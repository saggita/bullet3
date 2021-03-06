#include "btSoftBodySimulationSolverOpenCL.h"
#include "btSoftbodyCL.h"
#include "../RigidBody/b3GpuNarrowPhase.h"

#include "Bullet3OpenCL/Initialize/b3OpenCLInclude.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "kernels/SoftBodyKernals.h"


#define B3_SOFTBODY_KERNELS_PATH "src/Bullet3OpenCL/SoftBody/kernels/SoftBodyKernals.cl"
	
struct float4s
{	
	float x,y,z,w;		
};

B3_ATTRIBUTE_ALIGNED16(struct) btSoftBodyVertexCL
{
	B3_DECLARE_ALIGNED_ALLOCATOR();
	
	float4s m_Pos;
	float4s m_PosNext;
	float4s m_Vel;
	float4s m_Accel;

	float4s m_AABBMin;
	float4s m_AABBMax;
	
	float m_InvMass;
	unsigned int m_Index; // local index. Only unique inside the cloth which it belongs to.
	unsigned int m_IndexGlobal; // global index
	unsigned int m_PinIndex;
	unsigned int m_ClothIndex;
};

B3_ATTRIBUTE_ALIGNED16(struct) btSoftBodySpringCL
{
	B3_DECLARE_ALIGNED_ALLOCATOR();
	
	unsigned int m_Index; // local index. Only unique inside the cloth which it belongs to.
	unsigned int m_IndexGlobal; // global index
	unsigned int m_IndexGlobalVrx0; // global vertex index
	unsigned int m_IndexGlobalVrx1; // global vertex index

	float4s m_RestLength;

	unsigned int m_ClothIndex;
};


B3_ATTRIBUTE_ALIGNED16(struct) btSoftBodyLinkCL
{
	B3_DECLARE_ALIGNED_ALLOCATOR();

	unsigned int m_Index;
	float4s m_Pos;
};

B3_ATTRIBUTE_ALIGNED16(struct) btSoftBodyInfoCL
{
	B3_DECLARE_ALIGNED_ALLOCATOR();
	
	unsigned int m_Index;
	unsigned int m_NumVertices;
	unsigned int m_NumStretchSprings;
	unsigned int m_NumBendingSprings;
		
	float4s m_AABBMin;
	float4s m_AABBMax;

	unsigned int m_OffsetVertices;
	unsigned int m_OffsetStretchSprings;
	unsigned int m_OffsetBendingSprings;

	float m_Margin;
	float m_Kst;
	float m_Kb;
};

__inline
float4s ToFloat4s(float x, float y, float z, float w = 0.f)
{
	float4s v;
	v.x = x; v.y = y; v.z = z; v.w = w;
	return v;
}

__inline
float4s ToFloat4s(const b3Vector3& vec)
{
	float4s v;
	v.x = vec[0]; v.y = vec[1]; v.z = vec[2]; v.w = 0.f;
	return v;
}

__inline
b3Vector3 TobtVector3(const float4s& f4s)
{
	return b3Vector3(f4s.x, f4s.y, f4s.z);
}

inline float clamp(float val, float low, float high)
{
	if ( val < low )
		return low;
	else if ( val > high )
		return high;

	return val;
}

extern b3GpuNarrowPhase* narrowphaseAndSolver;

#define RELEASE_CL_KERNEL(kernelName) {if( kernelName ){ clReleaseKernel( kernelName ); kernelName = 0; }}

btSoftBodySimulationSolverOpenCL::btSoftBodySimulationSolverOpenCL(cl_context ctx, cl_device_id device, cl_command_queue  q) : 
m_context(ctx),
m_device(device),
m_queue(q),
m_bBuildCLKernels(false), m_HBVertexCL(NULL), m_HBStretchSpringCL(NULL), m_HBBendSpringCL(NULL),
m_HBClothInfoCL(NULL), m_DBVertices(NULL), m_DBStrechSprings(NULL), m_DBBendSprings(NULL), 
m_DBClothInfo(NULL), m_pMergedSoftBody(NULL), m_HBLinkCL(NULL)
{

	m_ClearForcesKernel = NULL;
	m_ComputeNextVertexPositionsKernel = NULL;
	m_ApplyGravityKernel = NULL;
	m_ApplyForcesKernel = NULL;
	m_EnforceEdgeConstraintsKernel = NULL;
	m_UpdateVelocitiesKernel = NULL;
	m_AdvancePositionKernel = NULL;
	m_UpdateVertexBoundingVolumeKernel = NULL;
	m_UpdateClothBoundingVolumeKernel = NULL;
	m_ResolveCollisionKernel = NULL;

	m_Gravity = b3Vector3(0, -9.8f, 0);
	m_NumIterForConstraintSolver = 10;
}

btSoftBodySimulationSolverOpenCL::~btSoftBodySimulationSolverOpenCL(void)
{
	if ( m_DBVertices )
		clReleaseMemObject(m_DBVertices);
	
	if ( m_DBStrechSprings )
		clReleaseMemObject(m_DBStrechSprings);

	if ( m_DBBendSprings )
		clReleaseMemObject(m_DBBendSprings);

	if ( m_DBClothInfo )
		clReleaseMemObject(m_DBClothInfo);

	releaseKernels();	
	
	if ( m_HBVertexCL )
		delete [] m_HBVertexCL;

	if ( m_HBStretchSpringCL )
		delete [] m_HBStretchSpringCL;

	if ( m_HBBendSpringCL )
		delete [] m_HBBendSpringCL;

	if ( m_HBClothInfoCL )
		delete [] m_HBClothInfoCL;

	if ( m_HBLinkCL )
		delete [] m_HBLinkCL;

	if ( m_pMergedSoftBody )
	{
		delete m_pMergedSoftBody;
		m_pMergedSoftBody = NULL;
	}
}

void btSoftBodySimulationSolverOpenCL::addSoftBody(btSoftbodyCL* pCloth) 
{ 
	m_clothArray.push_back(pCloth); 
}

bool btSoftBodySimulationSolverOpenCL::buildCLKernels()
{
	if ( m_bBuildCLKernels )
		return true;

	releaseKernels();

	const char* softbodyKernelsSrc = SoftBodyKernals;
	cl_int errNum=0;

	cl_program clProg = b3OpenCLUtils::compileCLProgramFromString(m_context,m_device,softbodyKernelsSrc,&errNum,"",B3_SOFTBODY_KERNELS_PATH);
	b3Assert(errNum==CL_SUCCESS);

	m_ClearForcesKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,softbodyKernelsSrc, "ClearForcesKernel",&errNum,clProg );
	m_ComputeNextVertexPositionsKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,softbodyKernelsSrc, "ComputeNextVertexPositionsKernel",&errNum,clProg );
	m_ApplyGravityKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,softbodyKernelsSrc, "ApplyGravityKernel",&errNum,clProg );
	m_ApplyForcesKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,softbodyKernelsSrc, "ApplyForcesKernel",&errNum,clProg );
	m_EnforceEdgeConstraintsKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,softbodyKernelsSrc, "EnforceEdgeConstraintsKernel",&errNum,clProg );
	m_UpdateVelocitiesKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,softbodyKernelsSrc, "UpdateVelocitiesKernel",&errNum,clProg );
	m_AdvancePositionKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,softbodyKernelsSrc, "AdvancePositionKernel",&errNum,clProg );
	m_UpdateVertexBoundingVolumeKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,softbodyKernelsSrc, "UpdateVertexBoundingVolumeKernel",&errNum,clProg );
	m_UpdateClothBoundingVolumeKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,softbodyKernelsSrc, "UpdateClothBoundingVolumeKernel",&errNum,clProg );
	m_ResolveCollisionKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,softbodyKernelsSrc, "ResolveCollisionKernel",&errNum,clProg );

	m_bBuildCLKernels = true;

	return m_bBuildCLKernels;
}

void btSoftBodySimulationSolverOpenCL::initialize()
{
	if ( m_numVertices == 0 )
		return;

	buildCLKernels();

	//-------------------------------------
	// Count vertices, springs and clothes
	//-------------------------------------
	m_numVertices = 0;
	m_numStretchSprings = 0;
	m_numBendingSprings = 0;
	m_numClothes = m_clothArray.size();

	for ( int i = 0; i < m_numClothes; i++ )
	{
		const btSoftbodyCL* pCloth = m_clothArray[i];
		m_numVertices += pCloth->getVertexArray().size();
		m_numStretchSprings += pCloth->getStrechSpringArray().size();
		m_numBendingSprings += pCloth->getBendSpringArray().size();
	}

	//---------------------
	// Buffer for vertices
	//---------------------
	m_HBVertexCL = new btSoftBodyVertexCL[m_numVertices];
	m_DBVertices = clCreateBuffer(m_context, CL_MEM_READ_WRITE, sizeof(btSoftBodyVertexCL) * m_numVertices, NULL, NULL);

	//----------------------------
	// Buffer for stretch springs
	//----------------------------
	m_HBStretchSpringCL = new btSoftBodySpringCL[m_numStretchSprings];	
	m_DBStrechSprings = clCreateBuffer(m_context, CL_MEM_READ_WRITE, sizeof(btSoftBodySpringCL) * m_numStretchSprings, NULL, NULL);

	//----------------------------
	// Buffer for bending springs
	//----------------------------
	m_HBBendSpringCL = new btSoftBodySpringCL[m_numBendingSprings];	
	m_DBBendSprings = clCreateBuffer(m_context, CL_MEM_READ_WRITE, sizeof(btSoftBodySpringCL) * m_numBendingSprings, NULL, NULL);

	//-----------------------
	// Buffer for cloth info
	//-----------------------
	m_HBClothInfoCL = new btSoftBodyInfoCL[m_numClothes];	
	m_DBClothInfo = clCreateBuffer(m_context, CL_MEM_READ_WRITE, sizeof(btSoftBodyInfoCL) * m_numClothes, NULL, NULL);

	initializeBoundingVolumes();
	mergeSoftBodies();
	generateBatches();
	updateBuffers();

	// We don't need m_pMergedSoftBody anymore.
	/*if ( m_pMergedSoftBody )
	{
		delete m_pMergedSoftBody;
		m_pMergedSoftBody = NULL;
	}*/
}

void btSoftBodySimulationSolverOpenCL::mergeSoftBodies()
{
	if ( m_clothArray.size() == 0 )
		return;

	// create temporary softbody objects to make one merged softbody.
	for ( int i = 0; i < m_clothArray.size(); i++ )
	{
		btSoftbodyCL* pNewSoftBody = new btSoftbodyCL(*m_clothArray[i]);
		m_tempClothArray.push_back(pNewSoftBody);
	}

	m_HBClothInfoCL = new btSoftBodyInfoCL[m_numClothes];	

	unsigned int offsetVertices = 0;
	unsigned int offsetStretchSprings = 0;
	unsigned int offsetBendingSprings = 0;

	for ( int i = 0; i < m_numClothes; i++ )
	{
		btSoftbodyCL* pCloth = m_clothArray[i];

		m_HBClothInfoCL[i].m_Index = i;
		
		m_HBClothInfoCL[i].m_NumVertices = pCloth->m_VertexArray.size();
		m_HBClothInfoCL[i].m_NumStretchSprings = pCloth->m_StrechSpringArray.size();
		m_HBClothInfoCL[i].m_NumBendingSprings = pCloth->m_BendSpringArray.size();

		m_HBClothInfoCL[i].m_OffsetVertices = offsetVertices;
		m_HBClothInfoCL[i].m_OffsetStretchSprings = offsetStretchSprings;
		m_HBClothInfoCL[i].m_OffsetBendingSprings = offsetBendingSprings;
		
		m_HBClothInfoCL[i].m_AABBMin = ToFloat4s(pCloth->m_Aabb.Min());
		m_HBClothInfoCL[i].m_AABBMax = ToFloat4s(pCloth->m_Aabb.Max());

		m_HBClothInfoCL[i].m_Margin = pCloth->getMargin();
		m_HBClothInfoCL[i].m_Kst = pCloth->getKst();
		m_HBClothInfoCL[i].m_Kb = pCloth->getKb();

		offsetVertices += pCloth->m_VertexArray.size();
		offsetStretchSprings += pCloth->m_StrechSpringArray.size();
		offsetBendingSprings += pCloth->m_BendSpringArray.size();
	}

	for ( int clothIndex = 0; clothIndex < m_numClothes; clothIndex++ )
	{
		btSoftbodyCL* pCloth = m_tempClothArray[clothIndex];

		// vertices
		for ( int vertIndexLocal = 0; vertIndexLocal < pCloth->getVertexArray().size(); vertIndexLocal++ )
		{
			// vertex index
			btSoftbodyNodeCL& vert = pCloth->getVertexArray()[vertIndexLocal];
			b3Assert(vert.m_Index == vertIndexLocal);
			vert.m_Index = getVertexIndexGlobal(vert.m_Index, clothIndex);

			// convert connected stretch spring indexes
			for ( int stretchIter = 0; stretchIter < vert.m_StrechSpringIndexes.size(); stretchIter++ )
			{
				int connectedStretchIndexLocal = vert.m_StrechSpringIndexes[stretchIter];
				vert.m_StrechSpringIndexes[stretchIter] = getStretchSpringIndexGlobal(connectedStretchIndexLocal, clothIndex);
			}

			// convert connected bending spring indexes
			for ( int bendingIter = 0; bendingIter < vert.m_BendSpringIndexes.size(); bendingIter++ )
			{
				int connectedBendingIndexLocal = vert.m_BendSpringIndexes[bendingIter];
				vert.m_BendSpringIndexes[bendingIter] = getBendingSpringIndexGlobal(connectedBendingIndexLocal, clothIndex);
			}
		}

		// stretch springs
		for ( int stretchIndexLocal = 0; stretchIndexLocal < pCloth->getStrechSpringArray().size(); stretchIndexLocal++ )
		{
			// spring index
			btSoftbodyLinkCL& spring = pCloth->getStrechSpringArray()[stretchIndexLocal];
			spring.SetIndex(getStretchSpringIndexGlobal(spring.GetIndex(), clothIndex));

			// connected two vertex indexes
			spring.m_IndexVrx[0] = getVertexIndexGlobal(spring.m_IndexVrx[0], clothIndex);
			spring.m_IndexVrx[1] = getVertexIndexGlobal(spring.m_IndexVrx[1], clothIndex);
		}

		// bending springs
		for ( int bendingIndexLocal = 0; bendingIndexLocal < pCloth->getBendSpringArray().size(); bendingIndexLocal++ )
		{
			// spring index
			btSoftbodyLinkCL& spring = pCloth->getBendSpringArray()[bendingIndexLocal];
			spring.SetIndex(getBendingSpringIndexGlobal(spring.GetIndex(), clothIndex));

			// connected two vertex indexes
			spring.m_IndexVrx[0] = getVertexIndexGlobal(spring.m_IndexVrx[0], clothIndex);
			spring.m_IndexVrx[1] = getVertexIndexGlobal(spring.m_IndexVrx[1], clothIndex);
		}
	}

	// merge softbodies into one 
	m_pMergedSoftBody = new btSoftbodyCL();

	for ( int clothIndex = 0; clothIndex < m_numClothes; clothIndex++ )
	{
		btSoftbodyCL* pCloth = m_tempClothArray[clothIndex];
		
		for ( int vertIter = 0; vertIter < pCloth->getVertexArray().size(); vertIter++ )
		{
			pCloth->getVertexArray()[vertIter].m_IndexCloth = clothIndex;
			m_pMergedSoftBody->getVertexArray().push_back(pCloth->getVertexArray()[vertIter]);
		}

		for ( int stretchIter = 0; stretchIter < pCloth->getStrechSpringArray().size(); stretchIter++ )
		{
			pCloth->getStrechSpringArray()[stretchIter].m_IndexCloth = clothIndex;
			m_pMergedSoftBody->getStrechSpringArray().push_back(pCloth->getStrechSpringArray()[stretchIter]);
		}

		for ( int bendingIter = 0; bendingIter < pCloth->getBendSpringArray().size(); bendingIter++ )
		{
			pCloth->getBendSpringArray()[bendingIter].m_IndexCloth = clothIndex;
			m_pMergedSoftBody->getBendSpringArray().push_back(pCloth->getBendSpringArray()[bendingIter]);
		}
	}

	b3Assert(m_numVertices == m_pMergedSoftBody->getVertexArray().size());
	b3Assert(m_numStretchSprings == m_pMergedSoftBody->getStrechSpringArray().size());
	b3Assert(m_numBendingSprings == m_pMergedSoftBody->getBendSpringArray().size());

	// clear temporary softbodyq objects. 
	for ( int i = 0; i < m_tempClothArray.size(); i++ )
	{
		delete m_tempClothArray[i];
	}

	m_tempClothArray.clear();

}

// Must be called after mergeSoftBodies().
void btSoftBodySimulationSolverOpenCL::generateBatches(bool bBatchEachSoftBodyFirst/*= false*/)
{

	m_pMergedSoftBody->generateBatches();

	m_BatchStretchSpringIndexGlobalArray = m_pMergedSoftBody->getBatchStretchSpringIndexArray();
	m_BatchBendSpringIndexGlobalArray = m_pMergedSoftBody->getBatchBendSpringIndexArray();

	//if ( bBatchEachSoftBodyFirst )
	//{
	//	for ( int i = 0; i < m_numClothes; i++ )
	//	{
	//		btSoftbodyCL* pCloth = m_clothArray[i];
	//		pCloth->GenerateBatches();
	//	}
	//}

	//m_BatchStretchSpringIndexGlobalArray.clear();
	//m_BatchBendSpringIndexGlobalArray.clear();

	//// stretch springs
	//int index = 0;

	//for ( int i = 0; i < m_numClothes; i++ )
	//{
	//	btSoftbodyCL* pCloth = m_clothArray[i];
	//	const b3AlignedObjectArray<int>& batchSprings = pCloth->GetBatchStretchSpringIndexArray();

	//	for ( int j = 0; j < batchSprings.size(); j++ )
	//	{
	//		int batchIndex =  m_HBClothInfoCL[i].m_OffsetStretchSprings +  batchSprings[index];
	//		m_BatchStretchSpringIndexGlobalArray.push_back(batchIndex);
	//	}
	//}

	//// bending springs
	//index = 0;

	//for ( int i = 0; i < m_numClothes; i++ )
	//{
	//	btSoftbodyCL* pCloth = m_clothArray[i];
	//	const b3AlignedObjectArray<int>& batchSprings = pCloth->GetBatchBendSpringIndexArray();

	//	for ( int j = 0; j < batchSprings.size(); j++ )
	//	{
	//		int batchIndex =  m_HBClothInfoCL[i].m_OffsetBendingSprings +  batchSprings[index];
	//		m_BatchBendSpringIndexGlobalArray.push_back(batchIndex);
	//	}
	//}
}

void btSoftBodySimulationSolverOpenCL::updateBuffers()
{
	if ( m_numVertices == 0 )
		return;

	b3Assert(m_pMergedSoftBody != NULL);
	cl_int result;

	//-----------------------
	// Buffer for cloth info
	//-----------------------
	result = clEnqueueWriteBuffer(m_queue, m_DBClothInfo, CL_TRUE, 0, sizeof(btSoftBodyInfoCL) * m_numClothes, m_HBClothInfoCL, 0, NULL, NULL);
	b3Assert(result == CL_SUCCESS);

	//---------------------
	// Buffer for vertices
	//---------------------	
	for ( int i = 0; i < m_numVertices; i++ )
	{
		btSoftBodyVertexCL vertexData;
		const btSoftbodyNodeCL& vert = m_pMergedSoftBody->m_VertexArray[i];
		
		vertexData.m_Index = vert.GetIndex();
		vertexData.m_IndexGlobal = vert.GetIndex();;
		vertexData.m_Pos = ToFloat4s(vert.m_Pos);
		vertexData.m_Vel = ToFloat4s(vert.m_Vel);
		vertexData.m_Accel = ToFloat4s(vert.m_Accel);
		vertexData.m_InvMass = vert.m_InvMass;
		vertexData.m_PinIndex = vert.m_PinIndex;
		vertexData.m_ClothIndex = vert.m_IndexCloth;

		m_HBVertexCL[i] = vertexData;
	}		
	
	result = clEnqueueWriteBuffer(m_queue, m_DBVertices, CL_TRUE, 0, sizeof(btSoftBodyVertexCL) * m_numVertices, m_HBVertexCL, 0, NULL, NULL);
	b3Assert(result == CL_SUCCESS);	

	//----------------------------
	// Buffer for stretch springs
	//----------------------------
	for ( int i = 0; i < m_numStretchSprings; i++ )
	{
		const btSoftbodyLinkCL& springData = m_pMergedSoftBody->m_StrechSpringArray[i];

		btSoftBodySpringCL springDataCL;

		springDataCL.m_Index = springData.GetIndex();
		springDataCL.m_IndexGlobal = springData.GetIndex();
		springDataCL.m_IndexGlobalVrx0 = springData.GetVertexIndex(0);
		springDataCL.m_IndexGlobalVrx1 = springData.GetVertexIndex(1);
		springDataCL.m_ClothIndex = springData.m_IndexCloth;
		springDataCL.m_RestLength.x = springData.GetRestLength();
		m_HBStretchSpringCL[i] = springDataCL;
	}

	result = clEnqueueWriteBuffer(m_queue, m_DBStrechSprings, CL_TRUE, 0, sizeof(btSoftBodySpringCL) * m_numStretchSprings, m_HBStretchSpringCL, 0, NULL, NULL);
	b3Assert(result == CL_SUCCESS);

	//----------------------------
	// Buffer for bending springs
	//----------------------------
	for ( int i = 0; i < m_numBendingSprings; i++ )
	{
		const btSoftbodyLinkCL& springData = m_pMergedSoftBody->getBendSpringArray()[i];

		btSoftBodySpringCL springDataCL;

		springDataCL.m_Index = springData.GetIndex();
		springDataCL.m_IndexGlobal = springData.GetIndex();
		springDataCL.m_IndexGlobalVrx0 = springData.GetVertexIndex(0);
		springDataCL.m_IndexGlobalVrx1 = springData.GetVertexIndex(1);
		springDataCL.m_ClothIndex = springData.m_IndexCloth;
		springDataCL.m_RestLength.x = springData.GetRestLength();
		m_HBBendSpringCL[i] = springDataCL;
	}
		
	result = clEnqueueWriteBuffer(m_queue, m_DBBendSprings, CL_TRUE, 0, sizeof(btSoftBodySpringCL) * m_numBendingSprings, m_HBBendSpringCL, 0, NULL, NULL);
	b3Assert(result == CL_SUCCESS);
}

int btSoftBodySimulationSolverOpenCL::getVertexIndexGlobal(int vertexIndexLocal, int clothIndex)
{
	return m_HBClothInfoCL[clothIndex].m_OffsetVertices + vertexIndexLocal;
}

int btSoftBodySimulationSolverOpenCL::getStretchSpringIndexGlobal(int stretchSpringIndexLocal, int clothIndex)
{
	return m_HBClothInfoCL[clothIndex].m_OffsetStretchSprings + stretchSpringIndexLocal;
}

int btSoftBodySimulationSolverOpenCL::getBendingSpringIndexGlobal(int bendingSpringIndexLocal, int clothIndex)
{
	return m_HBClothInfoCL[clothIndex].m_OffsetBendingSprings + bendingSpringIndexLocal;
}

bool btSoftBodySimulationSolverOpenCL::integrate(float dt)
{
	if ( m_numVertices == 0 )
		return true;
	
	//-------------------
	// ClearForcesKernel
	//-------------------
	{
		cl_int ciErrNum;	
		ciErrNum = clSetKernelArg(m_ClearForcesKernel, 0, sizeof(unsigned int), &m_numVertices);
		ciErrNum = clSetKernelArg(m_ClearForcesKernel, 1, sizeof(float), &dt);
		ciErrNum = clSetKernelArg(m_ClearForcesKernel, 2, sizeof(cl_mem), &m_DBVertices);

		b3Assert(ciErrNum == CL_SUCCESS);
		
		size_t m_defaultWorkGroupSize = 64;
		size_t numWorkItems = m_defaultWorkGroupSize*((m_numVertices + (m_defaultWorkGroupSize-1)) / m_defaultWorkGroupSize);

		clEnqueueNDRangeKernel(m_queue, m_ClearForcesKernel, 1, NULL, &numWorkItems, &m_defaultWorkGroupSize, 0,0,0);
	}
	
	//-------------------
	// ApplyGravityKernel
	//-------------------
	{
		float4s gravity = ToFloat4s(m_Gravity);

		cl_int ciErrNum;	
		ciErrNum = clSetKernelArg(m_ApplyGravityKernel, 0, sizeof(unsigned int), &m_numVertices);
		ciErrNum = clSetKernelArg(m_ApplyGravityKernel, 1, sizeof(float4s), &gravity);
		ciErrNum = clSetKernelArg(m_ApplyGravityKernel, 2, sizeof(float), &dt);
		ciErrNum = clSetKernelArg(m_ApplyGravityKernel, 3, sizeof(cl_mem), &m_DBVertices);

		b3Assert(ciErrNum == CL_SUCCESS);
		
		size_t m_defaultWorkGroupSize = 64;
		size_t numWorkItems = m_defaultWorkGroupSize*((m_numVertices + (m_defaultWorkGroupSize-1)) / m_defaultWorkGroupSize);

		clEnqueueNDRangeKernel(m_queue, m_ApplyGravityKernel, 1, NULL, &numWorkItems, &m_defaultWorkGroupSize, 0,0,0);
	}

	//------------------
	// ApplyForcesKernel
	//------------------
	{
		cl_int ciErrNum;	
		ciErrNum = clSetKernelArg(m_ApplyForcesKernel, 0, sizeof(unsigned int), &m_numVertices);
		ciErrNum = clSetKernelArg(m_ApplyForcesKernel, 1, sizeof(float), &dt);
		ciErrNum = clSetKernelArg(m_ApplyForcesKernel, 2, sizeof(cl_mem), &m_DBVertices);

		b3Assert(ciErrNum == CL_SUCCESS);
		
		size_t m_defaultWorkGroupSize = 64;
		size_t numWorkItems = m_defaultWorkGroupSize*((m_numVertices + (m_defaultWorkGroupSize-1)) / m_defaultWorkGroupSize);

		clEnqueueNDRangeKernel(m_queue, m_ApplyForcesKernel, 1, NULL, &numWorkItems, &m_defaultWorkGroupSize, 0,0,0);
	}

	//-------------------
	// ClearForcesKernel
	//-------------------
	{
		cl_int ciErrNum;	
		ciErrNum = clSetKernelArg(m_ClearForcesKernel, 0, sizeof(unsigned int), &m_numVertices);
		ciErrNum = clSetKernelArg(m_ClearForcesKernel, 1, sizeof(float), &dt);
		ciErrNum = clSetKernelArg(m_ClearForcesKernel, 2, sizeof(cl_mem), &m_DBVertices);

		b3Assert(ciErrNum == CL_SUCCESS);
		
		size_t m_defaultWorkGroupSize = 64;
		size_t numWorkItems = m_defaultWorkGroupSize*((m_numVertices + (m_defaultWorkGroupSize-1)) / m_defaultWorkGroupSize);

		clEnqueueNDRangeKernel(m_queue, m_ClearForcesKernel, 1, NULL, &numWorkItems, &m_defaultWorkGroupSize, 0,0,0);
	}

	//----------------------------------
	// ComputeNextVertexPositionsKernel
	//----------------------------------
	{
		cl_int ciErrNum;	
		ciErrNum = clSetKernelArg(m_ComputeNextVertexPositionsKernel, 0, sizeof(unsigned int), &m_numVertices);
		ciErrNum = clSetKernelArg(m_ComputeNextVertexPositionsKernel, 1, sizeof(float), &dt);
		ciErrNum = clSetKernelArg(m_ComputeNextVertexPositionsKernel, 2, sizeof(cl_mem), &m_DBVertices);

		b3Assert(ciErrNum == CL_SUCCESS);
		
		size_t m_defaultWorkGroupSize = 64;
		size_t numWorkItems = m_defaultWorkGroupSize*((m_numVertices + (m_defaultWorkGroupSize-1)) / m_defaultWorkGroupSize);

		clEnqueueNDRangeKernel(m_queue, m_ComputeNextVertexPositionsKernel, 1, NULL, &numWorkItems, &m_defaultWorkGroupSize, 0,0,0);
	}

	//-------------------------------
	// EnforceEdgeConstraintsKernel
	//-------------------------------

	// below code should be moved to kernel
	//b3Assert(0 <= m_pCloth->m_Kst && m_pCloth->m_Kst <= 1.0f);
	//b3Assert(0 <= m_pCloth->m_Kb && m_pCloth->m_Kb <= 1.0f);

	//float Kst = 1.0f - pow((1.0f - m_pCloth->m_Kst), 1.0f/m_pCloth->m_NumIterForConstraintSolver);
	//float Kb = 1.0f - pow((1.0f - m_pCloth->m_Kb), 1.0f/m_pCloth->m_NumIterForConstraintSolver);

	///*float Kst = m_Kst;
	//float Kb = m_Kb;*/

	//clamp(Kst, 0, 1.0f);
	//clamp(Kb, 0, 1.0f);
	
	int numIteration = 0;

	while ( numIteration < m_NumIterForConstraintSolver )
	{
		// stretch springs
		int springType = 0; // stretch

		for ( int batch = 0; batch < m_BatchStretchSpringIndexGlobalArray.size()-1; batch++ )
		{
			int startSpringIndex = m_BatchStretchSpringIndexGlobalArray[batch];
			int endSpringIndex = m_BatchStretchSpringIndexGlobalArray[batch+1]-1;
			int numSpringsInBatch = endSpringIndex - startSpringIndex + 1;

			cl_int ciErrNum;	
			ciErrNum = clSetKernelArg(m_EnforceEdgeConstraintsKernel, 0, sizeof(unsigned int), &numSpringsInBatch);
			ciErrNum = clSetKernelArg(m_EnforceEdgeConstraintsKernel, 1, sizeof(unsigned int), &startSpringIndex);
			ciErrNum = clSetKernelArg(m_EnforceEdgeConstraintsKernel, 2, sizeof(float), &dt);
			ciErrNum = clSetKernelArg(m_EnforceEdgeConstraintsKernel, 3, sizeof(int), &springType);
			ciErrNum = clSetKernelArg(m_EnforceEdgeConstraintsKernel, 4, sizeof(cl_mem), &m_DBClothInfo);
			ciErrNum = clSetKernelArg(m_EnforceEdgeConstraintsKernel, 5, sizeof(cl_mem), &m_DBVertices);
			ciErrNum = clSetKernelArg(m_EnforceEdgeConstraintsKernel, 6, sizeof(cl_mem), &m_DBStrechSprings);
		
			b3Assert(ciErrNum == CL_SUCCESS);
		
			size_t m_defaultWorkGroupSize = 64;
			size_t numWorkItems = m_defaultWorkGroupSize*((numSpringsInBatch + (m_defaultWorkGroupSize-1)) / m_defaultWorkGroupSize);

			clEnqueueNDRangeKernel(m_queue, m_EnforceEdgeConstraintsKernel, 1, NULL, &numWorkItems, &m_defaultWorkGroupSize, 0,0,0);
		}
	
		clFinish(m_queue);

		// bending springs
		springType = 1; // bending
		for ( int batch = 0; batch < m_BatchBendSpringIndexGlobalArray.size()-1; batch++ )
		{
			int startSpringIndex = m_BatchBendSpringIndexGlobalArray[batch];
			int endSpringIndex = m_BatchBendSpringIndexGlobalArray[batch+1]-1;
			int numSpringsInBatch = endSpringIndex - startSpringIndex + 1;

			cl_int ciErrNum;	
			ciErrNum = clSetKernelArg(m_EnforceEdgeConstraintsKernel, 0, sizeof(unsigned int), &numSpringsInBatch);
			ciErrNum = clSetKernelArg(m_EnforceEdgeConstraintsKernel, 1, sizeof(unsigned int), &startSpringIndex);
			ciErrNum = clSetKernelArg(m_EnforceEdgeConstraintsKernel, 2, sizeof(float), &dt);
			ciErrNum = clSetKernelArg(m_EnforceEdgeConstraintsKernel, 3, sizeof(int), &springType);
			ciErrNum = clSetKernelArg(m_EnforceEdgeConstraintsKernel, 4, sizeof(cl_mem), &m_DBClothInfo);
			ciErrNum = clSetKernelArg(m_EnforceEdgeConstraintsKernel, 5, sizeof(cl_mem), &m_DBVertices);
			ciErrNum = clSetKernelArg(m_EnforceEdgeConstraintsKernel, 6, sizeof(cl_mem), &m_DBBendSprings);
		
			b3Assert(ciErrNum == CL_SUCCESS);
		
			size_t m_defaultWorkGroupSize = 64;
			size_t numWorkItems = m_defaultWorkGroupSize*((numSpringsInBatch + (m_defaultWorkGroupSize-1)) / m_defaultWorkGroupSize);

			clEnqueueNDRangeKernel(m_queue, m_EnforceEdgeConstraintsKernel, 1, NULL, &numWorkItems, &m_defaultWorkGroupSize, 0,0,0);
		}

		clFinish(m_queue);
			
		++numIteration;
	}

	//-----------------------
	// UpdateVelocitiesKernel
	//-----------------------
	{
		cl_int ciErrNum;	
		ciErrNum = clSetKernelArg(m_UpdateVelocitiesKernel, 0, sizeof(unsigned int), &m_numVertices);
		ciErrNum = clSetKernelArg(m_UpdateVelocitiesKernel, 1, sizeof(float), &dt);
		ciErrNum = clSetKernelArg(m_UpdateVelocitiesKernel, 2, sizeof(cl_mem), &m_DBVertices);

		b3Assert(ciErrNum == CL_SUCCESS);
		
		size_t m_defaultWorkGroupSize = 64;
		size_t numWorkItems = m_defaultWorkGroupSize*((m_numVertices + (m_defaultWorkGroupSize-1)) / m_defaultWorkGroupSize);

		clEnqueueNDRangeKernel(m_queue, m_UpdateVelocitiesKernel, 1, NULL, &numWorkItems, &m_defaultWorkGroupSize, 0,0,0);
	}

	return true;
}

bool btSoftBodySimulationSolverOpenCL::advancePosition(float dt)
{	
	if ( m_numVertices == 0 )
		return true;

	//-----------------------
	// AdvancePositionKernel
	//-----------------------
	{
		cl_int ciErrNum;	
		ciErrNum = clSetKernelArg(m_AdvancePositionKernel, 0, sizeof(unsigned int), &m_numVertices);
		ciErrNum = clSetKernelArg(m_AdvancePositionKernel, 1, sizeof(float), &dt);
		ciErrNum = clSetKernelArg(m_AdvancePositionKernel, 2, sizeof(cl_mem), &m_DBVertices);

		b3Assert(ciErrNum == CL_SUCCESS);
		
		size_t m_defaultWorkGroupSize = 64;
		size_t numWorkItems = m_defaultWorkGroupSize*((m_numVertices + (m_defaultWorkGroupSize-1)) / m_defaultWorkGroupSize);

		clEnqueueNDRangeKernel(m_queue, m_AdvancePositionKernel, 1, NULL, &numWorkItems, &m_defaultWorkGroupSize, 0,0,0);
	}

	return true;
}

bool btSoftBodySimulationSolverOpenCL::resolveCollision(float dt)
{
	if ( m_numVertices == 0 )
		return true;

	//const CustomDispatchData* narrowphaseData = narrowphaseAndSolver->getCustomDispatchData();

	//cl_mem bodies = narrowphaseData->m_bodyBufferGPU->getBufferCL();
	//cl_mem collidables = narrowphaseData->m_collidablesGPU->getBufferCL();
	//cl_mem convexPolyhedra = narrowphaseData->m_convexPolyhedraGPU->getBufferCL();
	//cl_mem faces = narrowphaseData->m_convexFacesGPU->getBufferCL();
	//cl_mem convexIndices = narrowphaseData->m_convexIndicesGPU->getBufferCL();
	//cl_mem convexVertices = narrowphaseData->m_convexVerticesGPU->getBufferCL();

	//int numRigidBodies = narrowphaseData->m_bodyBufferGPU->size();

	////-----------------------
	//// ResolveCollisionKernel
	////-----------------------
	//{
	//	cl_int ciErrNum;	
	//	ciErrNum = clSetKernelArg(m_ResolveCollisionKernel, 0, sizeof(unsigned int), &m_numVertices);
	//	ciErrNum = clSetKernelArg(m_ResolveCollisionKernel, 1, sizeof(float), &dt);
	//	ciErrNum = clSetKernelArg(m_ResolveCollisionKernel, 2, sizeof(cl_mem), &m_DBVertices);
	//	ciErrNum = clSetKernelArg(m_ResolveCollisionKernel, 3, sizeof(cl_mem), (void*)(&bodies));
	//	ciErrNum = clSetKernelArg(m_ResolveCollisionKernel, 4, sizeof(unsigned int), &numRigidBodies);
	//	ciErrNum = clSetKernelArg(m_ResolveCollisionKernel, 5, sizeof(cl_mem), (void*)(&collidables));
	//	ciErrNum = clSetKernelArg(m_ResolveCollisionKernel, 6, sizeof(cl_mem), (void*)(&convexPolyhedra));
	//	ciErrNum = clSetKernelArg(m_ResolveCollisionKernel, 7, sizeof(cl_mem), (void*)(&faces));
	//	ciErrNum = clSetKernelArg(m_ResolveCollisionKernel, 8, sizeof(cl_mem), (void*)(&convexIndices));
	//	ciErrNum = clSetKernelArg(m_ResolveCollisionKernel, 9, sizeof(cl_mem), (void*)(&convexVertices));

	//	b3Assert(ciErrNum == CL_SUCCESS);
	//	
	//	size_t m_defaultWorkGroupSize = 64;
	//	size_t numWorkItems = m_defaultWorkGroupSize*((m_numVertices + (m_defaultWorkGroupSize-1)) / m_defaultWorkGroupSize);

	//	clEnqueueNDRangeKernel(m_queue, m_ResolveCollisionKernel, 1, NULL, &numWorkItems, &m_defaultWorkGroupSize, 0,0,0);
	//}

	return true;
}

// Assumes planeEqn[0], planeEqn[1] and planeEqn[2] forms unit normal vector.
b3Scalar signedDistanceFromPointToPlane(const b3Vector3& point, const b3Scalar* planeEqn, b3Vector3* closestPointInFace/* = NULL*/)
{
	b3Vector3 n(planeEqn[0], planeEqn[1], planeEqn[2]);

	if ( n.length2() < 1e-6 )
		return 0;

	if ( point == b3Vector3(0, 0, 0) )
	{
		b3Scalar dist = planeEqn[3];

		if ( closestPointInFace )
			*closestPointInFace = - dist * n;

		return dist;
	}
	else
	{
		b3Scalar dist = b3Dot(n, point) + planeEqn[3];

		if ( closestPointInFace )
			*closestPointInFace = point - dist * n;

		return dist;
	}
}

bool btSoftBodySimulationSolverOpenCL::resolveCollisionCPU(float dt)
{
	//ReadBackFromGPU();

	//const CustomDispatchData* narrowphaseData = narrowphaseAndSolver->getCustomDispatchData();
	//const b3AlignedObjectArray<RigidBodyBase::Body>& bodyArrayCPU = *narrowphaseData->m_bodyBufferCPU;
	//const b3AlignedObjectArray<btCollidable>& collidables = narrowphaseData->m_collidablesCPU;
	//const b3AlignedObjectArray<ConvexPolyhedronCL>& convexPolyhedra = narrowphaseData->m_convexPolyhedra;
	//const b3AlignedObjectArray<btGpuFace>& faces = narrowphaseData->m_convexFaces;
	//const b3AlignedObjectArray<int>& convexIndices = narrowphaseData->m_convexIndices;
	//const b3AlignedObjectArray<b3Vector3>& convexVertices = narrowphaseData->m_convexVertices;

	//for ( int index = 0; index < m_numVertices; index++ )
	//{
	//	btSoftBodyVertexCL& vertexData = m_HBVertexCL[index];
	//	index++;

	//	b3Vector3 vertPos = b3Vector3(vertexData.m_Pos.x, vertexData.m_Pos.y, vertexData.m_Pos.z);

	//	// check if vertex is colliding with rigidbody using face info from convex polyhedron.
	//	

	//	for ( int i = 0; i < bodyArrayCPU.size(); i++ )
	//	{
	//		const RigidBodyBase::Body& body = bodyArrayCPU[i];

	//		u32 collidableIndex = body.m_collidableIdx;

	//		b3Vector3 pos(body.m_pos.x, body.m_pos.y, body.m_pos.z);
	//		btQuaternion rot(body.m_quat.x, body.m_quat.y, body.m_quat.z, body.m_quat.w);
	//		
	//		btTransform tr(rot, pos);

	//		//int shapeType = collidables[collidableIndex].m_shapeType;
	//		int shapeIndex = collidables[collidableIndex].m_shapeIndex;
	//		const ConvexPolyhedronCL& convexShape = convexPolyhedra[shapeIndex];

	//		int numFaces = convexShape.m_numFaces;			
	//		btTransform trRot(rot, b3Vector3(0, 0, 0));
	//		b3Vector3 closestPnt;
	//		float minDist = -BT_LARGE_FLOAT;
	//		bool bCollide = true;

	//		for ( int f = 0; f < numFaces; f++ ) 
	//		{
	//			// plane equation
	//			b3Scalar planeEqn[4];

	//			const btGpuFace& face = faces[convexShape.m_faceOffset + f];
	//			b3Vector3 n(face.m_plane.x, face.m_plane.y, face.m_plane.z);
	//			n = trRot*n;

	//			planeEqn[0] = n[0];
	//			planeEqn[1] = n[1];
	//			planeEqn[2] = n[2];
	//							
	//			b3Vector3 v = tr * convexVertices[convexShape.m_vertexOffset + convexIndices[face.m_indexOffset + 0]];

	//			planeEqn[3] = -btDot(n, v);

	//			b3Vector3 pntReturn;
	//			b3Scalar dist = signedDistanceFromPointToPlane(vertPos, planeEqn, &pntReturn);

	//			// If the distance is positive, the plane is a separating plane. 
	//			if ( dist > 0 )
	//			{
	//				bCollide = false;
	//				break;
	//			}

	//			if ( dist > minDist )
	//			{
	//				minDist = dist;
	//				closestPnt = pntReturn;
	//			}
	//		}

	//		// If there is a collision
	//		if ( bCollide )
	//		{
	//			vertexData.m_Pos = ToFloat4s(closestPnt);
	//			vertexData.m_PosNext = vertexData.m_Pos;
	//			vertexData.m_Vel = ToFloat4s(b3Vector3(0, 0, 0)); // TODO: the velocity should be the one from the closted point.
	//		}
	//	}
	//	
	//}

	//cl_int result = clEnqueueWriteBuffer(m_queue, m_DBVertices, CL_TRUE, 0, sizeof(btSoftBodyVertexCL) * m_numVertices, m_HBVertexCL, 0, NULL, NULL);
	//b3Assert(result == CL_SUCCESS);

	return true;
}

bool btSoftBodySimulationSolverOpenCL::readBackFromGPU()
{
	if ( m_numVertices == 0 )
		return true;

	//------------------------
	// Read data back to CPU
	//------------------------
	clFinish(m_queue);

	{
		cl_int ciErrNum = clEnqueueReadBuffer(m_queue, m_DBVertices, CL_TRUE, 0, sizeof(btSoftBodyVertexCL) * m_numVertices, m_HBVertexCL, 0, NULL, NULL);
	}
	
	{		
		cl_int ciErrNum = clEnqueueReadBuffer(m_queue, m_DBClothInfo, CL_TRUE, 0, sizeof(btSoftBodyInfoCL) * m_numClothes, m_HBClothInfoCL, 0, NULL, NULL);
	}

	// TODO: Even though ciErrNum is 0, the following line doesn't work. 
	/*if ( ciErrNum != CL_SUCCESS );
		return false;*/


	int index = 0;
	for ( int i = 0; i < m_numClothes; i++ )
	{
		btSoftbodyCL* pCloth = m_clothArray[i];

		for ( int j = 0; j < pCloth->getVertexArray().size(); j++ )
		{
			const btSoftBodyVertexCL& vertexData = m_HBVertexCL[index];
			index++;

			pCloth->m_VertexArray[j].m_Pos = b3Vector3(vertexData.m_Pos.x, vertexData.m_Pos.y, vertexData.m_Pos.z);
			pCloth->m_VertexArray[j].m_PosNext = b3Vector3(vertexData.m_PosNext.x, vertexData.m_PosNext.y, vertexData.m_PosNext.z);
			pCloth->m_VertexArray[j].m_Vel = b3Vector3(vertexData.m_Vel.x, vertexData.m_Vel.y, vertexData.m_Vel.z);
			pCloth->m_AABBVertexArray[j].Min() = b3Vector3(vertexData.m_AABBMin.x, vertexData.m_AABBMin.y, vertexData.m_AABBMin.z);
			pCloth->m_AABBVertexArray[j].Max() = b3Vector3(vertexData.m_AABBMax.x, vertexData.m_AABBMax.y, vertexData.m_AABBMax.z);
		}

		//pCloth->updateSoftBodyCPU();

		pCloth->m_Aabb.Min() = TobtVector3(m_HBClothInfoCL[i].m_AABBMin);
		pCloth->m_Aabb.Max() = TobtVector3(m_HBClothInfoCL[i].m_AABBMax);

		// TODO:Below is a CPU version of updating AABB. Should be removed. 
		pCloth->m_Aabb.Empty();

		for ( int j = 0; j < pCloth->m_AABBVertexArray.size(); j++ )
		{
			pCloth->m_Aabb += pCloth->m_AABBVertexArray[j];
		}
	}
	
	return true;
}

//bool btSoftBodySimulationSolverOpenCL::ResolveCollision(CCollisionObject& convexObject, float dt)
//{
//	return btSoftbodyCL::ResolveCollision(convexObject, dt);
//}

void btSoftBodySimulationSolverOpenCL::initializeBoundingVolumes()
{
	for ( int i = 0; i < m_numClothes; i++ )
	{
		btSoftbodyCL* pCloth = m_clothArray[i];
		pCloth->initializeBoundingVolumes();
	}
}

void btSoftBodySimulationSolverOpenCL::updateBoundingVolumes(float dt)
{	
	if ( m_numVertices == 0 )
		return;

	//----------------------------------
	// UpdateVertexBoundingVolumeKernel
	//----------------------------------
	{
		cl_int ciErrNum;	
		ciErrNum = clSetKernelArg(m_UpdateVertexBoundingVolumeKernel, 0, sizeof(unsigned int), &m_numVertices);
		ciErrNum = clSetKernelArg(m_UpdateVertexBoundingVolumeKernel, 1, sizeof(float), &dt);
		ciErrNum = clSetKernelArg(m_UpdateVertexBoundingVolumeKernel, 2, sizeof(cl_mem), &m_DBVertices);
		ciErrNum = clSetKernelArg(m_UpdateVertexBoundingVolumeKernel, 3, sizeof(cl_mem), &m_DBClothInfo);

		b3Assert(ciErrNum == CL_SUCCESS);
		
		size_t m_defaultWorkGroupSize = 64;
		size_t numWorkItems = m_defaultWorkGroupSize*((m_numVertices + (m_defaultWorkGroupSize-1)) / m_defaultWorkGroupSize);

		clEnqueueNDRangeKernel(m_queue, m_UpdateVertexBoundingVolumeKernel, 1, NULL, &numWorkItems, &m_defaultWorkGroupSize, 0,0,0);
	}

	//---------------------------------
	// UpdateClothBoundingVolumeKernel
	//---------------------------------
	clFinish(m_queue);
	
	{
		cl_int ciErrNum;	
		ciErrNum = clSetKernelArg(m_UpdateClothBoundingVolumeKernel, 0, sizeof(unsigned int), &m_numClothes);
		ciErrNum = clSetKernelArg(m_UpdateClothBoundingVolumeKernel, 1, sizeof(cl_mem), &m_DBClothInfo);
		ciErrNum = clSetKernelArg(m_UpdateClothBoundingVolumeKernel, 2, sizeof(cl_mem), &m_DBVertices);

		b3Assert(ciErrNum == CL_SUCCESS);
		
		size_t m_defaultWorkGroupSize = 64;
		size_t numWorkItems = m_defaultWorkGroupSize*((m_numClothes + (m_defaultWorkGroupSize-1)) / m_defaultWorkGroupSize);

		clEnqueueNDRangeKernel(m_queue, m_UpdateClothBoundingVolumeKernel, 1, NULL, &numWorkItems, &m_defaultWorkGroupSize, 0,0,0);
	}
}

void btSoftBodySimulationSolverOpenCL::releaseKernels()
{
	if ( !m_bBuildCLKernels )
		return;

	RELEASE_CL_KERNEL(m_ClearForcesKernel);
	RELEASE_CL_KERNEL(m_ComputeNextVertexPositionsKernel);	
	RELEASE_CL_KERNEL(m_ApplyGravityKernel);
	RELEASE_CL_KERNEL(m_ApplyForcesKernel);
	RELEASE_CL_KERNEL(m_EnforceEdgeConstraintsKernel);
	RELEASE_CL_KERNEL(m_UpdateVelocitiesKernel);
	RELEASE_CL_KERNEL(m_AdvancePositionKernel);
	RELEASE_CL_KERNEL(m_UpdateVertexBoundingVolumeKernel);
	RELEASE_CL_KERNEL(m_UpdateClothBoundingVolumeKernel);
	RELEASE_CL_KERNEL(m_ResolveCollisionKernel);
}





