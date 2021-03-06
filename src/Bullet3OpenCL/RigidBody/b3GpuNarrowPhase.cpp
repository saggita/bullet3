#include "b3GpuNarrowPhase.h"


#include "Bullet3OpenCL/ParallelPrimitives/b3OpenCLArray.h"
#include "Bullet3OpenCL/NarrowphaseCollision/b3ConvexPolyhedronCL.h"
#include "Bullet3OpenCL/NarrowphaseCollision/b3ConvexHullContact.h"
#include "Bullet3OpenCL/BroadphaseCollision/b3SapAabb.h"
#include <string.h>
#include "b3Config.h"
#include "Bullet3OpenCL/NarrowphaseCollision/b3OptimizedBvh.h"
#include "Bullet3OpenCL/NarrowphaseCollision/b3TriangleIndexVertexArray.h"
#include "Bullet3Geometry/b3AabbUtil.h"
#include "Bullet3OpenCL/NarrowphaseCollision/b3BvhInfo.h"

struct b3GpuNarrowPhaseInternalData
{
	b3AlignedObjectArray<b3ConvexUtility*>* m_convexData;
    
	b3AlignedObjectArray<b3ConvexPolyhedronCL> m_convexPolyhedra;
	b3AlignedObjectArray<b3Vector3> m_uniqueEdges;
	b3AlignedObjectArray<b3Vector3> m_convexVertices;
	b3AlignedObjectArray<int> m_convexIndices;
    
	b3OpenCLArray<b3ConvexPolyhedronCL>* m_convexPolyhedraGPU;
	b3OpenCLArray<b3Vector3>* m_uniqueEdgesGPU;
	b3OpenCLArray<b3Vector3>* m_convexVerticesGPU;
	b3OpenCLArray<int>* m_convexIndicesGPU;
    
    b3OpenCLArray<b3Vector3>* m_worldVertsB1GPU;
    b3OpenCLArray<b3Int4>* m_clippingFacesOutGPU;
    b3OpenCLArray<b3Vector3>* m_worldNormalsAGPU;
    b3OpenCLArray<b3Vector3>* m_worldVertsA1GPU;
    b3OpenCLArray<b3Vector3>* m_worldVertsB2GPU;
    
	b3AlignedObjectArray<b3GpuChildShape> m_cpuChildShapes;
	b3OpenCLArray<b3GpuChildShape>*	m_gpuChildShapes;
    
	b3AlignedObjectArray<b3GpuFace> m_convexFaces;
	b3OpenCLArray<b3GpuFace>* m_convexFacesGPU;
    
	GpuSatCollision*	m_gpuSatCollision;
	    
	b3AlignedObjectArray<b3Int2>* m_pBufPairsCPU;
    
	b3OpenCLArray<b3Int2>* m_convexPairsOutGPU;
	b3OpenCLArray<b3Int2>* m_planePairs;
    
	b3OpenCLArray<b3Contact4>* m_pBufContactOutGPU;
	b3AlignedObjectArray<b3Contact4>* m_pBufContactOutCPU;
	
    
	b3AlignedObjectArray<b3RigidBodyCL>* m_bodyBufferCPU;
	b3OpenCLArray<b3RigidBodyCL>* m_bodyBufferGPU;
    
	b3AlignedObjectArray<b3InertiaCL>*	m_inertiaBufferCPU;
	b3OpenCLArray<b3InertiaCL>*	m_inertiaBufferGPU;
    
	int m_numAcceleratedShapes;
	int m_numAcceleratedRigidBodies;
    
	b3AlignedObjectArray<b3Collidable>	m_collidablesCPU;
	b3OpenCLArray<b3Collidable>*	m_collidablesGPU;

	b3OpenCLArray<b3SapAabb>* m_localShapeAABBGPU;
	b3AlignedObjectArray<b3SapAabb>* m_localShapeAABBCPU;

	b3AlignedObjectArray<class b3OptimizedBvh*> m_bvhData;

	b3AlignedObjectArray<b3QuantizedBvhNode>	m_treeNodesCPU;
	b3AlignedObjectArray<b3BvhSubtreeInfo>	m_subTreesCPU;

	b3AlignedObjectArray<b3BvhInfo>	m_bvhInfoCPU;
	b3OpenCLArray<b3BvhInfo>*			m_bvhInfoGPU;
	
	b3OpenCLArray<b3QuantizedBvhNode>*	m_treeNodesGPU;
	b3OpenCLArray<b3BvhSubtreeInfo>*	m_subTreesGPU;
	

	b3Config	m_config;
    
};





b3GpuNarrowPhase::b3GpuNarrowPhase(cl_context ctx, cl_device_id device, cl_command_queue queue, const b3Config& config)
:m_data(0) ,m_planeBodyIndex(-1),m_static0Index(-1),
m_context(ctx),
m_device(device),
m_queue(queue)
{
    
	m_data = new b3GpuNarrowPhaseInternalData();
	memset(m_data,0,sizeof(b3GpuNarrowPhaseInternalData));
    
	m_data->m_config = config;
	
	m_data->m_gpuSatCollision = new GpuSatCollision(ctx,device,queue);
	m_data->m_pBufPairsCPU = new b3AlignedObjectArray<b3Int2>;
	m_data->m_pBufPairsCPU->resize(config.m_maxBroadphasePairs);
	
	m_data->m_convexPairsOutGPU = new b3OpenCLArray<b3Int2>(ctx,queue,config.m_maxBroadphasePairs,false);
	m_data->m_planePairs = new b3OpenCLArray<b3Int2>(ctx,queue,config.m_maxBroadphasePairs,false);
    
	m_data->m_pBufContactOutCPU = new b3AlignedObjectArray<b3Contact4>();
	m_data->m_pBufContactOutCPU->resize(config.m_maxBroadphasePairs);
	m_data->m_bodyBufferCPU = new b3AlignedObjectArray<b3RigidBodyCL>();
	m_data->m_bodyBufferCPU->resize(config.m_maxConvexBodies);
    
	m_data->m_inertiaBufferCPU = new b3AlignedObjectArray<b3InertiaCL>();
	m_data->m_inertiaBufferCPU->resize(config.m_maxConvexBodies);
	
	m_data->m_pBufContactOutGPU = new b3OpenCLArray<b3Contact4>(ctx,queue, config.m_maxContactCapacity,true);
	
	m_data->m_inertiaBufferGPU = new b3OpenCLArray<b3InertiaCL>(ctx,queue,config.m_maxConvexBodies,false);
	m_data->m_collidablesGPU = new b3OpenCLArray<b3Collidable>(ctx,queue,config.m_maxConvexShapes);

	m_data->m_localShapeAABBCPU = new b3AlignedObjectArray<b3SapAabb>;
	m_data->m_localShapeAABBGPU = new b3OpenCLArray<b3SapAabb>(ctx,queue,config.m_maxConvexShapes);
    
    
	//m_data->m_solverDataGPU = adl::Solver<adl::TYPE_CL>::allocate(ctx,queue, config.m_maxBroadphasePairs,false);
	m_data->m_bodyBufferGPU = new b3OpenCLArray<b3RigidBodyCL>(ctx,queue, config.m_maxConvexBodies,false);

	m_data->m_convexFacesGPU = new b3OpenCLArray<b3GpuFace>(ctx,queue,config.m_maxConvexShapes*config.m_maxFacesPerShape,false);
	m_data->m_gpuChildShapes = new b3OpenCLArray<b3GpuChildShape>(ctx,queue,config.m_maxCompoundChildShapes,false);
	
	m_data->m_convexPolyhedraGPU = new b3OpenCLArray<b3ConvexPolyhedronCL>(ctx,queue,config.m_maxConvexShapes,false);
	m_data->m_uniqueEdgesGPU = new b3OpenCLArray<b3Vector3>(ctx,queue,config.m_maxConvexUniqueEdges,true);
	m_data->m_convexVerticesGPU = new b3OpenCLArray<b3Vector3>(ctx,queue,config.m_maxConvexVertices,true);
	m_data->m_convexIndicesGPU = new b3OpenCLArray<int>(ctx,queue,config.m_maxConvexIndices,true);
    
    
	m_data->m_worldVertsB1GPU = new b3OpenCLArray<b3Vector3>(ctx,queue,config.m_maxConvexBodies*config.m_maxVerticesPerFace);
    m_data->m_clippingFacesOutGPU = new  b3OpenCLArray<b3Int4>(ctx,queue,config.m_maxConvexBodies);
    m_data->m_worldNormalsAGPU = new  b3OpenCLArray<b3Vector3>(ctx,queue,config.m_maxConvexBodies);
	m_data->m_worldVertsA1GPU = new b3OpenCLArray<b3Vector3>(ctx,queue,config.m_maxConvexBodies*config.m_maxVerticesPerFace);
    m_data->m_worldVertsB2GPU = new  b3OpenCLArray<b3Vector3>(ctx,queue,config.m_maxConvexBodies*config.m_maxVerticesPerFace);
    
    

	m_data->m_convexData = new b3AlignedObjectArray<b3ConvexUtility* >();
    

	m_data->m_convexData->resize(config.m_maxConvexShapes);
	m_data->m_convexPolyhedra.resize(config.m_maxConvexShapes);
    
	m_data->m_numAcceleratedShapes = 0;
	m_data->m_numAcceleratedRigidBodies = 0;
    
		
	m_data->m_subTreesGPU = new b3OpenCLArray<b3BvhSubtreeInfo>(this->m_context,this->m_queue);
	m_data->m_treeNodesGPU = new b3OpenCLArray<b3QuantizedBvhNode>(this->m_context,this->m_queue);
	m_data->m_bvhInfoGPU = new b3OpenCLArray<b3BvhInfo>(this->m_context,this->m_queue);

	//m_data->m_contactCGPU = new b3OpenCLArray<Constraint4>(ctx,queue,config.m_maxBroadphasePairs,false);
	//m_data->m_frictionCGPU = new b3OpenCLArray<adl::Solver<adl::TYPE_CL>::allocateFrictionConstraint( m_data->m_deviceCL, config.m_maxBroadphasePairs);
 
	

}


b3GpuNarrowPhase::~b3GpuNarrowPhase()
{
	delete m_data->m_gpuSatCollision;
	delete m_data->m_pBufPairsCPU;
	delete m_data->m_convexPairsOutGPU;
	delete m_data->m_planePairs;
	delete m_data->m_pBufContactOutCPU;
	delete m_data->m_bodyBufferCPU;
	delete m_data->m_inertiaBufferCPU;
	delete m_data->m_pBufContactOutGPU;
	delete m_data->m_inertiaBufferGPU;
	delete m_data->m_collidablesGPU;
	delete m_data->m_localShapeAABBCPU;
	delete m_data->m_localShapeAABBGPU;
	delete m_data->m_bodyBufferGPU;
	delete m_data->m_convexFacesGPU;
	delete m_data->m_gpuChildShapes;
	delete m_data->m_convexPolyhedraGPU;
	delete m_data->m_uniqueEdgesGPU;
	delete m_data->m_convexVerticesGPU;
	delete m_data->m_convexIndicesGPU;
	delete m_data->m_worldVertsB1GPU;
    delete m_data->m_clippingFacesOutGPU;
    delete m_data->m_worldNormalsAGPU;
	delete m_data->m_worldVertsA1GPU;
    delete m_data->m_worldVertsB2GPU;
    
	delete m_data->m_bvhInfoGPU;

	delete m_data->m_treeNodesGPU;
	delete m_data->m_subTreesGPU;

    
    delete m_data->m_convexData;
	delete m_data;
}


int	b3GpuNarrowPhase::allocateCollidable()
{
	int curSize = m_data->m_collidablesCPU.size();
	m_data->m_collidablesCPU.expand();
	return curSize;
}





int		b3GpuNarrowPhase::registerSphereShape(float radius)
{
	int collidableIndex = allocateCollidable();

	b3Collidable& col = getCollidableCpu(collidableIndex);
	col.m_shapeType = SHAPE_SPHERE;
	col.m_shapeIndex = 0;
	col.m_radius = radius;
	
	if (col.m_shapeIndex>=0)
	{
		b3SapAabb aabb;
		b3Vector3 myAabbMin(-radius,-radius,-radius);
		b3Vector3 myAabbMax(radius,radius,radius);

		aabb.m_min[0] = myAabbMin[0];//s_convexHeightField->m_aabb.m_min.x;
		aabb.m_min[1] = myAabbMin[1];//s_convexHeightField->m_aabb.m_min.y;
		aabb.m_min[2] = myAabbMin[2];//s_convexHeightField->m_aabb.m_min.z;
		aabb.m_minIndices[3] = 0;

		aabb.m_max[0] = myAabbMax[0];//s_convexHeightField->m_aabb.m_max.x;
		aabb.m_max[1] = myAabbMax[1];//s_convexHeightField->m_aabb.m_max.y;
		aabb.m_max[2] = myAabbMax[2];//s_convexHeightField->m_aabb.m_max.z;
		aabb.m_signedMaxIndices[3] = 0;

		m_data->m_localShapeAABBCPU->push_back(aabb);
		m_data->m_localShapeAABBGPU->push_back(aabb);
		clFinish(m_queue);
	}
	
	return collidableIndex;
}


int b3GpuNarrowPhase::registerFace(const b3Vector3& faceNormal, float faceConstant)
{
	int faceOffset = m_data->m_convexFaces.size();
	b3GpuFace& face = m_data->m_convexFaces.expand();
	face.m_plane[0] = faceNormal.getX();
	face.m_plane[1] = faceNormal.getY();
	face.m_plane[2] = faceNormal.getZ();
	face.m_plane[3] = faceConstant;
	m_data->m_convexFacesGPU->copyFromHost(m_data->m_convexFaces);
	return faceOffset;
}

int		b3GpuNarrowPhase::registerPlaneShape(const b3Vector3& planeNormal, float planeConstant)
{
	int collidableIndex = allocateCollidable();

	b3Collidable& col = getCollidableCpu(collidableIndex);
	col.m_shapeType = SHAPE_PLANE;
	col.m_shapeIndex = registerFace(planeNormal,planeConstant);
	col.m_radius = planeConstant;
	
	if (col.m_shapeIndex>=0)
	{
		b3SapAabb aabb;
		aabb.m_min[0] = -1e30f;
		aabb.m_min[1] = -1e30f;
		aabb.m_min[2] = -1e30f;
		aabb.m_minIndices[3] = 0;
		
		aabb.m_max[0] = 1e30f;
		aabb.m_max[1] = 1e30f;
		aabb.m_max[2] = 1e30f;
		aabb.m_signedMaxIndices[3] = 0;

		m_data->m_localShapeAABBCPU->push_back(aabb);
		m_data->m_localShapeAABBGPU->push_back(aabb);
		clFinish(m_queue);
	}
	
	return collidableIndex;
}


int b3GpuNarrowPhase::registerConvexHullShape(b3ConvexUtility* convexPtr,b3Collidable& col)
{
	m_data->m_convexData->resize(m_data->m_numAcceleratedShapes+1);
	m_data->m_convexPolyhedra.resize(m_data->m_numAcceleratedShapes+1);
	
    
	b3ConvexPolyhedronCL& convex = m_data->m_convexPolyhedra.at(m_data->m_convexPolyhedra.size()-1);
	convex.mC = convexPtr->mC;
	convex.mE = convexPtr->mE;
	convex.m_extents= convexPtr->m_extents;
	convex.m_localCenter = convexPtr->m_localCenter;
	convex.m_radius = convexPtr->m_radius;
	
	convex.m_numUniqueEdges = convexPtr->m_uniqueEdges.size();
	int edgeOffset = m_data->m_uniqueEdges.size();
	convex.m_uniqueEdgesOffset = edgeOffset;
	
	m_data->m_uniqueEdges.resize(edgeOffset+convex.m_numUniqueEdges);
    
	//convex data here
	int i;
	for ( i=0;i<convexPtr->m_uniqueEdges.size();i++)
	{
		m_data->m_uniqueEdges[edgeOffset+i] = convexPtr->m_uniqueEdges[i];
	}
    
	int faceOffset = m_data->m_convexFaces.size();
	convex.m_faceOffset = faceOffset;
	convex.m_numFaces = convexPtr->m_faces.size();
	m_data->m_convexFaces.resize(faceOffset+convex.m_numFaces);
	for (i=0;i<convexPtr->m_faces.size();i++)
	{
		m_data->m_convexFaces[convex.m_faceOffset+i].m_plane[0] = convexPtr->m_faces[i].m_plane[0];
		m_data->m_convexFaces[convex.m_faceOffset+i].m_plane[1] = convexPtr->m_faces[i].m_plane[1];
		m_data->m_convexFaces[convex.m_faceOffset+i].m_plane[2] = convexPtr->m_faces[i].m_plane[2];
		m_data->m_convexFaces[convex.m_faceOffset+i].m_plane[3] = convexPtr->m_faces[i].m_plane[3];
		int indexOffset = m_data->m_convexIndices.size();
		int numIndices = convexPtr->m_faces[i].m_indices.size();
		m_data->m_convexFaces[convex.m_faceOffset+i].m_numIndices = numIndices;
		m_data->m_convexFaces[convex.m_faceOffset+i].m_indexOffset = indexOffset;
		m_data->m_convexIndices.resize(indexOffset+numIndices);
		for (int p=0;p<numIndices;p++)
		{
			m_data->m_convexIndices[indexOffset+p] = convexPtr->m_faces[i].m_indices[p];
		}
	}
    
	convex.m_numVertices = convexPtr->m_vertices.size();
	int vertexOffset = m_data->m_convexVertices.size();
	convex.m_vertexOffset =vertexOffset;
	m_data->m_convexVertices.resize(vertexOffset+convex.m_numVertices);
	for (int i=0;i<convexPtr->m_vertices.size();i++)
	{
		m_data->m_convexVertices[vertexOffset+i] = convexPtr->m_vertices[i];
	}

	(*m_data->m_convexData)[m_data->m_numAcceleratedShapes] = convexPtr;
	
	m_data->m_convexFacesGPU->copyFromHost(m_data->m_convexFaces);
    
	m_data->m_convexPolyhedraGPU->copyFromHost(m_data->m_convexPolyhedra);
	m_data->m_uniqueEdgesGPU->copyFromHost(m_data->m_uniqueEdges);
	m_data->m_convexVerticesGPU->copyFromHost(m_data->m_convexVertices);
	m_data->m_convexIndicesGPU->copyFromHost(m_data->m_convexIndices);
    
    
	return m_data->m_numAcceleratedShapes++;
}


int		b3GpuNarrowPhase::registerConvexHullShape(const float* vertices, int strideInBytes, int numVertices, const float* scaling)
{
	b3AlignedObjectArray<b3Vector3> verts;
	
	unsigned char* vts = (unsigned char*) vertices;
	for (int i=0;i<numVertices;i++)
	{
		float* vertex = (float*) &vts[i*strideInBytes];
		verts.push_back(b3Vector3(vertex[0]*scaling[0],vertex[1]*scaling[1],vertex[2]*scaling[2]));
	}

	b3ConvexUtility* utilPtr = new b3ConvexUtility();
	bool merge = true;
	if (numVertices)
	{
		utilPtr->initializePolyhedralFeatures(&verts[0],verts.size(),merge);
	}

	int collidableIndex = registerConvexHullShape(utilPtr);
	return collidableIndex;
}

int		b3GpuNarrowPhase::registerConvexHullShape(b3ConvexUtility* utilPtr)
{
	int collidableIndex = allocateCollidable();
	b3Collidable& col = getCollidableCpu(collidableIndex);
	col.m_shapeType = SHAPE_CONVEX_HULL;
	col.m_shapeIndex = -1;
	
	
	{
		b3Vector3 localCenter(0,0,0);
		for (int i=0;i<utilPtr->m_vertices.size();i++)
			localCenter+=utilPtr->m_vertices[i];
		localCenter*= (1.f/utilPtr->m_vertices.size());
		utilPtr->m_localCenter = localCenter;

		col.m_shapeIndex = registerConvexHullShape(utilPtr,col);
	}

	if (col.m_shapeIndex>=0)
	{
		b3SapAabb aabb;
		
		b3Vector3 myAabbMin(1e30f,1e30f,1e30f);
		b3Vector3 myAabbMax(-1e30f,-1e30f,-1e30f);

		for (int i=0;i<utilPtr->m_vertices.size();i++)
		{
			myAabbMin.setMin(utilPtr->m_vertices[i]);
			myAabbMax.setMax(utilPtr->m_vertices[i]);
		}
		aabb.m_min[0] = myAabbMin[0];
		aabb.m_min[1] = myAabbMin[1];
		aabb.m_min[2] = myAabbMin[2];
		aabb.m_minIndices[3] = 0;

		aabb.m_max[0] = myAabbMax[0];
		aabb.m_max[1] = myAabbMax[1];
		aabb.m_max[2] = myAabbMax[2];
		aabb.m_signedMaxIndices[3] = 0;

		m_data->m_localShapeAABBCPU->push_back(aabb);
		m_data->m_localShapeAABBGPU->push_back(aabb);
	}
	
	return collidableIndex;

}

int		b3GpuNarrowPhase::registerCompoundShape(b3AlignedObjectArray<b3GpuChildShape>* childShapes)
{
	
	int collidableIndex = allocateCollidable();
	b3Collidable& col = getCollidableCpu(collidableIndex);
	col.m_shapeType = SHAPE_COMPOUND_OF_CONVEX_HULLS;
	
	col.m_shapeIndex = m_data->m_cpuChildShapes.size();
	{
		b3Assert(col.m_shapeIndex+childShapes->size()<m_data->m_config.m_maxCompoundChildShapes);
		for (int i=0;i<childShapes->size();i++)
		{
			m_data->m_cpuChildShapes.push_back(childShapes->at(i));
		}
		//if writing the data directly is too slow, we can delay it and do it all at once in
		m_data->m_gpuChildShapes->copyFromHost(m_data->m_cpuChildShapes);
	}



	col.m_numChildShapes = childShapes->size();
	
	
	b3SapAabb aabbWS;
	b3Vector3 myAabbMin(1e30f,1e30f,1e30f);
	b3Vector3 myAabbMax(-1e30f,-1e30f,-1e30f);
	
	//compute local AABB of the compound of all children
	for (int i=0;i<childShapes->size();i++)
	{
		int childColIndex = childShapes->at(i).m_shapeIndex;
		b3Collidable& childCol = getCollidableCpu(childColIndex);
		b3SapAabb aabbLoc =m_data->m_localShapeAABBCPU->at(childColIndex);

		b3Vector3 childLocalAabbMin(aabbLoc.m_min[0],aabbLoc.m_min[1],aabbLoc.m_min[2]);
		b3Vector3 childLocalAabbMax(aabbLoc.m_max[0],aabbLoc.m_max[1],aabbLoc.m_max[2]);
		b3Vector3 aMin,aMax;
		b3Scalar margin(0.f);
		b3Transform childTr;
		childTr.setIdentity();

		childTr.setOrigin(b3Vector3(childShapes->at(i).m_childPosition[0],
									childShapes->at(i).m_childPosition[1],
									childShapes->at(i).m_childPosition[2]));
		childTr.setRotation(b3Quaternion(childShapes->at(i).m_childOrientation[0],
										 childShapes->at(i).m_childOrientation[1],
										 childShapes->at(i).m_childOrientation[2],
										 childShapes->at(i).m_childOrientation[3]));
		b3TransformAabb(childLocalAabbMin,childLocalAabbMax,margin,childTr,aMin,aMax);
		myAabbMin.setMin(aMin);
		myAabbMax.setMax(aMax);		
	}
	
	aabbWS.m_min[0] = myAabbMin[0];//s_convexHeightField->m_aabb.m_min.x;
	aabbWS.m_min[1]= myAabbMin[1];//s_convexHeightField->m_aabb.m_min.y;
	aabbWS.m_min[2]= myAabbMin[2];//s_convexHeightField->m_aabb.m_min.z;
	aabbWS.m_minIndices[3] = 0;
	
	aabbWS.m_max[0] = myAabbMax[0];//s_convexHeightField->m_aabb.m_max.x;
	aabbWS.m_max[1]= myAabbMax[1];//s_convexHeightField->m_aabb.m_max.y;
	aabbWS.m_max[2]= myAabbMax[2];//s_convexHeightField->m_aabb.m_max.z;
	aabbWS.m_signedMaxIndices[3] = 0;
	
	m_data->m_localShapeAABBCPU->push_back(aabbWS);
	m_data->m_localShapeAABBGPU->push_back(aabbWS);
	clFinish(m_queue);
	return collidableIndex;
	
}


int		b3GpuNarrowPhase::registerConcaveMesh(b3AlignedObjectArray<b3Vector3>* vertices, b3AlignedObjectArray<int>* indices,const float* scaling1)
{
	

	b3Vector3 scaling(scaling1[0],scaling1[1],scaling1[2]);

	int collidableIndex = allocateCollidable();
	b3Collidable& col = getCollidableCpu(collidableIndex);
	
	col.m_shapeType = SHAPE_CONCAVE_TRIMESH;
	col.m_shapeIndex = registerConcaveMeshShape(vertices,indices,col,scaling);
	col.m_bvhIndex = m_data->m_bvhInfoCPU.size();
		

	b3SapAabb aabb;
	b3Vector3 myAabbMin(1e30f,1e30f,1e30f);
	b3Vector3 myAabbMax(-1e30f,-1e30f,-1e30f);

	for (int i=0;i<vertices->size();i++)
	{
		b3Vector3 vtx(vertices->at(i)*scaling);
		myAabbMin.setMin(vtx);
		myAabbMax.setMax(vtx);
	}
	aabb.m_min[0] = myAabbMin[0];
	aabb.m_min[1] = myAabbMin[1];
	aabb.m_min[2] = myAabbMin[2];
	aabb.m_minIndices[3] = 0;

	aabb.m_max[0] = myAabbMax[0];
	aabb.m_max[1]= myAabbMax[1];
	aabb.m_max[2]= myAabbMax[2];
	aabb.m_signedMaxIndices[3]= 0;

	m_data->m_localShapeAABBCPU->push_back(aabb);
	m_data->m_localShapeAABBGPU->push_back(aabb);

	b3OptimizedBvh* bvh = new b3OptimizedBvh();
	//void b3OptimizedBvh::build(b3StridingMeshInterface* triangles, bool useQuantizedAabbCompression, const b3Vector3& bvhAabbMin, const b3Vector3& bvhAabbMax)
	
	bool useQuantizedAabbCompression = true;
	b3TriangleIndexVertexArray* meshInterface=new b3TriangleIndexVertexArray();
	b3IndexedMesh mesh;
	mesh.m_numTriangles = indices->size()/3;
	mesh.m_numVertices = vertices->size();
	mesh.m_vertexBase = (const unsigned char *)&vertices->at(0).getX();
	mesh.m_vertexStride = sizeof(b3Vector3);
	mesh.m_triangleIndexStride = 3 * sizeof(int);// or sizeof(int)
	mesh.m_triangleIndexBase = (const unsigned char *)&indices->at(0);
	
	meshInterface->addIndexedMesh(mesh);
	bvh->build(meshInterface, useQuantizedAabbCompression, (b3Vector3&)aabb.m_min, (b3Vector3&)aabb.m_max);
	m_data->m_bvhData.push_back(bvh);
	int numNodes = bvh->getQuantizedNodeArray().size();
	//b3OpenCLArray<b3QuantizedBvhNode>*	treeNodesGPU = new b3OpenCLArray<b3QuantizedBvhNode>(this->m_context,this->m_queue,numNodes);
	//treeNodesGPU->copyFromHost(bvh->getQuantizedNodeArray());
	int numSubTrees = bvh->getSubtreeInfoArray().size();

	b3BvhInfo bvhInfo;
	
	bvhInfo.m_aabbMin = bvh->m_bvhAabbMin;
	bvhInfo.m_aabbMax = bvh->m_bvhAabbMax;
	bvhInfo.m_quantization = bvh->m_bvhQuantization;
	bvhInfo.m_numNodes = numNodes;
	bvhInfo.m_numSubTrees = numSubTrees;
	bvhInfo.m_nodeOffset = m_data->m_treeNodesCPU.size();
	bvhInfo.m_subTreeOffset = m_data->m_subTreesCPU.size();

	m_data->m_bvhInfoCPU.push_back(bvhInfo);
	m_data->m_bvhInfoGPU->copyFromHost(m_data->m_bvhInfoCPU);


	int numNewSubtrees = bvh->getSubtreeInfoArray().size();
	m_data->m_subTreesCPU.reserve(m_data->m_subTreesCPU.size()+numNewSubtrees);
	for (int i=0;i<numNewSubtrees;i++)
	{
		m_data->m_subTreesCPU.push_back(bvh->getSubtreeInfoArray()[i]);
	}
	int numNewTreeNodes = bvh->getQuantizedNodeArray().size();

	for (int i=0;i<numNewTreeNodes;i++)
	{
		m_data->m_treeNodesCPU.push_back(bvh->getQuantizedNodeArray()[i]);
	}

	//b3OpenCLArray<b3BvhSubtreeInfo>* subTreesGPU = new b3OpenCLArray<b3BvhSubtreeInfo>(this->m_context,this->m_queue,numSubTrees);
	//subTreesGPU->copyFromHost(bvh->getSubtreeInfoArray());

	m_data->m_treeNodesGPU->copyFromHost(m_data->m_treeNodesCPU);
	m_data->m_subTreesGPU->copyFromHost(m_data->m_subTreesCPU);


	return collidableIndex;
}

int b3GpuNarrowPhase::registerConcaveMeshShape(b3AlignedObjectArray<b3Vector3>* vertices, b3AlignedObjectArray<int>* indices,b3Collidable& col, const float* scaling1)
{


	b3Vector3 scaling(scaling1[0],scaling1[1],scaling1[2]);

	m_data->m_convexData->resize(m_data->m_numAcceleratedShapes+1);
	m_data->m_convexPolyhedra.resize(m_data->m_numAcceleratedShapes+1);
	
    
	b3ConvexPolyhedronCL& convex = m_data->m_convexPolyhedra.at(m_data->m_convexPolyhedra.size()-1);
	convex.mC = b3Vector3(0,0,0);
	convex.mE = b3Vector3(0,0,0);
	convex.m_extents= b3Vector3(0,0,0);
	convex.m_localCenter = b3Vector3(0,0,0);
	convex.m_radius = 0.f;
	
	convex.m_numUniqueEdges = 0;
	int edgeOffset = m_data->m_uniqueEdges.size();
	convex.m_uniqueEdgesOffset = edgeOffset;
	
	int faceOffset = m_data->m_convexFaces.size();
	convex.m_faceOffset = faceOffset;
	
	convex.m_numFaces = indices->size()/3;
	m_data->m_convexFaces.resize(faceOffset+convex.m_numFaces);
	m_data->m_convexIndices.reserve(convex.m_numFaces*3);
	for (int i=0;i<convex.m_numFaces;i++)
	{
		if (i%256==0)
		{
			//printf("i=%d out of %d", i,convex.m_numFaces);
		}
		b3Vector3 vert0(vertices->at(indices->at(i*3))*scaling);
		b3Vector3 vert1(vertices->at(indices->at(i*3+1))*scaling);
		b3Vector3 vert2(vertices->at(indices->at(i*3+2))*scaling);

		b3Vector3 normal = ((vert1-vert0).cross(vert2-vert0)).normalize();
		b3Scalar c = -(normal.dot(vert0));

		m_data->m_convexFaces[convex.m_faceOffset+i].m_plane[0] = normal.getX();
		m_data->m_convexFaces[convex.m_faceOffset+i].m_plane[1] = normal.getY();
		m_data->m_convexFaces[convex.m_faceOffset+i].m_plane[2] = normal.getZ();
		m_data->m_convexFaces[convex.m_faceOffset+i].m_plane[3] = c;
		int indexOffset = m_data->m_convexIndices.size();
		int numIndices = 3;
		m_data->m_convexFaces[convex.m_faceOffset+i].m_numIndices = numIndices;
		m_data->m_convexFaces[convex.m_faceOffset+i].m_indexOffset = indexOffset;
		m_data->m_convexIndices.resize(indexOffset+numIndices);
		for (int p=0;p<numIndices;p++)
		{
			int vi = indices->at(i*3+p);
			m_data->m_convexIndices[indexOffset+p] = vi;//convexPtr->m_faces[i].m_indices[p];
		}
	}
    
	convex.m_numVertices = vertices->size();
	int vertexOffset = m_data->m_convexVertices.size();
	convex.m_vertexOffset =vertexOffset;
	m_data->m_convexVertices.resize(vertexOffset+convex.m_numVertices);
	for (int i=0;i<vertices->size();i++)
	{
		m_data->m_convexVertices[vertexOffset+i] = vertices->at(i)*scaling;
	}

	(*m_data->m_convexData)[m_data->m_numAcceleratedShapes] = 0;
	
	m_data->m_convexFacesGPU->copyFromHost(m_data->m_convexFaces);
    
	m_data->m_convexPolyhedraGPU->copyFromHost(m_data->m_convexPolyhedra);
	m_data->m_uniqueEdgesGPU->copyFromHost(m_data->m_uniqueEdges);
	m_data->m_convexVerticesGPU->copyFromHost(m_data->m_convexVertices);
	m_data->m_convexIndicesGPU->copyFromHost(m_data->m_convexIndices);
  
	return m_data->m_numAcceleratedShapes++;
}



cl_mem	b3GpuNarrowPhase::getBodiesGpu()
{
	return (cl_mem)m_data->m_bodyBufferGPU->getBufferCL();
}

const struct b3RigidBodyCL* b3GpuNarrowPhase::getBodiesCpu() const
{
	return &m_data->m_bodyBufferCPU->at(0);
};

int	b3GpuNarrowPhase::getNumBodiesGpu() const
{
	return m_data->m_bodyBufferGPU->size();
}

cl_mem	b3GpuNarrowPhase::getBodyInertiasGpu()
{
	return (cl_mem)m_data->m_inertiaBufferGPU->getBufferCL();
}

int	b3GpuNarrowPhase::getNumBodyInertiasGpu() const
{
	return m_data->m_inertiaBufferGPU->size();
}


b3Collidable& b3GpuNarrowPhase::getCollidableCpu(int collidableIndex)
{
	return m_data->m_collidablesCPU[collidableIndex];
}

const b3Collidable& b3GpuNarrowPhase::getCollidableCpu(int collidableIndex) const
{
	return m_data->m_collidablesCPU[collidableIndex];
}

cl_mem b3GpuNarrowPhase::getCollidablesGpu()
{
	return m_data->m_collidablesGPU->getBufferCL();
}

const struct b3Collidable* b3GpuNarrowPhase::getCollidablesCpu() const
{
	return &m_data->m_collidablesCPU[0];
}


cl_mem	b3GpuNarrowPhase::getAabbBufferGpu()
{
	return m_data->m_localShapeAABBGPU->getBufferCL();
}
int	b3GpuNarrowPhase::getNumCollidablesGpu() const
{
	return m_data->m_collidablesGPU->size();
}





int	b3GpuNarrowPhase::getNumContactsGpu() const
{
	return m_data->m_pBufContactOutGPU->size();
}
cl_mem b3GpuNarrowPhase::getContactsGpu()
{
	return m_data->m_pBufContactOutGPU->getBufferCL();
}

const b3Contact4* b3GpuNarrowPhase::getContactsCPU() const
{
	m_data->m_pBufContactOutGPU->copyToHost(*m_data->m_pBufContactOutCPU);
	return &m_data->m_pBufContactOutCPU->at(0);
}

void b3GpuNarrowPhase::computeContacts(cl_mem broadphasePairs, int numBroadphasePairs, cl_mem aabbsWS, int numObjects)
{
	int nContactOut = 0;

	int maxTriConvexPairCapacity = m_data->m_config.m_maxTriConvexPairCapacity;
	b3OpenCLArray<b3Int4> triangleConvexPairs(m_context,m_queue, maxTriConvexPairCapacity);
	int numTriConvexPairsOut=0;
	
	b3OpenCLArray<b3Int2> broadphasePairsGPU(m_context,m_queue);
	broadphasePairsGPU.setFromOpenCLBuffer(broadphasePairs,numBroadphasePairs);
	b3OpenCLArray<b3YetAnotherAabb> clAabbArray(this->m_context,this->m_queue);
	clAabbArray.setFromOpenCLBuffer(aabbsWS,numObjects);

	m_data->m_gpuSatCollision->computeConvexConvexContactsGPUSAT(
		&broadphasePairsGPU, numBroadphasePairs,
		m_data->m_bodyBufferGPU,
		m_data->m_pBufContactOutGPU,
		nContactOut,
		m_data->m_config.m_maxContactCapacity,
		*m_data->m_convexPolyhedraGPU,
		*m_data->m_convexVerticesGPU,
		*m_data->m_uniqueEdgesGPU,
		*m_data->m_convexFacesGPU,
		*m_data->m_convexIndicesGPU,
		*m_data->m_collidablesGPU,
		*m_data->m_gpuChildShapes,
		clAabbArray,
		*m_data->m_worldVertsB1GPU,
		*m_data->m_clippingFacesOutGPU,
		*m_data->m_worldNormalsAGPU,
		*m_data->m_worldVertsA1GPU,
		*m_data->m_worldVertsB2GPU,
		m_data->m_bvhData,
		m_data->m_treeNodesGPU,
		m_data->m_subTreesGPU,
		m_data->m_bvhInfoGPU,
		numObjects,
		maxTriConvexPairCapacity,
		triangleConvexPairs,
		numTriConvexPairsOut
		);

}

const b3SapAabb& b3GpuNarrowPhase::getLocalSpaceAabb(int collidableIndex) const
{
	return m_data->m_localShapeAABBCPU->at(collidableIndex);
}





int b3GpuNarrowPhase::registerRigidBody(int collidableIndex, float mass, const float* position, const float* orientation , const float* aabbMinPtr, const float* aabbMaxPtr,bool writeToGpu)
{
	b3Vector3 aabbMin(aabbMinPtr[0],aabbMinPtr[1],aabbMinPtr[2]);
	b3Vector3 aabbMax (aabbMaxPtr[0],aabbMaxPtr[1],aabbMaxPtr[2]);
	
	b3Assert(m_data->m_numAcceleratedRigidBodies< (m_data->m_config.m_maxConvexBodies-1));
    
	m_data->m_bodyBufferGPU->resize(m_data->m_numAcceleratedRigidBodies+1);
    
	b3RigidBodyCL& body = m_data->m_bodyBufferCPU->at(m_data->m_numAcceleratedRigidBodies);
    
	float friction = 1.f;
	float restitution = 0.f;
    
	body.m_frictionCoeff = friction;
	body.m_restituitionCoeff = restitution;
	body.m_angVel.setZero();
	body.m_linVel.setValue(0,0,0);//.setZero();
	body.m_pos.setValue(position[0],position[1],position[2]);
	body.m_quat.setValue(orientation[0],orientation[1],orientation[2],orientation[3]);
	body.m_collidableIdx = collidableIndex;
	if (collidableIndex>=0)
	{
//		body.m_shapeType = m_data->m_collidablesCPU.at(collidableIndex).m_shapeType;
	} else
	{
	//	body.m_shapeType = CollisionShape::SHAPE_PLANE;
		m_planeBodyIndex = m_data->m_numAcceleratedRigidBodies;
	}
	//body.m_shapeType = shapeType;
	
	
	body.m_invMass = mass? 1.f/mass : 0.f;
    
	if (writeToGpu)
	{
		m_data->m_bodyBufferGPU->copyFromHostPointer(&body,1,m_data->m_numAcceleratedRigidBodies);
	}
    
	b3InertiaCL& shapeInfo = m_data->m_inertiaBufferCPU->at(m_data->m_numAcceleratedRigidBodies);
    
	if (mass==0.f)
	{
		if (m_data->m_numAcceleratedRigidBodies==0)
			m_static0Index = 0;
        
		shapeInfo.m_initInvInertia.setValue(0,0,0,0,0,0,0,0,0);
		shapeInfo.m_invInertiaWorld.setValue(0,0,0,0,0,0,0,0,0);
	} else
	{
        
		assert(body.m_collidableIdx>=0);
        
		//approximate using the aabb of the shape
        
		//Aabb aabb = (*m_data->m_shapePointers)[shapeIndex]->m_aabb;
		b3Vector3 halfExtents = (aabbMax-aabbMin);//*0.5f;//fake larger inertia makes demos more stable ;-)
        
		b3Vector3 localInertia;
        
		float lx=2.f*halfExtents[0];
		float ly=2.f*halfExtents[1];
		float lz=2.f*halfExtents[2];
        
		localInertia.setValue( (mass/12.0f) * (ly*ly + lz*lz),
                                   (mass/12.0f) * (lx*lx + lz*lz),
                                   (mass/12.0f) * (lx*lx + ly*ly));
        
		b3Vector3 invLocalInertia;
		invLocalInertia[0] = 1.f/localInertia[0];
		invLocalInertia[1] = 1.f/localInertia[1];
		invLocalInertia[2] = 1.f/localInertia[2];
		invLocalInertia[3] = 0.f;
        
		shapeInfo.m_initInvInertia.setValue(
			invLocalInertia[0],		0,						0,
			0,						invLocalInertia[1],		0,
			0,						0,						invLocalInertia[2]);

		b3Matrix3x3 m (body.m_quat);

		shapeInfo.m_invInertiaWorld = m.scaled(invLocalInertia) * m.transpose();
        
	}
    
	if (writeToGpu)
		m_data->m_inertiaBufferGPU->copyFromHostPointer(&shapeInfo,1,m_data->m_numAcceleratedRigidBodies);
    
    
    
	return m_data->m_numAcceleratedRigidBodies++;
}

int b3GpuNarrowPhase::getNumRigidBodies() const
{
	return m_data->m_numAcceleratedRigidBodies;
}

void	b3GpuNarrowPhase::writeAllBodiesToGpu()
{
	m_data->m_bodyBufferGPU->resize(m_data->m_numAcceleratedRigidBodies);
	m_data->m_inertiaBufferGPU->resize(m_data->m_numAcceleratedRigidBodies);
    
	if (m_data->m_numAcceleratedRigidBodies)
	{
		m_data->m_bodyBufferGPU->copyFromHostPointer(&m_data->m_bodyBufferCPU->at(0),m_data->m_numAcceleratedRigidBodies);
		m_data->m_inertiaBufferGPU->copyFromHostPointer(&m_data->m_inertiaBufferCPU->at(0),m_data->m_numAcceleratedRigidBodies);
	}
    if (m_data->m_collidablesCPU.size())
	{
		m_data->m_collidablesGPU->copyFromHost(m_data->m_collidablesCPU);
	}
	
    
}

void	b3GpuNarrowPhase::readbackAllBodiesToCpu()
{
	m_data->m_bodyBufferGPU->copyToHostPointer(&m_data->m_bodyBufferCPU->at(0),m_data->m_numAcceleratedRigidBodies);
}
void	b3GpuNarrowPhase::getObjectTransformFromCpu(float* position, float* orientation , int bodyIndex) const
{
	position[0] = m_data->m_bodyBufferCPU->at(bodyIndex).m_pos.x;
	position[1] = m_data->m_bodyBufferCPU->at(bodyIndex).m_pos.y;
	position[2] = m_data->m_bodyBufferCPU->at(bodyIndex).m_pos.z;
	position[3] = 1.f;//or 1

	orientation[0] = m_data->m_bodyBufferCPU->at(bodyIndex).m_quat.x;
	orientation[1] = m_data->m_bodyBufferCPU->at(bodyIndex).m_quat.y;
	orientation[2] = m_data->m_bodyBufferCPU->at(bodyIndex).m_quat.z;
	orientation[3] = m_data->m_bodyBufferCPU->at(bodyIndex).m_quat.w;
}
