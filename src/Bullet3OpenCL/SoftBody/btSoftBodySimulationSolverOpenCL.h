#ifndef B3_SOFTBODY_SOLVER_CL_H
#define B3_SOFTBODY_SOLVER_CL_H

#include "../Initialize/b3OpenCLInclude.h"

#include "LinearMath\btVector3.h"
#include "LinearMath\btAlignedObjectArray.h"
#include "..\NarrowphaseCollision\b3ConvexPolyhedronCL.h"

class btSoftbodyCL;
struct float4;
struct float4s;
struct btSoftBodySpringCL;
struct btSoftBodyVertexCL;
struct btSoftBodyInfoCL;
struct btSoftBodyLinkCL;

class btSoftBodySimulationSolverOpenCL
{
public:
	btSoftBodySimulationSolverOpenCL(cl_context ctx, cl_device_id device, cl_command_queue  q);
	virtual ~btSoftBodySimulationSolverOpenCL(void);

	btSoftbodyCL* m_pMergedSoftBody;

protected:
	btAlignedObjectArray<btSoftbodyCL*> m_clothArray;
	btAlignedObjectArray<btSoftbodyCL*> m_tempClothArray;
	btVector3 m_Gravity;
	int m_NumIterForConstraintSolver;

	int m_numVertices;
	int m_numStretchSprings;
	int m_numBendingSprings;
	int m_numClothes;

	// for batches
	btAlignedObjectArray<int> m_BatchStretchSpringIndexGlobalArray; // index is global
	btAlignedObjectArray<int> m_BatchBendSpringIndexGlobalArray; // index is global
	void generateBatches(bool bBatchEachSoftBodyFirst = false); 
	void mergeSoftBodies();

	cl_context				m_context;
	cl_device_id			m_device;
	cl_command_queue		m_queue;

	cl_mem m_DBVertices;
	cl_mem m_DBStrechSprings;
	cl_mem m_DBBendSprings;
	cl_mem m_DBClothInfo;

	btSoftBodyVertexCL* m_HBVertexCL;
	btSoftBodySpringCL* m_HBStretchSpringCL;
	btSoftBodySpringCL* m_HBBendSpringCL;
	btSoftBodyInfoCL* m_HBClothInfoCL;
	btSoftBodyLinkCL* m_HBLinkCL;
	
	bool m_bBuildCLKernels;
	bool buildCLKernels();
	void releaseKernels();	
	void updateBuffers();	

	// OpenCL kernels
	cl_kernel m_ClearForcesKernel;
	cl_kernel m_ComputeNextVertexPositionsKernel;
	cl_kernel m_ApplyGravityKernel;
	cl_kernel m_ApplyForcesKernel;
	cl_kernel m_EnforceEdgeConstraintsKernel;
	cl_kernel m_UpdateVelocitiesKernel;
	cl_kernel m_AdvancePositionKernel;
	cl_kernel m_UpdateVertexBoundingVolumeKernel;
	cl_kernel m_UpdateClothBoundingVolumeKernel;
	cl_kernel m_ResolveCollisionKernel;

	int getVertexIndexGlobal(int vertexIndexLocal, int clothIndex);
	int getStretchSpringIndexGlobal(int stretchSpringIndexLocal, int clothIndex);
	int getBendingSpringIndexGlobal(int bendingSpringIndexLocal, int clothIndex);

public:
	void setGravity(const btVector3& gravity) { m_Gravity = gravity; }

	void initialize();
	bool integrate(float dt);
	bool advancePosition(float dt);
	bool resolveCollision(float dt);
	bool resolveCollisionCPU(float dt);
	void initializeBoundingVolumes();
	void updateBoundingVolumes(float dt);
	bool readBackFromGPU();
		
	btAlignedObjectArray<btSoftbodyCL*>& getSoftBodies() { return m_clothArray; }
	const btAlignedObjectArray<btSoftbodyCL*>& getSoftBodies() const { return m_clothArray; }
	void addSoftBody(btSoftbodyCL* pCloth);
};

#endif // B3_SOFTBODY_SOLVER_CL_H