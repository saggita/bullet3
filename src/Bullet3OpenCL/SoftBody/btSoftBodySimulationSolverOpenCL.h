#pragma once

#include "CLFunctions.h"

#include "LinearMath\btVector3.h"
#include "LinearMath\btAlignedObjectArray.h"
#include "..\NarrowphaseCollision\b3ConvexPolyhedronCL.h"

struct CLPhysicsDemo;
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
	btSoftBodySimulationSolverOpenCL(CLPhysicsDemo* gpuPhysics);
	virtual ~btSoftBodySimulationSolverOpenCL(void);

	btSoftbodyCL* m_pMergedSoftBody;
	CLPhysicsDemo*		m_gpuPhysics;

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
	void GenerateBatches(bool bBatchEachSoftBodyFirst = false); 
	void mergeSoftBodies();

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
	bool BuildCLKernels();
	void ReleaseKernels();	
	void UpdateBuffers();	

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

	CLFunctions m_clFunctions;
		
	int getVertexIndexGlobal(int vertexIndexLocal, int clothIndex);
	int getStretchSpringIndexGlobal(int stretchSpringIndexLocal, int clothIndex);
	int getBendingSpringIndexGlobal(int bendingSpringIndexLocal, int clothIndex);

public:
	void setGravity(const btVector3& gravity) { m_Gravity = gravity; }

	void Initialize();
	bool Integrate(float dt);
	bool AdvancePosition(float dt);
	bool ResolveCollision(float dt);
	bool ResolveCollisionCPU(float dt);
	void InitializeBoundingVolumes();
	void UpdateBoundingVolumes(float dt);
	bool ReadBackFromGPU();
		
	btAlignedObjectArray<btSoftbodyCL*>& getSoftBodies() { return m_clothArray; }
	const btAlignedObjectArray<btSoftbodyCL*>& getSoftBodies() const { return m_clothArray; }
	void addSoftBody(btSoftbodyCL* pCloth);
};
