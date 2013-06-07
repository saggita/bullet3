#ifndef B3_GPU_DYNAMICS_WORLD_H
#define B3_GPU_DYNAMICS_WORLD_H

#include "LinearMath/btVector3.h"

class btRigidBody;
class btCollisionObject;
struct b3GpuInternalData;//use this struct to avoid 'leaking' all OpenCL headers into clients code base
//class CLPhysicsDemo;
class btActionInterface;
class btSoftbodyCL;

#include "LinearMath/btAlignedObjectArray.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"

class b3GpuDynamicsWorld : public btSoftRigidDynamicsWorld
{
	
	btAlignedObjectArray<const class  btCollisionShape*> m_uniqueShapes;
	btAlignedObjectArray<int> m_uniqueShapeMapping;


	class b3GpuRigidBodyPipeline* m_rigidBodyPipeline;
	class b3GpuNarrowPhase* m_np;
	class b3GpuSapBroadphase* m_bp;

	class btSoftBodySimulationSolverOpenCL* m_softbodySolver;
	
	btVector3			m_gravity;
	bool	m_cpuGpuSync;
	
		
	int findOrRegisterCollisionShape(const btCollisionShape* colShape);

	
public:
	b3GpuDynamicsWorld(class b3GpuSapBroadphase* bp,class b3GpuNarrowPhase* np, class b3GpuRigidBodyPipeline* rigidBodyPipeline, class btSoftBodySimulationSolverOpenCL* softbodySolverCL);

	virtual ~b3GpuDynamicsWorld();

	virtual int		stepSimulation( btScalar timeStep,int maxSubSteps=1, btScalar fixedTimeStep=btScalar(1.)/btScalar(60.));

	virtual void	synchronizeMotionStates()
	{
		btAssert(0);
	}

	void	debugDrawWorld() {}

	void	setGravity(const btVector3& gravity);

	void	addRigidBody(btRigidBody* body);

	void	removeCollisionObject(btCollisionObject* colObj);

	void	rayTest(const btVector3& rayFromWorld, const btVector3& rayToWorld, RayResultCallback& resultCallback) const;

	btAlignedObjectArray<class btCollisionObject*>& getCollisionObjectArray();

	const btAlignedObjectArray<class btCollisionObject*>& getCollisionObjectArray() const;


	btVector3 getGravity () const
	{
		return m_gravity;
	}

	virtual void	addRigidBody(btRigidBody* body, short group, short mask)
	{
		addRigidBody(body);
	}

	// I think btSoftRigidDynamicsWorld::addSoftBody and btSoftRigidDynamicsWorld::removeSoftBody should be virtual functions.
	void addSoftBodyCl(btSoftbodyCL* softBody);

	virtual void	removeRigidBody(btRigidBody* body)
	{
		btAssert(0);
	}


	virtual void	addAction(btActionInterface* action) 
	{
		btAssert(0);
	}

	virtual void	removeAction(btActionInterface* action)
	{
		btAssert(0);
	}

	virtual void	setConstraintSolver(btConstraintSolver* solver)
	{
		btAssert(0);
	}

	virtual btConstraintSolver* getConstraintSolver()
	{
		btAssert(0);
		return 0;
	}
		

	virtual void	clearForces()
	{
		btAssert(0);
	}

	virtual btDynamicsWorldType	getWorldType() const
	{
		return BT_GPU_DYNAMICS_WORLD;
	}

	///this can be useful to synchronize a single rigid body -> graphics object
	void	synchronizeSingleMotionState(btRigidBody* body);
};

#endif //B3_GPU_DYNAMICS_WORLD_H
