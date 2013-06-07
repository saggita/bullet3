#include "b3GpuDynamicsWorld.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

//#include "../../../opencl/gpu_rigidbody_pipeline2/CLPhysicsDemo.h"
//#include "../../../opencl/gpu_rigidbody_pipeline/b3GpuNarrowPhaseAndSolver.h"
#include "BulletCollision/CollisionShapes/btPolyhedralConvexShape.h"
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btStaticPlaneShape.h"
#include "Bullet3OpenCL/SoftBody/btSoftBodySimulationSolverOpenCL.h"
#include "Bullet3OpenCL/SoftBody/btSoftbodyCL.h"

#include "LinearMath/btQuickprof.h"
#include "Bullet3OpenCL/BroadphaseCollision/b3GpuSapBroadphase.h"
#include "Bullet3OpenCL/RigidBody/b3GpuNarrowPhase.h"
#include "Bullet3OpenCL/RigidBody/b3GpuRigidBodyPipeline.h"




#ifdef _WIN32
	#include <windows.h>
#endif



b3GpuDynamicsWorld::b3GpuDynamicsWorld(class b3GpuSapBroadphase* bp,class b3GpuNarrowPhase* np, class b3GpuRigidBodyPipeline* rigidBodyPipeline, class btSoftBodySimulationSolverOpenCL* softbodySolverCL)
	:btSoftRigidDynamicsWorld(NULL, NULL, NULL, NULL),
	m_gravity(0,-10,0),
m_cpuGpuSync(true),
m_bp(bp),
m_np(np),
m_rigidBodyPipeline(rigidBodyPipeline),
m_softbodySolver(softbodySolverCL)
{

}

b3GpuDynamicsWorld::~b3GpuDynamicsWorld()
{
	// TODO: Should be done in other place..
	if ( m_softbodySolver )
	{
		delete m_softbodySolver;
		m_softbodySolver = NULL;
	}
}




int		b3GpuDynamicsWorld::stepSimulation( btScalar timeStep, int maxSubSteps, btScalar fixedTimeStep)
{
#ifndef BT_NO_PROFILE
	CProfileManager::Reset();
#endif //BT_NO_PROFILE


	BT_PROFILE("stepSimulation");

	//convert all shapes now, and if any change, reset all (todo)
	
	
	if (m_cpuGpuSync)
	{
		m_cpuGpuSync = false;
		m_np->writeAllBodiesToGpu();
		m_bp->writeAabbsToGpu();
		m_rigidBodyPipeline->writeAllInstancesToGpu();

		m_softbodySolver->Initialize();
	}

	// dispatch preTick callback
	if(0 != m_internalPreTickCallback) {
		(*m_internalPreTickCallback)(this, fixedTimeStep);
	}	

	m_rigidBodyPipeline->stepSimulation(fixedTimeStep);

	// simulate softbodies
	m_softbodySolver->Integrate(timeStep);
	m_softbodySolver->AdvancePosition(timeStep);
	m_softbodySolver->UpdateBoundingVolumes(timeStep);
	m_softbodySolver->ResolveCollision(timeStep);
	//m_softbodySolver->ResolveCollisionCPU(timeStep);
	m_softbodySolver->ReadBackFromGPU();


	{
		{
			BT_PROFILE("readbackBodiesToCpu");
			//now copy info back to rigid bodies....
			m_np->readbackAllBodiesToCpu();
		}

		{
			BT_PROFILE("scatter transforms into rigidbody (CPU)");
			for (int i=0;i<this->m_collisionObjects.size();i++)
			{
				btVector3 pos;
				btQuaternion orn;
				m_np->getObjectTransformFromCpu(&pos[0],&orn[0],i);
				btTransform newTrans;
				newTrans.setOrigin(pos);
				newTrans.setRotation(orn);

				btCollisionObject* colObj = this->m_collisionObjects[i];
				colObj->setWorldTransform(newTrans);

				// synchronize motionstate
				btRigidBody* body = btRigidBody::upcast(colObj);
				if (body)
					this->synchronizeSingleMotionState(body);
			}
		}
	}


#ifndef B3_NO_PROFILE
	CProfileManager::Increment_Frame_Counter();
#endif //B3_NO_PROFILE



	return 1;
}


void	b3GpuDynamicsWorld::setGravity(const btVector3& gravity)
{
}

int b3GpuDynamicsWorld::findOrRegisterCollisionShape(const btCollisionShape* colShape)
{
	int index = m_uniqueShapes.findLinearSearch(colShape);
	if (index==m_uniqueShapes.size())
	{
		if (colShape->isPolyhedral())
		{
			m_uniqueShapes.push_back(colShape);
			
			btPolyhedralConvexShape* convex = (btPolyhedralConvexShape*)colShape;
			int numVertices=convex->getNumVertices();
			
			int strideInBytes=sizeof(btVector3);
			btAlignedObjectArray<btVector3> tmpVertices;
			tmpVertices.resize(numVertices);
			for (int i=0;i<numVertices;i++)
				convex->getVertex(i,tmpVertices[i]);
			const float scaling[4]={1,1,1,1};
			//bool noHeightField=true;
			
			int gpuShapeIndex = m_np->registerConvexHullShape(&tmpVertices[0].getX(), strideInBytes, numVertices, scaling);
			m_uniqueShapeMapping.push_back(gpuShapeIndex);
		} else
		{
			if (colShape->getShapeType()==TRIANGLE_MESH_SHAPE_PROXYTYPE)
			{
				m_uniqueShapes.push_back(colShape);
				
				btBvhTriangleMeshShape* trimesh = (btBvhTriangleMeshShape*) colShape;
				btStridingMeshInterface* meshInterface = trimesh->getMeshInterface();
				b3AlignedObjectArray<b3Vector3> vertices;
				b3AlignedObjectArray<int> indices;
				
				btVector3 trimeshScaling(1,1,1);
				for (int partId=0;partId<meshInterface->getNumSubParts();partId++)
				{
					
					const unsigned char *vertexbase = 0;
					int numverts = 0;
					PHY_ScalarType type = PHY_INTEGER;
					int stride = 0;
					const unsigned char *indexbase = 0;
					int indexstride = 0;
					int numfaces = 0;
					PHY_ScalarType indicestype = PHY_INTEGER;
					//PHY_ScalarType indexType=0;
					
					b3Vector3 triangleVerts[3];
					meshInterface->getLockedReadOnlyVertexIndexBase(&vertexbase,numverts,	type,stride,&indexbase,indexstride,numfaces,indicestype,partId);
					btVector3 aabbMin,aabbMax;
					
					for (int triangleIndex = 0 ; triangleIndex < numfaces;triangleIndex++)
					{
						unsigned int* gfxbase = (unsigned int*)(indexbase+triangleIndex*indexstride);
						
						for (int j=2;j>=0;j--)
						{
							
							int graphicsindex = indicestype==PHY_SHORT?((unsigned short*)gfxbase)[j]:gfxbase[j];
							if (type == PHY_FLOAT)
							{
								float* graphicsbase = (float*)(vertexbase+graphicsindex*stride);
								triangleVerts[j] = b3Vector3(
															 graphicsbase[0]*trimeshScaling.getX(),
															 graphicsbase[1]*trimeshScaling.getY(),
															 graphicsbase[2]*trimeshScaling.getZ());
							}
							else
							{
								double* graphicsbase = (double*)(vertexbase+graphicsindex*stride);
								triangleVerts[j] = b3Vector3( btScalar(graphicsbase[0]*trimeshScaling.getX()),
															 btScalar(graphicsbase[1]*trimeshScaling.getY()),
															 btScalar(graphicsbase[2]*trimeshScaling.getZ()));
							}
						}
						vertices.push_back(triangleVerts[0]);
						vertices.push_back(triangleVerts[1]);
						vertices.push_back(triangleVerts[2]);
						indices.push_back(indices.size());
						indices.push_back(indices.size());
						indices.push_back(indices.size());
					}
				}
				//GraphicsShape* gfxShape = 0;//b3BulletDataExtractor::createGraphicsShapeFromWavefrontObj(objData);
				
				//GraphicsShape* gfxShape = b3BulletDataExtractor::createGraphicsShapeFromConvexHull(&sUnitSpherePoints[0],MY_UNITSPHERE_POINTS);
				float meshScaling[4] = {1,1,1,1};
				//int shapeIndex = renderer.registerShape(gfxShape->m_vertices,gfxShape->m_numvertices,gfxShape->m_indices,gfxShape->m_numIndices);
				//float groundPos[4] = {0,0,0,0};
				
				//renderer.registerGraphicsInstance(shapeIndex,groundPos,rotOrn,color,meshScaling);
				if (vertices.size() && indices.size())
				{
					int gpuShapeIndex = m_np->registerConcaveMesh(&vertices,&indices, meshScaling);
					m_uniqueShapeMapping.push_back(gpuShapeIndex);
				} else
				{
					printf("Error: no vertices in mesh in b3GpuDynamicsWorld::addRigidBody\n");
					index = -1;
					b3Assert(0);
				}
				
				
			} else
			{
				if (colShape->getShapeType()==COMPOUND_SHAPE_PROXYTYPE)
				{
					
					btCompoundShape* compound = (btCompoundShape*) colShape;
					b3AlignedObjectArray<b3GpuChildShape> childShapes;
					
					for (int i=0;i<compound->getNumChildShapes();i++)
					{
						//for now, only support polyhedral child shapes
						b3Assert(compound->getChildShape(i)->isPolyhedral());
						b3GpuChildShape child;
						child.m_shapeIndex = findOrRegisterCollisionShape(compound->getChildShape(i));
						btVector3 pos = compound->getChildTransform(i).getOrigin();
						btQuaternion orn = compound->getChildTransform(i).getRotation();
						for (int v=0;v<4;v++)
						{
							child.m_childPosition[v] = pos[v];
							child.m_childOrientation[v] = orn[v];
						}
						childShapes.push_back(child);
					}
					index = m_uniqueShapes.size();
					m_uniqueShapes.push_back(colShape);
					
					int gpuShapeIndex = m_np->registerCompoundShape(&childShapes);
					m_uniqueShapeMapping.push_back(gpuShapeIndex);

					
					
					
					/*printf("Error: unsupported compound type (%d) in b3GpuDynamicsWorld::addRigidBody\n",colShape->getShapeType());
					index = -1;
					b3Assert(0);
					 */
				} else
				{
					if (colShape->getShapeType()==SPHERE_SHAPE_PROXYTYPE)
					{
						m_uniqueShapes.push_back(colShape);
						btSphereShape* sphere = (btSphereShape*)colShape;
						
						int gpuShapeIndex = m_np->registerSphereShape(sphere->getRadius());
						m_uniqueShapeMapping.push_back(gpuShapeIndex);
					} else
					{
						if (colShape->getShapeType()==STATIC_PLANE_PROXYTYPE)
						{
							m_uniqueShapes.push_back(colShape);
							btStaticPlaneShape* plane = (btStaticPlaneShape*)colShape;
						
							int gpuShapeIndex = m_np->registerPlaneShape((b3Vector3&)plane->getPlaneNormal(),plane->getPlaneConstant());
							m_uniqueShapeMapping.push_back(gpuShapeIndex);
						} else
						{
							printf("Error: unsupported shape type (%d) in b3GpuDynamicsWorld::addRigidBody\n",colShape->getShapeType());
							index = -1;
							b3Assert(0);
						}
					}
				}
			}
		}
		
	}
	
	return index;
}

void	b3GpuDynamicsWorld::addRigidBody(btRigidBody* body)
{
	m_cpuGpuSync = true;
	//body->setMotionState(0);
	

	int index = findOrRegisterCollisionShape(body->getCollisionShape());

	if (index>=0)
	{
		int gpuShapeIndex= m_uniqueShapeMapping[index];
		float mass = body->getInvMass() ? 1.f/body->getInvMass() : 0.f;
		btVector3 pos = body->getWorldTransform().getOrigin();
		btQuaternion orn = body->getWorldTransform().getRotation();
	
		m_rigidBodyPipeline->registerPhysicsInstance(mass,&pos.getX(),&orn.getX(),gpuShapeIndex,m_collisionObjects.size(),false);

		m_collisionObjects.push_back(body);
		//btDynamicsWorld::addCollisionObject(
	}
}

void    b3GpuDynamicsWorld::addSoftBodyCl(btSoftbodyCL* softBody)
{
	//const CAabb& aabb = softBody->GetAabb();
	//btVector3 aabbMin(aabb.Min()[0], aabb.Min()[1], aabb.Min()[2]);
	//btVector3 aabbMax(aabb.Max()[0], aabb.Max()[1], aabb.Max()[2]);
	
	m_softbodySolver->addSoftBody(softBody);
}

void	b3GpuDynamicsWorld::removeCollisionObject(btCollisionObject* colObj)
{
	m_cpuGpuSync = true;
	btDynamicsWorld::removeCollisionObject(colObj);
}

void	b3GpuDynamicsWorld::rayTest(const btVector3& rayFromWorld, const btVector3& rayToWorld, RayResultCallback& resultCallback) const
{

}

void	b3GpuDynamicsWorld::synchronizeSingleMotionState(btRigidBody* body)
{
	btAssert(body);

	if (body->getMotionState() && !body->isStaticOrKinematicObject())
	{
		//we need to call the update at least once, even for sleeping objects
		//otherwise the 'graphics' transform never updates properly
		const btTransform& centerOfMassWorldTrans = body->getWorldTransform();
		body->getMotionState()->setWorldTransform(centerOfMassWorldTrans);
	}
}

