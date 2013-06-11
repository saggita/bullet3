/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


///create 125 (5x5x5) dynamic object
//#define ARRAY_SIZE_X 25
//#define ARRAY_SIZE_Y 20
//#define ARRAY_SIZE_Z 25

#define ARRAY_SIZE_X 10
#define ARRAY_SIZE_Y 10
#define ARRAY_SIZE_Z 10

//maximum number of objects (and allow user to shoot additional boxes)
#define MAX_PROXIES (ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z + 1024)

///scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1.
#define START_POS_X -5
#define START_POS_Y -5
#define START_POS_Z -3

#include "BasicGpuDemo.h"
#include "GlutStuff.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include "b3GpuDynamicsWorld.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"



#include "Bullet3OpenCL/BroadphaseCollision/b3GpuSapBroadphase.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3LauncherCL.h"
#include "Bullet3OpenCL/RigidBody/b3GpuRigidBodyPipeline.h"
#include "Bullet3OpenCL/RigidBody/b3GpuNarrowPhase.h"
#include "Bullet3OpenCL/RigidBody/b3Config.h"
#include "Bullet3OpenCL/SoftBody/btSoftBodySimulationSolverOpenCL.h"
#include "Bullet3OpenCL/SoftBody/btSoftbodyCL.h"
#include "Bullet3Collision/BroadPhaseCollision/b3DynamicBvhBroadphase.h"
#include "vectormath/vmInclude.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"

#include <stdio.h> //printf debugging
#include "GLDebugDrawer.h"

static GLDebugDrawer gDebugDraw;

void BasicGpuDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();
	
	///step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();

		// draw softbodies
		const btSoftBodySimulationSolverOpenCL* softbodySolverCL = ((b3GpuDynamicsWorld*)m_dynamicsWorld)->getSoftBodySolverCL();

		if ( softbodySolverCL )
		{
			for ( int i = 0; i < softbodySolverCL->getSoftBodies().size(); i++ )
			{
				btSoftBody* softbodyCPU = softbodySolverCL->getSoftBodies()[i]->getSoftBodyCPU();

				int drawFlags = fDrawFlags::Links | fDrawFlags::Faces;
				btSoftBodyHelpers::Draw(softbodyCPU, m_dynamicsWorld->getDebugDrawer(), drawFlags);
			
				// draw AABB of softbody
				

				btVector3 minAABB(softbodySolverCL->getSoftBodies()[i]->getAabb().Min());
				btVector3 maxAABB(softbodySolverCL->getSoftBodies()[i]->getAabb().Max());

				glColor3f(1.0f, 1.0f, 1.0f);

				glLineWidth(1.0);

				glBegin(GL_LINE_STRIP);
					glVertex3d(minAABB[0], minAABB[1], minAABB[2]);
					glVertex3d(maxAABB[0], minAABB[1], minAABB[2]);
					glVertex3d(maxAABB[0], minAABB[1], maxAABB[2]);
					glVertex3d(minAABB[0], minAABB[1], maxAABB[2]);
					glVertex3d(minAABB[0], minAABB[1], minAABB[2]);
				glEnd();

				glBegin(GL_LINE_STRIP);
					glVertex3d(minAABB[0], maxAABB[1], minAABB[2]);
					glVertex3d(maxAABB[0], maxAABB[1], minAABB[2]);
					glVertex3d(maxAABB[0], maxAABB[1], maxAABB[2]);
					glVertex3d(minAABB[0], maxAABB[1], maxAABB[2]);
					glVertex3d(minAABB[0], maxAABB[1], minAABB[2]);
				glEnd();

				glBegin(GL_LINES);
					glVertex3d(minAABB[0], minAABB[1], minAABB[2]);
					glVertex3d(minAABB[0], maxAABB[1], minAABB[2]);

					glVertex3d(maxAABB[0], minAABB[1], minAABB[2]);
					glVertex3d(maxAABB[0], maxAABB[1], minAABB[2]);

					glVertex3d(maxAABB[0], minAABB[1], maxAABB[2]);
					glVertex3d(maxAABB[0], maxAABB[1], maxAABB[2]);

					glVertex3d(minAABB[0], minAABB[1], maxAABB[2]);
					glVertex3d(minAABB[0], maxAABB[1], maxAABB[2]);
				glEnd();


			}
		}

	}
		
	renderme(); 

	glFlush();

	swapBuffers();

}



void BasicGpuDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	
	renderme();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	swapBuffers();
}

struct btInternalData
{
	cl_context	m_clContext;
	cl_device_id	m_clDevice;
	cl_command_queue	m_clQueue;
	const char* m_clDeviceName;
	bool	m_clInitialized;

	btInternalData()
	{
		m_clContext = 0;
		m_clDevice = 0;
		m_clQueue = 0;
		m_clDeviceName = 0;
		m_clInitialized =false;
	}
};

void BasicGpuDemo::initCL(int preferredDeviceIndex, int preferredPlatformIndex)
{
	void* glCtx=0;
	void* glDC = 0;
	
	
    
	int ciErrNum = 0;
	//#ifdef CL_PLATFORM_INTEL
	//cl_device_type deviceType = CL_DEVICE_TYPE_ALL;
	//#else
	cl_device_type deviceType = CL_DEVICE_TYPE_GPU;
	//#endif
	
	cl_platform_id platformId;
	
	//	if (useInterop)
	//	{
	//		m_data->m_clContext = b3OpenCLUtils::createContextFromType(deviceType, &ciErrNum, glCtx, glDC);
	//	} else
	{
		m_clData->m_clContext = b3OpenCLUtils::createContextFromType(deviceType, &ciErrNum, 0,0,preferredDeviceIndex, preferredPlatformIndex,&platformId);
		b3OpenCLUtils::printPlatformInfo(platformId);
	}
	
	
	oclCHECKERROR(ciErrNum, CL_SUCCESS);
	
	int numDev = b3OpenCLUtils::getNumDevices(m_clData->m_clContext);
	
	if (numDev>0)
	{
		m_clData->m_clDevice= b3OpenCLUtils::getDevice(m_clData->m_clContext,0);
		m_clData->m_clQueue = clCreateCommandQueue(m_clData->m_clContext, m_clData->m_clDevice, 0, &ciErrNum);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
        
        b3OpenCLUtils::printDeviceInfo(m_clData->m_clDevice);
		b3OpenCLDeviceInfo info;
		b3OpenCLUtils::getDeviceInfo(m_clData->m_clDevice,&info);
		m_clData->m_clDeviceName = info.m_deviceName;
		m_clData->m_clInitialized = true;
		
	}
	
}

void BasicGpuDemo::exitCL()
{

}

BasicGpuDemo::BasicGpuDemo()
{
	m_clData = new btInternalData;
	setCameraDistance(btScalar(SCALING*60.));
	this->setAzi(45);
	this->setEle(45);

}

BasicGpuDemo::~BasicGpuDemo()
{
	exitPhysics();
	exitCL();
	delete m_clData;
}

bool testAndAddLink( btAlignedObjectArray<int> &trianglesForLinks, btSoftBody *softBody, int triangle, int *triangleVertexIndexArray, int numVertices, int vertex0, int vertex1, int nonLinkVertex, btSoftBody::Material *structuralMaterial, bool createBendLinks, btSoftBody::Material *bendMaterial )
{		
	if( trianglesForLinks[ numVertices * vertex0 + vertex1 ] >= 0 && createBendLinks)
	{
		// Already have link so find other triangle and generate cross link

		int otherTriangle = trianglesForLinks[numVertices * vertex0 + vertex1];
		int otherIndices[3] = {triangleVertexIndexArray[otherTriangle * 3], triangleVertexIndexArray[otherTriangle * 3 + 1], triangleVertexIndexArray[otherTriangle * 3 + 2]};

		int nodeA;
		// Test all links of the other triangle against this link. The one that's not part of it is what we want.
		if( otherIndices[0] != vertex0 && otherIndices[0] != vertex1 )
			nodeA = otherIndices[0];
		if( otherIndices[1] != vertex0 && otherIndices[1] != vertex1 )
			nodeA = otherIndices[1];
		if( otherIndices[2] != vertex0 && otherIndices[2] != vertex1 )
			nodeA = otherIndices[2];

		softBody->appendLink( nodeA, nonLinkVertex, bendMaterial );
	} else {
		// Don't yet have link so create it
		softBody->appendLink( vertex0, vertex1, structuralMaterial );

		// If we added a new link, set the triangle array
		trianglesForLinks[numVertices * vertex0 + vertex1] = triangle;
		trianglesForLinks[numVertices * vertex1 + vertex0] = triangle;

	}

	return true;
}

btSoftBody* createFromIndexedMesh(btSoftBodyWorldInfo* softbodyWorldInfo, btVector3 *vertexArray, int numVertices, int *triangleVertexIndexArray, int numTriangles, bool createBendLinks )
{
	btSoftBody* softBody = new btSoftBody(softbodyWorldInfo, numVertices, vertexArray, 0);
	btSoftBody::Material * structuralMaterial = softBody->appendMaterial();
	btSoftBody::Material * bendMaterial;
	if( createBendLinks )
	{
		bendMaterial = softBody->appendMaterial();
		bendMaterial->m_kLST = 0.7f;
	} else {
		bendMaterial = NULL;
	}
	structuralMaterial->m_kLST = 1.0;
	

	// List of values for each link saying which triangle is associated with that link
	// -1 to start. Once a value is entered we know the "other" triangle
	// and can add a link across the link
	btAlignedObjectArray<int> triangleForLinks;
	triangleForLinks.resize( numVertices * numVertices, -1 );
	int numLinks = 0;
	for( int triangle = 0; triangle < numTriangles; ++triangle )
	{
		int index[3] = {triangleVertexIndexArray[triangle * 3], triangleVertexIndexArray[triangle * 3 + 1], triangleVertexIndexArray[triangle * 3 + 2]};
		softBody->appendFace( index[0], index[1], index[2] );
		
		// Generate the structural links directly from the triangles
		testAndAddLink( triangleForLinks, softBody, triangle, triangleVertexIndexArray, numVertices, index[0], index[1], index[2], structuralMaterial, createBendLinks, bendMaterial );
		testAndAddLink( triangleForLinks, softBody, triangle, triangleVertexIndexArray, numVertices, index[1], index[2], index[0], structuralMaterial, createBendLinks, bendMaterial );
		testAndAddLink( triangleForLinks, softBody, triangle, triangleVertexIndexArray, numVertices, index[2], index[0], index[1], structuralMaterial, createBendLinks, bendMaterial);
	}

	return softBody;
}

btSoftBody* createFlag(btSoftBodyWorldInfo* softbodyWorldInfo, int width, int height, float xTranslate = 0)
{
	// First create a triangle mesh to represent a flag

	using Vectormath::Aos::Matrix3;
	using Vectormath::Aos::Vector3;

	// Allocate a simple mesh consisting of a vertex array and a triangle index array
	btIndexedMesh mesh;
	mesh.m_numVertices = width*height;
	mesh.m_numTriangles = 2*(width-1)*(height-1);

	btVector3 *vertexArray = new btVector3[mesh.m_numVertices];

	mesh.m_vertexBase = reinterpret_cast<const unsigned char*>(vertexArray);
	int *triangleVertexIndexArray = new int[3*mesh.m_numTriangles];	
	mesh.m_triangleIndexBase = reinterpret_cast<const unsigned char*>(triangleVertexIndexArray);
	mesh.m_triangleIndexStride = sizeof(int)*3;
	mesh.m_vertexStride = sizeof(Vector3);

	// Generate normalised object space vertex coordinates for a rectangular flag
	float zCoordinate = 15.0f;
	
	Matrix3 defaultScale(Vector3(5.f, 0.f, 0.f), Vector3(0.f, 20.f, 0.f), Vector3(0.f, 0.f, 1.f));
	for( int y = 0; y < height; ++y )
	{
		float yCoordinate = y*2.0f/float(height) - 1.0f;
		for( int x = 0; x < width; ++x )
		{			
			float xCoordinate = x*2.0f/float(width) - 1.0f;

			Vector3 vertex(xCoordinate, yCoordinate, zCoordinate);
			Vector3 transformedVertex = defaultScale*vertex;

			vertexArray[y*width + x] = btVector3(transformedVertex.getX(), transformedVertex.getY(), transformedVertex.getZ() );

		}
	}

	// Generate vertex indices for triangles
	for( int y = 0; y < (height-1); ++y )
	{
		for( int x = 0; x < (width-1); ++x )
		{	
			// Triangle 0
			// Top left of square on mesh
			{
				int vertex0 = y*width + x;
				int vertex1 = vertex0 + 1;
				int vertex2 = vertex0 + width;
				int triangleIndex = 2*y*(width-1) + 2*x;
				triangleVertexIndexArray[(mesh.m_triangleIndexStride*triangleIndex)/sizeof(int)] = vertex0;
				triangleVertexIndexArray[(mesh.m_triangleIndexStride*triangleIndex+1)/sizeof(int)+1] = vertex1;
				triangleVertexIndexArray[(mesh.m_triangleIndexStride*triangleIndex+2)/sizeof(int)+2] = vertex2;
			}

			// Triangle 1
			// Bottom right of square on mesh
			{
				int vertex0 = y*width + x + 1;
				int vertex1 = vertex0 + width;
				int vertex2 = vertex1 - 1;
				int triangleIndex = 2*y*(width-1) + 2*x + 1;
				triangleVertexIndexArray[(mesh.m_triangleIndexStride*triangleIndex)/sizeof(int)] = vertex0;
				triangleVertexIndexArray[(mesh.m_triangleIndexStride*triangleIndex)/sizeof(int)+1] = vertex1;
				triangleVertexIndexArray[(mesh.m_triangleIndexStride*triangleIndex)/sizeof(int)+2] = vertex2;
			}
		}
	}

	
	float rotateAngleRoundZ = 0.0;
	//float rotateAngleRoundX = 1.0;
	float rotateAngleRoundX = 3.14159f/2.0f;
	btMatrix3x3 defaultRotate;
	defaultRotate[0] = btVector3(cos(rotateAngleRoundZ), sin(rotateAngleRoundZ), 0.f); 
	defaultRotate[1] = btVector3(-sin(rotateAngleRoundZ), cos(rotateAngleRoundZ), 0.f);
	defaultRotate[2] = btVector3(0.f, 0.f, 1.f);
	btMatrix3x3 defaultRotateX;
	defaultRotateX[0] = btVector3(1.f, 0.f, 0.f);
	defaultRotateX[1] = btVector3( 0.f, cos(rotateAngleRoundX), sin(rotateAngleRoundX));
	defaultRotateX[2] = btVector3(0.f, -sin(rotateAngleRoundX), cos(rotateAngleRoundX));

	btMatrix3x3 defaultRotateAndScale( (defaultRotateX*defaultRotate) );

	// Construct the sequence flags applying a slightly different translation to each one to arrange them
	// appropriately in the scene.

	btVector3 defaultTranslate(xTranslate, 20.f, 0);

	btTransform transform( defaultRotateAndScale, defaultTranslate );
	transform.setOrigin(defaultTranslate);


	btSoftBody *softBody = createFromIndexedMesh(softbodyWorldInfo, vertexArray, mesh.m_numVertices, triangleVertexIndexArray, mesh.m_numTriangles, true );


	for( int i = 0; i < mesh.m_numVertices; ++i )
	{
		softBody->setMass(i, 10.f/mesh.m_numVertices);
	}

	softBody->setMass((height-1)*(width), 0.f);
	softBody->setMass((height-1)*(width) + width - 1, 0.f);
	softBody->setMass((height-1)*width + width/2, 0.f);
	softBody->m_cfg.collisions = btSoftBody::fCollision::CL_SS+btSoftBody::fCollision::CL_RS;	

	softBody->transform( transform );
		
	/*softBody->m_cfg.kLF = 0.15f;
	softBody->m_cfg.kDG	= 0.01f;*/
	softBody->m_cfg.kLF = 0.0f;
	softBody->m_cfg.kDG	= 0.0f;
	softBody->m_cfg.piterations = 10;
	softBody->m_cfg.aeromodel	=	btSoftBody::eAeroModel::V_TwoSidedLiftDrag;
	//softBody->setWindVelocity(btVector3(5.0, 0.0, 25.0));
	softBody->setWindVelocity(btVector3(0.0, 0.0, 0.0));

	delete [] vertexArray;
	delete [] triangleVertexIndexArray;

	return softBody;
}



void	BasicGpuDemo::initPhysics()
{
	setTexturing(true);
	setShadows(false);//too slow with many objects
	
	
	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = 0;
	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = 0;

	m_broadphase = 0;

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	
	m_solver = 0;

	initCL(-1,-1);


	if (!m_clData->m_clInitialized)
	{
		printf("Error: cannot initialize OpenCL\n");
		exit(0);
	}

	b3Config config;
	b3GpuNarrowPhase* np = new b3GpuNarrowPhase(m_clData->m_clContext,m_clData->m_clDevice,m_clData->m_clQueue,config);
	b3GpuSapBroadphase* bp = new b3GpuSapBroadphase(m_clData->m_clContext,m_clData->m_clDevice,m_clData->m_clQueue);
	//m_data->m_np = np;
	//m_data->m_bp = bp;
	b3DynamicBvhBroadphase* broadphaseDbvt = new b3DynamicBvhBroadphase(config.m_maxConvexBodies);
	
	b3GpuRigidBodyPipeline* rbp = new b3GpuRigidBodyPipeline(m_clData->m_clContext,m_clData->m_clDevice,m_clData->m_clQueue, np, bp,broadphaseDbvt);

	btSoftBodySimulationSolverOpenCL* sfs = new btSoftBodySimulationSolverOpenCL(m_clData->m_clContext,m_clData->m_clDevice,m_clData->m_clQueue);

	m_dynamicsWorld = new b3GpuDynamicsWorld(bp,np,rbp,sfs);
	
	m_dynamicsWorld->setDebugDrawer(&gDebugDraw);
	
	m_dynamicsWorld->setGravity(btVector3(0,-10,0));

	///create a few basic rigid bodies
	btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(150.),btScalar(50.),btScalar(150.)));
	//groundShape->initializePolyhedralFeatures();
//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
	
	m_collisionShapes.push_back(groundShape);
	if (0)
	{
		btTransform tr;
		tr.setIdentity();
		btVector3 faraway(-1e30,-1e30,-1e30);

		tr.setOrigin(faraway);
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(tr);
		btSphereShape* dummyShape = new btSphereShape(0.f);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,dummyShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);


	}
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-50,0));
		//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
	{
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		//btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,0,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);
		body->setWorldTransform(groundTransform);

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}
	
	
	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btBoxShape* colShape = new btBoxShape(btVector3(SCALING*1,SCALING*1,SCALING*1));
		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);

		float start_x = START_POS_X - ARRAY_SIZE_X/2;
		float start_y = START_POS_Y;
		float start_z = START_POS_Z - ARRAY_SIZE_Z/2;

		for (int k=0;k<ARRAY_SIZE_Y;k++)
		{
			for (int i=0;i<ARRAY_SIZE_X;i++)
			{
				for(int j = 0;j<ARRAY_SIZE_Z;j++)
				{
					startTransform.setOrigin(SCALING*btVector3(
										btScalar(2.*i + start_x),
										btScalar(6+2.0*k + start_y),
										btScalar(2.*j + start_z)));

			
					//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
					//btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
					btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,0,colShape,localInertia);
					btRigidBody* body = new btRigidBody(rbInfo);
					body->setWorldTransform(startTransform);
					

					m_dynamicsWorld->addRigidBody(body);
				}
			}
		}
	}

	// softbody
	int numOfFlags = 10;
	float xTranslateFlags = 15;

	for ( int i = 0; i < numOfFlags; i++ )
	{
		btSoftBodyWorldInfo softbodyInfo;
		btSoftBody* softbody = createFlag(&softbodyInfo, 10, 10, xTranslateFlags * i);

		

		((b3GpuDynamicsWorld*)m_dynamicsWorld)->addSoftBody(softbody);		
	}



	// transfer data to GPU
	np->writeAllBodiesToGpu();
	bp->writeAabbsToGpu();
	rbp->writeAllInstancesToGpu();

}
void	BasicGpuDemo::clientResetScene()
{
	exitPhysics();
	initPhysics();
}
	

void	BasicGpuDemo::exitPhysics()
{

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
	m_collisionShapes.clear();

	delete m_dynamicsWorld;
	
	delete m_solver;
	
	delete m_broadphase;
	
	delete m_dispatcher;

	delete m_collisionConfiguration;

	
}




