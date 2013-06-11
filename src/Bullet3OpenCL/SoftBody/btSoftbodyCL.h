#pragma once


#include <assert.h>

#include <iostream>
#include <fstream>

#include "aabb.h"

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btVector3.h"

class btVector3;
class btSoftbodyTriangleCL;
class btSoftBody;

class btSoftbodyLinkCL
{
	friend class btSoftbodyCL;

public:
	btSoftbodyLinkCL() 
	{ 
		m_IndexVrx[0] = -1; 
		m_IndexVrx[1] = -1; 
		m_IndexTriangle[0] = -1;
		m_IndexTriangle[1] = -1;
		m_Index = -1;
		m_Coloring = -1;
	}

	btSoftbodyLinkCL(int indexVrx0, int indexVrx1) 
	{ 
		m_IndexVrx[0] = indexVrx0; 
		m_IndexVrx[1] = indexVrx1;
		m_IndexTriangle[0] = -1;
		m_IndexTriangle[1] = -1;
		m_Index = -1; 
		m_Coloring = -1;
	}

	btSoftbodyLinkCL(const btSoftbodyLinkCL& other) 
	{
		for ( int i = 0; i < 2; i++ )
		{
			m_IndexVrx[i] = other.m_IndexVrx[i];
			m_IndexTriangle[i] = other.m_IndexTriangle[i];
		}

		m_Index = other.m_Index;
		m_IndexCloth = other.m_IndexCloth;
		m_RestLength = other.m_RestLength;
		m_Coloring = other.m_Coloring;
	}
	
	virtual ~btSoftbodyLinkCL() { }

public:
	int m_Index;
	int m_IndexVrx[2];
	int m_IndexTriangle[2];
	int m_IndexCloth;
	float m_RestLength;

public:
	int m_Coloring;

	int GetVertexIndex(int i) const 
	{
		assert( 0 <= i && i <= 1 );

		return m_IndexVrx[i];
	}

	float GetRestLength() const { return m_RestLength; }
	void SetRestLength(float restLength) { m_RestLength = restLength; }

	int GetIndex() const { return m_Index; }
	void SetIndex(int index) { m_Index = index; }

	int GetTriangleIndex(int i) const 
	{ 
		assert( 0 <= i && i <= 1 );
		return m_IndexTriangle[i]; 
	}

	int GetTheOtherVertexIndex(int indexVert)
	{
		assert(indexVert == m_IndexVrx[0] || indexVert == m_IndexVrx[1]);

		if ( indexVert == m_IndexVrx[0] )
			return m_IndexVrx[1];
		else
			return m_IndexVrx[0];
	}

	bool operator==(const btSoftbodyLinkCL& other)
	{
		if ( ( m_IndexVrx[0] == other.m_IndexVrx[0] && m_IndexVrx[1] == other.m_IndexVrx[1] ) ||
			 ( m_IndexVrx[0] == other.m_IndexVrx[1] && m_IndexVrx[1] == other.m_IndexVrx[0] ) )
			 return true;
		else
			return false;
	}

	btSoftbodyLinkCL& operator=(const btSoftbodyLinkCL& other)
	{
		for ( int i = 0; i < 2; i++ )
		{
			m_IndexVrx[i] = other.m_IndexVrx[i];
			m_IndexTriangle[i] = other.m_IndexTriangle[i];
		}

		m_Index = other.m_Index;
		m_IndexCloth = other.m_IndexCloth;
		m_RestLength = other.m_RestLength;
		m_Coloring = other.m_Coloring;

		return (*this);
	}
};

//----------------------------------------------
// btSoftbodyNodeCL
//----------------------------------------------
class btSoftbodyNodeCL
{
public:
	btSoftbodyNodeCL() : m_InvMass(1.0), m_Index(-1), m_IndexCloth(-1), m_PinIndex(-1), m_Vel(0, 0, 0), m_Accel(0, 0, 0)
	{
	}

	virtual ~btSoftbodyNodeCL() {};

	int m_Index;
	int m_IndexCloth;
	btVector3 m_Pos;
	btVector3 m_PosNext;
	btVector3 m_Vel;
	float m_InvMass; // Inverse mass. To pin the vertex, set zero to make the mass infinite. 
	btVector3 m_Accel;
	int m_PinIndex;
		
	// array of indexes of stretch springs connected to this vertex 
	btAlignedObjectArray<int> m_StrechSpringIndexes; 

	// array of indexes of bend springs connected to this vertex 
	btAlignedObjectArray<int> m_BendSpringIndexes; 

public:
	int GetIndex() const { return m_Index; }
	void SetIndex(int index) { m_Index = index; }
};

class btSoftbodyCL
{
public:
	btSoftbodyCL();
	btSoftbodyCL(btSoftBody* softbody);
	btSoftbodyCL(const btSoftbodyCL& other);
	virtual ~btSoftbodyCL(void);

public:
	float m_dt;
	float m_Kst; // stretch stiffness for position-based dynamics. It should be 0..1
	float m_Kb; // bending stiffness for position-based dynamics. It should be 0..1

	float m_Kd;
	float m_Mu; // friction
	btVector3 m_Gravity;
	//CBVHTree* m_pBVHTree;
	bool m_bDeformable;
	
	CAabb m_Aabb;

	btAlignedObjectArray<btSoftbodyNodeCL> m_VertexArray;
	btAlignedObjectArray<btSoftbodyLinkCL> m_StrechSpringArray;
	btAlignedObjectArray<btSoftbodyLinkCL> m_BendSpringArray;
	btAlignedObjectArray<btVector3> m_NormalVecArray;
	btAlignedObjectArray<btSoftbodyTriangleCL> m_TriangleArray;

	btAlignedObjectArray<CAabb> m_AABBVertexArray;

	btAlignedObjectArray<int> m_BatchStretchSpringIndexArray;
	btAlignedObjectArray<int> m_BatchBendSpringIndexArray;
	int m_numBatchStretchSpring;
	int m_numBatchBendingSpring;
	
protected:	
	btSoftBody* m_pSoftBodyCPU;

	// for debug
	bool m_bShowBV; // toggle showing bounding volume
	
	// margin for collision detection
	float m_Margin;	

public:
	unsigned int m_NumIter;
	bool m_bEqualVertexMass;
	int m_NumIterForConstraintSolver;

	virtual bool load(const char* filename);
	virtual void initialize();
	float getKst() const { return m_Kst; };
	float getKb() const { return m_Kb; };
	float getFrictionCoef() const { return m_Mu; }
	void setKst(float Kst) { assert(0 <= Kst && Kst <= 1.0f); m_Kst = Kst; };
	void setKb(float Kb) { assert(0 <= Kb && Kb <= 1.0f); m_Kb = Kb; };
	void setFrictionCoef(float mu) { assert(mu >= 0 && mu <= 1.0f); m_Mu = mu; }
	float getdt() const { return m_dt; } 
	void setdt(float dt) { m_dt = dt; }
	void setGravity(const btVector3& gravity);
	const btVector3& GetGravity() const;
	bool getShowBV() { return m_bShowBV; }
	void setShowBV(bool bShowBV) { m_bShowBV = bShowBV; }
	void setMassDensity(float massDensity);
	void setVertexMass(float vertexMass);
	void setTotalMass(float totalMass);
	void setNumIterForConstraintSolver(int numIter) { m_NumIterForConstraintSolver = numIter; }
	const CAabb& getAabb() const { return m_Aabb; }
	void setAabb(const CAabb& aabb) { m_Aabb = aabb; }

	float getMargin() const { return m_Margin; }
	void setMargin(float margin) { m_Margin = margin; }

	btAlignedObjectArray<btSoftbodyNodeCL>& getVertexArray() { return m_VertexArray; }
	const btAlignedObjectArray<btSoftbodyNodeCL>& getVertexArray() const { return m_VertexArray; }

	btAlignedObjectArray<btSoftbodyLinkCL>& getStrechSpringArray() { return m_StrechSpringArray; }
	const btAlignedObjectArray<btSoftbodyLinkCL>& getStrechSpringArray() const { return m_StrechSpringArray; }

	btAlignedObjectArray<btSoftbodyLinkCL>& getBendSpringArray() { return m_BendSpringArray; }
	const btAlignedObjectArray<btSoftbodyLinkCL>& getBendSpringArray() const { return m_BendSpringArray; }

	btAlignedObjectArray<btSoftbodyTriangleCL>& getTriangleArray() { return m_TriangleArray; }
	const btAlignedObjectArray<btSoftbodyTriangleCL>& getTriangleArray() const { return m_TriangleArray; }

	const btAlignedObjectArray<int>&  getBatchStretchSpringIndexArray() { return m_BatchStretchSpringIndexArray; }
	const btAlignedObjectArray<int>&  getBatchBendSpringIndexArray() { return  m_BatchBendSpringIndexArray; }

	bool isDeformable() const { return m_bDeformable; }
	void setDeformable(bool bDeformable) { m_bDeformable = bDeformable; }

	btSoftBody* getSoftBodyCPU() { return m_pSoftBodyCPU; }
	const btSoftBody* getSoftBodyCPU() const { return m_pSoftBodyCPU; }

	void clear();
	
	void generateBatches();
	int getNumBatchStretchSpring() { return m_numBatchStretchSpring; }
	int getNumBatchBendingSpring() { return m_numBatchBendingSpring; }

	virtual bool integrate(float dt);
	virtual bool advancePosition(float dt);

	/*virtual bool ResolveCollision(CCollisionObject& convexObject, float dt);*/

	virtual void initializeBoundingVolumes();
	virtual void updateBoundingVolumes(float dt);
	
	void updateSoftBodyCPU();

protected:
	void fillSpringArray();
	void applyGravity(float dt);
	void applyForces(float dt); 
	void clearForces();	
	void computeNextVertexPositions(float dt); 
	float clcConstraint(int indexEdge, int indexVertex, float dt, btVector3* pGradientOfConstraint = NULL);
	void enforceEdgeConstraints(float k, float dt);
	void enforceBendingConstraints(float k, float dt);
	void enforceEdgeConstraintsBatched(float k, float dt);
	void enforceBendingConstraintsBatched(float k, float dt);
	void updateVelocities(float dt);
	
public:
	btSoftbodyCL& operator=(const btSoftbodyCL& other);
};


