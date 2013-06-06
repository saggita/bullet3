#pragma once


#include <assert.h>

#include <iostream>
#include <fstream>

#include "aabb.h"

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btVector3.h"

class btVector3;
class btSoftbodyTriangleCL;

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
	btSoftbodyCL(void);
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
	// for debug
	bool m_bShowBV; // toggle showing bounding volume
	
	// margin for collision detection
	float m_Margin;	

public:
	unsigned int m_NumIter;
	bool m_bEqualVertexMass;
	int m_NumIterForConstraintSolver;

	virtual bool Load(const char* filename);
	virtual void Initialize();
	float GetKst() const { return m_Kst; };
	float GetKb() const { return m_Kb; };
	float GetFrictionCoef() const { return m_Mu; }
	void SetKst(float Kst) { assert(0 <= Kst && Kst <= 1.0f); m_Kst = Kst; };
	void SetKb(float Kb) { assert(0 <= Kb && Kb <= 1.0f); m_Kb = Kb; };
	void SetFrictionCoef(float mu) { assert(mu >= 0 && mu <= 1.0f); m_Mu = mu; }
	float Getdt() const { return m_dt; } 
	void Setdt(float dt) { m_dt = dt; }
	void SetGravity(const btVector3& gravity);
	const btVector3& GetGravity() const;
	bool GetShowBV() { return m_bShowBV; }
	void SetShowBV(bool bShowBV) { m_bShowBV = bShowBV; }
	void SetMassDensity(float massDensity);
	void SetVertexMass(float vertexMass);
	void SetTotalMass(float totalMass);
	void SetNumIterForConstraintSolver(int numIter) { m_NumIterForConstraintSolver = numIter; }
	const CAabb& GetAabb() const { return m_Aabb; }
	void SetAabb(const CAabb& aabb) { m_Aabb = aabb; }

	float GetMargin() const { return m_Margin; }
	void SetMargin(float margin) { m_Margin = margin; }

	btAlignedObjectArray<btSoftbodyNodeCL>& GetVertexArray() { return m_VertexArray; }
	const btAlignedObjectArray<btSoftbodyNodeCL>& GetVertexArray() const { return m_VertexArray; }

	btAlignedObjectArray<btSoftbodyLinkCL>& GetStrechSpringArray() { return m_StrechSpringArray; }
	const btAlignedObjectArray<btSoftbodyLinkCL>& GetStrechSpringArray() const { return m_StrechSpringArray; }

	btAlignedObjectArray<btSoftbodyLinkCL>& GetBendSpringArray() { return m_BendSpringArray; }
	const btAlignedObjectArray<btSoftbodyLinkCL>& GetBendSpringArray() const { return m_BendSpringArray; }

	btAlignedObjectArray<btSoftbodyTriangleCL>& GetTriangleArray() { return m_TriangleArray; }
	const btAlignedObjectArray<btSoftbodyTriangleCL>& GetTriangleArray() const { return m_TriangleArray; }

	const btAlignedObjectArray<int>&  GetBatchStretchSpringIndexArray() { return m_BatchStretchSpringIndexArray; }
	const btAlignedObjectArray<int>&  GetBatchBendSpringIndexArray() { return  m_BatchBendSpringIndexArray; }

	bool IsDeformable() const { return m_bDeformable; }
	void SetDeformable(bool bDeformable) { m_bDeformable = bDeformable; }

	void Clear();
	
	void GenerateBatches();
	int GetNumBatchStretchSpring() { return m_numBatchStretchSpring; }
	int GetNumBatchBendingSpring() { return m_numBatchBendingSpring; }

	virtual bool Integrate(float dt);
	virtual bool AdvancePosition(float dt);
	
	virtual void Render(bool bBBox = false);
	int RenderBatch(int i) const;

	/*virtual bool ResolveCollision(CCollisionObject& convexObject, float dt);*/

	virtual void InitializeBoundingVolumes();
	virtual void UpdateBoundingVolumes(float dt);

	virtual void TranslateW(float x, float y, float z);

protected:
	void FillSpringArray();
	void ApplyGravity(float dt);
	void ApplyForces(float dt);
	void ClearForces();	
	void ComputeNextVertexPositions(float dt);
	float CalcConstraint(int indexEdge, int indexVertex, float dt, btVector3* pGradientOfConstraint = NULL);
	void EnforceEdgeConstraints(float k, float dt);
	void EnforceBendingConstraints(float k, float dt);
	void EnforceEdgeConstraintsBatched(float k, float dt);
	void EnforceBendingConstraintsBatched(float k, float dt);
	void UpdateVelocities(float dt);
	
public:
	btSoftbodyCL& operator=(const btSoftbodyCL& other);
};


