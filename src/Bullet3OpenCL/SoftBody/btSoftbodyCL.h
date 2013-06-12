#ifndef B3_SOFTBODY_CL_H
#define B3_SOFTBODY_CL_H

#include "aabb.h"

#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "btSoftbodyTriangleCL.h"

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
		b3Assert( 0 <= i && i <= 1 );

		return m_IndexVrx[i];
	}

	float GetRestLength() const { return m_RestLength; }
	void SetRestLength(float restLength) { m_RestLength = restLength; }

	int GetIndex() const { return m_Index; }
	void SetIndex(int index) { m_Index = index; }

	int GetTriangleIndex(int i) const 
	{ 
		b3Assert( 0 <= i && i <= 1 );
		return m_IndexTriangle[i]; 
	}

	int GetTheOtherVertexIndex(int indexVert)
	{
		b3Assert(indexVert == m_IndexVrx[0] || indexVert == m_IndexVrx[1]);

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
	b3Vector3 m_Pos;
	b3Vector3 m_PosNext;
	b3Vector3 m_Vel;
	float m_InvMass; // Inverse mass. To pin the vertex, set zero to make the mass infinite. 
	b3Vector3 m_Accel;
	int m_PinIndex;
		
	// array of indexes of stretch springs connected to this vertex 
	b3AlignedObjectArray<int> m_StrechSpringIndexes; 

	// array of indexes of bend springs connected to this vertex 
	b3AlignedObjectArray<int> m_BendSpringIndexes; 

public:
	int GetIndex() const { return m_Index; }
	void SetIndex(int index) { m_Index = index; }
};

class btSoftbodyCL
{
public:
	btSoftbodyCL();
	btSoftbodyCL(btSoftBody* softbody);
	virtual ~btSoftbodyCL(void);

public:
	float m_dt;
	float m_Kst; // stretch stiffness for position-based dynamics. It should be 0..1
	float m_Kb; // bending stiffness for position-based dynamics. It should be 0..1

	float m_Kd;
	float m_Mu; // friction
	b3Vector3 m_Gravity;
	bool m_bDeformable;
	
	CAabb m_Aabb;

	b3AlignedObjectArray<btSoftbodyNodeCL> m_VertexArray;
	b3AlignedObjectArray<btSoftbodyLinkCL> m_StrechSpringArray;
	b3AlignedObjectArray<btSoftbodyLinkCL> m_BendSpringArray;
	b3AlignedObjectArray<b3Vector3> m_NormalVecArray;
	b3AlignedObjectArray<btSoftbodyTriangleCL> m_TriangleArray;

	b3AlignedObjectArray<CAabb> m_AABBVertexArray;

	b3AlignedObjectArray<int> m_BatchStretchSpringIndexArray;
	b3AlignedObjectArray<int> m_BatchBendSpringIndexArray;
	int m_numBatchStretchSpring;
	int m_numBatchBendingSpring;
	
protected:	
	// margin for collision detection
	float m_Margin;	

public:
	unsigned int m_NumIter;
	bool m_bEqualVertexMass;
	int m_NumIterForConstraintSolver;

	virtual void initialize();
	float getKst() const { return m_Kst; };
	float getKb() const { return m_Kb; };
	float getFrictionCoef() const { return m_Mu; }
	void setKst(float Kst) { b3Assert(0 <= Kst && Kst <= 1.0f); m_Kst = Kst; };
	void setKb(float Kb) { b3Assert(0 <= Kb && Kb <= 1.0f); m_Kb = Kb; };
	void setFrictionCoef(float mu) { b3Assert(mu >= 0 && mu <= 1.0f); m_Mu = mu; }
	float getdt() const { return m_dt; } 
	void setdt(float dt) { m_dt = dt; }
	void setGravity(const b3Vector3& gravity);
	const b3Vector3& GetGravity() const;
	void setMassDensity(float massDensity);
	void setVertexMass(float vertexMass);
	void setTotalMass(float totalMass);
	void setNumIterForConstraintSolver(int numIter) { m_NumIterForConstraintSolver = numIter; }
	const CAabb& getAabb() const { return m_Aabb; }
	void setAabb(const CAabb& aabb) { m_Aabb = aabb; }

	float getMargin() const { return m_Margin; }
	void setMargin(float margin) { m_Margin = margin; }

	b3AlignedObjectArray<btSoftbodyNodeCL>& getVertexArray() { return m_VertexArray; }
	const b3AlignedObjectArray<btSoftbodyNodeCL>& getVertexArray() const { return m_VertexArray; }

	b3AlignedObjectArray<btSoftbodyLinkCL>& getStrechSpringArray() { return m_StrechSpringArray; }
	const b3AlignedObjectArray<btSoftbodyLinkCL>& getStrechSpringArray() const { return m_StrechSpringArray; }

	b3AlignedObjectArray<btSoftbodyLinkCL>& getBendSpringArray() { return m_BendSpringArray; }
	const b3AlignedObjectArray<btSoftbodyLinkCL>& getBendSpringArray() const { return m_BendSpringArray; }

	b3AlignedObjectArray<btSoftbodyTriangleCL>& getTriangleArray() { return m_TriangleArray; }
	const b3AlignedObjectArray<btSoftbodyTriangleCL>& getTriangleArray() const { return m_TriangleArray; }

	const b3AlignedObjectArray<int>&  getBatchStretchSpringIndexArray() { return m_BatchStretchSpringIndexArray; }
	const b3AlignedObjectArray<int>&  getBatchBendSpringIndexArray() { return  m_BatchBendSpringIndexArray; }

	bool isDeformable() const { return m_bDeformable; }
	void setDeformable(bool bDeformable) { m_bDeformable = bDeformable; }

	void clear();
	
	void generateBatches();
	int getNumBatchStretchSpring() { return m_numBatchStretchSpring; }
	int getNumBatchBendingSpring() { return m_numBatchBendingSpring; }

	virtual bool integrate(float dt);
	virtual bool advancePosition(float dt);

	/*virtual bool ResolveCollision(CCollisionObject& convexObject, float dt);*/

	virtual void initializeBoundingVolumes();
	virtual void updateBoundingVolumes(float dt);

protected:
	void fillSpringArray();
	void applyGravity(float dt);
	void applyForces(float dt); 
	void clearForces();	
	void computeNextVertexPositions(float dt); 
	float clcConstraint(int indexEdge, int indexVertex, float dt, b3Vector3* pGradientOfConstraint = NULL);
	void enforceEdgeConstraints(float k, float dt);
	void enforceBendingConstraints(float k, float dt);
	void enforceEdgeConstraintsBatched(float k, float dt);
	void enforceBendingConstraintsBatched(float k, float dt);
	void updateVelocities(float dt);
};

#endif // B3_SOFTBODY_CL_H

