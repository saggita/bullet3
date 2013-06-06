#pragma once

#include <assert.h>

class btSoftbodyCL;
class btVector3;

class btSoftbodyTriangleCL
{
	friend class btSoftbodyCL;

public:
	btSoftbodyTriangleCL();
	btSoftbodyTriangleCL(const btSoftbodyTriangleCL& other);
	virtual ~btSoftbodyTriangleCL();

protected:
	int m_Index;
	int m_IndexVrx[3];
	int m_IndexEdge[3];
	int m_IndexNormalVec;
	
public:
	float A; // initial area without any deformation. Calculated before simulation starts and won't change over the simulation.

public:
	int GetVertexIndex(int i) const 
	{
		assert( 0 <= i && i < 3 );

		return m_IndexVrx[i];
	}

	void SetVertexIndex(int i, int vertexIndex)
	{
		assert( 0 <= i && i < 3 );
		 m_IndexVrx[i] = vertexIndex;
	}

	int GetEdgeIndex(int i) const 
	{
		assert( 0 <= i && i < 3 );

		return m_IndexEdge[i];
	}

	int GetIndex() const { return m_Index; }
	void SetIndex(int index) { m_Index = index; }
	int GetNormalVectIndex() const { return m_IndexNormalVec; }
	btVector3 GetPointByBaryCoord(const btSoftbodyCL* pCloth, float a, float b, float c) const;
	btVector3 GetVelocityByBaryCoord(const btSoftbodyCL* pCloth, float a, float b, float c) const;
	
	btSoftbodyTriangleCL& operator=(const btSoftbodyTriangleCL& other);
};
