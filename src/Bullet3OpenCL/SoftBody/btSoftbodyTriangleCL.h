#ifndef B3_SOFTBODY_TRIANGLE_CL_H
#define B3_SOFTBODY_TRIANGLE_CL_H

#include "Bullet3Common/b3Vector3.h"

class btSoftbodyCL;

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
		b3Assert( 0 <= i && i < 3 );

		return m_IndexVrx[i];
	}

	void SetVertexIndex(int i, int vertexIndex)
	{
		b3Assert( 0 <= i && i < 3 );
		 m_IndexVrx[i] = vertexIndex;
	}

	int GetEdgeIndex(int i) const 
	{
		b3Assert( 0 <= i && i < 3 );

		return m_IndexEdge[i];
	}

	int GetIndex() const { return m_Index; }
	void SetIndex(int index) { m_Index = index; }
	int GetNormalVectIndex() const { return m_IndexNormalVec; }
	b3Vector3 GetPointByBaryCoord(const btSoftbodyCL* pCloth, float a, float b, float c) const;
	b3Vector3 GetVelocityByBaryCoord(const btSoftbodyCL* pCloth, float a, float b, float c) const;
	
	btSoftbodyTriangleCL& operator=(const btSoftbodyTriangleCL& other);
};

#endif // B3_SOFTBODY_TRIANGLE_CL_H