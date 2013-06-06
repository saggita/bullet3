#include "btSoftbodyTriangleCL.h"
#include "btSoftbodyCL.h"

btSoftbodyTriangleCL::btSoftbodyTriangleCL()
{
	for ( int i = 0; i < 3; i++ )
	{
		m_IndexVrx[i] = -1;
		m_IndexEdge[i] = -1;			
	}
		
	m_Index = -1;
	m_IndexNormalVec = -1;
}

btSoftbodyTriangleCL::btSoftbodyTriangleCL(const btSoftbodyTriangleCL& other)
{
	for ( int i = 0; i < 3; i++ )
	{
		m_IndexVrx[i] = other.m_IndexVrx[i];
		m_IndexEdge[i] = other.m_IndexEdge[i];
	}

	m_Index = other.m_Index;
	m_IndexNormalVec = other.m_IndexNormalVec;
}

btSoftbodyTriangleCL::~btSoftbodyTriangleCL() 
{
}

btVector3 btSoftbodyTriangleCL::GetPointByBaryCoord(const btSoftbodyCL* pCloth, float a, float b, float c) const
{
	const btSoftbodyNodeCL& v0 = pCloth->GetVertexArray()[m_IndexVrx[0]];
	const btSoftbodyNodeCL& v1 = pCloth->GetVertexArray()[m_IndexVrx[1]];
	const btSoftbodyNodeCL& v2 = pCloth->GetVertexArray()[m_IndexVrx[2]];

	return btVector3(v0.m_Pos*a + v1.m_Pos*b + v2.m_Pos*c);
}

btVector3 btSoftbodyTriangleCL::GetVelocityByBaryCoord(const btSoftbodyCL* pCloth, float a, float b, float c) const
{
	const btSoftbodyNodeCL& v0 = pCloth->GetVertexArray()[m_IndexVrx[0]];
	const btSoftbodyNodeCL& v1 = pCloth->GetVertexArray()[m_IndexVrx[1]];
	const btSoftbodyNodeCL& v2 = pCloth->GetVertexArray()[m_IndexVrx[2]];

	return btVector3(v0.m_Vel*a + v1.m_Vel*b + v2.m_Vel*c);
}


btSoftbodyTriangleCL& btSoftbodyTriangleCL::operator=(const btSoftbodyTriangleCL& other)
{
	for ( int i = 0; i < 3; i++ )
	{
		m_IndexVrx[i] = other.m_IndexVrx[i];
		m_IndexEdge[i] = other.m_IndexEdge[i];
	}

	m_Index = other.m_Index;
	m_IndexNormalVec = other.m_IndexNormalVec;
	return (*this);
}

