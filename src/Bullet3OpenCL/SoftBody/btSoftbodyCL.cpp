#include "btSoftbodyCL.h"
#include "btSoftbodyTriangleCL.h"
//#include "../BulletSoftBody/btSoftBody.h"

inline float clamp(float val, float low, float high)
{
	if ( val < low )
		return low;
	else if ( val > high )
		return high;

	return val;
}

using namespace std;

btSoftbodyCL::btSoftbodyCL() : m_bDeformable(true), m_Gravity(0.0f, -9.8f, 0.0f), m_dt(0.0f), m_Margin(0.01)
{
	m_Kst = 0.95f;
	m_Kb = 0.4f;
	m_Kd = 0.0f;
	m_Mu = 0.3f;
	
	m_bEqualVertexMass = true;
	m_NumIterForConstraintSolver = 7;

	 m_numBatchStretchSpring = 0;
	 m_numBatchBendingSpring = 0;
}

btSoftbodyCL::~btSoftbodyCL(void)
{
	clear();
}

void btSoftbodyCL::clear()
{
	m_VertexArray.clear();
	m_StrechSpringArray.clear();
	m_BendSpringArray.clear();
}

void btSoftbodyCL::initialize()
{
	m_bDeformable = true;

	fillSpringArray();
	initializeBoundingVolumes();
}

void btSoftbodyCL::fillSpringArray()
{
	//---------------
	// Stretch springs
	//---------------
	m_StrechSpringArray.clear();

	for ( int iterTri = 0; iterTri < m_TriangleArray.size(); iterTri++ )
	{
		const btSoftbodyTriangleCL& tri = m_TriangleArray[iterTri];

		for ( int i = 0; i < 3; i++ )
		{
			int j = ((i != 2) ? i+1 : 0);

			btSoftbodyLinkCL edge(tri.GetVertexIndex(i), tri.GetVertexIndex(j));	
			int iterEdge = m_StrechSpringArray.findLinearSearch(edge);

			if ( iterEdge == m_StrechSpringArray.size() )
			{
				edge.m_IndexTriangle[0] = tri.GetIndex();
				edge.m_IndexVrx[0] = tri.GetVertexIndex(i);
				edge.m_IndexVrx[1] = tri.GetVertexIndex(j);

				edge.m_Index = m_StrechSpringArray.size();
				m_StrechSpringArray.push_back(edge);
			}
			else
				m_StrechSpringArray[iterEdge].m_IndexTriangle[1] = tri.GetIndex();
		}		
	}

	for ( int iterTri = 0; iterTri < m_TriangleArray.size(); iterTri++ )
	{
		btSoftbodyTriangleCL& tri = m_TriangleArray[iterTri];

		for ( int i = 0; i < 3; i++ )
		{
			int j = ((i != 2) ? i+1 : 0);

			btSoftbodyLinkCL edge(tri.GetVertexIndex(i), tri.GetVertexIndex(j));	
			int iterEdge = m_StrechSpringArray.findLinearSearch(edge);

			if ( iterEdge == m_StrechSpringArray.size() )
				b3Assert(0); // must not reach here!

			tri.m_IndexEdge[i] = m_StrechSpringArray[iterEdge].GetIndex();
		}		
	}

	// Set rest length for stretch springs.
	for ( int iterEdge = 0; iterEdge < m_StrechSpringArray.size(); iterEdge++ )
	{
		btSoftbodyLinkCL& edge = m_StrechSpringArray[iterEdge];
		const b3Vector3& ver0 = m_VertexArray[edge.GetVertexIndex(0)].m_Pos;
		const b3Vector3& ver1 = m_VertexArray[edge.GetVertexIndex(1)].m_Pos;

		edge.SetRestLength((ver0 - ver1).length());
	}

	//----------------
	// Bending springs
	//----------------	
	for ( int iterEdge = 0; iterEdge < m_StrechSpringArray.size(); iterEdge++ )
	{
		const btSoftbodyLinkCL& edge = m_StrechSpringArray[iterEdge];
		const b3Vector3& ver0 = m_VertexArray[edge.GetVertexIndex(0)].m_Pos;
		const b3Vector3& ver1 = m_VertexArray[edge.GetVertexIndex(1)].m_Pos;

		int indexTri0 = edge.m_IndexTriangle[0];
		int indexTri1 = edge.m_IndexTriangle[1];

		if ( indexTri0 != -1 && indexTri1 != -1 )
		{
			const btSoftbodyTriangleCL& tri0 = m_TriangleArray[indexTri0];
			const btSoftbodyTriangleCL& tri1 = m_TriangleArray[indexTri1];

			int indexVer0 = -1;
			int indexVer1 = -1;

			// find two vertices whose connecting line crosses the current edge.
			if ( tri0.GetVertexIndex(0) != edge.GetVertexIndex(0) && tri0.GetVertexIndex(0) != edge.GetVertexIndex(1) )
				indexVer0 = tri0.GetVertexIndex(0);
			else if ( tri0.GetVertexIndex(1) != edge.GetVertexIndex(0) && tri0.GetVertexIndex(1) != edge.GetVertexIndex(1) )
				indexVer0 = tri0.GetVertexIndex(1);
			else if ( tri0.GetVertexIndex(2) != edge.GetVertexIndex(0) && tri0.GetVertexIndex(2) != edge.GetVertexIndex(1) )
				indexVer0 = tri0.GetVertexIndex(2);

			if ( tri1.GetVertexIndex(0) != edge.GetVertexIndex(0) && tri1.GetVertexIndex(0) != edge.GetVertexIndex(1) )
				indexVer1 = tri1.GetVertexIndex(0);
			else if ( tri1.GetVertexIndex(1) != edge.GetVertexIndex(0) && tri1.GetVertexIndex(1) != edge.GetVertexIndex(1) )
				indexVer1 = tri1.GetVertexIndex(1);
			else if ( tri1.GetVertexIndex(2) != edge.GetVertexIndex(0) && tri1.GetVertexIndex(2) != edge.GetVertexIndex(1) )
				indexVer1 = tri1.GetVertexIndex(2);

			b3Assert(indexVer0 != -1 && indexVer1 != -1);

			btSoftbodyLinkCL bendSpring(indexVer0, indexVer1);
			int iterCheck = m_BendSpringArray.findLinearSearch(bendSpring);

			// there is already the same edge in the array.
			if ( iterCheck != m_BendSpringArray.size() )
				continue;

			bendSpring.m_Index = m_BendSpringArray.size();
			m_BendSpringArray.push_back(bendSpring);
		}		
	}		
	
	// Set rest length for bending springs.
	for ( int iterEdge = 0; iterEdge < m_BendSpringArray.size(); iterEdge++ )
	{
		btSoftbodyLinkCL& edge = m_BendSpringArray[iterEdge];
		const b3Vector3& ver0 = m_VertexArray[edge.GetVertexIndex(0)].m_Pos;
		const b3Vector3& ver1 = m_VertexArray[edge.GetVertexIndex(1)].m_Pos;

		edge.SetRestLength((ver0 - ver1).length());
	}

	// Clear m_StrechSpringIndexes and m_BendSpringIndexes in each vertex
	for ( int i = 0; i < m_VertexArray.size(); i++ )
	{
		btSoftbodyNodeCL& vert = m_VertexArray[i];
		vert.m_StrechSpringIndexes.clear();
		vert.m_BendSpringIndexes.clear();
	}

	// Set m_StrechSpringIndexes in each vertex
	for ( int iterEdge = 0; iterEdge < m_StrechSpringArray.size(); iterEdge++ )
	{
		btSoftbodyLinkCL& edge = m_StrechSpringArray[iterEdge];
		btSoftbodyNodeCL& ver0 = m_VertexArray[edge.GetVertexIndex(0)];
		btSoftbodyNodeCL& ver1 = m_VertexArray[edge.GetVertexIndex(1)];

		ver0.m_StrechSpringIndexes.push_back(edge.GetIndex());
		ver1.m_StrechSpringIndexes.push_back(edge.GetIndex());
	}

	// Set m_BendSpringIndexes in each vertex
	for ( int iterEdge = 0; iterEdge < m_BendSpringArray.size(); iterEdge++ )
	{
		btSoftbodyLinkCL& edge = m_BendSpringArray[iterEdge];
		btSoftbodyNodeCL& ver0 = m_VertexArray[edge.GetVertexIndex(0)];
		btSoftbodyNodeCL& ver1 = m_VertexArray[edge.GetVertexIndex(1)];

		ver0.m_BendSpringIndexes.push_back(edge.GetIndex());
		ver1.m_BendSpringIndexes.push_back(edge.GetIndex());
	}
}

void btSoftbodyCL::setGravity(const b3Vector3& gravity)
{
	m_Gravity = gravity;
}

const b3Vector3& btSoftbodyCL::GetGravity() const
{
	return m_Gravity;
}

void btSoftbodyCL::setVertexMass(float vertexMass)
{
	m_bEqualVertexMass = true;

	b3Assert(vertexMass > 0 );

	float invMass =  1.0f / vertexMass;

	for ( int i = 0; i < m_VertexArray.size(); i++ )
	{
		btSoftbodyNodeCL& vert = m_VertexArray[i];
		vert.m_InvMass = invMass;
	}
}

void btSoftbodyCL::setTotalMass(float totalMass)
{
	b3Assert(totalMass > 0);

	m_bEqualVertexMass = true;

	float invMass =  m_VertexArray.size() / totalMass;

	for ( int i = 0; i < m_VertexArray.size(); i++ )
	{
		btSoftbodyNodeCL& vert = m_VertexArray[i];
		vert.m_InvMass = invMass;
	}
}

class CColoringCompare
{
public:
	bool operator() (const btSoftbodyLinkCL& a, const btSoftbodyLinkCL& b) const
	{
		return a.m_Coloring < b.m_Coloring;
	}

};

void btSoftbodyCL::generateBatches()
{
	//---------------------
	// Clean up m_Coloring
	//---------------------
	for ( int i = 0; i < (int)m_StrechSpringArray.size(); i++ )
	{
		btSoftbodyLinkCL& spring = m_StrechSpringArray[i];
		spring.m_Coloring = -1;
	}

	for ( int i = 0; i < (int)m_BendSpringArray.size(); i++ )
	{
		btSoftbodyLinkCL& spring = m_BendSpringArray[i];
		spring.m_Coloring = -1;
	}

	//---------
	// Stretch
	//---------
	m_BatchStretchSpringIndexArray.clear();

	for ( int i = 0; i < (int)m_StrechSpringArray.size(); i++ )
	{
		btSoftbodyLinkCL& spring = m_StrechSpringArray[i];

		const btSoftbodyNodeCL& vert0 = m_VertexArray[spring.GetVertexIndex(0)];
		const btSoftbodyNodeCL& vert1 = m_VertexArray[spring.GetVertexIndex(1)];

		int coloring = 0;		

		while ( true ) 
		{
			bool bFound0 = false;
			bool bFound1 = false;

			for ( int a = 0; a < (int)vert0.m_StrechSpringIndexes.size(); a++ )
			{
				const btSoftbodyLinkCL& otherSpring = m_StrechSpringArray[vert0.m_StrechSpringIndexes[a]];

				// skip if the neighbor spring is actually itself
				if ( otherSpring.GetIndex() == spring.GetIndex() )
					continue;

				if ( otherSpring.m_Coloring == coloring )
				{
					bFound0 = true;
					break;
				}				
			}
			
			for ( int a = 0; a < (int)vert1.m_StrechSpringIndexes.size(); a++ )
			{
				const btSoftbodyLinkCL& otherSpring = m_StrechSpringArray[vert1.m_StrechSpringIndexes[a]];

				// skip if the neighbor spring is actually itself
				if ( otherSpring.GetIndex() == spring.GetIndex() )
					continue;

				if ( otherSpring.m_Coloring == coloring )
				{
					bFound1 = true;
					break;
				}				
			}

			if ( bFound0 || bFound1 )
				coloring++;
			else
				break;
		} 

		spring.m_Coloring = coloring;
	}

#ifdef _DEBUG

	for ( int i = 0; i < (int)m_StrechSpringArray.size(); i++ )
	{
		btSoftbodyLinkCL& spring = m_StrechSpringArray[i];

		const btSoftbodyNodeCL& vert0 = m_VertexArray[spring.GetVertexIndex(0)];
		const btSoftbodyNodeCL& vert1 = m_VertexArray[spring.GetVertexIndex(1)];

		int coloring = spring.m_Coloring;
		bool bFound0 = false;
		bool bFound1 = false;

		for ( int a = 0; a < (int)vert0.m_StrechSpringIndexes.size(); a++ )
		{
			const btSoftbodyLinkCL& otherSpring = m_StrechSpringArray[vert0.m_StrechSpringIndexes[a]];

			// skip if the neighbor spring is actually itself
			if ( otherSpring.GetIndex() == spring.GetIndex() )
				continue;

			if ( otherSpring.m_Coloring == coloring )
			{
				bFound0 = true;
				break;
			}				
		}
		
		for ( int a = 0; a < (int)vert1.m_StrechSpringIndexes.size(); a++ )
		{
			const btSoftbodyLinkCL& otherSpring = m_StrechSpringArray[vert1.m_StrechSpringIndexes[a]];

			// skip if the neighbor spring is actually itself
			if ( otherSpring.GetIndex() == spring.GetIndex() )
				continue;

			if ( otherSpring.m_Coloring == coloring )
			{
				bFound1 = true;
				break;
			}				
		}

		b3Assert(!bFound0 && !bFound1);
	}
#endif

	// Count how many batches were generated
	m_numBatchStretchSpring = 0;

	for ( int i = 0; i < (int)m_StrechSpringArray.size(); i++ )
	{
		btSoftbodyLinkCL& spring = m_StrechSpringArray[i];

		if ( spring.m_Coloring > m_numBatchStretchSpring )
			m_numBatchStretchSpring = spring.m_Coloring;
	}

	m_numBatchStretchSpring++;

	m_StrechSpringArray.quickSort(CColoringCompare());

	m_BatchStretchSpringIndexArray.push_back(0);

	if ( m_StrechSpringArray.size() > 1 )
	{
		int i = 0;

		for ( i = 0; i < (int)m_StrechSpringArray.size()-1; i++ )
		{
			btSoftbodyLinkCL& spring = m_StrechSpringArray[i];
			btSoftbodyLinkCL& springNext = m_StrechSpringArray[i+1];

#ifdef _DEBUG
			b3Assert(spring.m_Coloring <= springNext.m_Coloring);
#endif

			if ( spring.m_Coloring < springNext.m_Coloring )
				m_BatchStretchSpringIndexArray.push_back(i+1);
		}

		m_BatchStretchSpringIndexArray.push_back(i);
	}

#ifdef _DEBUG
	for ( int i = 0; i < (int)m_BatchStretchSpringIndexArray.size()-1; i++ )
	{
		int startIndex = m_BatchStretchSpringIndexArray[i];
		int endIndex = m_BatchStretchSpringIndexArray[i+1] - 1;

		for ( int j = startIndex; j <= endIndex; j++ )
		{
			b3Assert(m_StrechSpringArray[j].m_Coloring == m_StrechSpringArray[startIndex].m_Coloring);
		}
	}
#endif

	//---------
	// Bending
	//---------
	m_BatchBendSpringIndexArray.clear();

	for ( int i = 0; i < (int)m_BendSpringArray.size(); i++ )
	{
		btSoftbodyLinkCL& spring = m_BendSpringArray[i];

		const btSoftbodyNodeCL& vert0 = m_VertexArray[spring.GetVertexIndex(0)];
		const btSoftbodyNodeCL& vert1 = m_VertexArray[spring.GetVertexIndex(1)];

		int coloring = 0;		

		while ( true ) 
		{
			bool bFound0 = false;
			bool bFound1 = false;

			for ( int a = 0; a < (int)vert0.m_BendSpringIndexes.size(); a++ )
			{
				const btSoftbodyLinkCL& otherSpring = m_BendSpringArray[vert0.m_BendSpringIndexes[a]];

				// skip if the neighbor spring is actually itself
				if ( otherSpring.GetIndex() == spring.GetIndex() )
					continue;

				if ( otherSpring.m_Coloring == coloring )
				{
					bFound0 = true;
					break;
				}				
			}
			
			for ( int a = 0; a < (int)vert1.m_BendSpringIndexes.size(); a++ )
			{
				const btSoftbodyLinkCL& otherSpring = m_BendSpringArray[vert1.m_BendSpringIndexes[a]];

				// skip if the neighbor spring is actually itself
				if ( otherSpring.GetIndex() == spring.GetIndex() )
					continue;

				if ( otherSpring.m_Coloring == coloring )
				{
					bFound1 = true;
					break;
				}				
			}

			if ( bFound0 || bFound1 )
				coloring++;
			else
				break;
		} 

		spring.m_Coloring = coloring;
	}

#ifdef _DEBUG

	for ( int i = 0; i < (int)m_BendSpringArray.size(); i++ )
	{
		btSoftbodyLinkCL& spring = m_BendSpringArray[i];

		const btSoftbodyNodeCL& vert0 = m_VertexArray[spring.GetVertexIndex(0)];
		const btSoftbodyNodeCL& vert1 = m_VertexArray[spring.GetVertexIndex(1)];

		int coloring = spring.m_Coloring;
		bool bFound0 = false;
		bool bFound1 = false;

		for ( int a = 0; a < (int)vert0.m_BendSpringIndexes.size(); a++ )
		{
			const btSoftbodyLinkCL& otherSpring = m_BendSpringArray[vert0.m_BendSpringIndexes[a]];

			// skip if the neighbor spring is actually itself
			if ( otherSpring.GetIndex() == spring.GetIndex() )
				continue;

			if ( otherSpring.m_Coloring == coloring )
			{
				bFound0 = true;
				break;
			}				
		}
		
		for ( int a = 0; a < (int)vert1.m_BendSpringIndexes.size(); a++ )
		{
			const btSoftbodyLinkCL& otherSpring = m_BendSpringArray[vert1.m_BendSpringIndexes[a]];

			// skip if the neighbor spring is actually itself
			if ( otherSpring.GetIndex() == spring.GetIndex() )
				continue;

			if ( otherSpring.m_Coloring == coloring )
			{
				bFound1 = true;
				break;
			}				
		}

		b3Assert(!bFound0 && !bFound1);
	}
#endif

	// Count how many batches were generated
	m_numBatchBendingSpring = 0;

	for ( int i = 0; i < (int)m_BendSpringArray.size(); i++ )
	{
		btSoftbodyLinkCL& spring = m_BendSpringArray[i];

		if ( spring.m_Coloring > m_numBatchBendingSpring )
			m_numBatchBendingSpring = spring.m_Coloring;
	}

	m_numBatchBendingSpring++;

	//std::sort(m_BendSpringArray.begin(), m_BendSpringArray.end(), ColoringCompare);
	m_BendSpringArray.quickSort(CColoringCompare());

	m_BatchBendSpringIndexArray.push_back(0);

	if ( m_BendSpringArray.size() > 1 )
	{
		int i = 0;

		for ( i = 0; i < (int)m_BendSpringArray.size()-1; i++ )
		{
			btSoftbodyLinkCL& spring = m_BendSpringArray[i];
			btSoftbodyLinkCL& springNext = m_BendSpringArray[i+1];

#ifdef _DEBUG
			b3Assert(spring.m_Coloring <= springNext.m_Coloring);
#endif

			if ( spring.m_Coloring < springNext.m_Coloring )
				m_BatchBendSpringIndexArray.push_back(i+1);
		}

		m_BatchBendSpringIndexArray.push_back(i);
	}

#ifdef _DEBUG
	for ( int i = 0; i < (int)m_BatchBendSpringIndexArray.size()-1; i++ )
	{
		int startIndex = m_BatchBendSpringIndexArray[i];
		int endIndex = m_BatchBendSpringIndexArray[i+1] - 1;

		for ( int j = startIndex; j <= endIndex; j++ )
		{
			b3Assert(m_BendSpringArray[j].m_Coloring == m_BendSpringArray[startIndex].m_Coloring);
		}
	}
#endif
}

void btSoftbodyCL::applyForces(float dt)
{
	for ( int i = 0; i < (int)m_VertexArray.size(); i++ )
	{
		btSoftbodyNodeCL& vert = m_VertexArray[i];		
		vert.m_Vel += vert.m_Accel * dt;
	}
}

void btSoftbodyCL::applyGravity(float dt)
{
	for ( int i = 0; i < (int)m_VertexArray.size(); i++ )
	{
		btSoftbodyNodeCL& vert = m_VertexArray[i];	
		vert.m_Accel += m_Gravity;
	}
}

void btSoftbodyCL::clearForces()
{
	for ( int i = 0; i < (int)m_VertexArray.size(); i++ )
	{
		btSoftbodyNodeCL& vert = m_VertexArray[i];		
		vert.m_Accel = b3Vector3(0, 0, 0);
	}
}

void btSoftbodyCL::computeNextVertexPositions(float dt)
{
	for ( int i = 0; i < (int)m_VertexArray.size(); i++ )
	{
		btSoftbodyNodeCL& vert = m_VertexArray[i];		
		vert.m_PosNext = vert.m_Pos;
	}
}

// k is a adjusted stiffness with a considertion of number of iterations
void btSoftbodyCL::enforceEdgeConstraints(float k, float dt) 
{
	m_dt = dt;
	
	for ( int i = 0; i < (int)m_StrechSpringArray.size(); i++ )
	{
		int indexEdge = i;

		bool bNeedLimiting = false;

		const btSoftbodyLinkCL& spring = m_StrechSpringArray[indexEdge];

		btSoftbodyNodeCL& vert0 = m_VertexArray[spring.GetVertexIndex(0)];
		btSoftbodyNodeCL& vert1 = m_VertexArray[spring.GetVertexIndex(1)];

		b3Vector3 vecNewSpring = vert0.m_PosNext - vert1.m_PosNext;

		float newLen = vecNewSpring.length();
		float restLen = spring.GetRestLength();

		b3Vector3 cji = (newLen-restLen)*vecNewSpring.normalize() / (vert0.m_InvMass + vert1.m_InvMass);

		b3Vector3 dVert0(0, 0, 0);
		b3Vector3 dVert1(0, 0, 0);			

		dVert0 = -cji * vert0.m_InvMass;
		dVert1 = cji * vert1.m_InvMass;

		vert0.m_PosNext +=  k * dVert0;
		vert1.m_PosNext += k * dVert1;			  
	}	
}

// k is a adjusted stiffness with a considertion of number of iterations
void btSoftbodyCL::enforceBendingConstraints(float k, float dt)
{
	m_dt = dt;
	
	for ( int i = 0; i < (int)m_BendSpringArray.size(); i++ )
	{
		int indexEdge = i;

		bool bNeedLimiting = false;

		const btSoftbodyLinkCL& spring = m_BendSpringArray[indexEdge];

		btSoftbodyNodeCL& vert0 = m_VertexArray[spring.GetVertexIndex(0)];
		btSoftbodyNodeCL& vert1 = m_VertexArray[spring.GetVertexIndex(1)];

		b3Vector3 vecNewSpring = vert0.m_PosNext - vert1.m_PosNext;

		float newLen = vecNewSpring.length();
		float restLen = spring.GetRestLength();

		b3Vector3 cji = (newLen-restLen)*vecNewSpring.normalize() / (vert0.m_InvMass + vert1.m_InvMass);

		b3Vector3 dVert0(0, 0, 0);
		b3Vector3 dVert1(0, 0, 0);			

		dVert0 = -cji * vert0.m_InvMass;
		dVert1 = cji * vert1.m_InvMass;

		vert0.m_PosNext += k * dVert0;
		vert1.m_PosNext += k * dVert1;			  
	}	
}

void btSoftbodyCL::enforceEdgeConstraintsBatched(float k, float dt)
{
	m_dt = dt;
	
	for ( int batch = 0; batch < (int)m_BatchStretchSpringIndexArray.size()-1; batch++ )
	{
		int startIndex = m_BatchStretchSpringIndexArray[batch];
		int endIndex = m_BatchStretchSpringIndexArray[batch+1] - 1;

		for ( int j = startIndex; j <= endIndex; j++ )
		{
			int indexEdge = j;

			bool bNeedLimiting = false;

			const btSoftbodyLinkCL& spring = m_StrechSpringArray[indexEdge];

			btSoftbodyNodeCL& vert0 = m_VertexArray[spring.GetVertexIndex(0)];
			btSoftbodyNodeCL& vert1 = m_VertexArray[spring.GetVertexIndex(1)];
			
			b3Vector3 vecNewSpring = vert0.m_PosNext - vert1.m_PosNext;

			float newLen = vecNewSpring.length();
			float restLen = spring.GetRestLength();

			b3Vector3 cji = (newLen-restLen)*vecNewSpring.normalize() / (vert0.m_InvMass + vert1.m_InvMass);

			b3Vector3 dVert0(0, 0, 0);
			b3Vector3 dVert1(0, 0, 0);			

			dVert0 = -cji * vert0.m_InvMass;
			dVert1 = cji * vert1.m_InvMass;

			vert0.m_PosNext += k * dVert0;
			vert1.m_PosNext += k * dVert1;			  
		}
	}	
}

// k is a adjusted stiffness with a considertion of number of iterations
void btSoftbodyCL::enforceBendingConstraintsBatched(float k, float dt)
{
	m_dt = dt;
	
	for ( int batch = 0; batch < (int)m_BatchBendSpringIndexArray.size()-1; batch++ )
	{
		int startIndex = m_BatchBendSpringIndexArray[batch];
		int endIndex = m_BatchBendSpringIndexArray[batch+1] - 1;

		for ( int j = startIndex; j <= endIndex; j++ )
		{
			int indexEdge = j;

			bool bNeedLimiting = false;

			const btSoftbodyLinkCL& spring = m_BendSpringArray[indexEdge];

			btSoftbodyNodeCL& vert0 = m_VertexArray[spring.GetVertexIndex(0)];
			btSoftbodyNodeCL& vert1 = m_VertexArray[spring.GetVertexIndex(1)];

			b3Vector3 vecNewSpring = vert0.m_PosNext - vert1.m_PosNext;

			float newLen = vecNewSpring.length();
			float restLen = spring.GetRestLength();

			b3Vector3 cji = (newLen-restLen)*vecNewSpring.normalize() / (vert0.m_InvMass + vert1.m_InvMass);

			b3Vector3 dVert0(0, 0, 0);
			b3Vector3 dVert1(0, 0, 0);			
		
			dVert0 = -cji * vert0.m_InvMass;
			dVert1 = cji * vert1.m_InvMass;

			vert0.m_PosNext += k * dVert0;
			vert1.m_PosNext += k * dVert1;			  
		}
	}	
}

bool btSoftbodyCL::advancePosition(float dt)
{
	m_dt = dt;

	for ( int i = 0; i < (int)m_VertexArray.size(); i++ )
	{
		btSoftbodyNodeCL& vert = m_VertexArray[i];	
		vert.m_Pos = vert.m_Pos + vert.m_Vel * dt;
	}

	return true;
}

bool btSoftbodyCL::integrate(float dt)
{
	m_dt = dt;

	clearForces();
	applyGravity(dt);

	// TODO: ApplyExternalForces() should be here..

	applyForces(dt);
	computeNextVertexPositions(dt);

	b3Assert(m_NumIterForConstraintSolver > 0 );
	b3Assert(0 <= m_Kst && m_Kst <= 1.0);
	b3Assert(0 <= m_Kb && m_Kb <= 1.0);

	float Kst = 1.0f - pow((1.0f - m_Kst), 1.0f/m_NumIterForConstraintSolver);
	float Kb = 1.0f - pow((1.0f - m_Kb), 1.0f/m_NumIterForConstraintSolver);

	clamp(Kst, 0, 1.0);
	clamp(Kb, 0, 1.0);

	int numIteration = 0;

	while ( numIteration < m_NumIterForConstraintSolver )
	{
		/*EnforceEdgeConstraints(Kst, dt);		
		EnforceBendingConstraints(Kb, dt);*/
		
		enforceEdgeConstraintsBatched(Kst, dt);
		enforceBendingConstraintsBatched(Kb, dt);

		++numIteration;
	}

	updateVelocities(dt);

	m_NumIter = numIteration;
	return true;
}

void btSoftbodyCL::updateVelocities(float dt)
{
	for ( int i = 0; i < m_VertexArray.size(); i++ )
	{
		btSoftbodyNodeCL& vert = m_VertexArray[i];
		vert.m_Vel = (vert.m_PosNext - vert.m_Pos)/dt;
	}
}

void btSoftbodyCL::initializeBoundingVolumes()
{
	m_Aabb.Empty();

	bool bEmpty = m_Aabb.IsEmpty();

	m_AABBVertexArray.clear();

	for ( int i = 0; i < m_VertexArray.size(); i++ )
	{
		btSoftbodyNodeCL& vert = m_VertexArray[i];
	
		CAabb aabb;
		aabb += vert.m_Pos;
		aabb.Enlarge(getMargin());

		m_AABBVertexArray.push_back(aabb);
		m_Aabb += aabb;
	}
}

void btSoftbodyCL::updateBoundingVolumes(float dt)
{
	b3Assert(m_AABBVertexArray.size() == m_VertexArray.size());

	m_Aabb.Empty();

	for ( int i = 0; i < m_AABBVertexArray.size(); i++ )
	{
		btSoftbodyNodeCL& vert = m_VertexArray[i];
	
		m_AABBVertexArray[i].Empty();
		m_AABBVertexArray[i] += vert.m_Pos;
		m_AABBVertexArray[i] += vert.m_Pos + vert.m_Vel * dt;
		m_AABBVertexArray[i].Enlarge(getMargin());
		
		m_Aabb += m_AABBVertexArray[i];
	}
}





