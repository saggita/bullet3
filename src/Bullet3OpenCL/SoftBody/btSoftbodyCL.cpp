#include <omp.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

#include "btSoftbodyCL.h"
#include "StringTokenizer.h"
#include "btSoftbodyTriangleCL.h"
#include "../BulletSoftBody/btSoftBody.h"

inline float clamp(float val, float low, float high)
{
	if ( val < low )
		return low;
	else if ( val > high )
		return high;

	return val;
}

using namespace std;

btSoftbodyCL::btSoftbodyCL() : m_pSoftBodyCPU(NULL), m_bDeformable(true), m_Gravity(0.0f, -9.8f, 0.0f), m_dt(0.0f), m_bShowBV(false), m_Margin(0.01)
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

btSoftbodyCL::btSoftbodyCL(btSoftBody* softbody) : m_pSoftBodyCPU(softbody), m_bDeformable(true), m_Gravity(0.0f, -9.8f, 0.0f), m_dt(0.0f), m_bShowBV(false), m_Margin(0.01)
{
	assert(softbody);

	m_Kst = 0.95f;
	m_Kb = 0.4f;
	m_Kd = 0.0f;
	m_Mu = 0.3f;
	
	m_bEqualVertexMass = true;
	m_NumIterForConstraintSolver = 7;

	 m_numBatchStretchSpring = 0;
	 m_numBatchBendingSpring = 0;
		
	 // nodes
	 int index = 0;
	 for ( int i = 0; i < softbody->m_nodes.size(); i++ )
	 {
		btSoftbodyNodeCL nodeCl;
		nodeCl.m_Index = index++;
		nodeCl.m_Pos = softbody->m_nodes[i].m_x;
		nodeCl.m_InvMass = softbody->m_nodes[i].m_im;
		softbody->m_nodes[i].m_tag = (void*)nodeCl.m_Index;
		m_VertexArray.push_back(nodeCl);
	 }
	
	 // faces
	 for ( int i = 0; i < softbody->m_faces.size(); i++ )
	 {
		 btSoftbodyTriangleCL faceCl;

		 faceCl.SetVertexIndex(0, (int)softbody->m_faces[i].m_n[0]->m_tag);
		 faceCl.SetVertexIndex(1, (int)softbody->m_faces[i].m_n[1]->m_tag);
		 faceCl.SetVertexIndex(2, (int)softbody->m_faces[i].m_n[2]->m_tag);
		 faceCl.m_Index = (int)m_TriangleArray.size();
		 m_TriangleArray.push_back(faceCl);
	 }
	
	FillSpringArray();
}

btSoftbodyCL::btSoftbodyCL(const btSoftbodyCL& other)
{
	m_Kst = other.m_Kst;
	m_Kb = other.m_Kb;
	m_Kd = other.m_Kd;
	m_Gravity = other.m_Gravity;

	m_VertexArray = other.m_VertexArray;
	m_StrechSpringArray = other.m_StrechSpringArray;
	m_BendSpringArray = other.m_BendSpringArray;
	m_TriangleArray = other.m_TriangleArray;

	m_bDeformable = other.m_bDeformable;
	m_bEqualVertexMass = other.m_bEqualVertexMass;
	m_bShowBV = other.m_bShowBV;
}

btSoftbodyCL::~btSoftbodyCL(void)
{
	Clear();
}

void btSoftbodyCL::Clear()
{
	m_VertexArray.clear();
	m_StrechSpringArray.clear();
	m_BendSpringArray.clear();
}

void btSoftbodyCL::Initialize()
{
	m_bDeformable = true;

	InitializeBoundingVolumes();
}

bool btSoftbodyCL::Load(const char* filename)
{
	// Loading wavefront obj file.
	ifstream inFile(filename);
	string sLine;
	vector<string> sTokens;

	if ( !inFile.is_open() )
		return false;

	m_VertexArray.clear();
	m_StrechSpringArray.clear();
	m_BendSpringArray.clear();

	while (!inFile.eof() )
	{
		getline(inFile, sLine);
		sTokens.clear(); 
		int numFound = StringTokenizer(sLine, string(" "), sTokens, false);

		if ( numFound == 0 )
			continue;

		vector <string>::iterator iter;
		string sToken; 

		iter = sTokens.begin();
		sToken = *(iter);
		
		if ( sToken == "#" ) // comment
			continue;
		else if ( sToken == "v" ) // vertex
		{
			btVector3 pnt;
			
			// x
			++iter;
			sToken = (*iter);			
			pnt[0] = (float)atof(sToken.c_str());

			// y
			++iter;
			sToken = (*iter);			
			pnt[1] = (float)atof(sToken.c_str());

			// z
			++iter;
			sToken = (*iter);			
			pnt[2] = (float)atof(sToken.c_str());

			btSoftbodyNodeCL vert;
			vert.m_Pos = pnt;

			m_VertexArray.push_back(vert);
		}
		else if ( sToken == "vn" ) // vertex normal
		{
			// skip this.
		}
		else if ( sToken == "f" ) // face
		{
			btSoftbodyTriangleCL tri;
			vector<string> sTokens2;

			int i = 0;

			for ( iter = sTokens.begin() + 1; iter != sTokens.end(); iter++ )
			{
				sToken = (*iter);
				sTokens2.clear();
				numFound = StringTokenizer(sToken, string("/"), sTokens2, false);

				if ( numFound > 0 )
				{
					tri.m_IndexVrx[i++] = atoi(sTokens2[0].c_str())-1;
				}
				else if ( numFound == 0 && sToken != "" )
				{
					tri.m_IndexVrx[i++] = atoi(sToken.c_str())-1;
				}
			}		

			tri.m_Index = (int)m_TriangleArray.size();
			m_TriangleArray.push_back(tri);	
		}		
	}

	inFile.close();
	
	// Set up indexes for vertices
	int index = 0;
	for ( int i = 0; i < m_VertexArray.size(); i++ )
	{
		btSoftbodyNodeCL& vert = m_VertexArray[i];
		vert.m_Index = index;
		index++;
	}
	
	FillSpringArray();
		
	return true;
}

void btSoftbodyCL::FillSpringArray()
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
				assert(0); // must not reach here!

			tri.m_IndexEdge[i] = m_StrechSpringArray[iterEdge].GetIndex();
		}		
	}

	// Set rest length for stretch springs.
	for ( int iterEdge = 0; iterEdge < m_StrechSpringArray.size(); iterEdge++ )
	{
		btSoftbodyLinkCL& edge = m_StrechSpringArray[iterEdge];
		const btVector3& ver0 = m_VertexArray[edge.GetVertexIndex(0)].m_Pos;
		const btVector3& ver1 = m_VertexArray[edge.GetVertexIndex(1)].m_Pos;

		edge.SetRestLength((ver0 - ver1).length());
	}

	//----------------
	// Bending springs
	//----------------	
	for ( int iterEdge = 0; iterEdge < m_StrechSpringArray.size(); iterEdge++ )
	{
		const btSoftbodyLinkCL& edge = m_StrechSpringArray[iterEdge];
		const btVector3& ver0 = m_VertexArray[edge.GetVertexIndex(0)].m_Pos;
		const btVector3& ver1 = m_VertexArray[edge.GetVertexIndex(1)].m_Pos;

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

			assert(indexVer0 != -1 && indexVer1 != -1);

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
		const btVector3& ver0 = m_VertexArray[edge.GetVertexIndex(0)].m_Pos;
		const btVector3& ver1 = m_VertexArray[edge.GetVertexIndex(1)].m_Pos;

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

void btSoftbodyCL::SetGravity(const btVector3& gravity)
{
	m_Gravity = gravity;
}

const btVector3& btSoftbodyCL::GetGravity() const
{
	return m_Gravity;
}

void btSoftbodyCL::SetVertexMass(float vertexMass)
{
	m_bEqualVertexMass = true;

	assert(vertexMass > 0 );

	float invMass =  1.0f / vertexMass;

	for ( int i = 0; i < m_VertexArray.size(); i++ )
	{
		btSoftbodyNodeCL& vert = m_VertexArray[i];
		vert.m_InvMass = invMass;
	}
}

void btSoftbodyCL::SetTotalMass(float totalMass)
{
	assert(totalMass > 0);

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

void btSoftbodyCL::GenerateBatches()
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

		assert(!bFound0 && !bFound1);
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
			assert(spring.m_Coloring <= springNext.m_Coloring);
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
			assert(m_StrechSpringArray[j].m_Coloring == m_StrechSpringArray[startIndex].m_Coloring);
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

		assert(!bFound0 && !bFound1);
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
			assert(spring.m_Coloring <= springNext.m_Coloring);
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
			assert(m_BendSpringArray[j].m_Coloring == m_BendSpringArray[startIndex].m_Coloring);
		}
	}
#endif
}

void btSoftbodyCL::Render(bool bBBox/* = false*/)
{
	////-----------
	//// triangles
	////-----------
	//glEnable(GL_LIGHTING);
	//glEnable(GL_LIGHT0);

	////GLfloat white[] = {1.0f, 1.0f, 1.0f, 1.0f};
	//GLfloat color[]  = { 0.5f, 0.5f, 0.5f, 1.0f};

	//glMaterialfv(GL_FRONT, GL_DIFFUSE, color);
	//
	//glEnable(GL_POLYGON_OFFSET_FILL);
	//glPolygonOffset(1.0, 1.0);

	//glBegin(GL_TRIANGLES);

	//glEnd();

	//color[0] = 0.3f;
	//color[1] = 0.3f;
	//color[2] = 0.3f;
	//glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color);

	//glBegin(GL_TRIANGLES);
	//
	//for ( int iterTri = 0; iterTri < m_TriangleArray.size(); iterTri++ )
	//{
	//	const btSoftbodyTriangleCL& tri = m_TriangleArray[iterTri];

	//	const btSoftbodyNodeCL& v0 = m_VertexArray[tri.GetVertexIndex(0)];
	//	const btSoftbodyNodeCL& v1 = m_VertexArray[tri.GetVertexIndex(1)];
	//	const btSoftbodyNodeCL& v2 = m_VertexArray[tri.GetVertexIndex(2)];

	//	btVector3 n = btCross((v0.m_Pos - v1.m_Pos), (v2.m_Pos-v1.m_Pos)).normalize();

	//	glNormal3d(n[0], n[1], n[2]);
	//	glVertex3d(v0.m_Pos[0], v0.m_Pos[1], v0.m_Pos[2]);
	//	glVertex3d(v1.m_Pos[0], v1.m_Pos[1], v1.m_Pos[2]);
	//	glVertex3d(v2.m_Pos[0], v2.m_Pos[1], v2.m_Pos[2]);
	//}	

	//glEnd();

	//glDisable(GL_POLYGON_OFFSET_FILL);

	////----------
	//// vertices
	////----------
	//glDisable(GL_LIGHTING);
	//glColor3f(0, 0, 0);
	//
	////----------------
	//// Strech springs
	////----------------
	//glColor3f(0.0f, 0.0f, 0.0f);
	//glLineWidth(1.0);
	//
	//for ( int iterEdge = 0; iterEdge < m_StrechSpringArray.size(); iterEdge++ )
	//{
	//	const btSoftbodyLinkCL& edge = m_StrechSpringArray[iterEdge];
	//	const btVector3& ver0 = m_VertexArray[edge.GetVertexIndex(0)].m_Pos;
	//	const btVector3& ver1 = m_VertexArray[edge.GetVertexIndex(1)].m_Pos;

	//	btVector3 pos0 = (ver0);
	//	btVector3 pos1 = (ver1);

	//	glBegin(GL_LINES);
	//	glVertex3d(pos0[0], pos0[1], pos0[2]);
	//	glVertex3d(pos1[0], pos1[1], pos1[2]);
	//	glEnd();
	//}

	////--------------
	//// Bend springs
	////--------------
	//if ( 0 )
	//{
	//	glColor3f(0.0f, 0.6f, 0.0f);
	//	glLineWidth(1.0);
	//	
	//	for ( int iterEdge = 0; iterEdge < m_BendSpringArray.size(); iterEdge++ )
	//	{
	//		const btSoftbodyLinkCL& edge = m_BendSpringArray[iterEdge];
	//		const btVector3& ver0 = m_VertexArray[edge.GetVertexIndex(0)].m_Pos;
	//		const btVector3& ver1 = m_VertexArray[edge.GetVertexIndex(1)].m_Pos;

	//		btVector3 pos0 = (ver0);
	//		btVector3 pos1 = (ver1);

	//		glBegin(GL_LINES);
	//		glVertex3d(pos0[0], pos0[1], pos0[2]);
	//		glVertex3d(pos1[0], pos1[1], pos1[2]);
	//		glEnd();
	//	}
	//}

	////------------------
	//// Bounding volumes
	////------------------
	//if ( bBBox )
	//{
	//	m_Aabb.Visualize(true);

	//	for ( int i = 0; i < (int)m_AABBVertexArray.size(); i++ )
	//	{
	//		m_AABBVertexArray[i].Visualize(true);
	//	}
	//}
	//
	//glEnable(GL_LIGHTING);
}

int btSoftbodyCL::RenderBatch(int i) const
{
	/*glDisable(GL_LIGHTING);
	glLineWidth(3.0f);
	glColor3f(1.0f, 1.0f, 1.0f);

	if ( i >= (int)m_BatchStretchSpringIndexArray.size() - 1 )
		i = i - ((int)m_BatchStretchSpringIndexArray.size() - 1);

	if ( 0 <= i && i < (int)m_BatchStretchSpringIndexArray.size() - 1  )
	{
		int startIndex = m_BatchStretchSpringIndexArray[i];
		int endIndex = m_BatchStretchSpringIndexArray[i+1] - 1;

		for ( int j = startIndex; j <= endIndex; j++ )
		{
			const btVector3& v0 = m_VertexArray[m_StrechSpringArray[j].GetVertexIndex(0)].m_Pos;
			const btVector3& v1 = m_VertexArray[m_StrechSpringArray[j].GetVertexIndex(1)].m_Pos;

			glBegin(GL_LINES);
			glVertex3f(v0[0], v0[1], v0[2]);
			glVertex3f(v1[0], v1[1], v1[2]);
			glEnd();
		}
	}

	glColor3f(1.0f, 1.0f, 0.0f);

	if ( i >= (int)m_BatchBendSpringIndexArray.size() - 1 )
		i = i - ((int)m_BatchBendSpringIndexArray.size() - 1);

	if ( 0 <= i && i < (int)m_BatchBendSpringIndexArray.size() - 1  )
	{
		int startIndex = m_BatchBendSpringIndexArray[i];
		int endIndex = m_BatchBendSpringIndexArray[i+1] - 1;

		for ( int j = startIndex; j <= endIndex; j++ )
		{
			const btVector3& v0 = m_VertexArray[m_BendSpringArray[j].GetVertexIndex(0)].m_Pos;
			const btVector3& v1 = m_VertexArray[m_BendSpringArray[j].GetVertexIndex(1)].m_Pos;

			glBegin(GL_LINES);
			glVertex3f(v0[0], v0[1], v0[2]);
			glVertex3f(v1[0], v1[1], v1[2]);
			glEnd();
		}
	}
	
	glEnable(GL_LIGHTING);

	return i;*/

	return 0;
}

void btSoftbodyCL::ApplyForces(float dt)
{
	for ( int i = 0; i < (int)m_VertexArray.size(); i++ )
	{
		btSoftbodyNodeCL& vert = m_VertexArray[i];		
		vert.m_Vel += vert.m_Accel * dt;
	}
}

void btSoftbodyCL::ApplyGravity(float dt)
{
	for ( int i = 0; i < (int)m_VertexArray.size(); i++ )
	{
		btSoftbodyNodeCL& vert = m_VertexArray[i];	
		vert.m_Accel += m_Gravity;
	}
}

void btSoftbodyCL::ClearForces()
{
	for ( int i = 0; i < (int)m_VertexArray.size(); i++ )
	{
		btSoftbodyNodeCL& vert = m_VertexArray[i];		
		vert.m_Accel = btVector3(0, 0, 0);
	}
}

void btSoftbodyCL::ComputeNextVertexPositions(float dt)
{
	for ( int i = 0; i < (int)m_VertexArray.size(); i++ )
	{
		btSoftbodyNodeCL& vert = m_VertexArray[i];		
		vert.m_PosNext = vert.m_Pos;
	}
}

// k is a adjusted stiffness with a considertion of number of iterations
void btSoftbodyCL::EnforceEdgeConstraints(float k, float dt) 
{
	m_dt = dt;
	
	for ( int i = 0; i < (int)m_StrechSpringArray.size(); i++ )
	{
		int indexEdge = i;

		bool bNeedLimiting = false;

		const btSoftbodyLinkCL& spring = m_StrechSpringArray[indexEdge];

		btSoftbodyNodeCL& vert0 = m_VertexArray[spring.GetVertexIndex(0)];
		btSoftbodyNodeCL& vert1 = m_VertexArray[spring.GetVertexIndex(1)];

		btVector3 vecNewSpring = vert0.m_PosNext - vert1.m_PosNext;

		float newLen = vecNewSpring.length();
		float restLen = spring.GetRestLength();

		btVector3 cji = (newLen-restLen)*vecNewSpring.normalize() / (vert0.m_InvMass + vert1.m_InvMass);

		btVector3 dVert0(0, 0, 0);
		btVector3 dVert1(0, 0, 0);			

		dVert0 = -cji * vert0.m_InvMass;
		dVert1 = cji * vert1.m_InvMass;

		vert0.m_PosNext +=  k * dVert0;
		vert1.m_PosNext += k * dVert1;			  
	}	
}

// k is a adjusted stiffness with a considertion of number of iterations
void btSoftbodyCL::EnforceBendingConstraints(float k, float dt)
{
	m_dt = dt;
	
	for ( int i = 0; i < (int)m_BendSpringArray.size(); i++ )
	{
		int indexEdge = i;

		bool bNeedLimiting = false;

		const btSoftbodyLinkCL& spring = m_BendSpringArray[indexEdge];

		btSoftbodyNodeCL& vert0 = m_VertexArray[spring.GetVertexIndex(0)];
		btSoftbodyNodeCL& vert1 = m_VertexArray[spring.GetVertexIndex(1)];

		btVector3 vecNewSpring = vert0.m_PosNext - vert1.m_PosNext;

		float newLen = vecNewSpring.length();
		float restLen = spring.GetRestLength();

		btVector3 cji = (newLen-restLen)*vecNewSpring.normalize() / (vert0.m_InvMass + vert1.m_InvMass);

		btVector3 dVert0(0, 0, 0);
		btVector3 dVert1(0, 0, 0);			

		dVert0 = -cji * vert0.m_InvMass;
		dVert1 = cji * vert1.m_InvMass;

		vert0.m_PosNext += k * dVert0;
		vert1.m_PosNext += k * dVert1;			  
	}	
}

void btSoftbodyCL::EnforceEdgeConstraintsBatched(float k, float dt)
{
	m_dt = dt;
	
	#pragma omp parallel for
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
			
			btVector3 vecNewSpring = vert0.m_PosNext - vert1.m_PosNext;

			float newLen = vecNewSpring.length();
			float restLen = spring.GetRestLength();

			btVector3 cji = (newLen-restLen)*vecNewSpring.normalize() / (vert0.m_InvMass + vert1.m_InvMass);

			btVector3 dVert0(0, 0, 0);
			btVector3 dVert1(0, 0, 0);			

			dVert0 = -cji * vert0.m_InvMass;
			dVert1 = cji * vert1.m_InvMass;

			vert0.m_PosNext += k * dVert0;
			vert1.m_PosNext += k * dVert1;			  
		}
	}	
}

// k is a adjusted stiffness with a considertion of number of iterations
void btSoftbodyCL::EnforceBendingConstraintsBatched(float k, float dt)
{
	m_dt = dt;
	
	#pragma omp parallel for
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

			btVector3 vecNewSpring = vert0.m_PosNext - vert1.m_PosNext;

			float newLen = vecNewSpring.length();
			float restLen = spring.GetRestLength();

			btVector3 cji = (newLen-restLen)*vecNewSpring.normalize() / (vert0.m_InvMass + vert1.m_InvMass);

			btVector3 dVert0(0, 0, 0);
			btVector3 dVert1(0, 0, 0);			
		
			dVert0 = -cji * vert0.m_InvMass;
			dVert1 = cji * vert1.m_InvMass;

			vert0.m_PosNext += k * dVert0;
			vert1.m_PosNext += k * dVert1;			  
		}
	}	
}

bool btSoftbodyCL::AdvancePosition(float dt)
{
	m_dt = dt;

	#pragma omp parallel for
	for ( int i = 0; i < (int)m_VertexArray.size(); i++ )
	{
		btSoftbodyNodeCL& vert = m_VertexArray[i];	
		vert.m_Pos = vert.m_Pos + vert.m_Vel * dt;
	}

	return true;
}

bool btSoftbodyCL::Integrate(float dt)
{
	m_dt = dt;

	ClearForces();
	ApplyGravity(dt);

	// TODO: ApplyExternalForces() should be here..

	ApplyForces(dt);
	ComputeNextVertexPositions(dt);

	assert(m_NumIterForConstraintSolver > 0 );
	assert(0 <= m_Kst && m_Kst <= 1.0);
	assert(0 <= m_Kb && m_Kb <= 1.0);

	float Kst = 1.0f - pow((1.0f - m_Kst), 1.0f/m_NumIterForConstraintSolver);
	float Kb = 1.0f - pow((1.0f - m_Kb), 1.0f/m_NumIterForConstraintSolver);

	clamp(Kst, 0, 1.0);
	clamp(Kb, 0, 1.0);

	int numIteration = 0;

	while ( numIteration < m_NumIterForConstraintSolver )
	{
		/*EnforceEdgeConstraints(Kst, dt);		
		EnforceBendingConstraints(Kb, dt);*/
		
		EnforceEdgeConstraintsBatched(Kst, dt);
		EnforceBendingConstraintsBatched(Kb, dt);

		++numIteration;
	}

	UpdateVelocities(dt);

	m_NumIter = numIteration;
	return true;
}

void btSoftbodyCL::UpdateVelocities(float dt)
{
	for ( int i = 0; i < m_VertexArray.size(); i++ )
	{
		btSoftbodyNodeCL& vert = m_VertexArray[i];
		vert.m_Vel = (vert.m_PosNext - vert.m_Pos)/dt;
	}
}

void btSoftbodyCL::InitializeBoundingVolumes()
{
	m_Aabb.Empty();

	bool bEmpty = m_Aabb.IsEmpty();

	m_AABBVertexArray.clear();

	for ( int i = 0; i < m_VertexArray.size(); i++ )
	{
		btSoftbodyNodeCL& vert = m_VertexArray[i];
	
		CAabb aabb;
		aabb += vert.m_Pos;
		aabb.Enlarge(GetMargin());

		m_AABBVertexArray.push_back(aabb);
		m_Aabb += aabb;
	}
}

void btSoftbodyCL::UpdateBoundingVolumes(float dt)
{
	assert(m_AABBVertexArray.size() == m_VertexArray.size());

	m_Aabb.Empty();

	for ( int i = 0; i < m_AABBVertexArray.size(); i++ )
	{
		btSoftbodyNodeCL& vert = m_VertexArray[i];
	
		m_AABBVertexArray[i].Empty();
		m_AABBVertexArray[i] += vert.m_Pos;
		m_AABBVertexArray[i] += vert.m_Pos + vert.m_Vel * dt;
		m_AABBVertexArray[i].Enlarge(this->GetMargin());
		
		m_Aabb += m_AABBVertexArray[i];
	}
}

void btSoftbodyCL::TranslateW(float x, float y, float z)
{
	for ( int i = 0; i < m_VertexArray.size(); i++ )
	{
		btSoftbodyNodeCL& vert = m_VertexArray[i];
				
		vert.m_Pos += btVector3(x, y, z);
	}
}

void btSoftbodyCL::UpdateSoftBodyCPU()
{
	if ( !m_pSoftBodyCPU )
		return;

	for ( int i = 0; i < m_pSoftBodyCPU->m_nodes.size(); i++ )
	 {
		m_pSoftBodyCPU->m_nodes[i].m_x = m_VertexArray[i].m_Pos;
	 }
}

btSoftbodyCL& btSoftbodyCL::operator=(const btSoftbodyCL& other) 
{ 
	m_Kst = other.m_Kst;
	m_Kb = other.m_Kb;
	m_Kd = other.m_Kd;

	// ToDo: Need to do something with m_pBVHTree

	m_VertexArray = other.m_VertexArray;
	m_StrechSpringArray = other.m_StrechSpringArray;
	m_BendSpringArray = other.m_BendSpringArray;
	m_TriangleArray = other.m_TriangleArray;

	m_bDeformable = other.m_bDeformable;
	m_bEqualVertexMass = other.m_bEqualVertexMass;
	m_bShowBV = other.m_bShowBV;

	return *this; 
}




