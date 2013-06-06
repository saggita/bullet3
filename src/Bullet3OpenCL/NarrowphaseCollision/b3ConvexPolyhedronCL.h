#ifndef CONVEX_POLYHEDRON_CL
#define CONVEX_POLYHEDRON_CL

#include "Bullet3Common/b3AlignedObjectArray.h"
#include "Bullet3Common/b3Transform.h"

struct b3GpuFace
{
	b3Vector4 m_plane;
	int m_indexOffset;
	int m_numIndices;
	int m_unusedPadding1;
	int m_unusedPadding2;
};

B3_ATTRIBUTE_ALIGNED16(struct) b3ConvexPolyhedronCL
{
	b3Vector3		m_localCenter;
	b3Vector3		m_extents;
	b3Vector3		mC;
	b3Vector3		mE;

	b3Scalar		m_radius;
	int	m_faceOffset;
	int m_numFaces;
	int	m_numVertices;

	int m_vertexOffset;
	int	m_uniqueEdgesOffset;
	int	m_numUniqueEdges;
	int m_unused;	
	


	inline void project(const b3Transform& trans, const b3Vector3& dir, const b3AlignedObjectArray<b3Vector3>& vertices, b3Scalar& min, b3Scalar& max) const
	{
		min = FLT_MAX;
		max = -FLT_MAX;
		int numVerts = m_numVertices;

		const b3Vector3 localDir = trans.getBasis().transpose()*dir;
		const b3Vector3 localDi2 = b3QuatRotate(trans.getRotation().inverse(),dir);
		
		b3Scalar offset = trans.getOrigin().dot(dir);

		for(int i=0;i<numVerts;i++)
		{
			//b3Vector3 pt = trans * vertices[m_vertexOffset+i];
			//b3Scalar dp = pt.dot(dir);
			b3Scalar dp = vertices[m_vertexOffset+i].dot(localDir);
			//b3Assert(dp==dpL);
			if(dp < min)	min = dp;
			if(dp > max)	max = dp;
		}
		if(min>max)
		{
			b3Scalar tmp = min;
			min = max;
			max = tmp;
		}
		min += offset;
		max += offset;
	}

};

#endif //CONVEX_POLYHEDRON_CL
