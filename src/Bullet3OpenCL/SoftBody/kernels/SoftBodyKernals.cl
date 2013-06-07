

// From MathCl.h
typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;


typedef struct 
{
	float4 m_Pos;
	float4 m_PosNext;
	float4 m_Vel;	
	float4 m_Accel;

	float4 m_AABBMin;
	float4 m_AABBMax;

	float m_InvMass;
	u32 m_Index; // local index. Only unique inside the cloth which it belongs to.
	u32 m_IndexGlobal; // global index
	u32 m_PinIndex; // if no pin is attached, -1. 
	u32 m_ClothIndex;
} VertexClothData;

typedef struct 
{
	u32 m_Index; // local index. Only unique inside the cloth which it belongs to.
	u32 m_IndexGlobal; // global index
	u32 m_IndexGlobalVrx0; // global vertex index
	u32 m_IndexGlobalVrx1; // global vertex index

	float4 m_RestLength;

	u32 m_ClothIndex;
} SpringClothData;

typedef struct 
{
	u32 m_Index;
	float4 m_Pos;
} SoftBodyLink;


typedef struct 
{
	u32 m_Index;
	u32 m_NumVertices;
	u32 m_NumStretchSprings;
	u32 m_NumBendingSprings;
	
	float4 m_AABBMin;
	float4 m_AABBMax;

	u32 m_OffsetVertices;
	u32 m_OffsetStretchSprings;
	u32 m_OffsetBendingSprings;

	float m_Margin;
	float m_Ks;
	float m_Kb;
} ClothInfoData;

///keep this in sync with btCollidable.h
typedef struct
{
	int m_shapeType;
	int m_shapeIndex;
	
} btCollidableGpu;

typedef struct
{
	float4 m_pos;
	float4 m_quat;
	float4 m_linVel;
	float4 m_angVel;

	u32 m_collidableIdx;	
	float m_invMass;
	float m_restituitionCoeff;
	float m_frictionCoeff;
} BodyData;


typedef struct  
{
	float4		m_localCenter;
	float4		m_extents;
	float4		mC;
	float4		mE;
	
	float			m_radius;
	int	m_faceOffset;
	int m_numFaces;
	int	m_numVertices;
	
	int m_vertexOffset;
	int	m_uniqueEdgesOffset;
	int	m_numUniqueEdges;
	int m_unused;

} ConvexPolyhedronCL;

typedef struct
{
	float4 m_plane;
	int m_indexOffset;
	int m_numIndices;
} btGpuFace;

typedef struct 
{
	union
	{
		float4	m_min;
		float   m_minElems[4];
		int			m_minIndices[4];
	};
	union
	{
		float4	m_max;
		float   m_maxElems[4];
		int			m_maxIndices[4];
	};
} btAabbCL;

__inline
float dot3F4(float4 a, float4 b)
{
	float4 a1 = (float4)(a.xyz,0.f);
	float4 b1 = (float4)(b.xyz,0.f);
	return dot(a1, b1);
}

__inline
float4 cross3(float4 a, float4 b)
{
	return cross(a,b);
}


__inline
float4 fastNormalize4(float4 v)
{
	return fast_normalize(v);
}


///////////////////////////////////////
//	Quaternion
///////////////////////////////////////

typedef float4 Quaternion;

__inline
Quaternion qtMul(Quaternion a, Quaternion b);

__inline
Quaternion qtNormalize(Quaternion in);

__inline
float4 qtRotate(Quaternion q, float4 vec);

__inline
Quaternion qtInvert(Quaternion q);




__inline
Quaternion qtMul(Quaternion a, Quaternion b)
{
	Quaternion ans;
	ans = cross3( a, b );
	ans += a.w*b+b.w*a;
//	ans.w = a.w*b.w - (a.x*b.x+a.y*b.y+a.z*b.z);
	ans.w = a.w*b.w - dot3F4(a, b);
	return ans;
}

__inline
Quaternion qtNormalize(Quaternion in)
{
	return fastNormalize4(in);
//	in /= length( in );
//	return in;
}
__inline
float4 qtRotate(Quaternion q, float4 vec)
{
	Quaternion qInv = qtInvert( q );
	float4 vcpy = vec;
	vcpy.w = 0.f;
	float4 out = qtMul(qtMul(q,vcpy),qInv);
	return out;
}

__inline
Quaternion qtInvert(Quaternion q)
{
	return (Quaternion)(-q.xyz, q.w);
}

__inline
float4 qtInvRotate(const Quaternion q, float4 vec)
{
	return qtRotate( qtInvert( q ), vec );
}

__inline
float4 transform(const float4* p, const float4* translation, const Quaternion* orientation)
{
	return qtRotate( *orientation, *p ) + (*translation);
}



__inline
float4 normalize3(const float4 a)
{
	float4 n = (float4)(a.x, a.y, a.z, 0.f);
	return fastNormalize4( n );
}

__kernel 
void ClearForcesKernel(const u32 numVertices, const float dt, __global VertexClothData* gVertexClothData)
{
	u32 vertexID = get_global_id(0);

	if( vertexID < numVertices )
	{		
		VertexClothData vertexData = gVertexClothData[vertexID];
		vertexData.m_Accel = (float4)(0.f, 0.f, 0.f, 0.f);
		gVertexClothData[vertexID] = vertexData;
	}
}

__kernel 
void ComputeNextVertexPositionsKernel(const u32 numVertices, const float dt, __global VertexClothData* gVertexClothData)
{
	u32 vertexID = get_global_id(0);

	if( vertexID < numVertices )
	{		
		VertexClothData vertexData = gVertexClothData[vertexID];	
		vertexData.m_PosNext = vertexData.m_Pos + vertexData.m_Vel * dt;
		gVertexClothData[vertexID] = vertexData;
	}
}

__kernel 
void ApplyGravityKernel(const u32 numVertices, const float4 gravity, const float dt, __global VertexClothData* gVertexClothData)
{
	u32 vertexID = get_global_id(0);

	if( vertexID < numVertices )
	{		
		VertexClothData vertexData = gVertexClothData[vertexID];
		vertexData.m_Accel += gravity;
		gVertexClothData[vertexID] = vertexData;
	}
}

__kernel 
void ApplyForcesKernel(const u32 numVertices, const float dt, __global VertexClothData* gVertexClothData)
{
	u32 vertexID = get_global_id(0);

	if( vertexID < numVertices )
	{		
		VertexClothData vertexData = gVertexClothData[vertexID];

		if ( vertexData.m_InvMass > 0 )
		{
			vertexData.m_Vel += vertexData.m_Accel * dt;
			gVertexClothData[vertexID] = vertexData;		
		}
	}
}

// springType - 0 for stretch and 1 for bending
__kernel 
void EnforceEdgeConstraintsKernel(const u32 numSpringsInBatch, const u32 startSpringIndex, const float dt, const int springType, __global ClothInfoData* gClothInfoData, __global VertexClothData* gVertexClothData, __global SpringClothData* gSpringClothData)
{
	u32 springID = get_global_id(0) + startSpringIndex;

	if( get_global_id(0) < numSpringsInBatch )
	{
		SpringClothData springData = gSpringClothData[springID];

		u32 indexVrx0 = springData.m_IndexGlobalVrx0;
		u32 indexVrx1 = springData.m_IndexGlobalVrx1;

		VertexClothData vertexData0 = gVertexClothData[indexVrx0];
		VertexClothData vertexData1 = gVertexClothData[indexVrx1];

		float Kstiff = 0;

		if ( springType == 0 ) // stretch
			Kstiff = gClothInfoData[springData.m_ClothIndex].m_Ks;
		else if ( springType == 1 ) // bending
			Kstiff = gClothInfoData[springData.m_ClothIndex].m_Kb;
		
		float4 vecNewSpring = vertexData0.m_PosNext - vertexData1.m_PosNext;
		vecNewSpring.w = 0;

		float newLen = length(vecNewSpring);
		float4 restLen = springData.m_RestLength;
		
		if ( (vertexData0.m_InvMass + vertexData1.m_InvMass) > 0 )
		{
			float4 cji = (newLen-restLen.x)*normalize(vecNewSpring) / (vertexData0.m_InvMass + vertexData1.m_InvMass);
			
			float4 dVert0;
			float4 dVert1;

			dVert0 = -Kstiff * cji * vertexData0.m_InvMass;
			dVert1 = Kstiff * cji * vertexData1.m_InvMass;
		
			vertexData0.m_PosNext += dVert0;
			vertexData1.m_PosNext += dVert1;

			gVertexClothData[indexVrx0] = vertexData0; 
			gVertexClothData[indexVrx1] = vertexData1;
		}
	}
}

__kernel 
void UpdateVelocitiesKernel(const u32 numVertices, const float dt, __global VertexClothData* gVertexClothData)
{
	u32 vertexID = get_global_id(0);

	if( vertexID < numVertices )
	{		
		VertexClothData vertexData = gVertexClothData[vertexID];

		if ( vertexData.m_InvMass > 0 )
		{
			vertexData.m_Vel = (vertexData.m_PosNext - vertexData.m_Pos)/dt;
			gVertexClothData[vertexID] = vertexData;		
		}
	}
}

__kernel 
void AdvancePositionKernel(const u32 numVertices, const float dt, __global VertexClothData* gVertexClothData)
{
	u32 vertexID = get_global_id(0);

	if( vertexID < numVertices )
	{		
		VertexClothData vertexData = gVertexClothData[vertexID];
		vertexData.m_Pos = vertexData.m_Pos + vertexData.m_Vel * dt;
		gVertexClothData[vertexID] = vertexData;
	}
}

__kernel 
void UpdateVertexBoundingVolumeKernel(const u32 numVertices, const float dt, __global VertexClothData* gVertexClothData, __global ClothInfoData* gClothInfoData)
{
	u32 vertexID = get_global_id(0);

	if( vertexID < numVertices )
	{	
		VertexClothData vertexData = gVertexClothData[vertexID];

		const float margin = gClothInfoData[vertexData.m_ClothIndex].m_Margin;
		float4 newPos = vertexData.m_Pos + vertexData.m_Vel * dt;
		
		vertexData.m_AABBMin = vertexData.m_Pos;
		vertexData.m_AABBMax = vertexData.m_Pos;

		vertexData.m_AABBMin.x = min(vertexData.m_AABBMin.x, newPos.x) - margin;
		vertexData.m_AABBMin.y = min(vertexData.m_AABBMin.y, newPos.y) - margin;
		vertexData.m_AABBMin.z = min(vertexData.m_AABBMin.z, newPos.z) - margin;

		vertexData.m_AABBMax.x = max(vertexData.m_AABBMax.x, newPos.x) + margin;
		vertexData.m_AABBMax.y = max(vertexData.m_AABBMax.y, newPos.y) + margin;
		vertexData.m_AABBMax.z = max(vertexData.m_AABBMax.z, newPos.z) + margin;
		
		gVertexClothData[vertexID] = vertexData;
	}
}

// TODO:Should be parallel in cloth node level.
__kernel 
void UpdateClothBoundingVolumeKernel(const u32 numClothes, __global ClothInfoData* gClothInfoData, __global VertexClothData* gVertexClothData)
{
	u32 clothID = get_global_id(0);

	if( clothID < numClothes )
	{	
		ClothInfoData clothInfo = gClothInfoData[clothID];
		u32 numVertices = clothInfo.m_NumVertices;

		clothInfo.m_AABBMin.x = 100000000.0f;
		clothInfo.m_AABBMin.y = 100000000.0f;
		clothInfo.m_AABBMin.z = 100000000.0f;

		clothInfo.m_AABBMax.x = -100000000.0f;
		clothInfo.m_AABBMax.y = -100000000.0f;
		clothInfo.m_AABBMax.z = -100000000.0f;

		for ( int vertexID = 0; vertexID < numVertices; vertexID++ )
		{
			VertexClothData vertexData = gVertexClothData[vertexID];
			
			/*if ( vertexID == 0 )
			{
				clothInfo.m_AABBMin = vertexData.m_AABBMin;
				clothInfo.m_AABBMax = vertexData.m_AABBMax;
			}
			else*/
			{
				clothInfo.m_AABBMin.x = min(clothInfo.m_AABBMin.x, vertexData.m_AABBMin.x);
				clothInfo.m_AABBMin.y = min(clothInfo.m_AABBMin.y, vertexData.m_AABBMin.y);
				clothInfo.m_AABBMin.z = min(clothInfo.m_AABBMin.z, vertexData.m_AABBMin.z);

				clothInfo.m_AABBMax.x = max(clothInfo.m_AABBMax.x, vertexData.m_AABBMax.x);
				clothInfo.m_AABBMax.y = max(clothInfo.m_AABBMax.y, vertexData.m_AABBMax.y);
				clothInfo.m_AABBMax.z = max(clothInfo.m_AABBMax.z, vertexData.m_AABBMax.z);
			}
		}

		gClothInfoData[clothID] = clothInfo;
	}
}

// Assumes planeEqn[0], planeEqn[1] and planeEqn[2] forms unit normal vector.
float signedDistanceFromPointToPlane(float4 point, float4 planeEqn, float4* closestPointOnFace)
{
	float4 n = (float4)(planeEqn.x, planeEqn.y, planeEqn.z, 0);

	if ( length(n) < 1e-6 )
		return 0;

	if ( point.x == 0 && point.y == 0 && point.z == 0 )
	{
		float dist = planeEqn.w;

		if ( closestPointOnFace )
			*closestPointOnFace = - dist * n;

		return dist;
	}
	else
	{
		float dist = dot3F4(n, point) + planeEqn.w;

		if ( closestPointOnFace )
			*closestPointOnFace = point - dist * n;

		return dist;
	}
}

__kernel 
void ResolveCollisionKernel(const u32 numSoftbodyVertices, 
							const float dt, 
							__global VertexClothData* gVertexClothData,
							__global const BodyData* rigidBodies, 
							const u32 numRigidBodies,
							__global const btCollidableGpu* collidables,
							__global const ConvexPolyhedronCL* convexShapes, 
							__global const btGpuFace* faces,
							__global const int* convexIndices, 
							__global const float4* convexVertices)
{
	u32 vertexID = get_global_id(0);

	if( vertexID < numSoftbodyVertices )
	{		
		float margin = 0.3f;

		VertexClothData vertexData = gVertexClothData[vertexID];
		
		for ( u32 i = 0; i < numRigidBodies; i++ )
		{
			float4 pos = rigidBodies[i].m_pos;
			float4 quat = rigidBodies[i].m_quat;

			int collidableIndex = rigidBodies[i].m_collidableIdx;
			int shapeIndex = collidables[collidableIndex].m_shapeIndex;
			int numFaces = convexShapes[shapeIndex].m_numFaces;
			float4 closestPnt = (float4)(0, 0, 0, 0);
			float minDist = -1000000.f; // TODO: What is the largest/smallest float?
			bool bCollide = true;

			for ( int f = 0; f < numFaces; f++ )
			{
				btGpuFace face = faces[convexShapes[shapeIndex].m_faceOffset+f];

				// set up a plane equation 
				float4 planeEqn;
				float4 n = qtRotate(quat, (float4)(face.m_plane.xyz, 0));
				planeEqn.xyz = n.xyz;

				float4 vertInFace = convexVertices[convexShapes[shapeIndex].m_vertexOffset + convexIndices[face.m_indexOffset + 0]];
				vertInFace = transform(&vertInFace, &pos, &quat);
				planeEqn.w = -dot3F4(n, vertInFace) - margin;

				// compute a signed distance from the vertex in cloth to the face of rigidbody.
				float4 pntReturn;
				float dist = signedDistanceFromPointToPlane(vertexData.m_PosNext, planeEqn, &pntReturn);

				// If the distance is positive, the plane is a separating plane. 
				if ( dist > 0 )
				{
					bCollide = false;
					break;
				}

				if ( dist > minDist )
				{
					minDist = dist;
					closestPnt = pntReturn;
				}
			}
				
			// If there is a collision
			if ( bCollide )
			{
				vertexData.m_Pos = closestPnt;
				vertexData.m_PosNext = vertexData.m_Pos;
				vertexData.m_Vel = (float4)(0, 0, 0, 0); // TODO: the velocity should be the one from the closest point.
			}
		}

		gVertexClothData[vertexID] = vertexData;		
	}
}
