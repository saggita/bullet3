//this file is autogenerated using stringify.bat (premake --stringify) in the build folder of this project
static const char* updateAabbsKernelCL= \
"\n"
"#define SHAPE_CONVEX_HULL 3\n"
"\n"
"typedef float4 Quaternion;\n"
"\n"
"__inline\n"
"float4 cross3(float4 a, float4 b)\n"
"{\n"
"	return cross(a,b);\n"
"}\n"
"\n"
"__inline\n"
"float dot3F4(float4 a, float4 b)\n"
"{\n"
"	float4 a1 = (float4)(a.xyz,0.f);\n"
"	float4 b1 = (float4)(b.xyz,0.f);\n"
"	return dot(a1, b1);\n"
"}\n"
"\n"
"\n"
"__inline\n"
"Quaternion qtMul(Quaternion a, Quaternion b)\n"
"{\n"
"	Quaternion ans;\n"
"	ans = cross3( a, b );\n"
"	ans += a.w*b+b.w*a;\n"
"	ans.w = a.w*b.w - dot3F4(a, b);\n"
"	return ans;\n"
"}\n"
"\n"
"__inline\n"
"Quaternion qtInvert(Quaternion q)\n"
"{\n"
"	return (Quaternion)(-q.xyz, q.w);\n"
"}\n"
"\n"
"__inline\n"
"float4 qtRotate(Quaternion q, float4 vec)\n"
"{\n"
"	Quaternion qInv = qtInvert( q );\n"
"	float4 vcpy = vec;\n"
"	vcpy.w = 0.f;\n"
"	float4 out = qtMul(qtMul(q,vcpy),qInv);\n"
"	return out;\n"
"}\n"
"\n"
"__inline\n"
"float4 transform(const float4* p, const float4* translation, const Quaternion* orientation)\n"
"{\n"
"	return qtRotate( *orientation, *p ) + (*translation);\n"
"}\n"
"\n"
"typedef struct\n"
"{\n"
"	float4	m_row[3];\n"
"} Matrix3x3;\n"
"\n"
"typedef unsigned int u32;\n"
"\n"
"\n"
"typedef struct\n"
"{\n"
"	float4 m_pos;\n"
"	float4 m_quat;\n"
"	float4 m_linVel;\n"
"	float4 m_angVel;\n"
"\n"
"	u32 m_collidableIdx;\n"
"	float m_invMass;\n"
"	float m_restituitionCoeff;\n"
"	float m_frictionCoeff;\n"
"} Body;\n"
"\n"
"typedef struct Collidable\n"
"{\n"
"	int m_unused1;\n"
"	int m_unused2;\n"
"	int m_shapeType;\n"
"	int m_shapeIndex;\n"
"} Collidable;\n"
"\n"
"\n"
"typedef struct\n"
"{\n"
"	Matrix3x3 m_invInertia;\n"
"	Matrix3x3 m_initInvInertia;\n"
"} Shape;\n"
"\n"
"\n"
"__inline\n"
"Matrix3x3 qtGetRotationMatrix(float4 quat)\n"
"{\n"
"	float4 quat2 = (float4)(quat.x*quat.x, quat.y*quat.y, quat.z*quat.z, 0.f);\n"
"	Matrix3x3 out;\n"
"\n"
"	out.m_row[0].x=fabs(1-2*quat2.y-2*quat2.z);\n"
"	out.m_row[0].y=fabs(2*quat.x*quat.y-2*quat.w*quat.z);\n"
"	out.m_row[0].z=fabs(2*quat.x*quat.z+2*quat.w*quat.y);\n"
"	out.m_row[0].w = 0.f;\n"
"\n"
"	out.m_row[1].x=fabs(2*quat.x*quat.y+2*quat.w*quat.z);\n"
"	out.m_row[1].y=fabs(1-2*quat2.x-2*quat2.z);\n"
"	out.m_row[1].z=fabs(2*quat.y*quat.z-2*quat.w*quat.x);\n"
"	out.m_row[1].w = 0.f;\n"
"\n"
"	out.m_row[2].x=fabs(2*quat.x*quat.z-2*quat.w*quat.y);\n"
"	out.m_row[2].y=fabs(2*quat.y*quat.z+2*quat.w*quat.x);\n"
"	out.m_row[2].z=fabs(1-2*quat2.x-2*quat2.y);\n"
"	out.m_row[2].w = 0.f;\n"
"\n"
"	return out;\n"
"}\n"
"\n"
"\n"
"typedef struct \n"
"{\n"
"	float			fx;\n"
"	float			fy;\n"
"	float			fz;\n"
"	int	uw;\n"
"} btAABBCL;\n"
"\n"
"__inline\n"
"Matrix3x3 mtTranspose(Matrix3x3 m)\n"
"{\n"
"	Matrix3x3 out;\n"
"	out.m_row[0] = (float4)(m.m_row[0].x, m.m_row[1].x, m.m_row[2].x, 0.f);\n"
"	out.m_row[1] = (float4)(m.m_row[0].y, m.m_row[1].y, m.m_row[2].y, 0.f);\n"
"	out.m_row[2] = (float4)(m.m_row[0].z, m.m_row[1].z, m.m_row[2].z, 0.f);\n"
"	return out;\n"
"}\n"
"\n"
"\n"
"\n"
"__inline\n"
"Matrix3x3 mtMul(Matrix3x3 a, Matrix3x3 b)\n"
"{\n"
"	Matrix3x3 transB;\n"
"	transB = mtTranspose( b );\n"
"	Matrix3x3 ans;\n"
"	//	why this doesn't run when 0ing in the for{}\n"
"	a.m_row[0].w = 0.f;\n"
"	a.m_row[1].w = 0.f;\n"
"	a.m_row[2].w = 0.f;\n"
"	for(int i=0; i<3; i++)\n"
"	{\n"
"//	a.m_row[i].w = 0.f;\n"
"		ans.m_row[i].x = dot3F4(a.m_row[i],transB.m_row[0]);\n"
"		ans.m_row[i].y = dot3F4(a.m_row[i],transB.m_row[1]);\n"
"		ans.m_row[i].z = dot3F4(a.m_row[i],transB.m_row[2]);\n"
"		ans.m_row[i].w = 0.f;\n"
"	}\n"
"	return ans;\n"
"}\n"
"\n"
"\n"
"__kernel void initializeGpuAabbsFull(  const int numNodes, __global Body* gBodies,__global Collidable* collidables, __global btAABBCL* plocalShapeAABB, __global btAABBCL* pAABB)\n"
"{\n"
"	int nodeID = get_global_id(0);\n"
"		\n"
"	if( nodeID < numNodes )\n"
"	{\n"
"		float4 position = gBodies[nodeID].m_pos;\n"
"		float4 orientation = gBodies[nodeID].m_quat;\n"
"		\n"
"			\n"
"		int collidableIndex = gBodies[nodeID].m_collidableIdx;\n"
"		int shapeIndex = collidables[collidableIndex].m_shapeIndex;\n"
"			\n"
"		if (shapeIndex>=0)\n"
"		{\n"
"			btAABBCL minAabb = plocalShapeAABB[collidableIndex*2];\n"
"			btAABBCL maxAabb = plocalShapeAABB[collidableIndex*2+1];\n"
"				\n"
"			float4 halfExtents = ((float4)(maxAabb.fx - minAabb.fx,maxAabb.fy - minAabb.fy,maxAabb.fz - minAabb.fz,0.f))*0.5f;\n"
"			float4 localCenter = ((float4)(maxAabb.fx + minAabb.fx,maxAabb.fy + minAabb.fy,maxAabb.fz + minAabb.fz,0.f))*0.5f;\n"
"				\n"
"			float4 worldCenter = transform(&localCenter,&position,&orientation);\n"
"				\n"
"			Matrix3x3 abs_b = qtGetRotationMatrix(orientation);\n"
"			float4 extent = (float4) (	dot(abs_b.m_row[0],halfExtents),dot(abs_b.m_row[1],halfExtents),dot(abs_b.m_row[2],halfExtents),0.f);\n"
"			\n"
"	\n"
"			pAABB[nodeID*2].fx = worldCenter.x-extent.x;\n"
"			pAABB[nodeID*2].fy = worldCenter.y-extent.y;\n"
"			pAABB[nodeID*2].fz = worldCenter.z-extent.z;\n"
"			pAABB[nodeID*2].uw = nodeID;\n"
"	\n"
"			pAABB[nodeID*2+1].fx = worldCenter.x+extent.x;\n"
"			pAABB[nodeID*2+1].fy = worldCenter.y+extent.y;\n"
"			pAABB[nodeID*2+1].fz = worldCenter.z+extent.z;\n"
"			pAABB[nodeID*2+1].uw = gBodies[nodeID].m_invMass==0.f? 0 : 1;\n"
"		}\n"
"	} \n"
"}\n"
"\n"
;
