/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B3_RIGID_BODY_CL
#define B3_RIGID_BODY_CL

#include "Bullet3Common/b3Scalar.h"
#include "Bullet3Common/b3Matrix3x3.h"

B3_ATTRIBUTE_ALIGNED16(struct) b3RigidBodyCL
{
	B3_DECLARE_ALIGNED_ALLOCATOR();

	b3Vector3 		m_pos;
	b3Quaternion 	m_quat;
	b3Vector3			m_linVel;
	b3Vector3			m_angVel;

	int 					m_collidableIdx;
	float 				m_invMass;
	float 				m_restituitionCoeff;
	float 				m_frictionCoeff;

	float	getInvMass() const
	{
			return m_invMass;
	}
};


struct b3InertiaCL
{
	b3Matrix3x3 m_invInertiaWorld;
	b3Matrix3x3 m_initInvInertia;
};


#endif//B3_RIGID_BODY_CL
