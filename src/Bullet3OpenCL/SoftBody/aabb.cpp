#ifdef WIN32
#  define WIN32_LEAN_AND_MEAN
#  define NOMINMAX
#  include <windows.h>
#  undef WIN32_LEAN_AND_MEAN
#  undef NOMINMAX
#endif

#define NOMINMAX
#include "Aabb.h"
#include <algorithm>
#include <limits>

#include "LinearMath/btVector3.h"

#include <assert.h>

using namespace std;

CAabb::CAabb(void) 
{
	Empty();
}

CAabb::CAabb(const CAabb& other) : IBoundingVolume(other)
{
	m_Min = other.m_Min;
	m_Max = other.m_Max;
}

CAabb::~CAabb(void)
{
}

void CAabb::Set(const btVector3& min, const btVector3& max)
{
	m_Min = min;
	m_Max = max;
}

void CAabb::Enlarge(float h)
{
	m_Min[0] -= h;
	m_Min[1] -= h;
	m_Min[2] -= h;

	m_Max[0] += h;
	m_Max[1] += h;
	m_Max[2] += h;
}

bool CAabb::Collide(const IBoundingVolume& other, float tolerance /*= 0*/) const
{
	const CAabb& box = (CAabb&)other;

	if ( m_Min[0] > box.m_Max[0] + tolerance )
		return false;

	if ( m_Min[1] > box.m_Max[1] + tolerance )
		return false;

	if ( m_Min[2] > box.m_Max[2] + tolerance )
		return false;

	if ( m_Max[0] < box.m_Min[0] + tolerance )
		return false;

	if ( m_Max[1] < box.m_Min[1] + tolerance )
		return false;

	if ( m_Max[2] < box.m_Min[2] + tolerance )
		return false;

	return true;
}

bool CAabb::Inside(const btVector3& point) const
{
	if ( point[0] < m_Min[0] || m_Max[0] < point[0] )
		return false;

	if ( point[1] < m_Min[1] || m_Max[1] < point[1] )
		return false;

	if ( point[2] < m_Min[2] || m_Max[2] < point[2] )
		return false;

	return true;
}

void CAabb::Empty()
{
	float max = numeric_limits<float>::max();
	float min = numeric_limits<float>::min();

	m_Min = btVector3(max, max, max);
	m_Max = btVector3(min, min, min);
}

bool CAabb::IsEmpty() const
{
	float max = numeric_limits<float>::max();
	float min = numeric_limits<float>::min();

	if ( m_Min[0] == max && 
		 m_Min[1] == max && 
		 m_Min[2] == max && 
		 m_Max[0] == min && 
		 m_Max[1] == min && 
		 m_Max[2] == min )
		 return true;
	else
		return false;
}

float CAabb::Height() const
{
	return m_Max[1] - m_Min[1];
}

float CAabb::Width() const
{
	return m_Max[0] - m_Min[0];
}

float CAabb::Length() const
{
	return m_Max[2] - m_Min[2];
}

btVector3 CAabb::Center() const
{
	return (m_Min + m_Max) * 0.5;
}

float CAabb::Volume() const
{
	return Width() * Length() * Height();
}

int CAabb::LongestSide() const
{
	float w = Width();
	float h = Height();
	float l = Length();

	if ( w >= h && w >= l )
		return 0;
	else if ( h >= w && h >= l )
		return 1;
	else // if ( l >= w && l >= h )
		return 2;
}

void CAabb::Split(IBoundingVolume*& leftBV, IBoundingVolume*& rightBV) const
{
	leftBV = new CAabb(*this);
	rightBV = new CAabb(*this);

	CAabb* lBox = (CAabb*)leftBV;
	CAabb* rBox = (CAabb*)rightBV;
	
	btVector3 c = Center();

	int longSide = LongestSide();

	if ( longSide == 0 )
	{
		lBox->Max()[0] = c[0];
		rBox->Min()[0] = c[0];
	}
	else if ( longSide == 1 )
	{
		lBox->Max()[1] = c[1];
		rBox->Min()[1] = c[1];
	}
	else // if ( longSide == 2 )
	{
		lBox->Max()[2] = c[2];
		rBox->Min()[2] = c[2];
	}
}

void CAabb::Visualize(bool bCollided) const
{
	/*btVector3 min(m_Min);
	btVector3 max(m_Max);

	if ( bCollided )
		glColor3f(1.0f, 1.0f, 1.0f);
	else
		glColor3f(0.5f, 0.5f, 0.5f);

	glLineWidth(1.0);

	glBegin(GL_LINE_STRIP);
		glVertex3d(min[0], min[1], min[2]);
		glVertex3d(max[0], min[1], min[2]);
		glVertex3d(max[0], min[1], max[2]);
		glVertex3d(min[0], min[1], max[2]);
		glVertex3d(min[0], min[1], min[2]);
	glEnd();

	glBegin(GL_LINE_STRIP);
		glVertex3d(min[0], max[1], min[2]);
		glVertex3d(max[0], max[1], min[2]);
		glVertex3d(max[0], max[1], max[2]);
		glVertex3d(min[0], max[1], max[2]);
		glVertex3d(min[0], max[1], min[2]);
	glEnd();

	glBegin(GL_LINES);
		glVertex3d(min[0], min[1], min[2]);
		glVertex3d(min[0], max[1], min[2]);

		glVertex3d(max[0], min[1], min[2]);
		glVertex3d(max[0], max[1], min[2]);

		glVertex3d(max[0], min[1], max[2]);
		glVertex3d(max[0], max[1], max[2]);

		glVertex3d(min[0], min[1], max[2]);
		glVertex3d(min[0], max[1], max[2]);
	glEnd();*/
}

IBoundingVolume& CAabb::operator=(const IBoundingVolume& other)
{
	const CAabb& bv = (CAabb&)other;
	return operator=(bv);
}

CAabb& CAabb::operator=(const CAabb& other)
{
	IBoundingVolume::operator=(other);

	m_Min = other.m_Min;
	m_Max = other.m_Max;

	return (*this);
}

IBoundingVolume& CAabb::operator+=(const btVector3& vec)
{
	if ( IsEmpty() )
	{
		m_Min[0] = vec[0];
		m_Min[1] = vec[1];
		m_Min[2] = vec[2];

		m_Max[0] = vec[0];
		m_Max[1] = vec[1];
		m_Max[2] = vec[2];
	}
	else
	{
		for ( int i = 0; i < 3; i++ )
		{
			m_Min[i] = std::min(m_Min[i], vec[i]);
			m_Max[i] = std::max(m_Max[i], vec[i]);
		}
	}

	return (*this);
}

IBoundingVolume& CAabb::operator+=(const IBoundingVolume& other)
{
	const CAabb& bv = (CAabb&)other;

	if ( IsEmpty() )
	{
		*this = other;
	}
	else
	{
		for ( int i = 0; i < 3; i++ )
		{
			m_Min[i] = std::min(m_Min[i], bv.m_Min[i]);
			m_Max[i] = std::max(m_Max[i], bv.m_Max[i]);
		}
	}

	return (*this);
}