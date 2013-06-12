#include "Aabb.h"

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

void CAabb::Set(const b3Vector3& min, const b3Vector3& max)
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

bool CAabb::Inside(const b3Vector3& point) const
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
	float max = 1e30;
	float min = -1e30;

	m_Min = b3Vector3(max, max, max);
	m_Max = b3Vector3(min, min, min);
}

bool CAabb::IsEmpty() const
{
	float max = 1e30;
	float min = -1e30;

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

b3Vector3 CAabb::Center() const
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
	
	b3Vector3 c = Center();

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

IBoundingVolume& CAabb::operator+=(const b3Vector3& vec)
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
			m_Min[i] = b3Min<float>(m_Min[i], vec[i]);
			m_Max[i] = b3Max<float>(m_Max[i], vec[i]);
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
			m_Min[i] = b3Min<float>(m_Min[i], bv.m_Min[i]);
			m_Max[i] = b3Max<float>(m_Max[i], bv.m_Max[i]);
		}
	}

	return (*this);
}