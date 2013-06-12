#ifndef B3_AABB_H
#define B3_AABB_H

#include "BoundingVolume.h"
#include "Bullet3Common/b3Vector3.h"

class b3Vector3;

class CAabb : public IBoundingVolume
{
public:
	CAabb(void);
	CAabb(const CAabb& other);
	virtual ~CAabb(void);

private:
	b3Vector3 m_Min;
	b3Vector3 m_Max;

public:
	const b3Vector3& Min() const { return m_Min; }
	b3Vector3& Min() { return m_Min; }
	const b3Vector3& Max() const { return m_Max; }
	b3Vector3& Max() { return m_Max; }
	void Set(const b3Vector3& min, const b3Vector3& max);
public:	

	virtual void Enlarge(float h);

	virtual bool Collide(const IBoundingVolume& other, float tolerance = 0) const;
	virtual bool Inside(const b3Vector3& point) const;

	virtual void Empty();
	virtual bool IsEmpty() const;
	virtual void Visualize(bool bCollided = false) const;

	virtual float Height() const;
	virtual float Width() const;
	virtual float Length() const;
	virtual b3Vector3 Center() const;
	virtual float Volume() const;

	// If width is longest, returns 0. If height is longest, returns 1. If length is longest, returns 2. 
	virtual int LongestSide() const;

	// Split this box into two CAabb boxes by cutting the longest side half
	virtual void Split(IBoundingVolume*& leftBV, IBoundingVolume*& rightBV) const;

	IBoundingVolume& operator=(const IBoundingVolume& other);
	CAabb& operator=(const CAabb& other);
	IBoundingVolume& operator+=(const b3Vector3& vec);
	IBoundingVolume& operator+=(const IBoundingVolume& other);
};

#endif // B3_AABB_H