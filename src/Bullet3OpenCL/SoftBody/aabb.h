#pragma once

#include "BoundingVolume.h"
#include "LinearMath/btVector3.h"

class btVector3;

class CAabb : public IBoundingVolume
{
public:
	CAabb(void);
	CAabb(const CAabb& other);
	virtual ~CAabb(void);

private:
	btVector3 m_Min;
	btVector3 m_Max;

public:
	const btVector3& Min() const { return m_Min; }
	btVector3& Min() { return m_Min; }
	const btVector3& Max() const { return m_Max; }
	btVector3& Max() { return m_Max; }
	void Set(const btVector3& min, const btVector3& max);
public:	

	virtual void Enlarge(float h);

	virtual bool Collide(const IBoundingVolume& other, float tolerance = 0) const;
	virtual bool Inside(const btVector3& point) const;

	virtual void Empty();
	virtual bool IsEmpty() const;
	virtual void Visualize(bool bCollided = false) const;

	virtual float Height() const;
	virtual float Width() const;
	virtual float Length() const;
	virtual btVector3 Center() const;
	virtual float Volume() const;

	// If width is longest, returns 0. If height is longest, returns 1. If length is longest, returns 2. 
	virtual int LongestSide() const;

	// Split this box into two CAabb boxes by cutting the longest side half
	virtual void Split(IBoundingVolume*& leftBV, IBoundingVolume*& rightBV) const;

	IBoundingVolume& operator=(const IBoundingVolume& other);
	CAabb& operator=(const CAabb& other);
	IBoundingVolume& operator+=(const btVector3& vec);
	IBoundingVolume& operator+=(const IBoundingVolume& other);
};
