#pragma once

class btVector3;

class IBoundingVolume
{
public:
	IBoundingVolume(void) { m_GlobalIndex = -1; }
	IBoundingVolume(const IBoundingVolume& other) { m_GlobalIndex = other.m_GlobalIndex; }
	virtual ~IBoundingVolume(void) {}

	virtual void Enlarge(float h) = 0;

	virtual bool Collide(const IBoundingVolume& other, float tolerance = 0) const = 0;
	virtual bool Inside(const btVector3& point) const = 0;

	virtual void Empty() = 0;
	virtual bool IsEmpty() const = 0;
	virtual void Visualize(bool bCollided = false) const = 0;

	virtual float Height() const = 0;
	virtual float Width() const = 0;
	virtual float Length() const = 0;
	virtual btVector3 Center() const = 0;
	virtual float Volume() const = 0;
	virtual int LongestSide() const = 0;

	virtual void Split(IBoundingVolume*& leftBV, IBoundingVolume*& rightBV) const = 0;

	int GetGlobalIndex() const { return m_GlobalIndex; }
	void SetGlobalIndex(int globalIndex) { m_GlobalIndex = globalIndex; }

	IBoundingVolume& operator=(const IBoundingVolume& other) { m_GlobalIndex = other.m_GlobalIndex; return (*this); }

protected:
	int m_GlobalIndex;
};


//#define USE_KDOP

#ifdef USE_KDOP
	class CkDOP18;
	typedef CkDOP18 CBoundingVolume;
#else
	class CAabb;
	typedef CAabb CBoundingVolume;
#endif

