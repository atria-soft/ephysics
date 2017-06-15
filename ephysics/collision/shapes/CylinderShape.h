/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/collision/shapes/ConvexShape.h>
#include <ephysics/body/CollisionBody.h>
#include <ephysics/mathematics/mathematics.h>


/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class CylinderShape
/**
 * This class represents a cylinder collision shape around the Y axis
 * and centered at the origin. The cylinder is defined by its height
 * and the radius of its base. The "transform" of the corresponding
 * rigid body gives an orientation and a position to the cylinder.
 * This collision shape uses an extra margin distance around it for collision
 * detection purpose. The default margin is 4cm (if your units are meters,
 * which is recommended). In case, you want to simulate small objects
 * (smaller than the margin distance), you might want to reduce the margin by
 * specifying your own margin distance using the "margin" parameter in the
 * constructor of the cylinder shape. Otherwise, it is recommended to use the
 * default margin distance by not using the "margin" parameter in the constructor.
 */
class CylinderShape : public ConvexShape {

	protected :

		// -------------------- Attributes -------------------- //

		/// Radius of the base
		float mRadius;

		/// Half height of the cylinder
		float m_halfHeight;

		// -------------------- Methods -------------------- //

		/// Private copy-constructor
		CylinderShape(const CylinderShape& shape);

		/// Private assignment operator
		CylinderShape& operator=(const CylinderShape& shape);

		/// Return a local support point in a given direction without the object margin
		virtual vec3 getLocalSupportPointWithoutMargin(const vec3& direction,
														  void** cachedCollisionData) const;

		/// Return true if a point is inside the collision shape
		virtual bool testPointInside(const vec3& localPoint, ProxyShape* proxyShape) const;

		/// Raycast method with feedback information
		virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const;

		/// Return the number of bytes used by the collision shape
		virtual size_t getSizeInBytes() const;

	public :

		// -------------------- Methods -------------------- //

		/// Constructor
		CylinderShape(float radius, float height, float margin = OBJECT_MARGIN);

		/// Destructor
		virtual ~CylinderShape();

		/// Return the radius
		float getRadius() const;

		/// Return the height
		float getHeight() const;

		/// Set the scaling vector of the collision shape
		virtual void setLocalScaling(const vec3& scaling);

		/// Return the local bounds of the shape in x, y and z directions
		virtual void getLocalBounds(vec3& min, vec3& max) const;

		/// Return the local inertia tensor of the collision shape
		virtual void computeLocalInertiaTensor(etk::Matrix3x3& tensor, float mass) const;
};

// Return the radius
/**
 * @return Radius of the cylinder (in meters)
 */
inline float CylinderShape::getRadius() const {
	return mRadius;
}

// Return the height
/**
 * @return Height of the cylinder (in meters)
 */
inline float CylinderShape::getHeight() const {
	return m_halfHeight + m_halfHeight;
}

// Set the scaling vector of the collision shape
inline void CylinderShape::setLocalScaling(const vec3& scaling) {

	m_halfHeight = (m_halfHeight / m_scaling.y()) * scaling.y();
	mRadius = (mRadius / m_scaling.x()) * scaling.x();

	CollisionShape::setLocalScaling(scaling);
}

// Return the number of bytes used by the collision shape
inline size_t CylinderShape::getSizeInBytes() const {
	return sizeof(CylinderShape);
}

// Return the local bounds of the shape in x, y and z directions
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
inline void CylinderShape::getLocalBounds(vec3& min, vec3& max) const {
	// Maximum bounds
	max.setX(mRadius + m_margin);
	max.setY(m_halfHeight + m_margin);
	max.setZ(max.x());
	// Minimum bounds
	min.setX(-max.x());
	min.setY(-max.y());
	min.setZ(min.x());
}

// Return the local inertia tensor of the cylinder
/**
 * @param[out] tensor The 3x3 inertia tensor matrix of the shape in local-space
 *					coordinates
 * @param mass Mass to use to compute the inertia tensor of the collision shape
 */
inline void CylinderShape::computeLocalInertiaTensor(etk::Matrix3x3& tensor, float mass) const {
	float height = float(2.0) * m_halfHeight;
	float diag = (1.0f / float(12.0)) * mass * (3 * mRadius * mRadius + height * height);
	tensor.setValue(diag, 0.0, 0.0, 0.0,
						0.5f * mass * mRadius * mRadius, 0.0,
						0.0, 0.0, diag);
}

// Return true if a point is inside the collision shape
inline bool CylinderShape::testPointInside(const vec3& localPoint, ProxyShape* proxyShape) const{
	return (    (localPoint.x() * localPoint.x() + localPoint.z() * localPoint.z()) < mRadius * mRadius
	         && localPoint.y() < m_halfHeight
	         && localPoint.y() > -m_halfHeight);
}

}


