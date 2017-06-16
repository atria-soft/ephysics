/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <cfloat>
#include <ephysics/collision/shapes/ConvexShape.h>
#include <ephysics/body/CollisionBody.h>
#include <ephysics/mathematics/mathematics.h>


/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class BoxShape
/**
 * This class represents a 3D box shape. Those axis are unit length.
 * The three extents are half-widths of the box along the three
 * axis x, y, z local axis. The "transform" of the corresponding
 * rigid body will give an orientation and a position to the box. This
 * collision shape uses an extra margin distance around it for collision
 * detection purpose. The default margin is 4cm (if your units are meters,
 * which is recommended). In case, you want to simulate small objects
 * (smaller than the margin distance), you might want to reduce the margin by
 * specifying your own margin distance using the "margin" parameter in the
 * constructor of the box shape. Otherwise, it is recommended to use the
 * default margin distance by not using the "margin" parameter in the constructor.
 */
class BoxShape : public ConvexShape {

	protected :
		/// Extent sizes of the box in the x, y and z direction
		vec3 m_extent;
		/// Private copy-constructor
		BoxShape(const BoxShape& shape);

		/// Private assignment operator
		BoxShape& operator=(const BoxShape& shape);

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
		/// Constructor
		BoxShape(const vec3& extent, float margin = OBJECT_MARGIN);

		/// Destructor
		virtual ~BoxShape() = default;

		/// Return the extents of the box
		vec3 getExtent() const;

		/// Set the scaling vector of the collision shape
		virtual void setLocalScaling(const vec3& scaling);

		/// Return the local bounds of the shape in x, y and z directions
		virtual void getLocalBounds(vec3& _min, vec3& _max) const;

		/// Return the local inertia tensor of the collision shape
		virtual void computeLocalInertiaTensor(etk::Matrix3x3& tensor, float mass) const;
};

// Return the extents of the box
/**
 * @return The vector with the three extents of the box shape (in meters)
 */
inline vec3 BoxShape::getExtent() const {
	return m_extent + vec3(m_margin, m_margin, m_margin);
}

// Set the scaling vector of the collision shape
inline void BoxShape::setLocalScaling(const vec3& scaling) {

	m_extent = (m_extent / m_scaling) * scaling;

	CollisionShape::setLocalScaling(scaling);
}

// Return the local bounds of the shape in x, y and z directions
/// This method is used to compute the AABB of the box
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
inline void BoxShape::getLocalBounds(vec3& _min, vec3& _max) const {

	// Maximum bounds
	_max = m_extent + vec3(m_margin, m_margin, m_margin);

	// Minimum bounds
	_min = -_max;
}

// Return the number of bytes used by the collision shape
inline size_t BoxShape::getSizeInBytes() const {
	return sizeof(BoxShape);
}

// Return a local support point in a given direction without the objec margin
inline vec3 BoxShape::getLocalSupportPointWithoutMargin(const vec3& direction,
														   void** cachedCollisionData) const {

	return vec3(direction.x() < 0.0 ? -m_extent.x() : m_extent.x(),
				   direction.y() < 0.0 ? -m_extent.y() : m_extent.y(),
				   direction.z() < 0.0 ? -m_extent.z() : m_extent.z());
}

// Return true if a point is inside the collision shape
inline bool BoxShape::testPointInside(const vec3& localPoint, ProxyShape* proxyShape) const {
	return (localPoint.x() < m_extent[0] && localPoint.x() > -m_extent[0] &&
			localPoint.y() < m_extent[1] && localPoint.y() > -m_extent[1] &&
			localPoint.z() < m_extent[2] && localPoint.z() > -m_extent[2]);
}

}
