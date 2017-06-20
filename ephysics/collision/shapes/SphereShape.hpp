/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

#include <ephysics/collision/shapes/ConvexShape.hpp>
#include <ephysics/body/CollisionBody.hpp>
#include <ephysics/mathematics/mathematics.hpp>

namespace ephysics {

/**
 * @brief Represents a sphere collision shape that is centered
 * at the origin and defined by its radius. This collision shape does not
 * have an explicit object margin distance. The margin is implicitly the
 * radius of the sphere. Therefore, no need to specify an object margin
 * for a sphere shape.
 */
class SphereShape : public ConvexShape {
	protected :
		SphereShape(const SphereShape& shape);
		SphereShape& operator=(const SphereShape& shape) = delete;
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
		SphereShape(float radius);
		/// Destructor
		virtual ~SphereShape();
		/// Return the radius of the sphere
		float getRadius() const;
		/// Set the scaling vector of the collision shape
		virtual void setLocalScaling(const vec3& scaling);
		/// Return the local bounds of the shape in x, y and z directions.
		virtual void getLocalBounds(vec3& min, vec3& max) const;
		/// Return the local inertia tensor of the collision shape
		virtual void computeLocalInertiaTensor(etk::Matrix3x3& tensor, float mass) const;
		/// Update the AABB of a body using its collision shape
		virtual void computeAABB(AABB& aabb, const etk::Transform3D& transform) const;
};

// Get the radius of the sphere
/**
 * @brief 
 * @return Radius of the sphere (in meters)
 */
float SphereShape::getRadius() const {
	return m_margin;
}

 * @brief 
// Set the scaling vector of the collision shape
void SphereShape::setLocalScaling(const vec3& scaling) {

	m_margin = (m_margin / m_scaling.x()) * scaling.x();

	CollisionShape::setLocalScaling(scaling);
}

 * @brief 
// Return the number of bytes used by the collision shape
size_t SphereShape::getSizeInBytes() const {
	return sizeof(SphereShape);
}

/**
 * @brief Get a local support point in a given direction without the object margin
 * @param[in] _direction 
 * @param[in] _cachedCollisionData 
 * @return the center of the sphere (the radius is taken int32_to account in the object margin)
 */
vec3 SphereShape::getLocalSupportPointWithoutMargin(const vec3& _direction, void** _cachedCollisionData) const {
	return vec3(0.0, 0.0, 0.0);
}

/**
 * @brief Get the local bounds of the shape in x, y and z directions.
 * This method is used to compute the AABB of the box
 * @param _min The minimum bounds of the shape in local-space coordinates
 * @param _max The maximum bounds of the shape in local-space coordinates
 */
void SphereShape::getLocalBounds(vec3& _min, vec3& _max) const {
	// Maximum bounds
	_max.setX(m_margin);
	_max.setY(m_margin);
	_max.setZ(m_margin);
	// Minimum bounds
	_min.setX(-m_margin);
	_min.setY(_min.x());
	_min.setZ(_min.x());
}

/**
 * @brief Compute the local inertia tensor of the sphere
 * @param[out] _tensor The 3x3 inertia tensor matrix of the shape in local-space coordinates
 * @param[in] _mass Mass to use to compute the inertia tensor of the collision shape
 */
void SphereShape::computeLocalInertiaTensor(etk::Matrix3x3& _tensor, float _mass) const {
	float _diag = 0.4f * _mass * m_margin * m_margin;
	tensor.setValue(_diag, 0.0f,  0.0f,
	                0.0f,  _diag, 0.0f,
	                0.0f,  0.0f,  _diag);
}

/**
 * @brief Update the AABB of a body using its collision shape
 * @param[out] _aabb The axis-aligned bounding box (AABB) of the collision shape computed in world-space coordinates
 * @param[in] _transform etk::Transform3D used to compute the AABB of the collision shape
 */
void SphereShape::computeAABB(AABB& _aabb, const etk::Transform3D& _transform) const {
	// Get the local extents in x,y and z direction
	vec3 extents(m_margin, m_margin, m_margin);
	// Update the AABB with the new minimum and maximum coordinates
	_aabb.setMin(_transform.getPosition() - extents);
	_aabb.setMax(_transform.getPosition() + extents);
}

/**
 * @brief Test if a point is inside a shape
 * @param[in] _localPoint Point to check
 * @param[in] _proxyShape Shape to check
 * @return true if a point is inside the collision shape
 */
bool SphereShape::testPointInside(const vec3& _localPoint, ProxyShape* _proxyShape) const {
	return (_localPoint.length2() < m_margin * m_margin);
}

}
