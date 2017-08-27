/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/shapes/BoxShape.hpp>
#include <ephysics/collision/ProxyShape.hpp>
#include <ephysics/configuration.hpp>
#include <etk/Vector.hpp>
#include <cassert>

using namespace ephysics;

// Constructor
/**
 * @param extent The vector with the three extents of the box (in meters)
 * @param margin The collision margin (in meters) around the collision shape
 */
BoxShape::BoxShape(const vec3& _extent, float _margin):
  ConvexShape(BOX, _margin),
  m_extent(_extent - vec3(_margin, _margin, _margin)) {
	assert(_extent.x() > 0.0f && _extent.x() > _margin);
	assert(_extent.y() > 0.0f && _extent.y() > _margin);
	assert(_extent.z() > 0.0f && _extent.z() > _margin);
}

// Return the local inertia tensor of the collision shape
/**
 * @param[out] tensor The 3x3 inertia tensor matrix of the shape in local-space
 *					coordinates
 * @param mass Mass to use to compute the inertia tensor of the collision shape
 */
void BoxShape::computeLocalInertiaTensor(etk::Matrix3x3& tensor, float mass) const {
	float factor = (1.0f / float(3.0)) * mass;
	vec3 realExtent = m_extent + vec3(m_margin, m_margin, m_margin);
	float xSquare = realExtent.x() * realExtent.x();
	float ySquare = realExtent.y() * realExtent.y();
	float zSquare = realExtent.z() * realExtent.z();
	tensor.setValue(factor * (ySquare + zSquare), 0.0, 0.0,
						0.0, factor * (xSquare + zSquare), 0.0,
						0.0, 0.0, factor * (xSquare + ySquare));
}

// Raycast method with feedback information
bool BoxShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const {

	vec3 rayDirection = ray.point2 - ray.point1;
	float tMin = DECIMAL_SMALLEST;
	float tMax = DECIMAL_LARGEST;
	vec3 normalDirection(float(0), float(0), float(0));
	vec3 currentNormal;

	// For each of the three slabs
	for (int32_t i=0; i<3; i++) {

		// If ray is parallel to the slab
		if (std::abs(rayDirection[i]) < MACHINE_EPSILON) {

			// If the ray's origin is not inside the slab, there is no hit
			if (ray.point1[i] > m_extent[i] || ray.point1[i] < -m_extent[i]) return false;
		}
		else {

			// Compute the int32_tersection of the ray with the near and far plane of the slab
			float oneOverD = 1.0f / rayDirection[i];
			float t1 = (-m_extent[i] - ray.point1[i]) * oneOverD;
			float t2 = (m_extent[i] - ray.point1[i]) * oneOverD;
			currentNormal[0] = (i == 0) ? -m_extent[i] : 0.0f;
			currentNormal[1] = (i == 1) ? -m_extent[i] : 0.0f;
			currentNormal[2] = (i == 2) ? -m_extent[i] : 0.0f;

			// Swap t1 and t2 if need so that t1 is int32_tersection with near plane and
			// t2 with far plane
			if (t1 > t2) {
				std::swap(t1, t2);
				currentNormal = -currentNormal;
			}

			// Compute the int32_tersection of the of slab int32_tersection int32_terval with previous slabs
			if (t1 > tMin) {
				tMin = t1;
				normalDirection = currentNormal;
			}
			tMax = etk::min(tMax, t2);

			// If tMin is larger than the maximum raycasting fraction, we return no hit
			if (tMin > ray.maxFraction) return false;

			// If the slabs int32_tersection is empty, there is no hit
			if (tMin > tMax) return false;
		}
	}

	// If tMin is negative, we return no hit
	if (tMin < 0.0f || tMin > ray.maxFraction) return false;

	// The ray int32_tersects the three slabs, we compute the hit point
	vec3 localHitPoint = ray.point1 + tMin * rayDirection;

	raycastInfo.body = proxyShape->getBody();
	raycastInfo.proxyShape = proxyShape;
	raycastInfo.hitFraction = tMin;
	raycastInfo.worldPoint = localHitPoint;
	raycastInfo.worldNormal = normalDirection;

	return true;
}




// Return the extents of the box
/**
 * @return The vector with the three extents of the box shape (in meters)
 */
vec3 BoxShape::getExtent() const {
	return m_extent + vec3(m_margin, m_margin, m_margin);
}

// Set the scaling vector of the collision shape
void BoxShape::setLocalScaling(const vec3& scaling) {

	m_extent = (m_extent / m_scaling) * scaling;

	CollisionShape::setLocalScaling(scaling);
}

// Return the local bounds of the shape in x, y and z directions
/// This method is used to compute the AABB of the box
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
void BoxShape::getLocalBounds(vec3& _min, vec3& _max) const {

	// Maximum bounds
	_max = m_extent + vec3(m_margin, m_margin, m_margin);

	// Minimum bounds
	_min = -_max;
}

// Return the number of bytes used by the collision shape
size_t BoxShape::getSizeInBytes() const {
	return sizeof(BoxShape);
}

// Return a local support point in a given direction without the objec margin
vec3 BoxShape::getLocalSupportPointWithoutMargin(const vec3& direction,
														   void** cachedCollisionData) const {

	return vec3(direction.x() < 0.0 ? -m_extent.x() : m_extent.x(),
				   direction.y() < 0.0 ? -m_extent.y() : m_extent.y(),
				   direction.z() < 0.0 ? -m_extent.z() : m_extent.z());
}

// Return true if a point is inside the collision shape
bool BoxShape::testPointInside(const vec3& localPoint, ProxyShape* proxyShape) const {
	return (localPoint.x() < m_extent[0] && localPoint.x() > -m_extent[0] &&
			localPoint.y() < m_extent[1] && localPoint.y() > -m_extent[1] &&
			localPoint.z() < m_extent[2] && localPoint.z() > -m_extent[2]);
}

