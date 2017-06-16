/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/shapes/BoxShape.h>
#include <ephysics/collision/ProxyShape.h>
#include <ephysics/configuration.h>
#include <vector>
#include <cassert>

using namespace reactphysics3d;

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
			tMax = std::min(tMax, t2);

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
