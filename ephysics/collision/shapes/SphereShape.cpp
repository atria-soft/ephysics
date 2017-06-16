/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/shapes/SphereShape.hpp>
#include <ephysics/collision/ProxyShape.hpp>
#include <ephysics/configuration.hpp>
#include <cassert>

using namespace ephysics;

// Constructor
/**
 * @param radius Radius of the sphere (in meters)
 */
SphereShape::SphereShape(float radius) : ConvexShape(SPHERE, radius) {
	assert(radius > 0.0f);
}

// Destructor
SphereShape::~SphereShape() {

}

// Raycast method with feedback information
bool SphereShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const {

	const vec3 m = ray.point1;
	float c = m.dot(m) - m_margin * m_margin;

	// If the origin of the ray is inside the sphere, we return no int32_tersection
	if (c < 0.0f) return false;

	const vec3 rayDirection = ray.point2 - ray.point1;
	float b = m.dot(rayDirection);

	// If the origin of the ray is outside the sphere and the ray
	// is pointing away from the sphere, there is no int32_tersection
	if (b > 0.0f) return false;

	float raySquareLength = rayDirection.length2();

	// Compute the discriminant of the quadratic equation
	float discriminant = b * b - raySquareLength * c;

	// If the discriminant is negative or the ray length is very small, there is no int32_tersection
	if (discriminant < 0.0f || raySquareLength < MACHINE_EPSILON) return false;

	// Compute the solution "t" closest to the origin
	float t = -b - std::sqrt(discriminant);

	assert(t >= 0.0f);

	// If the hit point is withing the segment ray fraction
	if (t < ray.maxFraction * raySquareLength) {

		// Compute the int32_tersection information
		t /= raySquareLength;
		raycastInfo.body = proxyShape->getBody();
		raycastInfo.proxyShape = proxyShape;
		raycastInfo.hitFraction = t;
		raycastInfo.worldPoint = ray.point1 + t * rayDirection;
		raycastInfo.worldNormal = raycastInfo.worldPoint;

		return true;
	}

	return false;
}
