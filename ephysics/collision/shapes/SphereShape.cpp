/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/shapes/SphereShape.h>
#include <ephysics/collision/ProxyShape.h>
#include <ephysics/configuration.h>
#include <cassert>

using namespace reactphysics3d;

// Constructor
/**
 * @param radius Radius of the sphere (in meters)
 */
SphereShape::SphereShape(float radius) : ConvexShape(SPHERE, radius) {
	assert(radius > float(0.0));
}

// Destructor
SphereShape::~SphereShape() {

}

// Raycast method with feedback information
bool SphereShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const {

	const Vector3 m = ray.point1;
	float c = m.dot(m) - mMargin * mMargin;

	// If the origin of the ray is inside the sphere, we return no int32_tersection
	if (c < float(0.0)) return false;

	const Vector3 rayDirection = ray.point2 - ray.point1;
	float b = m.dot(rayDirection);

	// If the origin of the ray is outside the sphere and the ray
	// is pointing away from the sphere, there is no int32_tersection
	if (b > float(0.0)) return false;

	float raySquareLength = rayDirection.lengthSquare();

	// Compute the discriminant of the quadratic equation
	float discriminant = b * b - raySquareLength * c;

	// If the discriminant is negative or the ray length is very small, there is no int32_tersection
	if (discriminant < float(0.0) || raySquareLength < MACHINE_EPSILON) return false;

	// Compute the solution "t" closest to the origin
	float t = -b - std::sqrt(discriminant);

	assert(t >= float(0.0));

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
