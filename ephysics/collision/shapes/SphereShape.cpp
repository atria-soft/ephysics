/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/shapes/SphereShape.hpp>
#include <ephysics/collision/ProxyShape.hpp>
#include <ephysics/configuration.hpp>

using namespace ephysics;

// Constructor
/**
 * @param radius Radius of the sphere (in meters)
 */
SphereShape::SphereShape(float radius) : ConvexShape(SPHERE, radius) {
	assert(radius > 0.0f);
}


void SphereShape::setLocalScaling(const vec3& _scaling) {
	m_margin = (m_margin / m_scaling.x()) * _scaling.x();
	CollisionShape::setLocalScaling(_scaling);
}

void SphereShape::computeLocalInertiaTensor(etk::Matrix3x3& _tensor, float _mass) const {
	float diag = 0.4f * _mass * m_margin * m_margin;
	_tensor.setValue(diag, 0.0f,  0.0f,
	                 0.0f,  diag, 0.0f,
	                 0.0f,  0.0f,  diag);
}

void SphereShape::computeAABB(AABB& _aabb, const etk::Transform3D& _transform) const {
	// Get the local extents in x,y and z direction
	vec3 extents(m_margin, m_margin, m_margin);
	// Update the AABB with the new minimum and maximum coordinates
	_aabb.setMin(_transform.getPosition() - extents);
	_aabb.setMax(_transform.getPosition() + extents);
}

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
	if (discriminant < 0.0f || raySquareLength < FLT_EPSILON) return false;

	// Compute the solution "t" closest to the origin
	float t = -b - etk::sqrt(discriminant);

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
