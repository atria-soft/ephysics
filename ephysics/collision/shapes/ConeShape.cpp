/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#include <ephysics/configuration.hpp>
#include <ephysics/collision/shapes/ConeShape.hpp>
#include <ephysics/collision/ProxyShape.hpp>

using namespace ephysics;

// Constructor
/**
 * @param radius Radius of the cone (in meters)
 * @param height Height of the cone (in meters)
 * @param margin Collision margin (in meters) around the collision shape
 */
ConeShape::ConeShape(float radius, float height, float margin):
  ConvexShape(CONE, margin),
  m_radius(radius),
  m_halfHeight(height * 0.5f) {
	assert(m_radius > 0.0f);
	assert(m_halfHeight > 0.0f);
	
	// Compute the sine of the semi-angle at the apex point
	m_sinTheta = m_radius / (sqrt(m_radius * m_radius + height * height));
}

// Return a local support point in a given direction without the object margin
vec3 ConeShape::getLocalSupportPointWithoutMargin(const vec3& direction,
													 void** cachedCollisionData) const {

	const vec3& v = direction;
	float sinThetaTimesLengthV = m_sinTheta * v.length();
	vec3 supportPoint;

	if (v.y() > sinThetaTimesLengthV) {
		supportPoint = vec3(0.0, m_halfHeight, 0.0);
	}
	else {
		float projectedLength = sqrt(v.x() * v.x() + v.z() * v.z());
		if (projectedLength > FLT_EPSILON) {
			float d = m_radius / projectedLength;
			supportPoint = vec3(v.x() * d, -m_halfHeight, v.z() * d);
		}
		else {
			supportPoint = vec3(0.0, -m_halfHeight, 0.0);
		}
	}

	return supportPoint;
}

// Raycast method with feedback information
// This implementation is based on the technique described by David Eberly in the article
// "Intersection of a Line and a Cone" that can be found at
// http://www.geometrictools.com/Documentation/IntersectionLineCone.pdf
bool ConeShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const {

	const vec3 r = ray.point2 - ray.point1;

	const float epsilon = float(0.00001);
	vec3 V(0, m_halfHeight, 0);
	vec3 centerBase(0, -m_halfHeight, 0);
	vec3 axis(0, float(-1.0), 0);
	float heightSquare = float(4.0) * m_halfHeight * m_halfHeight;
	float cosThetaSquare = heightSquare / (heightSquare + m_radius * m_radius);
	float factor = 1.0f - cosThetaSquare;
	vec3 delta = ray.point1 - V;
	float c0 = -cosThetaSquare * delta.x() * delta.x()  + factor * delta.y() * delta.y() -
				  cosThetaSquare * delta.z() * delta.z();
	float c1 = -cosThetaSquare * delta.x() * r.x() + factor * delta.y() * r.y() - cosThetaSquare * delta.z() * r.z();
	float c2 = -cosThetaSquare * r.x() * r.x()  + factor * r.y() * r.y() - cosThetaSquare * r.z() * r.z();
	float tHit[] = {float(-1.0), float(-1.0), float(-1.0)};
	vec3 localHitPoint[3];
	vec3 localNormal[3];

	// If c2 is different from zero
	if (etk::abs(c2) > FLT_EPSILON) {
		float gamma = c1 * c1 - c0 * c2;

		// If there is no real roots in the quadratic equation
		if (gamma < 0.0f) {
			return false;
		}
		else if (gamma > 0.0f) {	// The equation has two real roots

			// Compute two int32_tersections
			float sqrRoot = etk::sqrt(gamma);
			tHit[0] = (-c1 - sqrRoot) / c2;
			tHit[1] = (-c1 + sqrRoot) / c2;
		}
		else {  // If the equation has a single real root

			// Compute the int32_tersection
			tHit[0] = -c1 / c2;
		}
	}
	else {  // If c2 == 0

		// If c2 = 0 and c1 != 0
		if (etk::abs(c1) > FLT_EPSILON) {
			tHit[0] = -c0 / (float(2.0) * c1);
		}
		else {  // If c2 = c1 = 0

			// If c0 is different from zero, no solution and if c0 = 0, we have a
			// degenerate case, the whole ray is contained in the cone side
			// but we return no hit in this case
			return false;
		}
	}

	// If the origin of the ray is inside the cone, we return no hit
	if (testPointInside(ray.point1, NULL)) return false;

	localHitPoint[0] = ray.point1 + tHit[0] * r;
	localHitPoint[1] = ray.point1 + tHit[1] * r;

	// Only keep hit points in one side of the double cone (the cone we are int32_terested in)
	if (axis.dot(localHitPoint[0] - V) < 0.0f) {
		tHit[0] = float(-1.0);
	}
	if (axis.dot(localHitPoint[1] - V) < 0.0f) {
		tHit[1] = float(-1.0);
	}

	// Only keep hit points that are within the correct height of the cone
	if (localHitPoint[0].y() < float(-m_halfHeight)) {
		tHit[0] = float(-1.0);
	}
	if (localHitPoint[1].y() < float(-m_halfHeight)) {
		tHit[1] = float(-1.0);
	}

	// If the ray is in direction of the base plane of the cone
	if (r.y() > epsilon) {

		// Compute the int32_tersection with the base plane of the cone
		tHit[2] = (-ray.point1.y() - m_halfHeight) / (r.y());

		// Only keep this int32_tersection if it is inside the cone radius
		localHitPoint[2] = ray.point1 + tHit[2] * r;

		if ((localHitPoint[2] - centerBase).length2() > m_radius * m_radius) {
			tHit[2] = float(-1.0);
		}

		// Compute the normal direction
		localNormal[2] = axis;
	}

	// Find the smallest positive t value
	int32_t hitIndex = -1;
	float t = FLT_MAX;
	for (int32_t i=0; i<3; i++) {
		if (tHit[i] < 0.0f) continue;
		if (tHit[i] < t) {
			hitIndex = i;
			t = tHit[hitIndex];
		}
	}

	if (hitIndex < 0) return false;
	if (t > ray.maxFraction) return false;

	// Compute the normal direction for hit against side of the cone
	if (hitIndex != 2) {
		float h = float(2.0) * m_halfHeight;
		float value1 = (localHitPoint[hitIndex].x() * localHitPoint[hitIndex].x() +
						  localHitPoint[hitIndex].z() * localHitPoint[hitIndex].z());
		float rOverH = m_radius / h;
		float value2 = 1.0f + rOverH * rOverH;
		float factor = 1.0f / etk::sqrt(value1 * value2);
		float x = localHitPoint[hitIndex].x() * factor;
		float z = localHitPoint[hitIndex].z() * factor;
		localNormal[hitIndex].setX(x);
		localNormal[hitIndex].setY(etk::sqrt(x * x + z * z) * rOverH);
		localNormal[hitIndex].setZ(z);
	}

	raycastInfo.body = proxyShape->getBody();
	raycastInfo.proxyShape = proxyShape;
	raycastInfo.hitFraction = t;
	raycastInfo.worldPoint = localHitPoint[hitIndex];
	raycastInfo.worldNormal = localNormal[hitIndex];

	return true;
}

// Return the radius
/**
 * @return Radius of the cone (in meters)
 */
float ConeShape::getRadius() const {
	return m_radius;
}

// Return the height
/**
 * @return Height of the cone (in meters)
 */
float ConeShape::getHeight() const {
	return float(2.0) * m_halfHeight;
}

// Set the scaling vector of the collision shape
void ConeShape::setLocalScaling(const vec3& scaling) {

	m_halfHeight = (m_halfHeight / m_scaling.y()) * scaling.y();
	m_radius = (m_radius / m_scaling.x()) * scaling.x();

	CollisionShape::setLocalScaling(scaling);
}

// Return the number of bytes used by the collision shape
size_t ConeShape::getSizeInBytes() const {
	return sizeof(ConeShape);
}

// Return the local bounds of the shape in x, y and z directions
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
void ConeShape::getLocalBounds(vec3& min, vec3& max) const {

	// Maximum bounds
	max.setX(m_radius + m_margin);
	max.setY(m_halfHeight + m_margin);
	max.setZ(max.x());

	// Minimum bounds
	min.setX(-max.x());
	min.setY(-max.y());
	min.setZ(min.x());
}

// Return the local inertia tensor of the collision shape
/**
 * @param[out] tensor The 3x3 inertia tensor matrix of the shape in local-space
 *					coordinates
 * @param mass Mass to use to compute the inertia tensor of the collision shape
 */
void ConeShape::computeLocalInertiaTensor(etk::Matrix3x3& tensor, float mass) const {
	float rSquare = m_radius * m_radius;
	float diagXZ = float(0.15) * mass * (rSquare + m_halfHeight);
	tensor.setValue(diagXZ, 0.0, 0.0,
						0.0, float(0.3) * mass * rSquare,
						0.0, 0.0, 0.0, diagXZ);
}

// Return true if a point is inside the collision shape
bool ConeShape::testPointInside(const vec3& localPoint, ProxyShape* proxyShape) const {
	const float radiusHeight = m_radius * (-localPoint.y() + m_halfHeight) /
										  (m_halfHeight * float(2.0));
	return (localPoint.y() < m_halfHeight && localPoint.y() > -m_halfHeight) &&
		   (localPoint.x() * localPoint.x() + localPoint.z() * localPoint.z() < radiusHeight *radiusHeight);
}
