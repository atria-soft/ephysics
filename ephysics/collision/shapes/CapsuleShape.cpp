/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/shapes/CapsuleShape.hpp>
#include <ephysics/collision/ProxyShape.hpp>
#include <ephysics/configuration.hpp>

using namespace ephysics;

// Constructor
/**
 * @param _radius The radius of the capsule (in meters)
 * @param _height The height of the capsule (in meters)
 */
CapsuleShape::CapsuleShape(float _radius, float _height):
  ConvexShape(CAPSULE, _radius),
  m_halfHeight(_height * 0.5f) {
	assert(_radius > 0.0f);
	assert(_height > 0.0f);
}

// Destructor
CapsuleShape::~CapsuleShape() {

}

// Return the local inertia tensor of the capsule
/**
 * @param[out] tensor The 3x3 inertia tensor matrix of the shape in local-space
 *					coordinates
 * @param mass Mass to use to compute the inertia tensor of the collision shape
 */
void CapsuleShape::computeLocalInertiaTensor(etk::Matrix3x3& tensor, float mass) const {

	// The inertia tensor formula for a capsule can be found in : Game Engine Gems, Volume 1
	
	float height = m_halfHeight + m_halfHeight;
	float radiusSquare = m_margin * m_margin;
	float heightSquare = height * height;
	float radiusSquareDouble = radiusSquare + radiusSquare;
	float factor1 = float(2.0) * m_margin / (float(4.0) * m_margin + float(3.0) * height);
	float factor2 = float(3.0) * height / (float(4.0) * m_margin + float(3.0) * height);
	float sum1 = float(0.4) * radiusSquareDouble;
	float sum2 = float(0.75) * height * m_margin + 0.5f * heightSquare;
	float sum3 = float(0.25) * radiusSquare + float(1.0 / 12.0) * heightSquare;
	float IxxAndzz = factor1 * mass * (sum1 + sum2) + factor2 * mass * sum3;
	float Iyy = factor1 * mass * sum1 + factor2 * mass * float(0.25) * radiusSquareDouble;
	tensor.setValue(IxxAndzz, 0.0, 0.0,
						0.0, Iyy, 0.0,
						0.0, 0.0, IxxAndzz);
}

// Return true if a point is inside the collision shape
bool CapsuleShape::testPointInside(const vec3& localPoint, ProxyShape* proxyShape) const {

	const float diffYCenterSphere1 = localPoint.y() - m_halfHeight;
	const float diffYCenterSphere2 = localPoint.y() + m_halfHeight;
	const float xSquare = localPoint.x() * localPoint.x();
	const float zSquare = localPoint.z() * localPoint.z();
	const float squareRadius = m_margin * m_margin;

	// Return true if the point is inside the cylinder or one of the two spheres of the capsule
	return ((xSquare + zSquare) < squareRadius &&
			localPoint.y() < m_halfHeight && localPoint.y() > -m_halfHeight) ||
			(xSquare + zSquare + diffYCenterSphere1 * diffYCenterSphere1) < squareRadius ||
			(xSquare + zSquare + diffYCenterSphere2 * diffYCenterSphere2) < squareRadius;
}

// Raycast method with feedback information
bool CapsuleShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const {

	const vec3 n = ray.point2 - ray.point1;

	const float epsilon = float(0.01);
	vec3 p(float(0), -m_halfHeight, float(0));
	vec3 q(float(0), m_halfHeight, float(0));
	vec3 d = q - p;
	vec3 m = ray.point1 - p;
	float t;

	float mDotD = m.dot(d);
	float nDotD = n.dot(d);
	float dDotD = d.dot(d);

	// Test if the segment is outside the cylinder
	float vec1DotD = (ray.point1 - vec3(0.0f, -m_halfHeight - m_margin, float(0.0))).dot(d);
	if (vec1DotD < 0.0f && vec1DotD + nDotD < float(0.0)) return false;
	float ddotDExtraCaps = float(2.0) * m_margin * d.y();
	if (vec1DotD > dDotD + ddotDExtraCaps && vec1DotD + nDotD > dDotD + ddotDExtraCaps) return false;

	float nDotN = n.dot(n);
	float mDotN = m.dot(n);

	float a = dDotD * nDotN - nDotD * nDotD;
	float k = m.dot(m) - m_margin * m_margin;
	float c = dDotD * k - mDotD * mDotD;

	// If the ray is parallel to the capsule axis
	if (etk::abs(a) < epsilon) {

		// If the origin is outside the surface of the capusle's cylinder, we return no hit
		if (c > 0.0f) return false;

		// Here we know that the segment int32_tersect an endcap of the capsule

		// If the ray int32_tersects with the "p" endcap of the capsule
		if (mDotD < 0.0f) {

			// Check int32_tersection between the ray and the "p" sphere endcap of the capsule
			vec3 hitLocalPoint;
			float hitFraction;
			if (raycastWithSphereEndCap(ray.point1, ray.point2, p, ray.maxFraction, hitLocalPoint, hitFraction)) {
				raycastInfo.body = proxyShape->getBody();
				raycastInfo.proxyShape = proxyShape;
				raycastInfo.hitFraction = hitFraction;
				raycastInfo.worldPoint = hitLocalPoint;
				vec3 normalDirection = hitLocalPoint - p;
				raycastInfo.worldNormal = normalDirection;

				return true;
			}

			return false;
		}
		else if (mDotD > dDotD) {   // If the ray int32_tersects with the "q" endcap of the cylinder

			// Check int32_tersection between the ray and the "q" sphere endcap of the capsule
			vec3 hitLocalPoint;
			float hitFraction;
			if (raycastWithSphereEndCap(ray.point1, ray.point2, q, ray.maxFraction, hitLocalPoint, hitFraction)) {
				raycastInfo.body = proxyShape->getBody();
				raycastInfo.proxyShape = proxyShape;
				raycastInfo.hitFraction = hitFraction;
				raycastInfo.worldPoint = hitLocalPoint;
				vec3 normalDirection = hitLocalPoint - q;
				raycastInfo.worldNormal = normalDirection;

				return true;
			}

			return false;
		}
		else {  // If the origin is inside the cylinder, we return no hit
			return false;
		}
	}
	float b = dDotD * mDotN - nDotD * mDotD;
	float discriminant = b * b - a * c;

	// If the discriminant is negative, no real roots and therfore, no hit
	if (discriminant < 0.0f) return false;

	// Compute the smallest root (first int32_tersection along the ray)
	float t0 = t = (-b - etk::sqrt(discriminant)) / a;

	// If the int32_tersection is outside the finite cylinder of the capsule on "p" endcap side
	float value = mDotD + t * nDotD;
	if (value < 0.0f) {

		// Check int32_tersection between the ray and the "p" sphere endcap of the capsule
		vec3 hitLocalPoint;
		float hitFraction;
		if (raycastWithSphereEndCap(ray.point1, ray.point2, p, ray.maxFraction, hitLocalPoint, hitFraction)) {
			raycastInfo.body = proxyShape->getBody();
			raycastInfo.proxyShape = proxyShape;
			raycastInfo.hitFraction = hitFraction;
			raycastInfo.worldPoint = hitLocalPoint;
			vec3 normalDirection = hitLocalPoint - p;
			raycastInfo.worldNormal = normalDirection;

			return true;
		}

		return false;
	}
	else if (value > dDotD) {  // If the int32_tersection is outside the finite cylinder on the "q" side

		// Check int32_tersection between the ray and the "q" sphere endcap of the capsule
		vec3 hitLocalPoint;
		float hitFraction;
		if (raycastWithSphereEndCap(ray.point1, ray.point2, q, ray.maxFraction, hitLocalPoint, hitFraction)) {
			raycastInfo.body = proxyShape->getBody();
			raycastInfo.proxyShape = proxyShape;
			raycastInfo.hitFraction = hitFraction;
			raycastInfo.worldPoint = hitLocalPoint;
			vec3 normalDirection = hitLocalPoint - q;
			raycastInfo.worldNormal = normalDirection;

			return true;
		}

		return false;
	}

	t = t0;

	// If the int32_tersection is behind the origin of the ray or beyond the maximum
	// raycasting distance, we return no hit
	if (t < 0.0f || t > ray.maxFraction) return false;

	// Compute the hit information
	vec3 localHitPoint = ray.point1 + t * n;
	raycastInfo.body = proxyShape->getBody();
	raycastInfo.proxyShape = proxyShape;
	raycastInfo.hitFraction = t;
	raycastInfo.worldPoint = localHitPoint;
	vec3 v = localHitPoint - p;
	vec3 w = (v.dot(d) / d.length2()) * d;
	vec3 normalDirection = (localHitPoint - (p + w)).safeNormalized();
	raycastInfo.worldNormal = normalDirection;

	return true;
}

// Raycasting method between a ray one of the two spheres end cap of the capsule
bool CapsuleShape::raycastWithSphereEndCap(const vec3& point1, const vec3& point2,
										   const vec3& sphereCenter, float maxFraction,
										   vec3& hitLocalPoint, float& hitFraction) const {

	 const vec3 m = point1 - sphereCenter;
	float c = m.dot(m) - m_margin * m_margin;

	// If the origin of the ray is inside the sphere, we return no int32_tersection
	if (c < 0.0f) return false;

	const vec3 rayDirection = point2 - point1;
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
	if (t < maxFraction * raySquareLength) {

		// Compute the int32_tersection information
		t /= raySquareLength;
		hitFraction = t;
		hitLocalPoint = point1 + t * rayDirection;

		return true;
	}

	return false;
}



// Get the radius of the capsule
/**
 * @return The radius of the capsule shape (in meters)
 */
float CapsuleShape::getRadius() const {
	return m_margin;
}

// Return the height of the capsule
/**
 * @return The height of the capsule shape (in meters)
 */
float CapsuleShape::getHeight() const {
	return m_halfHeight + m_halfHeight;
}

// Set the scaling vector of the collision shape
void CapsuleShape::setLocalScaling(const vec3& scaling) {

	m_halfHeight = (m_halfHeight / m_scaling.y()) * scaling.y();
	m_margin = (m_margin / m_scaling.x()) * scaling.x();

	CollisionShape::setLocalScaling(scaling);
}

// Return the number of bytes used by the collision shape
size_t CapsuleShape::getSizeInBytes() const {
	return sizeof(CapsuleShape);
}

// Return the local bounds of the shape in x, y and z directions
// This method is used to compute the AABB of the box
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
void CapsuleShape::getLocalBounds(vec3& min, vec3& max) const {

	// Maximum bounds
	max.setX(m_margin);
	max.setY(m_halfHeight + m_margin);
	max.setZ(m_margin);

	// Minimum bounds
	min.setX(-m_margin);
	min.setY(-max.y());
	min.setZ(min.x());
}

// Return a local support point in a given direction without the object margin.
/// A capsule is the convex hull of two spheres S1 and S2. The support point in the direction "d"
/// of the convex hull of a set of convex objects is the support point "p" in the set of all
/// support points from all the convex objects with the maximum dot product with the direction "d".
/// Therefore, in this method, we compute the support points of both top and bottom spheres of
/// the capsule and return the point with the maximum dot product with the direction vector. Note
/// that the object margin is implicitly the radius and height of the capsule.
vec3 CapsuleShape::getLocalSupportPointWithoutMargin(const vec3& direction,
														void** cachedCollisionData) const {

	// Support point top sphere
	float dotProductTop = m_halfHeight * direction.y();

	// Support point bottom sphere
	float dotProductBottom = -m_halfHeight * direction.y();

	// Return the point with the maximum dot product
	if (dotProductTop > dotProductBottom) {
		return vec3(0, m_halfHeight, 0);
	}
	else {
		return vec3(0, -m_halfHeight, 0);
	}
}