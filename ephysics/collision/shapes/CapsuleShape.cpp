/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/shapes/CapsuleShape.h>
#include <ephysics/collision/ProxyShape.h>
#include <ephysics/configuration.h>
#include <cassert>

using namespace reactphysics3d;

// Constructor
/**
 * @param radius The radius of the capsule (in meters)
 * @param height The height of the capsule (in meters)
 */
CapsuleShape::CapsuleShape(float radius, float height)
			: ConvexShape(CAPSULE, radius), mHalfHeight(height * float(0.5)) {
	assert(radius > float(0.0));
	assert(height > float(0.0));
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
void CapsuleShape::computeLocalInertiaTensor(Matrix3x3& tensor, float mass) const {

	// The inertia tensor formula for a capsule can be found in : Game Engine Gems, Volume 1
	
	float height = mHalfHeight + mHalfHeight;
	float radiusSquare = mMargin * mMargin;
	float heightSquare = height * height;
	float radiusSquareDouble = radiusSquare + radiusSquare;
	float factor1 = float(2.0) * mMargin / (float(4.0) * mMargin + float(3.0) * height);
	float factor2 = float(3.0) * height / (float(4.0) * mMargin + float(3.0) * height);
	float sum1 = float(0.4) * radiusSquareDouble;
	float sum2 = float(0.75) * height * mMargin + float(0.5) * heightSquare;
	float sum3 = float(0.25) * radiusSquare + float(1.0 / 12.0) * heightSquare;
	float IxxAndzz = factor1 * mass * (sum1 + sum2) + factor2 * mass * sum3;
	float Iyy = factor1 * mass * sum1 + factor2 * mass * float(0.25) * radiusSquareDouble;
	tensor.setAllValues(IxxAndzz, 0.0, 0.0,
						0.0, Iyy, 0.0,
						0.0, 0.0, IxxAndzz);
}

// Return true if a point is inside the collision shape
bool CapsuleShape::testPointInside(const Vector3& localPoint, ProxyShape* proxyShape) const {

	const float diffYCenterSphere1 = localPoint.y - mHalfHeight;
	const float diffYCenterSphere2 = localPoint.y + mHalfHeight;
	const float xSquare = localPoint.x * localPoint.x;
	const float zSquare = localPoint.z * localPoint.z;
	const float squareRadius = mMargin * mMargin;

	// Return true if the point is inside the cylinder or one of the two spheres of the capsule
	return ((xSquare + zSquare) < squareRadius &&
			localPoint.y < mHalfHeight && localPoint.y > -mHalfHeight) ||
			(xSquare + zSquare + diffYCenterSphere1 * diffYCenterSphere1) < squareRadius ||
			(xSquare + zSquare + diffYCenterSphere2 * diffYCenterSphere2) < squareRadius;
}

// Raycast method with feedback information
bool CapsuleShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const {

	const Vector3 n = ray.point2 - ray.point1;

	const float epsilon = float(0.01);
	Vector3 p(float(0), -mHalfHeight, float(0));
	Vector3 q(float(0), mHalfHeight, float(0));
	Vector3 d = q - p;
	Vector3 m = ray.point1 - p;
	float t;

	float mDotD = m.dot(d);
	float nDotD = n.dot(d);
	float dDotD = d.dot(d);

	// Test if the segment is outside the cylinder
	float vec1DotD = (ray.point1 - Vector3(float(0.0), -mHalfHeight - mMargin, float(0.0))).dot(d);
	if (vec1DotD < float(0.0) && vec1DotD + nDotD < float(0.0)) return false;
	float ddotDExtraCaps = float(2.0) * mMargin * d.y;
	if (vec1DotD > dDotD + ddotDExtraCaps && vec1DotD + nDotD > dDotD + ddotDExtraCaps) return false;

	float nDotN = n.dot(n);
	float mDotN = m.dot(n);

	float a = dDotD * nDotN - nDotD * nDotD;
	float k = m.dot(m) - mMargin * mMargin;
	float c = dDotD * k - mDotD * mDotD;

	// If the ray is parallel to the capsule axis
	if (std::abs(a) < epsilon) {

		// If the origin is outside the surface of the capusle's cylinder, we return no hit
		if (c > float(0.0)) return false;

		// Here we know that the segment int32_tersect an endcap of the capsule

		// If the ray int32_tersects with the "p" endcap of the capsule
		if (mDotD < float(0.0)) {

			// Check int32_tersection between the ray and the "p" sphere endcap of the capsule
			Vector3 hitLocalPoint;
			float hitFraction;
			if (raycastWithSphereEndCap(ray.point1, ray.point2, p, ray.maxFraction, hitLocalPoint, hitFraction)) {
				raycastInfo.body = proxyShape->getBody();
				raycastInfo.proxyShape = proxyShape;
				raycastInfo.hitFraction = hitFraction;
				raycastInfo.worldPoint = hitLocalPoint;
				Vector3 normalDirection = hitLocalPoint - p;
				raycastInfo.worldNormal = normalDirection;

				return true;
			}

			return false;
		}
		else if (mDotD > dDotD) {   // If the ray int32_tersects with the "q" endcap of the cylinder

			// Check int32_tersection between the ray and the "q" sphere endcap of the capsule
			Vector3 hitLocalPoint;
			float hitFraction;
			if (raycastWithSphereEndCap(ray.point1, ray.point2, q, ray.maxFraction, hitLocalPoint, hitFraction)) {
				raycastInfo.body = proxyShape->getBody();
				raycastInfo.proxyShape = proxyShape;
				raycastInfo.hitFraction = hitFraction;
				raycastInfo.worldPoint = hitLocalPoint;
				Vector3 normalDirection = hitLocalPoint - q;
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
	if (discriminant < float(0.0)) return false;

	// Compute the smallest root (first int32_tersection along the ray)
	float t0 = t = (-b - std::sqrt(discriminant)) / a;

	// If the int32_tersection is outside the finite cylinder of the capsule on "p" endcap side
	float value = mDotD + t * nDotD;
	if (value < float(0.0)) {

		// Check int32_tersection between the ray and the "p" sphere endcap of the capsule
		Vector3 hitLocalPoint;
		float hitFraction;
		if (raycastWithSphereEndCap(ray.point1, ray.point2, p, ray.maxFraction, hitLocalPoint, hitFraction)) {
			raycastInfo.body = proxyShape->getBody();
			raycastInfo.proxyShape = proxyShape;
			raycastInfo.hitFraction = hitFraction;
			raycastInfo.worldPoint = hitLocalPoint;
			Vector3 normalDirection = hitLocalPoint - p;
			raycastInfo.worldNormal = normalDirection;

			return true;
		}

		return false;
	}
	else if (value > dDotD) {  // If the int32_tersection is outside the finite cylinder on the "q" side

		// Check int32_tersection between the ray and the "q" sphere endcap of the capsule
		Vector3 hitLocalPoint;
		float hitFraction;
		if (raycastWithSphereEndCap(ray.point1, ray.point2, q, ray.maxFraction, hitLocalPoint, hitFraction)) {
			raycastInfo.body = proxyShape->getBody();
			raycastInfo.proxyShape = proxyShape;
			raycastInfo.hitFraction = hitFraction;
			raycastInfo.worldPoint = hitLocalPoint;
			Vector3 normalDirection = hitLocalPoint - q;
			raycastInfo.worldNormal = normalDirection;

			return true;
		}

		return false;
	}

	t = t0;

	// If the int32_tersection is behind the origin of the ray or beyond the maximum
	// raycasting distance, we return no hit
	if (t < float(0.0) || t > ray.maxFraction) return false;

	// Compute the hit information
	Vector3 localHitPoint = ray.point1 + t * n;
	raycastInfo.body = proxyShape->getBody();
	raycastInfo.proxyShape = proxyShape;
	raycastInfo.hitFraction = t;
	raycastInfo.worldPoint = localHitPoint;
	Vector3 v = localHitPoint - p;
	Vector3 w = (v.dot(d) / d.lengthSquare()) * d;
	Vector3 normalDirection = (localHitPoint - (p + w)).getUnit();
	raycastInfo.worldNormal = normalDirection;

	return true;
}

// Raycasting method between a ray one of the two spheres end cap of the capsule
bool CapsuleShape::raycastWithSphereEndCap(const Vector3& point1, const Vector3& point2,
										   const Vector3& sphereCenter, float maxFraction,
										   Vector3& hitLocalPoint, float& hitFraction) const {

	 const Vector3 m = point1 - sphereCenter;
	float c = m.dot(m) - mMargin * mMargin;

	// If the origin of the ray is inside the sphere, we return no int32_tersection
	if (c < float(0.0)) return false;

	const Vector3 rayDirection = point2 - point1;
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
	if (t < maxFraction * raySquareLength) {

		// Compute the int32_tersection information
		t /= raySquareLength;
		hitFraction = t;
		hitLocalPoint = point1 + t * rayDirection;

		return true;
	}

	return false;
}
