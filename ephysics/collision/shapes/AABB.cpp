/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */


// Libraries
#include <ephysics/collision/shapes/AABB.h>
#include <ephysics/configuration.h>
#include <cassert>

using namespace reactphysics3d;
using namespace std;

// Constructor
AABB::AABB() {

}

// Constructor
AABB::AABB(const Vector3& minCoordinates, const Vector3& maxCoordinates)
	 :mMinCoordinates(minCoordinates), mMaxCoordinates(maxCoordinates) {

}

// Copy-constructor
AABB::AABB(const AABB& aabb)
	 : mMinCoordinates(aabb.mMinCoordinates), mMaxCoordinates(aabb.mMaxCoordinates) {

}

// Destructor
AABB::~AABB() {

}

// Merge the AABB in parameter with the current one
void AABB::mergeWithAABB(const AABB& aabb) {
	mMinCoordinates.x = std::min(mMinCoordinates.x, aabb.mMinCoordinates.x);
	mMinCoordinates.y = std::min(mMinCoordinates.y, aabb.mMinCoordinates.y);
	mMinCoordinates.z = std::min(mMinCoordinates.z, aabb.mMinCoordinates.z);

	mMaxCoordinates.x = std::max(mMaxCoordinates.x, aabb.mMaxCoordinates.x);
	mMaxCoordinates.y = std::max(mMaxCoordinates.y, aabb.mMaxCoordinates.y);
	mMaxCoordinates.z = std::max(mMaxCoordinates.z, aabb.mMaxCoordinates.z);
}

// Replace the current AABB with a new AABB that is the union of two AABBs in parameters
void AABB::mergeTwoAABBs(const AABB& aabb1, const AABB& aabb2) {
	mMinCoordinates.x = std::min(aabb1.mMinCoordinates.x, aabb2.mMinCoordinates.x);
	mMinCoordinates.y = std::min(aabb1.mMinCoordinates.y, aabb2.mMinCoordinates.y);
	mMinCoordinates.z = std::min(aabb1.mMinCoordinates.z, aabb2.mMinCoordinates.z);

	mMaxCoordinates.x = std::max(aabb1.mMaxCoordinates.x, aabb2.mMaxCoordinates.x);
	mMaxCoordinates.y = std::max(aabb1.mMaxCoordinates.y, aabb2.mMaxCoordinates.y);
	mMaxCoordinates.z = std::max(aabb1.mMaxCoordinates.z, aabb2.mMaxCoordinates.z);
}

// Return true if the current AABB contains the AABB given in parameter
bool AABB::contains(const AABB& aabb) const {

	bool isInside = true;
	isInside = isInside && mMinCoordinates.x <= aabb.mMinCoordinates.x;
	isInside = isInside && mMinCoordinates.y <= aabb.mMinCoordinates.y;
	isInside = isInside && mMinCoordinates.z <= aabb.mMinCoordinates.z;

	isInside = isInside && mMaxCoordinates.x >= aabb.mMaxCoordinates.x;
	isInside = isInside && mMaxCoordinates.y >= aabb.mMaxCoordinates.y;
	isInside = isInside && mMaxCoordinates.z >= aabb.mMaxCoordinates.z;
	return isInside;
}

// Create and return an AABB for a triangle
AABB AABB::createAABBForTriangle(const Vector3* trianglePoints) {

	Vector3 minCoords(trianglePoints[0].x, trianglePoints[0].y, trianglePoints[0].z);
	Vector3 maxCoords(trianglePoints[0].x, trianglePoints[0].y, trianglePoints[0].z);

	if (trianglePoints[1].x < minCoords.x) minCoords.x = trianglePoints[1].x;
	if (trianglePoints[1].y < minCoords.y) minCoords.y = trianglePoints[1].y;
	if (trianglePoints[1].z < minCoords.z) minCoords.z = trianglePoints[1].z;

	if (trianglePoints[2].x < minCoords.x) minCoords.x = trianglePoints[2].x;
	if (trianglePoints[2].y < minCoords.y) minCoords.y = trianglePoints[2].y;
	if (trianglePoints[2].z < minCoords.z) minCoords.z = trianglePoints[2].z;

	if (trianglePoints[1].x > maxCoords.x) maxCoords.x = trianglePoints[1].x;
	if (trianglePoints[1].y > maxCoords.y) maxCoords.y = trianglePoints[1].y;
	if (trianglePoints[1].z > maxCoords.z) maxCoords.z = trianglePoints[1].z;

	if (trianglePoints[2].x > maxCoords.x) maxCoords.x = trianglePoints[2].x;
	if (trianglePoints[2].y > maxCoords.y) maxCoords.y = trianglePoints[2].y;
	if (trianglePoints[2].z > maxCoords.z) maxCoords.z = trianglePoints[2].z;

	return AABB(minCoords, maxCoords);
}

// Return true if the ray int32_tersects the AABB
/// This method use the line vs AABB raycasting technique described in
/// Real-time Collision Detection by Christer Ericson.
bool AABB::testRayIntersect(const Ray& ray) const {

	const Vector3 point2 = ray.point1 + ray.maxFraction * (ray.point2 - ray.point1);
	const Vector3 e = mMaxCoordinates - mMinCoordinates;
	const Vector3 d = point2 - ray.point1;
	const Vector3 m = ray.point1 + point2 - mMinCoordinates - mMaxCoordinates;

	// Test if the AABB face normals are separating axis
	float adx = std::abs(d.x);
	if (std::abs(m.x) > e.x + adx) return false;
	float ady = std::abs(d.y);
	if (std::abs(m.y) > e.y + ady) return false;
	float adz = std::abs(d.z);
	if (std::abs(m.z) > e.z + adz) return false;

	// Add in an epsilon term to counteract arithmetic errors when segment is
	// (near) parallel to a coordinate axis (see text for detail)
	const float epsilon = 0.00001;
	adx += epsilon;
	ady += epsilon;
	adz += epsilon;

	// Test if the cross products between face normals and ray direction are
	// separating axis
	if (std::abs(m.y * d.z - m.z * d.y) > e.y * adz + e.z * ady) return false;
	if (std::abs(m.z * d.x - m.x * d.z) > e.x * adz + e.z * adx) return false;
	if (std::abs(m.x * d.y - m.y * d.x) > e.x * ady + e.y * adx) return false;

	// No separating axis has been found
	return true;
}
