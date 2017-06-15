/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/shapes/TriangleShape.h>
#include <ephysics/collision/ProxyShape.h>
#include <ephysics/engine/Profiler.h>
#include <ephysics/configuration.h>
#include <cassert>

using namespace reactphysics3d;

// Constructor
/**
 * @param point1 First point of the triangle
 * @param point2 Second point of the triangle
 * @param point3 Third point of the triangle
 * @param margin The collision margin (in meters) around the collision shape
 */
TriangleShape::TriangleShape(const vec3& point1, const vec3& point2, const vec3& point3, float margin)
			  : ConvexShape(TRIANGLE, margin) {
	mPoints[0] = point1;
	mPoints[1] = point2;
	mPoints[2] = point3;
	m_raycastTestType = FRONT;
}

// Destructor
TriangleShape::~TriangleShape() {

}

// Raycast method with feedback information
/// This method use the line vs triangle raycasting technique described in
/// Real-time Collision Detection by Christer Ericson.
bool TriangleShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const {

	PROFILE("TriangleShape::raycast()");

	const vec3 pq = ray.point2 - ray.point1;
	const vec3 pa = mPoints[0] - ray.point1;
	const vec3 pb = mPoints[1] - ray.point1;
	const vec3 pc = mPoints[2] - ray.point1;

	// Test if the line PQ is inside the eges BC, CA and AB. We use the triple
	// product for this test.
	const vec3 m = pq.cross(pc);
	float u = pb.dot(m);
	if (m_raycastTestType == FRONT) {
		if (u < 0.0f) return false;
	}
	else if (m_raycastTestType == BACK) {
		if (u > 0.0f) return false;
	}

	float v = -pa.dot(m);
	if (m_raycastTestType == FRONT) {
		if (v < 0.0f) return false;
	}
	else if (m_raycastTestType == BACK) {
		if (v > 0.0f) return false;
	}
	else if (m_raycastTestType == FRONT_AND_BACK) {
		if (!sameSign(u, v)) return false;
	}

	float w = pa.dot(pq.cross(pb));
	if (m_raycastTestType == FRONT) {
		if (w < 0.0f) return false;
	}
	else if (m_raycastTestType == BACK) {
		if (w > 0.0f) return false;
	}
	else if (m_raycastTestType == FRONT_AND_BACK) {
		if (!sameSign(u, w)) return false;
	}

	// If the line PQ is in the triangle plane (case where u=v=w=0)
	if (approxEqual(u, 0) && approxEqual(v, 0) && approxEqual(w, 0)) return false;

	// Compute the barycentric coordinates (u, v, w) to determine the
	// int32_tersection point R, R = u * a + v * b + w * c
	float denom = 1.0f / (u + v + w);
	u *= denom;
	v *= denom;
	w *= denom;

	// Compute the local hit point using the barycentric coordinates
	const vec3 localHitPoint = u * mPoints[0] + v * mPoints[1] + w * mPoints[2];
	const float hitFraction = (localHitPoint - ray.point1).length() / pq.length();

	if (hitFraction < 0.0f || hitFraction > ray.maxFraction) return false;

	vec3 localHitNormal = (mPoints[1] - mPoints[0]).cross(mPoints[2] - mPoints[0]);
	if (localHitNormal.dot(pq) > 0.0f) localHitNormal = -localHitNormal;

	raycastInfo.body = proxyShape->getBody();
	raycastInfo.proxyShape = proxyShape;
	raycastInfo.worldPoint = localHitPoint;
	raycastInfo.hitFraction = hitFraction;
	raycastInfo.worldNormal = localHitNormal;

	return true;
}

