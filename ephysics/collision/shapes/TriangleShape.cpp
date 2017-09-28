/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/shapes/TriangleShape.hpp>
#include <ephysics/collision/ProxyShape.hpp>
#include <ephysics/engine/Profiler.hpp>
#include <ephysics/configuration.hpp>

using namespace ephysics;

// Constructor
/**
 * @param point1 First point of the triangle
 * @param point2 Second point of the triangle
 * @param point3 Third point of the triangle
 * @param margin The collision margin (in meters) around the collision shape
 */
TriangleShape::TriangleShape(const vec3& point1, const vec3& point2, const vec3& point3, float margin)
			  : ConvexShape(TRIANGLE, margin) {
	m_points[0] = point1;
	m_points[1] = point2;
	m_points[2] = point3;
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
	const vec3 pa = m_points[0] - ray.point1;
	const vec3 pb = m_points[1] - ray.point1;
	const vec3 pc = m_points[2] - ray.point1;

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
	const vec3 localHitPoint = u * m_points[0] + v * m_points[1] + w * m_points[2];
	const float hitFraction = (localHitPoint - ray.point1).length() / pq.length();

	if (hitFraction < 0.0f || hitFraction > ray.maxFraction) return false;

	vec3 localHitNormal = (m_points[1] - m_points[0]).cross(m_points[2] - m_points[0]);
	if (localHitNormal.dot(pq) > 0.0f) localHitNormal = -localHitNormal;

	raycastInfo.body = proxyShape->getBody();
	raycastInfo.proxyShape = proxyShape;
	raycastInfo.worldPoint = localHitPoint;
	raycastInfo.hitFraction = hitFraction;
	raycastInfo.worldNormal = localHitNormal;

	return true;
}


// Return the number of bytes used by the collision shape
size_t TriangleShape::getSizeInBytes() const {
	return sizeof(TriangleShape);
}

// Return a local support point in a given direction without the object margin
vec3 TriangleShape::getLocalSupportPointWithoutMargin(const vec3& direction,
															  void** cachedCollisionData) const {
	vec3 dotProducts(direction.dot(m_points[0]), direction.dot(m_points[1]), direction.dot(m_points[2]));
	return m_points[dotProducts.getMaxAxis()];
}

// Return the local bounds of the shape in x, y and z directions.
// This method is used to compute the AABB of the box
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
void TriangleShape::getLocalBounds(vec3& min, vec3& max) const {

	const vec3 xAxis(m_points[0].x(), m_points[1].x(), m_points[2].x());
	const vec3 yAxis(m_points[0].y(), m_points[1].y(), m_points[2].y());
	const vec3 zAxis(m_points[0].z(), m_points[1].z(), m_points[2].z());
	min.setValue(xAxis.getMin(), yAxis.getMin(), zAxis.getMin());
	max.setValue(xAxis.getMax(), yAxis.getMax(), zAxis.getMax());

	min -= vec3(m_margin, m_margin, m_margin);
	max += vec3(m_margin, m_margin, m_margin);
}

// Set the local scaling vector of the collision shape
void TriangleShape::setLocalScaling(const vec3& scaling) {

	m_points[0] = (m_points[0] / m_scaling) * scaling;
	m_points[1] = (m_points[1] / m_scaling) * scaling;
	m_points[2] = (m_points[2] / m_scaling) * scaling;

	CollisionShape::setLocalScaling(scaling);
}

// Return the local inertia tensor of the triangle shape
/**
 * @param[out] tensor The 3x3 inertia tensor matrix of the shape in local-space
 *					coordinates
 * @param mass Mass to use to compute the inertia tensor of the collision shape
 */
void TriangleShape::computeLocalInertiaTensor(etk::Matrix3x3& tensor, float mass) const {
	tensor.setZero();
}

// Update the AABB of a body using its collision shape
/**
 * @param[out] aabb The axis-aligned bounding box (AABB) of the collision shape
 *				  computed in world-space coordinates
 * @param transform etk::Transform3D used to compute the AABB of the collision shape
 */
void TriangleShape::computeAABB(AABB& aabb, const etk::Transform3D& transform) const {

	const vec3 worldPoint1 = transform * m_points[0];
	const vec3 worldPoint2 = transform * m_points[1];
	const vec3 worldPoint3 = transform * m_points[2];

	const vec3 xAxis(worldPoint1.x(), worldPoint2.x(), worldPoint3.x());
	const vec3 yAxis(worldPoint1.y(), worldPoint2.y(), worldPoint3.y());
	const vec3 zAxis(worldPoint1.z(), worldPoint2.z(), worldPoint3.z());
	aabb.setMin(vec3(xAxis.getMin(), yAxis.getMin(), zAxis.getMin()));
	aabb.setMax(vec3(xAxis.getMax(), yAxis.getMax(), zAxis.getMax()));
}

// Return true if a point is inside the collision shape
bool TriangleShape::testPointInside(const vec3& localPoint, ProxyShape* proxyShape) const {
	return false;
}

// Return the raycast test type (front, back, front-back)
TriangleRaycastSide TriangleShape::getRaycastTestType() const {
	return m_raycastTestType;
}

// Set the raycast test type (front, back, front-back)
/**
 * @param testType Raycast test type for the triangle (front, back, front-back)
 */
void TriangleShape::setRaycastTestType(TriangleRaycastSide testType) {
	m_raycastTestType = testType;
}

// Return the coordinates of a given vertex of the triangle
/**
 * @param index Index (0 to 2) of a vertex of the triangle
 */
vec3 TriangleShape::getVertex(int32_t index) const {
	assert(index >= 0 && index < 3);
	return m_points[index];
}