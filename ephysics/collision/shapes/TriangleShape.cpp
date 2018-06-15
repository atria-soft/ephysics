/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#include <ephysics/collision/shapes/TriangleShape.hpp>
#include <ephysics/collision/ProxyShape.hpp>
#include <ephysics/engine/Profiler.hpp>
#include <ephysics/configuration.hpp>

// TODO: REMOVE this...
using namespace ephysics;

TriangleShape::TriangleShape(const vec3& _point1, const vec3& _point2, const vec3& _point3, float _margin)
			  : ConvexShape(TRIANGLE, _margin) {
	m_points[0] = _point1;
	m_points[1] = _point2;
	m_points[2] = _point3;
	m_raycastTestType = FRONT;
}

bool TriangleShape::raycast(const Ray& _ray, RaycastInfo& _raycastInfo, ProxyShape* _proxyShape) const {
	PROFILE("TriangleShape::raycast()");
	const vec3 pq = _ray.point2 - _ray.point1;
	const vec3 pa = m_points[0] - _ray.point1;
	const vec3 pb = m_points[1] - _ray.point1;
	const vec3 pc = m_points[2] - _ray.point1;
	// Test if the line PQ is inside the eges BC, CA and AB. We use the triple
	// product for this test.
	const vec3 m = pq.cross(pc);
	float u = pb.dot(m);
	if (m_raycastTestType == FRONT) {
		if (u < 0.0f) {
			return false;
		}
	} else if (m_raycastTestType == BACK) {
		if (u > 0.0f) {
			return false;
		}
	}
	float v = -pa.dot(m);
	if (m_raycastTestType == FRONT) {
		if (v < 0.0f) {
			return false;
		}
	} else if (m_raycastTestType == BACK) {
		if (v > 0.0f) {
			return false;
		}
	} else if (m_raycastTestType == FRONT_AND_BACK) {
		if (!sameSign(u, v)) {
			return false;
		}
	}
	float w = pa.dot(pq.cross(pb));
	if (m_raycastTestType == FRONT) {
		if (w < 0.0f) {
			return false;
		}
	} else if (m_raycastTestType == BACK) {
		if (w > 0.0f) {
			return false;
		}
	} else if (m_raycastTestType == FRONT_AND_BACK) {
		if (!sameSign(u, w)) {
			return false;
		}
	}
	// If the line PQ is in the triangle plane (case where u=v=w=0)
	if (    approxEqual(u, 0)
	     && approxEqual(v, 0)
	     && approxEqual(w, 0)) {
		return false;
	}
	// Compute the barycentric coordinates (u, v, w) to determine the
	// int32_tersection point R, R = u * a + v * b + w * c
	float denom = 1.0f / (u + v + w);
	u *= denom;
	v *= denom;
	w *= denom;
	// Compute the local hit point using the barycentric coordinates
	const vec3 localHitPoint = u * m_points[0] + v * m_points[1] + w * m_points[2];
	const float hitFraction = (localHitPoint - _ray.point1).length() / pq.length();
	if (    hitFraction < 0.0f
	     || hitFraction > _ray.maxFraction) {
		return false;
	}
	vec3 localHitNormal = (m_points[1] - m_points[0]).cross(m_points[2] - m_points[0]);
	if (localHitNormal.dot(pq) > 0.0f) {
		localHitNormal = -localHitNormal;
	}
	_raycastInfo.body = _proxyShape->getBody();
	_raycastInfo.proxyShape = _proxyShape;
	_raycastInfo.worldPoint = localHitPoint;
	_raycastInfo.hitFraction = hitFraction;
	_raycastInfo.worldNormal = localHitNormal;
	return true;
}

size_t TriangleShape::getSizeInBytes() const {
	return sizeof(TriangleShape);
}

vec3 TriangleShape::getLocalSupportPointWithoutMargin(const vec3& _direction,
                                                      void** _cachedCollisionData) const {
	vec3 dotProducts(_direction.dot(m_points[0]), _direction.dot(m_points[1]), _direction.dot(m_points[2]));
	return m_points[dotProducts.getMaxAxis()];
}

void TriangleShape::getLocalBounds(vec3& _min, vec3& _max) const {
	const vec3 xAxis(m_points[0].x(), m_points[1].x(), m_points[2].x());
	const vec3 yAxis(m_points[0].y(), m_points[1].y(), m_points[2].y());
	const vec3 zAxis(m_points[0].z(), m_points[1].z(), m_points[2].z());
	_min.setValue(xAxis.getMin(), yAxis.getMin(), zAxis.getMin());
	_max.setValue(xAxis.getMax(), yAxis.getMax(), zAxis.getMax());
	_min -= vec3(m_margin, m_margin, m_margin);
	_max += vec3(m_margin, m_margin, m_margin);
}

void TriangleShape::setLocalScaling(const vec3& _scaling) {
	m_points[0] = (m_points[0] / m_scaling) * _scaling;
	m_points[1] = (m_points[1] / m_scaling) * _scaling;
	m_points[2] = (m_points[2] / m_scaling) * _scaling;
	CollisionShape::setLocalScaling(_scaling);
}

void TriangleShape::computeLocalInertiaTensor(etk::Matrix3x3& _tensor, float _mass) const {
	_tensor.setZero();
}

void TriangleShape::computeAABB(AABB& _aabb, const etk::Transform3D& _transform) const {
	const vec3 worldPoint1 = _transform * m_points[0];
	const vec3 worldPoint2 = _transform * m_points[1];
	const vec3 worldPoint3 = _transform * m_points[2];
	const vec3 xAxis(worldPoint1.x(), worldPoint2.x(), worldPoint3.x());
	const vec3 yAxis(worldPoint1.y(), worldPoint2.y(), worldPoint3.y());
	const vec3 zAxis(worldPoint1.z(), worldPoint2.z(), worldPoint3.z());
	_aabb.setMin(vec3(xAxis.getMin(), yAxis.getMin(), zAxis.getMin()));
	_aabb.setMax(vec3(xAxis.getMax(), yAxis.getMax(), zAxis.getMax()));
}

bool TriangleShape::testPointInside(const vec3& _localPoint, ProxyShape* _proxyShape) const {
	return false;
}

TriangleRaycastSide TriangleShape::getRaycastTestType() const {
	return m_raycastTestType;
}


void TriangleShape::setRaycastTestType(TriangleRaycastSide _testType) {
	m_raycastTestType = _testType;
}

vec3 TriangleShape::getVertex(int32_t _index) const {
	assert(    _index >= 0
	        && _index < 3);
	return m_points[_index];
}

