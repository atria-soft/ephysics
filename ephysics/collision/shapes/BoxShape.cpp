/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
// Libraries
#include <ephysics/collision/shapes/BoxShape.hpp>
#include <ephysics/collision/ProxyShape.hpp>
#include <ephysics/configuration.hpp>
#include <etk/Vector.hpp>

using namespace ephysics;

BoxShape::BoxShape(const vec3& _extent, float _margin):
  ConvexShape(BOX, _margin),
  m_extent(_extent - vec3(_margin, _margin, _margin)) {
	assert(_extent.x() > 0.0f && _extent.x() > _margin);
	assert(_extent.y() > 0.0f && _extent.y() > _margin);
	assert(_extent.z() > 0.0f && _extent.z() > _margin);
}

void BoxShape::computeLocalInertiaTensor(etk::Matrix3x3& _tensor, float _mass) const {
	float factor = (1.0f / float(3.0)) * _mass;
	vec3 realExtent = m_extent + vec3(m_margin, m_margin, m_margin);
	float xSquare = realExtent.x() * realExtent.x();
	float ySquare = realExtent.y() * realExtent.y();
	float zSquare = realExtent.z() * realExtent.z();
	_tensor.setValue(factor * (ySquare + zSquare), 0.0, 0.0,
	                 0.0, factor * (xSquare + zSquare), 0.0,
	                 0.0, 0.0, factor * (xSquare + ySquare));
}

bool BoxShape::raycast(const Ray& _ray, RaycastInfo& _raycastInfo, ProxyShape* _proxyShape) const {
	vec3 rayDirection = _ray.point2 - _ray.point1;
	float tMin = FLT_MIN;
	float tMax = FLT_MAX;
	vec3 normalDirection(0,0,0);
	vec3 currentNormal(0,0,0);
	// For each of the three slabs
	for (int32_t iii=0; iii<3; ++iii) {
		// If ray is parallel to the slab
		if (etk::abs(rayDirection[iii]) < FLT_EPSILON) {
			// If the ray's origin is not inside the slab, there is no hit
			if (_ray.point1[iii] > m_extent[iii] || _ray.point1[iii] < -m_extent[iii]) {
				return false;
			}
		} else {
			// Compute the intersection of the ray with the near and far plane of the slab
			float oneOverD = 1.0f / rayDirection[iii];
			float t1 = (-m_extent[iii] - _ray.point1[iii]) * oneOverD;
			float t2 = (m_extent[iii] - _ray.point1[iii]) * oneOverD;
			currentNormal[0] = (iii == 0) ? -m_extent[iii] : 0.0f;
			currentNormal[1] = (iii == 1) ? -m_extent[iii] : 0.0f;
			currentNormal[2] = (iii == 2) ? -m_extent[iii] : 0.0f;
			// Swap t1 and t2 if need so that t1 is intersection with near plane and
			// t2 with far plane
			if (t1 > t2) {
				etk::swap(t1, t2);
				currentNormal = -currentNormal;
			}
			// Compute the intersection of the of slab intersection interval with previous slabs
			if (t1 > tMin) {
				tMin = t1;
				normalDirection = currentNormal;
			}
			tMax = etk::min(tMax, t2);
			// If tMin is larger than the maximum raycasting fraction, we return no hit
			if (tMin > _ray.maxFraction) {
				return false;
			}
			// If the slabs intersection is empty, there is no hit
			if (tMin > tMax) {
				return false;
			}
		}
	}
	// If tMin is negative, we return no hit
	if (    tMin < 0.0f
	     || tMin > _ray.maxFraction) {
		return false;
	}
	if (normalDirection == vec3(0,0,0)) {
		return false;
	}
	// The ray int32_tersects the three slabs, we compute the hit point
	vec3 localHitPoint = _ray.point1 + tMin * rayDirection;
	_raycastInfo.body = _proxyShape->getBody();
	_raycastInfo.proxyShape = _proxyShape;
	_raycastInfo.hitFraction = tMin;
	_raycastInfo.worldPoint = localHitPoint;
	_raycastInfo.worldNormal = normalDirection;
	return true;
}

vec3 BoxShape::getExtent() const {
	return m_extent + vec3(m_margin, m_margin, m_margin);
}

void BoxShape::setLocalScaling(const vec3& _scaling) {
	m_extent = (m_extent / m_scaling) * _scaling;
	CollisionShape::setLocalScaling(_scaling);
}

void BoxShape::getLocalBounds(vec3& _min, vec3& _max) const {
	// Maximum bounds
	_max = m_extent + vec3(m_margin, m_margin, m_margin);
	// Minimum bounds
	_min = -_max;
}

size_t BoxShape::getSizeInBytes() const {
	return sizeof(BoxShape);
}

vec3 BoxShape::getLocalSupportPointWithoutMargin(const vec3& _direction,
                                                 void** _cachedCollisionData) const {
	return vec3(_direction.x() < 0.0 ? -m_extent.x() : m_extent.x(),
	            _direction.y() < 0.0 ? -m_extent.y() : m_extent.y(),
	            _direction.z() < 0.0 ? -m_extent.z() : m_extent.z());
}

bool BoxShape::testPointInside(const vec3& _localPoint, ProxyShape* _proxyShape) const {
	return (    _localPoint.x() < m_extent[0]
	         && _localPoint.x() > -m_extent[0]
	         && _localPoint.y() < m_extent[1]
	         && _localPoint.y() > -m_extent[1]
	         && _localPoint.z() < m_extent[2]
	         && _localPoint.z() > -m_extent[2]);
}

