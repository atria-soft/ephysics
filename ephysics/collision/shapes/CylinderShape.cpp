/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#include <ephysics/collision/shapes/CylinderShape.hpp>
#include <ephysics/collision/ProxyShape.hpp>
#include <ephysics/configuration.hpp>

using namespace ephysics;

CylinderShape::CylinderShape(float _radius,
                             float _height,
                             float _margin):
  ConvexShape(CYLINDER, _margin), m_radius(_radius), m_halfHeight(_height/float(2.0)) {
	assert(_radius > 0.0f);
	assert(_height > 0.0f);
}

vec3 CylinderShape::getLocalSupportPointWithoutMargin(const vec3& _direction,
                                                      void** _cachedCollisionData) const {
	vec3 supportPoint(0.0, 0.0, 0.0);
	float uDotv = _direction.y();
	vec3 w(_direction.x(), 0.0, _direction.z());
	float lengthW = sqrt(_direction.x() * _direction.x() + _direction.z() * _direction.z());
	if (lengthW > FLT_EPSILON) {
		if (uDotv < 0.0) {
			supportPoint.setY(-m_halfHeight);
		} else {
			supportPoint.setY(m_halfHeight);
		}
		supportPoint += (m_radius / lengthW) * w;
	} else {
		if (uDotv < 0.0) {
			supportPoint.setY(-m_halfHeight);
		} else {
			supportPoint.setY(m_halfHeight);
		}
	}
	return supportPoint;
}

bool CylinderShape::raycast(const Ray& _ray, RaycastInfo& _raycastInfo, ProxyShape* _proxyShape) const {
	const vec3 n = _ray.point2 - _ray.point1;
	const float epsilon = float(0.01);
	vec3 p(float(0), -m_halfHeight, float(0));
	vec3 q(float(0), m_halfHeight, float(0));
	vec3 d = q - p;
	vec3 m = _ray.point1 - p;
	float t;
	float mDotD = m.dot(d);
	float nDotD = n.dot(d);
	float dDotD = d.dot(d);
	// Test if the segment is outside the cylinder
	if (mDotD < 0.0f && mDotD + nDotD < float(0.0)) {
		return false;
	}
	if (mDotD > dDotD && mDotD + nDotD > dDotD) {
		return false;
	}
	float nDotN = n.dot(n);
	float mDotN = m.dot(n);
	float a = dDotD * nDotN - nDotD * nDotD;
	float k = m.dot(m) - m_radius * m_radius;
	float c = dDotD * k - mDotD * mDotD;
	// If the ray is parallel to the cylinder axis
	if (etk::abs(a) < epsilon) {
		// If the origin is outside the surface of the cylinder, we return no hit
		if (c > 0.0f) {
			return false;
		}
		// Here we know that the segment int32_tersect an endcap of the cylinder
		// If the ray int32_tersects with the "p" endcap of the cylinder
		if (mDotD < 0.0f) {
			t = -mDotN / nDotN;
			// If the int32_tersection is behind the origin of the ray or beyond the maximum
			// raycasting distance, we return no hit
			if (t < 0.0f || t > _ray.maxFraction) {
				return false;
			}
			// Compute the hit information
			vec3 localHitPoint = _ray.point1 + t * n;
			_raycastInfo.body = _proxyShape->getBody();
			_raycastInfo.proxyShape = _proxyShape;
			_raycastInfo.hitFraction = t;
			_raycastInfo.worldPoint = localHitPoint;
			vec3 normalDirection(0, float(-1), 0);
			_raycastInfo.worldNormal = normalDirection;
			return true;
		}
		// If the ray int32_tersects with the "q" endcap of the cylinder
		if (mDotD > dDotD) {
			t = (nDotD - mDotN) / nDotN;
			// If the int32_tersection is behind the origin of the ray or beyond the maximum
			// raycasting distance, we return no hit
			if (t < 0.0f || t > _ray.maxFraction) {
				return false;
			}
			// Compute the hit information
			vec3 localHitPoint = _ray.point1 + t * n;
			_raycastInfo.body = _proxyShape->getBody();
			_raycastInfo.proxyShape = _proxyShape;
			_raycastInfo.hitFraction = t;
			_raycastInfo.worldPoint = localHitPoint;
			vec3 normalDirection(0, 1.0f, 0);
			_raycastInfo.worldNormal = normalDirection;
			return true;
		}
		// If the origin is inside the cylinder, we return no hit
		return false;
	}
	float b = dDotD * mDotN - nDotD * mDotD;
	float discriminant = b * b - a * c;
	// If the discriminant is negative, no real roots and therfore, no hit
	if (discriminant < 0.0f) {
		return false;
	}
	// Compute the smallest root (first int32_tersection along the ray)
	float t0 = t = (-b - etk::sqrt(discriminant)) / a;
	// If the int32_tersection is outside the cylinder on "p" endcap side
	float value = mDotD + t * nDotD;
	if (value < 0.0f) {
		// If the ray is pointing away from the "p" endcap, we return no hit
		if (nDotD <= 0.0f) {
			return false;
		}
		// Compute the int32_tersection against the "p" endcap (int32_tersection agains whole plane)
		t = -mDotD / nDotD;
		// Keep the int32_tersection if the it is inside the cylinder radius
		if (k + t * (float(2.0) * mDotN + t) > 0.0f) {
			return false;
		}
		// If the int32_tersection is behind the origin of the ray or beyond the maximum
		// raycasting distance, we return no hit
		if (t < 0.0f || t > _ray.maxFraction) {
			return false;
		}
		// Compute the hit information
		vec3 localHitPoint = _ray.point1 + t * n;
		_raycastInfo.body = _proxyShape->getBody();
		_raycastInfo.proxyShape = _proxyShape;
		_raycastInfo.hitFraction = t;
		_raycastInfo.worldPoint = localHitPoint;
		vec3 normalDirection(0, float(-1.0), 0);
		_raycastInfo.worldNormal = normalDirection;
		return true;
	}
	// If the int32_tersection is outside the cylinder on the "q" side
	if (value > dDotD) {
		// If the ray is pointing away from the "q" endcap, we return no hit
		if (nDotD >= 0.0f) {
			return false;
		}
		// Compute the int32_tersection against the "q" endcap (int32_tersection against whole plane)
		t = (dDotD - mDotD) / nDotD;
		// Keep the int32_tersection if it is inside the cylinder radius
		if (k + dDotD - float(2.0) * mDotD + t * (float(2.0) * (mDotN - nDotD) + t) > 0.0f) {
			return false;
		}
		// If the int32_tersection is behind the origin of the ray or beyond the maximum
		// raycasting distance, we return no hit
		if (t < 0.0f || t > _ray.maxFraction) {
			return false;
		}
		// Compute the hit information
		vec3 localHitPoint = _ray.point1 + t * n;
		_raycastInfo.body = _proxyShape->getBody();
		_raycastInfo.proxyShape = _proxyShape;
		_raycastInfo.hitFraction = t;
		_raycastInfo.worldPoint = localHitPoint;
		vec3 normalDirection(0, 1.0f, 0);
		_raycastInfo.worldNormal = normalDirection;
		return true;
	}
	t = t0;
	// If the int32_tersection is behind the origin of the ray or beyond the maximum
	// raycasting distance, we return no hit
	if (t < 0.0f || t > _ray.maxFraction) {
		return false;
	}
	// Compute the hit information
	vec3 localHitPoint = _ray.point1 + t * n;
	_raycastInfo.body = _proxyShape->getBody();
	_raycastInfo.proxyShape = _proxyShape;
	_raycastInfo.hitFraction = t;
	_raycastInfo.worldPoint = localHitPoint;
	vec3 v = localHitPoint - p;
	vec3 w = (v.dot(d) / d.length2()) * d;
	vec3 normalDirection = (localHitPoint - (p + w));
	_raycastInfo.worldNormal = normalDirection;
	return true;
}

float CylinderShape::getRadius() const {
	return m_radius;
}

float CylinderShape::getHeight() const {
	return m_halfHeight + m_halfHeight;
}

void CylinderShape::setLocalScaling(const vec3& _scaling) {
	m_halfHeight = (m_halfHeight / m_scaling.y()) * _scaling.y();
	m_radius = (m_radius / m_scaling.x()) * _scaling.x();
	CollisionShape::setLocalScaling(_scaling);
}

size_t CylinderShape::getSizeInBytes() const {
	return sizeof(CylinderShape);
}

void CylinderShape::getLocalBounds(vec3& _min, vec3& _max) const {
	// Maximum bounds
	_max.setX(m_radius + m_margin);
	_max.setY(m_halfHeight + m_margin);
	_max.setZ(_max.x());
	// Minimum bounds
	_min.setX(-_max.x());
	_min.setY(-_max.y());
	_min.setZ(_min.x());
}

void CylinderShape::computeLocalInertiaTensor(etk::Matrix3x3& _tensor, float _mass) const {
	float height = float(2.0) * m_halfHeight;
	float diag = (1.0f / float(12.0)) * _mass * (3 * m_radius * m_radius + height * height);
	_tensor.setValue(diag, 0.0, 0.0, 0.0,
	                 0.5f * _mass * m_radius * m_radius, 0.0,
	                 0.0, 0.0, diag);
}

bool CylinderShape::testPointInside(const vec3& _localPoint, ProxyShape* _proxyShape) const{
	return (    (_localPoint.x() * _localPoint.x() + _localPoint.z() * _localPoint.z()) < m_radius * m_radius
	         && _localPoint.y() < m_halfHeight
	         && _localPoint.y() > -m_halfHeight);
}
