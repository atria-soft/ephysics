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

ConeShape::ConeShape(float _radius, float _height, float _margin):
  ConvexShape(CONE, _margin),
  m_radius(_radius),
  m_halfHeight(_height * 0.5f) {
	assert(m_radius > 0.0f);
	assert(m_halfHeight > 0.0f);
	// Compute the sine of the semi-angle at the apex point
	m_sinTheta = m_radius / (sqrt(m_radius * m_radius + _height * _height));
}

vec3 ConeShape::getLocalSupportPointWithoutMargin(const vec3& _direction, void** _cachedCollisionData) const {
	const vec3& v = _direction;
	float sinThetaTimesLengthV = m_sinTheta * v.length();
	vec3 supportPoint;
	if (v.y() > sinThetaTimesLengthV) {
		supportPoint = vec3(0.0, m_halfHeight, 0.0);
	} else {
		float projectedLength = sqrt(v.x() * v.x() + v.z() * v.z());
		if (projectedLength > FLT_EPSILON) {
			float d = m_radius / projectedLength;
			supportPoint = vec3(v.x() * d, -m_halfHeight, v.z() * d);
		} else {
			supportPoint = vec3(0.0, -m_halfHeight, 0.0);
		}
	}
	return supportPoint;
}

bool ConeShape::raycast(const Ray& _ray, RaycastInfo& _raycastInfo, ProxyShape* _proxyShape) const {
	const vec3 r = _ray.point2 - _ray.point1;
	const float epsilon = float(0.00001);
	vec3 V(0, m_halfHeight, 0);
	vec3 centerBase(0, -m_halfHeight, 0);
	vec3 axis(0, float(-1.0), 0);
	float heightSquare = float(4.0) * m_halfHeight * m_halfHeight;
	float cosThetaSquare = heightSquare / (heightSquare + m_radius * m_radius);
	float factor = 1.0f - cosThetaSquare;
	vec3 delta = _ray.point1 - V;
	float c0 = -cosThetaSquare * delta.x() * delta.x()  + factor * delta.y() * delta.y() - cosThetaSquare * delta.z() * delta.z();
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
		} else if (gamma > 0.0f) {	// The equation has two real roots
			// Compute two int32_tersections
			float sqrRoot = etk::sqrt(gamma);
			tHit[0] = (-c1 - sqrRoot) / c2;
			tHit[1] = (-c1 + sqrRoot) / c2;
		} else {  // If the equation has a single real root
			// Compute the int32_tersection
			tHit[0] = -c1 / c2;
		}
	} else {
		// If c2 == 0
		if (etk::abs(c1) > FLT_EPSILON) {
			// If c2 = 0 and c1 != 0
			tHit[0] = -c0 / (float(2.0) * c1);
		} else {
			// If c2 = c1 = 0
			// If c0 is different from zero, no solution and if c0 = 0, we have a
			// degenerate case, the whole ray is contained in the cone side
			// but we return no hit in this case
			return false;
		}
	}
	// If the origin of the ray is inside the cone, we return no hit
	if (testPointInside(_ray.point1, NULL)) {
		return false;
	}
	localHitPoint[0] = _ray.point1 + tHit[0] * r;
	localHitPoint[1] = _ray.point1 + tHit[1] * r;
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
		tHit[2] = (-_ray.point1.y() - m_halfHeight) / (r.y());
		// Only keep this int32_tersection if it is inside the cone radius
		localHitPoint[2] = _ray.point1 + tHit[2] * r;
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
		if (tHit[i] < 0.0f) {
			continue;
		}
		if (tHit[i] < t) {
			hitIndex = i;
			t = tHit[hitIndex];
		}
	}
	if (hitIndex < 0) {
		return false;
	}
	if (t > _ray.maxFraction) {
		return false;
	}
	// Compute the normal direction for hit against side of the cone
	if (hitIndex != 2) {
		float h = float(2.0) * m_halfHeight;
		float value1 = (localHitPoint[hitIndex].x() * localHitPoint[hitIndex].x() + localHitPoint[hitIndex].z() * localHitPoint[hitIndex].z());
		float rOverH = m_radius / h;
		float value2 = 1.0f + rOverH * rOverH;
		float factor = 1.0f / etk::sqrt(value1 * value2);
		float x = localHitPoint[hitIndex].x() * factor;
		float z = localHitPoint[hitIndex].z() * factor;
		localNormal[hitIndex].setX(x);
		localNormal[hitIndex].setY(etk::sqrt(x * x + z * z) * rOverH);
		localNormal[hitIndex].setZ(z);
	}
	_raycastInfo.body = _proxyShape->getBody();
	_raycastInfo.proxyShape = _proxyShape;
	_raycastInfo.hitFraction = t;
	_raycastInfo.worldPoint = localHitPoint[hitIndex];
	_raycastInfo.worldNormal = localNormal[hitIndex];
	return true;
}

float ConeShape::getRadius() const {
	return m_radius;
}

float ConeShape::getHeight() const {
	return float(2.0) * m_halfHeight;
}

void ConeShape::setLocalScaling(const vec3& _scaling) {
	m_halfHeight = (m_halfHeight / m_scaling.y()) * _scaling.y();
	m_radius = (m_radius / m_scaling.x()) * _scaling.x();
	CollisionShape::setLocalScaling(_scaling);
}

size_t ConeShape::getSizeInBytes() const {
	return sizeof(ConeShape);
}

void ConeShape::getLocalBounds(vec3& _min, vec3& _max) const {
	// Maximum bounds
	_max.setX(m_radius + m_margin);
	_max.setY(m_halfHeight + m_margin);
	_max.setZ(_max.x());
	// Minimum bounds
	_min.setX(-_max.x());
	_min.setY(-_max.y());
	_min.setZ(_min.x());
}

void ConeShape::computeLocalInertiaTensor(etk::Matrix3x3& _tensor, float _mass) const {
	float rSquare = m_radius * m_radius;
	float diagXZ = float(0.15) * _mass * (rSquare + m_halfHeight);
	_tensor.setValue(diagXZ, 0.0, 0.0,
	                 0.0, float(0.3) * _mass * rSquare,
	                 0.0, 0.0, 0.0, diagXZ);
}

bool ConeShape::testPointInside(const vec3& _localPoint, ProxyShape* _proxyShape) const {
	const float radiusHeight =   m_radius
	                           * (-_localPoint.y() + m_halfHeight)
	                           / (m_halfHeight * float(2.0));
	return (    _localPoint.y() < m_halfHeight
	         && _localPoint.y() > -m_halfHeight)
	         && (_localPoint.x() * _localPoint.x() + _localPoint.z() * _localPoint.z() < radiusHeight *radiusHeight);
}
