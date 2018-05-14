/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#include <ephysics/collision/shapes/CapsuleShape.hpp>
#include <ephysics/collision/ProxyShape.hpp>
#include <ephysics/configuration.hpp>

using namespace ephysics;

CapsuleShape::CapsuleShape(float _radius, float _height):
  ConvexShape(CAPSULE, _radius),
  m_halfHeight(_height * 0.5f) {
	assert(_radius > 0.0f);
	assert(_height > 0.0f);
}

void CapsuleShape::computeLocalInertiaTensor(etk::Matrix3x3& _tensor, float _mass) const {
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
	float IxxAndzz = factor1 * _mass * (sum1 + sum2) + factor2 * _mass * sum3;
	float Iyy = factor1 * _mass * sum1 + factor2 * _mass * float(0.25) * radiusSquareDouble;
	_tensor.setValue(IxxAndzz, 0.0, 0.0,
	                 0.0, Iyy, 0.0,
	                 0.0, 0.0, IxxAndzz);
}

bool CapsuleShape::testPointInside(const vec3& _localPoint, ProxyShape* _proxyShape) const {
	const float diffYCenterSphere1 = _localPoint.y() - m_halfHeight;
	const float diffYCenterSphere2 = _localPoint.y() + m_halfHeight;
	const float xSquare = _localPoint.x() * _localPoint.x();
	const float zSquare = _localPoint.z() * _localPoint.z();
	const float squareRadius = m_margin * m_margin;
	// Return true if the point is inside the cylinder or one of the two spheres of the capsule
	return ((xSquare + zSquare) < squareRadius &&
			_localPoint.y() < m_halfHeight && _localPoint.y() > -m_halfHeight) ||
			(xSquare + zSquare + diffYCenterSphere1 * diffYCenterSphere1) < squareRadius ||
			(xSquare + zSquare + diffYCenterSphere2 * diffYCenterSphere2) < squareRadius;
}

bool CapsuleShape::raycast(const Ray& _ray, RaycastInfo& _raycastInfo, ProxyShape* _proxyShape) const {
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
	float vec1DotD = (_ray.point1 - vec3(0.0f, -m_halfHeight - m_margin, float(0.0))).dot(d);
	if (    vec1DotD < 0.0f
	     && vec1DotD + nDotD < float(0.0)) {
		return false;
	}
	float ddotDExtraCaps = float(2.0) * m_margin * d.y();
	if (    vec1DotD > dDotD + ddotDExtraCaps
	     && vec1DotD + nDotD > dDotD + ddotDExtraCaps) {
		return false;
	}
	float nDotN = n.dot(n);
	float mDotN = m.dot(n);
	float a = dDotD * nDotN - nDotD * nDotD;
	float k = m.dot(m) - m_margin * m_margin;
	float c = dDotD * k - mDotD * mDotD;
	// If the ray is parallel to the capsule axis
	if (etk::abs(a) < epsilon) {
		// If the origin is outside the surface of the capusle's cylinder, we return no hit
		if (c > 0.0f) {
			return false;
		}
		// Here we know that the segment int32_tersect an endcap of the capsule
		// If the ray int32_tersects with the "p" endcap of the capsule
		if (mDotD < 0.0f) {
			// Check int32_tersection between the ray and the "p" sphere endcap of the capsule
			vec3 hitLocalPoint;
			float hitFraction;
			if (raycastWithSphereEndCap(_ray.point1, _ray.point2, p, _ray.maxFraction, hitLocalPoint, hitFraction)) {
				_raycastInfo.body = _proxyShape->getBody();
				_raycastInfo.proxyShape = _proxyShape;
				_raycastInfo.hitFraction = hitFraction;
				_raycastInfo.worldPoint = hitLocalPoint;
				vec3 normalDirection = hitLocalPoint - p;
				_raycastInfo.worldNormal = normalDirection;
				return true;
			}
			return false;
		} else if (mDotD > dDotD) {   // If the ray int32_tersects with the "q" endcap of the cylinder
			// Check int32_tersection between the ray and the "q" sphere endcap of the capsule
			vec3 hitLocalPoint;
			float hitFraction;
			if (raycastWithSphereEndCap(_ray.point1, _ray.point2, q, _ray.maxFraction, hitLocalPoint, hitFraction)) {
				_raycastInfo.body = _proxyShape->getBody();
				_raycastInfo.proxyShape = _proxyShape;
				_raycastInfo.hitFraction = hitFraction;
				_raycastInfo.worldPoint = hitLocalPoint;
				vec3 normalDirection = hitLocalPoint - q;
				_raycastInfo.worldNormal = normalDirection;
				return true;
			}
			return false;
		} else {
			// If the origin is inside the cylinder, we return no hit
			return false;
		}
	}
	float b = dDotD * mDotN - nDotD * mDotD;
	float discriminant = b * b - a * c;
	// If the discriminant is negative, no real roots and therfore, no hit
	if (discriminant < 0.0f) {
		return false;
	}
	// Compute the smallest root (first int32_tersection along the ray)
	float t0 = t = (-b - etk::sqrt(discriminant)) / a;
	// If the int32_tersection is outside the finite cylinder of the capsule on "p" endcap side
	float value = mDotD + t * nDotD;
	if (value < 0.0f) {
		// Check int32_tersection between the ray and the "p" sphere endcap of the capsule
		vec3 hitLocalPoint;
		float hitFraction;
		if (raycastWithSphereEndCap(_ray.point1, _ray.point2, p, _ray.maxFraction, hitLocalPoint, hitFraction)) {
			_raycastInfo.body = _proxyShape->getBody();
			_raycastInfo.proxyShape = _proxyShape;
			_raycastInfo.hitFraction = hitFraction;
			_raycastInfo.worldPoint = hitLocalPoint;
			vec3 normalDirection = hitLocalPoint - p;
			_raycastInfo.worldNormal = normalDirection;
			return true;
		}
		return false;
	} else if (value > dDotD) {  // If the int32_tersection is outside the finite cylinder on the "q" side
		// Check int32_tersection between the ray and the "q" sphere endcap of the capsule
		vec3 hitLocalPoint;
		float hitFraction;
		if (raycastWithSphereEndCap(_ray.point1, _ray.point2, q, _ray.maxFraction, hitLocalPoint, hitFraction)) {
			_raycastInfo.body = _proxyShape->getBody();
			_raycastInfo.proxyShape = _proxyShape;
			_raycastInfo.hitFraction = hitFraction;
			_raycastInfo.worldPoint = hitLocalPoint;
			vec3 normalDirection = hitLocalPoint - q;
			_raycastInfo.worldNormal = normalDirection;
			return true;
		}
		return false;
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
	vec3 normalDirection = (localHitPoint - (p + w)).safeNormalized();
	_raycastInfo.worldNormal = normalDirection;
	return true;
}

bool CapsuleShape::raycastWithSphereEndCap(const vec3& _point1,
                                           const vec3& _point2,
                                           const vec3& _sphereCenter,
                                           float _maxFraction,
                                           vec3& _hitLocalPoint,
                                           float& _hitFraction) const {
	 const vec3 m = _point1 - _sphereCenter;
	float c = m.dot(m) - m_margin * m_margin;
	// If the origin of the ray is inside the sphere, we return no int32_tersection
	if (c < 0.0f) {
		return false;
	}
	const vec3 rayDirection = _point2 - _point1;
	float b = m.dot(rayDirection);
	// If the origin of the ray is outside the sphere and the ray
	// is pointing away from the sphere, there is no int32_tersection
	if (b > 0.0f) {
		return false;
	}
	float raySquareLength = rayDirection.length2();
	// Compute the discriminant of the quadratic equation
	float discriminant = b * b - raySquareLength * c;
	// If the discriminant is negative or the ray length is very small, there is no int32_tersection
	if (    discriminant < 0.0f
	     || raySquareLength < FLT_EPSILON) {
		return false;
	}
	// Compute the solution "t" closest to the origin
	float t = -b - etk::sqrt(discriminant);
	assert(t >= 0.0f);
	// If the hit point is withing the segment ray fraction
	if (t < _maxFraction * raySquareLength) {
		// Compute the int32_tersection information
		t /= raySquareLength;
		_hitFraction = t;
		_hitLocalPoint = _point1 + t * rayDirection;
		return true;
	}
	return false;
}

float CapsuleShape::getRadius() const {
	return m_margin;
}

float CapsuleShape::getHeight() const {
	return m_halfHeight + m_halfHeight;
}

void CapsuleShape::setLocalScaling(const vec3& _scaling) {
	m_halfHeight = (m_halfHeight / m_scaling.y()) * _scaling.y();
	m_margin = (m_margin / m_scaling.x()) * _scaling.x();
	CollisionShape::setLocalScaling(_scaling);
}

size_t CapsuleShape::getSizeInBytes() const {
	return sizeof(CapsuleShape);
}

void CapsuleShape::getLocalBounds(vec3& _min, vec3& _max) const {
	// Maximum bounds
	_max.setX(m_margin);
	_max.setY(m_halfHeight + m_margin);
	_max.setZ(m_margin);
	// Minimum bounds
	_min.setX(-m_margin);
	_min.setY(-_max.y());
	_min.setZ(_min.x());
}

vec3 CapsuleShape::getLocalSupportPointWithoutMargin(const vec3& _direction,
                                                     void** _cachedCollisionData) const {
	// Support point top sphere
	float dotProductTop = m_halfHeight * _direction.y();
	// Support point bottom sphere
	float dotProductBottom = -m_halfHeight * _direction.y();
	// Return the point with the maximum dot product
	if (dotProductTop > dotProductBottom) {
		return vec3(0, m_halfHeight, 0);
	}
	return vec3(0, -m_halfHeight, 0);
}
