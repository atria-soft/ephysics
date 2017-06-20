/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */


// Libraries
#include <ephysics/collision/shapes/AABB.hpp>
#include <ephysics/configuration.hpp>
#include <cassert>

using namespace ephysics;
using namespace std;

AABB::AABB():
  m_minCoordinates(0,0,0),
  m_maxCoordinates(0,0,0) {
	
}

AABB::AABB(const vec3& minCoordinates, const vec3& maxCoordinates):
  m_minCoordinates(minCoordinates),
  m_maxCoordinates(maxCoordinates) {
	
}

AABB::AABB(const AABB& _aabb):
  m_minCoordinates(_aabb.m_minCoordinates),
  m_maxCoordinates(_aabb.m_maxCoordinates) {
	
}

void AABB::mergeWithAABB(const AABB& _aabb) {
	m_minCoordinates.setX(std::min(m_minCoordinates.x(), _aabb.m_minCoordinates.x()));
	m_minCoordinates.setY(std::min(m_minCoordinates.y(), _aabb.m_minCoordinates.y()));
	m_minCoordinates.setZ(std::min(m_minCoordinates.z(), _aabb.m_minCoordinates.z()));
	m_maxCoordinates.setX(std::max(m_maxCoordinates.x(), _aabb.m_maxCoordinates.x()));
	m_maxCoordinates.setY(std::max(m_maxCoordinates.y(), _aabb.m_maxCoordinates.y()));
	m_maxCoordinates.setZ(std::max(m_maxCoordinates.z(), _aabb.m_maxCoordinates.z()));
}

void AABB::mergeTwoAABBs(const AABB& _aabb1, const AABB& _aabb2) {
	m_minCoordinates.setX(std::min(_aabb1.m_minCoordinates.x(), _aabb2.m_minCoordinates.x()));
	m_minCoordinates.setY(std::min(_aabb1.m_minCoordinates.y(), _aabb2.m_minCoordinates.y()));
	m_minCoordinates.setZ(std::min(_aabb1.m_minCoordinates.z(), _aabb2.m_minCoordinates.z()));
	m_maxCoordinates.setX(std::max(_aabb1.m_maxCoordinates.x(), _aabb2.m_maxCoordinates.x()));
	m_maxCoordinates.setY(std::max(_aabb1.m_maxCoordinates.y(), _aabb2.m_maxCoordinates.y()));
	m_maxCoordinates.setZ(std::max(_aabb1.m_maxCoordinates.z(), _aabb2.m_maxCoordinates.z()));
}

bool AABB::contains(const AABB& _aabb) const {
	bool isInside = true;
	isInside = isInside && m_minCoordinates.x() <= _aabb.m_minCoordinates.x();
	isInside = isInside && m_minCoordinates.y() <= _aabb.m_minCoordinates.y();
	isInside = isInside && m_minCoordinates.z() <= _aabb.m_minCoordinates.z();
	isInside = isInside && m_maxCoordinates.x() >= _aabb.m_maxCoordinates.x();
	isInside = isInside && m_maxCoordinates.y() >= _aabb.m_maxCoordinates.y();
	isInside = isInside && m_maxCoordinates.z() >= _aabb.m_maxCoordinates.z();
	return isInside;
}

AABB AABB::createAABBForTriangle(const vec3* _trianglePoints) {
	vec3 minCoords(_trianglePoints[0].x(), _trianglePoints[0].y(), _trianglePoints[0].z());
	vec3 maxCoords(_trianglePoints[0].x(), _trianglePoints[0].y(), _trianglePoints[0].z());
	if (_trianglePoints[1].x() < minCoords.x()) {
		minCoords.setX(_trianglePoints[1].x());
	}
	if (_trianglePoints[1].y() < minCoords.y()) {
		minCoords.setY(_trianglePoints[1].y());
	}
	if (_trianglePoints[1].z() < minCoords.z()) {
		minCoords.setZ(_trianglePoints[1].z());
	}
	if (_trianglePoints[2].x() < minCoords.x()) {
		minCoords.setX(_trianglePoints[2].x());
	}
	if (_trianglePoints[2].y() < minCoords.y()) {
		minCoords.setY(_trianglePoints[2].y());
	}
	if (_trianglePoints[2].z() < minCoords.z()) {
		minCoords.setZ(_trianglePoints[2].z());
	}
	if (_trianglePoints[1].x() > maxCoords.x()) {
		maxCoords.setX(_trianglePoints[1].x());
	}
	if (_trianglePoints[1].y() > maxCoords.y()) {
		maxCoords.setY(_trianglePoints[1].y());
	}
	if (_trianglePoints[1].z() > maxCoords.z()) {
		maxCoords.setZ(_trianglePoints[1].z());
	}
	if (_trianglePoints[2].x() > maxCoords.x()) {
		maxCoords.setX(_trianglePoints[2].x());
	}
	if (_trianglePoints[2].y() > maxCoords.y()) {
		maxCoords.setY(_trianglePoints[2].y());
	}
	if (_trianglePoints[2].z() > maxCoords.z()) {
		maxCoords.setZ(_trianglePoints[2].z());
	}
	return AABB(minCoords, maxCoords);
}

bool AABB::testRayIntersect(const Ray& _ray) const {
	const vec3 point2 = _ray.point1 + _ray.maxFraction * (_ray.point2 - _ray.point1);
	const vec3 e = m_maxCoordinates - m_minCoordinates;
	const vec3 d = point2 - _ray.point1;
	const vec3 m = _ray.point1 + point2 - m_minCoordinates - m_maxCoordinates;
	// Test if the AABB face normals are separating axis
	float adx = std::abs(d.x());
	if (std::abs(m.x()) > e.x() + adx) {
		return false;
	}
	float ady = std::abs(d.y());
	if (std::abs(m.y()) > e.y() + ady) {
		return false;
	}
	float adz = std::abs(d.z());
	if (std::abs(m.z()) > e.z() + adz) {
		return false;
	}
	// Add in an epsilon term to counteract arithmetic errors when segment is
	// (near) parallel to a coordinate axis (see text for detail)
	const float epsilon = 0.00001;
	adx += epsilon;
	ady += epsilon;
	adz += epsilon;
	// Test if the cross products between face normals and ray direction are
	// separating axis
	if (std::abs(m.y() * d.z() - m.z() * d.y()) > e.y() * adz + e.z() * ady) {
		return false;
	}
	if (std::abs(m.z() * d.x() - m.x() * d.z()) > e.x() * adz + e.z() * adx) {
		return false;
	}
	if (std::abs(m.x() * d.y() - m.y() * d.x()) > e.x() * ady + e.y() * adx) {
		return false;
	}
	// No separating axis has been found
	return true;
}

vec3 AABB::getExtent() const {
  return m_maxCoordinates - m_minCoordinates;
}

void AABB::inflate(float _dx, float _dy, float _dz) {
	m_maxCoordinates += vec3(_dx, _dy, _dz);
	m_minCoordinates -= vec3(_dx, _dy, _dz);
}

bool AABB::testCollision(const AABB& aabb) const {
	if (m_maxCoordinates.x() < aabb.m_minCoordinates.x() ||
		aabb.m_maxCoordinates.x() < m_minCoordinates.x()) return false;
	if (m_maxCoordinates.y() < aabb.m_minCoordinates.y() ||
		aabb.m_maxCoordinates.y() < m_minCoordinates.y()) return false;
	if (m_maxCoordinates.z() < aabb.m_minCoordinates.z()||
		aabb.m_maxCoordinates.z() < m_minCoordinates.z()) return false;
	return true;
}

float AABB::getVolume() const {
	const vec3 diff = m_maxCoordinates - m_minCoordinates;
	return (diff.x() * diff.y() * diff.z());
}

bool AABB::testCollisionTriangleAABB(const vec3* _trianglePoints) const {
	if (min3(_trianglePoints[0].x(), _trianglePoints[1].x(), _trianglePoints[2].x()) > m_maxCoordinates.x()) {
		return false;
	}
	if (min3(_trianglePoints[0].y(), _trianglePoints[1].y(), _trianglePoints[2].y()) > m_maxCoordinates.y()) {
		return false;
	}
	if (min3(_trianglePoints[0].z(), _trianglePoints[1].z(), _trianglePoints[2].z()) > m_maxCoordinates.z()) {
		return false;
	}
	if (max3(_trianglePoints[0].x(), _trianglePoints[1].x(), _trianglePoints[2].x()) < m_minCoordinates.x()) {
		return false;
	}
	if (max3(_trianglePoints[0].y(), _trianglePoints[1].y(), _trianglePoints[2].y()) < m_minCoordinates.y()) {
		return false;
	}
	if (max3(_trianglePoints[0].z(), _trianglePoints[1].z(), _trianglePoints[2].z()) < m_minCoordinates.z()) {
		return false;
	}
	return true;
}

bool AABB::contains(const vec3& _point) const {
	return    _point.x() >= m_minCoordinates.x() - MACHINE_EPSILON && _point.x() <= m_maxCoordinates.x() + MACHINE_EPSILON
	       && _point.y() >= m_minCoordinates.y() - MACHINE_EPSILON && _point.y() <= m_maxCoordinates.y() + MACHINE_EPSILON
	       && _point.z() >= m_minCoordinates.z() - MACHINE_EPSILON && _point.z() <= m_maxCoordinates.z() + MACHINE_EPSILON);
}

AABB& AABB::operator=(const AABB& _aabb) {
	if (this != &_aabb) {
		m_minCoordinates = _aabb.m_minCoordinates;
		m_maxCoordinates = _aabb.m_maxCoordinates;
	}
	return *this;
}

