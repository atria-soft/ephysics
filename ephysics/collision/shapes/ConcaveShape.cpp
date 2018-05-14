/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#include <ephysics/collision/shapes/ConcaveShape.hpp>


// We want to use the ReactPhysics3D namespace
using namespace ephysics;

// Constructor
ConcaveShape::ConcaveShape(CollisionShapeType _type):
  CollisionShape(_type),
  m_isSmoothMeshCollisionEnabled(false),
  m_triangleMargin(0),
  m_raycastTestType(FRONT) {
	
}

float ConcaveShape::getTriangleMargin() const {
	return m_triangleMargin;
}

bool ConcaveShape::isConvex() const {
	return false;
}

bool ConcaveShape::testPointInside(const vec3& _localPoint, ProxyShape* _proxyShape) const {
	return false;
}

bool ConcaveShape::getIsSmoothMeshCollisionEnabled() const {
	return m_isSmoothMeshCollisionEnabled;
}

void ConcaveShape::setIsSmoothMeshCollisionEnabled(bool _isEnabled) {
	m_isSmoothMeshCollisionEnabled = _isEnabled;
}

TriangleRaycastSide ConcaveShape::getRaycastTestType() const {
	return m_raycastTestType;
}

void ConcaveShape::setRaycastTestType(TriangleRaycastSide _testType) {
	m_raycastTestType = _testType;
}
