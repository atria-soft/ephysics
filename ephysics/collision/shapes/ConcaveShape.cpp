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
ConcaveShape::ConcaveShape(CollisionShapeType type)
			 : CollisionShape(type), m_isSmoothMeshCollisionEnabled(false),
			   m_triangleMargin(0), m_raycastTestType(FRONT) {

}

// Destructor
ConcaveShape::~ConcaveShape() {

}

// Return the triangle margin
float ConcaveShape::getTriangleMargin() const {
	return m_triangleMargin;
}

/// Return true if the collision shape is convex, false if it is concave
bool ConcaveShape::isConvex() const {
	return false;
}

// Return true if a point is inside the collision shape
bool ConcaveShape::testPointInside(const vec3& localPoint, ProxyShape* proxyShape) const {
	return false;
}

// Return true if the smooth mesh collision is enabled
bool ConcaveShape::getIsSmoothMeshCollisionEnabled() const {
	return m_isSmoothMeshCollisionEnabled;
}

// Enable/disable the smooth mesh collision algorithm
/// Smooth mesh collision is used to avoid collisions against some int32_ternal edges
/// of the triangle mesh. If it is enabled, collsions with the mesh will be smoother
/// but collisions computation is a bit more expensive.
void ConcaveShape::setIsSmoothMeshCollisionEnabled(bool isEnabled) {
	m_isSmoothMeshCollisionEnabled = isEnabled;
}

// Return the raycast test type (front, back, front-back)
TriangleRaycastSide ConcaveShape::getRaycastTestType() const {
	return m_raycastTestType;
}

// Set the raycast test type (front, back, front-back)
/**
 * @param testType Raycast test type for the triangle (front, back, front-back)
 */
void ConcaveShape::setRaycastTestType(TriangleRaycastSide testType) {
	m_raycastTestType = testType;
}
