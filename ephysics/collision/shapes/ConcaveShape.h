/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/collision/shapes/CollisionShape.h>
#include <ephysics/collision/shapes/TriangleShape.h>

// ReactPhysics3D namespace
namespace reactphysics3d {

// Class TriangleCallback
/**
 * This class is used to encapsulate a callback method for
 * a single triangle of a ConcaveMesh.
 */
class TriangleCallback {

	public:
		virtual ~TriangleCallback() = default;

		/// Report a triangle
		virtual void testTriangle(const vec3* trianglePoints)=0;

};


// Class ConcaveShape
/**
 * This abstract class represents a concave collision shape associated with a
 * body that is used during the narrow-phase collision detection.
 */
class ConcaveShape : public CollisionShape {

	protected :

		// -------------------- Attributes -------------------- //

		/// True if the smooth mesh collision algorithm is enabled
		bool m_isSmoothMeshCollisionEnabled;

		// Margin use for collision detection for each triangle
		float m_triangleMargin;

		/// Raycast test type for the triangle (front, back, front-back)
		TriangleRaycastSide m_raycastTestType;

		// -------------------- Methods -------------------- //

		/// Private copy-constructor
		ConcaveShape(const ConcaveShape& shape);

		/// Private assignment operator
		ConcaveShape& operator=(const ConcaveShape& shape);

		/// Return true if a point is inside the collision shape
		virtual bool testPointInside(const vec3& localPoint, ProxyShape* proxyShape) const;

	public :

		// -------------------- Methods -------------------- //

		/// Constructor
		ConcaveShape(CollisionShapeType type);

		/// Destructor
		virtual ~ConcaveShape();

		/// Return the triangle margin
		float getTriangleMargin() const;

		/// Return the raycast test type (front, back, front-back)
		TriangleRaycastSide getRaycastTestType() const;

		// Set the raycast test type (front, back, front-back)
		void setRaycastTestType(TriangleRaycastSide testType);

		/// Return true if the collision shape is convex, false if it is concave
		virtual bool isConvex() const;

		/// Use a callback method on all triangles of the concave shape inside a given AABB
		virtual void testAllTriangles(TriangleCallback& callback, const AABB& localAABB) const=0;

		/// Return true if the smooth mesh collision is enabled
		bool getIsSmoothMeshCollisionEnabled() const;

		/// Enable/disable the smooth mesh collision algorithm
		void setIsSmoothMeshCollisionEnabled(bool isEnabled);
};

// Return the triangle margin
inline float ConcaveShape::getTriangleMargin() const {
	return m_triangleMargin;
}

/// Return true if the collision shape is convex, false if it is concave
inline bool ConcaveShape::isConvex() const {
	return false;
}

// Return true if a point is inside the collision shape
inline bool ConcaveShape::testPointInside(const vec3& localPoint, ProxyShape* proxyShape) const {
	return false;
}

// Return true if the smooth mesh collision is enabled
inline bool ConcaveShape::getIsSmoothMeshCollisionEnabled() const {
	return m_isSmoothMeshCollisionEnabled;
}

// Enable/disable the smooth mesh collision algorithm
/// Smooth mesh collision is used to avoid collisions against some int32_ternal edges
/// of the triangle mesh. If it is enabled, collsions with the mesh will be smoother
/// but collisions computation is a bit more expensive.
inline void ConcaveShape::setIsSmoothMeshCollisionEnabled(bool isEnabled) {
	m_isSmoothMeshCollisionEnabled = isEnabled;
}

// Return the raycast test type (front, back, front-back)
inline TriangleRaycastSide ConcaveShape::getRaycastTestType() const {
	return m_raycastTestType;
}

// Set the raycast test type (front, back, front-back)
/**
 * @param testType Raycast test type for the triangle (front, back, front-back)
 */
inline void ConcaveShape::setRaycastTestType(TriangleRaycastSide testType) {
	m_raycastTestType = testType;
}

}


