/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/mathematics/mathematics.hpp>
#include <ephysics/collision/shapes/ConvexShape.hpp>

/// ReactPhysics3D namespace
namespace ephysics {

/// Raycast test side for the triangle
enum TriangleRaycastSide {

	/// Raycast against front triangle
	FRONT,

	/// Raycast against back triangle
	BACK,

	/// Raycast against front and back triangle
	FRONT_AND_BACK
};

// Class TriangleShape
/**
 * This class represents a triangle collision shape that is centered
 * at the origin and defined three points.
 */
class TriangleShape : public ConvexShape {

	protected:

		// -------------------- Attribute -------------------- //

		/// Three points of the triangle
		vec3 mPoints[3];

		/// Raycast test type for the triangle (front, back, front-back)
		TriangleRaycastSide m_raycastTestType;

		// -------------------- Methods -------------------- //

		/// Private copy-constructor
		TriangleShape(const TriangleShape& shape);

		/// Private assignment operator
		TriangleShape& operator=(const TriangleShape& shape);

		/// Return a local support point in a given direction without the object margin
		virtual vec3 getLocalSupportPointWithoutMargin(const vec3& direction,
														  void** cachedCollisionData) const;

		/// Return true if a point is inside the collision shape
		virtual bool testPointInside(const vec3& localPoint, ProxyShape* proxyShape) const;

		/// Raycast method with feedback information
		virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const;

		/// Return the number of bytes used by the collision shape
		virtual size_t getSizeInBytes() const;

	public:

		// -------------------- Methods -------------------- //

		/// Constructor
		TriangleShape(const vec3& point1, const vec3& point2, const vec3& point3,
					  float margin = OBJECT_MARGIN);

		/// Destructor
		virtual ~TriangleShape();

		/// Return the local bounds of the shape in x, y and z directions.
		virtual void getLocalBounds(vec3& min, vec3& max) const;

		/// Set the local scaling vector of the collision shape
		virtual void setLocalScaling(const vec3& scaling);

		/// Return the local inertia tensor of the collision shape
		virtual void computeLocalInertiaTensor(etk::Matrix3x3& tensor, float mass) const;

		/// Update the AABB of a body using its collision shape
		virtual void computeAABB(AABB& aabb, const etk::Transform3D& transform) const;

		/// Return the raycast test type (front, back, front-back)
		TriangleRaycastSide getRaycastTestType() const;

		// Set the raycast test type (front, back, front-back)
		void setRaycastTestType(TriangleRaycastSide testType);

		/// Return the coordinates of a given vertex of the triangle
		vec3 getVertex(int32_t index) const;

		// ---------- Friendship ---------- //

		friend class ConcaveMeshRaycastCallback;
		friend class TriangleOverlapCallback;
};

// Return the number of bytes used by the collision shape
inline size_t TriangleShape::getSizeInBytes() const {
	return sizeof(TriangleShape);
}

// Return a local support point in a given direction without the object margin
inline vec3 TriangleShape::getLocalSupportPointWithoutMargin(const vec3& direction,
															  void** cachedCollisionData) const {
	vec3 dotProducts(direction.dot(mPoints[0]), direction.dot(mPoints[1]), direction.dot(mPoints[2]));
	return mPoints[dotProducts.getMaxAxis()];
}

// Return the local bounds of the shape in x, y and z directions.
// This method is used to compute the AABB of the box
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
inline void TriangleShape::getLocalBounds(vec3& min, vec3& max) const {

	const vec3 xAxis(mPoints[0].x(), mPoints[1].x(), mPoints[2].x());
	const vec3 yAxis(mPoints[0].y(), mPoints[1].y(), mPoints[2].y());
	const vec3 zAxis(mPoints[0].z(), mPoints[1].z(), mPoints[2].z());
	min.setValue(xAxis.getMin(), yAxis.getMin(), zAxis.getMin());
	max.setValue(xAxis.getMax(), yAxis.getMax(), zAxis.getMax());

	min -= vec3(m_margin, m_margin, m_margin);
	max += vec3(m_margin, m_margin, m_margin);
}

// Set the local scaling vector of the collision shape
inline void TriangleShape::setLocalScaling(const vec3& scaling) {

	mPoints[0] = (mPoints[0] / m_scaling) * scaling;
	mPoints[1] = (mPoints[1] / m_scaling) * scaling;
	mPoints[2] = (mPoints[2] / m_scaling) * scaling;

	CollisionShape::setLocalScaling(scaling);
}

// Return the local inertia tensor of the triangle shape
/**
 * @param[out] tensor The 3x3 inertia tensor matrix of the shape in local-space
 *					coordinates
 * @param mass Mass to use to compute the inertia tensor of the collision shape
 */
inline void TriangleShape::computeLocalInertiaTensor(etk::Matrix3x3& tensor, float mass) const {
	tensor.setZero();
}

// Update the AABB of a body using its collision shape
/**
 * @param[out] aabb The axis-aligned bounding box (AABB) of the collision shape
 *				  computed in world-space coordinates
 * @param transform etk::Transform3D used to compute the AABB of the collision shape
 */
inline void TriangleShape::computeAABB(AABB& aabb, const etk::Transform3D& transform) const {

	const vec3 worldPoint1 = transform * mPoints[0];
	const vec3 worldPoint2 = transform * mPoints[1];
	const vec3 worldPoint3 = transform * mPoints[2];

	const vec3 xAxis(worldPoint1.x(), worldPoint2.x(), worldPoint3.x());
	const vec3 yAxis(worldPoint1.y(), worldPoint2.y(), worldPoint3.y());
	const vec3 zAxis(worldPoint1.z(), worldPoint2.z(), worldPoint3.z());
	aabb.setMin(vec3(xAxis.getMin(), yAxis.getMin(), zAxis.getMin()));
	aabb.setMax(vec3(xAxis.getMax(), yAxis.getMax(), zAxis.getMax()));
}

// Return true if a point is inside the collision shape
inline bool TriangleShape::testPointInside(const vec3& localPoint, ProxyShape* proxyShape) const {
	return false;
}

// Return the raycast test type (front, back, front-back)
inline TriangleRaycastSide TriangleShape::getRaycastTestType() const {
	return m_raycastTestType;
}

// Set the raycast test type (front, back, front-back)
/**
 * @param testType Raycast test type for the triangle (front, back, front-back)
 */
inline void TriangleShape::setRaycastTestType(TriangleRaycastSide testType) {
	m_raycastTestType = testType;
}

// Return the coordinates of a given vertex of the triangle
/**
 * @param index Index (0 to 2) of a vertex of the triangle
 */
inline vec3 TriangleShape::getVertex(int32_t index) const {
	assert(index >= 0 && index < 3);
	return mPoints[index];
}

}

