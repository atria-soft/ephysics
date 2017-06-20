/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

#include <ephysics/mathematics/mathematics.hpp>
#include <ephysics/collision/shapes/ConvexShape.hpp>

namespace ephysics {

/**
 * @brief Raycast test side for the triangle
 */
enum TriangleRaycastSide {
	FRONT, //!< Raycast against front triangle
	BACK, //!< Raycast against back triangle
	FRONT_AND_BACK //!< Raycast against front and back triangle
};

/**
 * This class represents a triangle collision shape that is centered
 * at the origin and defined three points.
 */
class TriangleShape : public ConvexShape {
	protected:
		vec3 m_points[3]; //!< Three points of the triangle
		TriangleRaycastSide m_raycastTestType; //!< Raycast test type for the triangle (front, back, front-back)
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
		friend class ConcaveMeshRaycastCallback;
		friend class TriangleOverlapCallback;
};

// Return the number of bytes used by the collision shape
size_t TriangleShape::getSizeInBytes() const {
	return sizeof(TriangleShape);
}

// Return a local support point in a given direction without the object margin
vec3 TriangleShape::getLocalSupportPointWithoutMargin(const vec3& direction,
															  void** cachedCollisionData) const {
	vec3 dotProducts(direction.dot(m_points[0]), direction.dot(m_points[1]), direction.dot(m_points[2]));
	return m_points[dotProducts.getMaxAxis()];
}

// Return the local bounds of the shape in x, y and z directions.
// This method is used to compute the AABB of the box
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
void TriangleShape::getLocalBounds(vec3& min, vec3& max) const {

	const vec3 xAxis(m_points[0].x(), m_points[1].x(), m_points[2].x());
	const vec3 yAxis(m_points[0].y(), m_points[1].y(), m_points[2].y());
	const vec3 zAxis(m_points[0].z(), m_points[1].z(), m_points[2].z());
	min.setValue(xAxis.getMin(), yAxis.getMin(), zAxis.getMin());
	max.setValue(xAxis.getMax(), yAxis.getMax(), zAxis.getMax());

	min -= vec3(m_margin, m_margin, m_margin);
	max += vec3(m_margin, m_margin, m_margin);
}

// Set the local scaling vector of the collision shape
void TriangleShape::setLocalScaling(const vec3& scaling) {

	m_points[0] = (m_points[0] / m_scaling) * scaling;
	m_points[1] = (m_points[1] / m_scaling) * scaling;
	m_points[2] = (m_points[2] / m_scaling) * scaling;

	CollisionShape::setLocalScaling(scaling);
}

// Return the local inertia tensor of the triangle shape
/**
 * @param[out] tensor The 3x3 inertia tensor matrix of the shape in local-space
 *					coordinates
 * @param mass Mass to use to compute the inertia tensor of the collision shape
 */
void TriangleShape::computeLocalInertiaTensor(etk::Matrix3x3& tensor, float mass) const {
	tensor.setZero();
}

// Update the AABB of a body using its collision shape
/**
 * @param[out] aabb The axis-aligned bounding box (AABB) of the collision shape
 *				  computed in world-space coordinates
 * @param transform etk::Transform3D used to compute the AABB of the collision shape
 */
void TriangleShape::computeAABB(AABB& aabb, const etk::Transform3D& transform) const {

	const vec3 worldPoint1 = transform * m_points[0];
	const vec3 worldPoint2 = transform * m_points[1];
	const vec3 worldPoint3 = transform * m_points[2];

	const vec3 xAxis(worldPoint1.x(), worldPoint2.x(), worldPoint3.x());
	const vec3 yAxis(worldPoint1.y(), worldPoint2.y(), worldPoint3.y());
	const vec3 zAxis(worldPoint1.z(), worldPoint2.z(), worldPoint3.z());
	aabb.setMin(vec3(xAxis.getMin(), yAxis.getMin(), zAxis.getMin()));
	aabb.setMax(vec3(xAxis.getMax(), yAxis.getMax(), zAxis.getMax()));
}

// Return true if a point is inside the collision shape
bool TriangleShape::testPointInside(const vec3& localPoint, ProxyShape* proxyShape) const {
	return false;
}

// Return the raycast test type (front, back, front-back)
TriangleRaycastSide TriangleShape::getRaycastTestType() const {
	return m_raycastTestType;
}

// Set the raycast test type (front, back, front-back)
/**
 * @param testType Raycast test type for the triangle (front, back, front-back)
 */
void TriangleShape::setRaycastTestType(TriangleRaycastSide testType) {
	m_raycastTestType = testType;
}

// Return the coordinates of a given vertex of the triangle
/**
 * @param index Index (0 to 2) of a vertex of the triangle
 */
vec3 TriangleShape::getVertex(int32_t index) const {
	assert(index >= 0 && index < 3);
	return m_points[index];
}

}

