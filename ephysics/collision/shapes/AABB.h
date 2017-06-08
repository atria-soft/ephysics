/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/mathematics/mathematics.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {
	
// Class AABB
/**
 * This class represents a bounding volume of type "Axis Aligned
 * Bounding Box". It's a box where all the edges are always aligned
 * with the world coordinate system. The AABB is defined by the
 * minimum and maximum world coordinates of the three axis.
 */
class AABB {

	private :

		// -------------------- Attributes -------------------- //

		/// Minimum world coordinates of the AABB on the x,y and z axis
		Vector3 m_minCoordinates;

		/// Maximum world coordinates of the AABB on the x,y and z axis
		Vector3 m_maxCoordinates;

	public :

		// -------------------- Methods -------------------- //

		/// Constructor
		AABB();

		/// Constructor
		AABB(const Vector3& minCoordinates, const Vector3& maxCoordinates);

		/// Copy-constructor
		AABB(const AABB& aabb);

		/// Destructor
		~AABB();

		/// Return the center point
		Vector3 getCenter() const;

		/// Return the minimum coordinates of the AABB
		const Vector3& getMin() const;

		/// Set the minimum coordinates of the AABB
		void setMin(const Vector3& min);

		/// Return the maximum coordinates of the AABB
		const Vector3& getMax() const;

		/// Set the maximum coordinates of the AABB
		void setMax(const Vector3& max);

		/// Return the size of the AABB in the three dimension x, y and z
		Vector3 getExtent() const;

		/// Inflate each side of the AABB by a given size
		void inflate(float dx, float dy, float dz);

		/// Return true if the current AABB is overlapping with the AABB in argument
		bool testCollision(const AABB& aabb) const;

		/// Return the volume of the AABB
		float getVolume() const;

		/// Merge the AABB in parameter with the current one
		void mergeWithAABB(const AABB& aabb);

		/// Replace the current AABB with a new AABB that is the union of two AABBs in parameters
		void mergeTwoAABBs(const AABB& aabb1, const AABB& aabb2);

		/// Return true if the current AABB contains the AABB given in parameter
		bool contains(const AABB& aabb) const;

		/// Return true if a point is inside the AABB
		bool contains(const Vector3& point) const;

		/// Return true if the AABB of a triangle int32_tersects the AABB
		bool testCollisionTriangleAABB(const Vector3* trianglePoints) const;

		/// Return true if the ray int32_tersects the AABB
		bool testRayIntersect(const Ray& ray) const;

		/// Create and return an AABB for a triangle
		static AABB createAABBForTriangle(const Vector3* trianglePoints);

		/// Assignment operator
		AABB& operator=(const AABB& aabb);

		// -------------------- Friendship -------------------- //

		friend class DynamicAABBTree;
};

// Return the center point of the AABB in world coordinates
inline Vector3 AABB::getCenter() const {
	return (m_minCoordinates + m_maxCoordinates) * float(0.5);
}

// Return the minimum coordinates of the AABB
inline const Vector3& AABB::getMin() const {
	return m_minCoordinates;
}

// Set the minimum coordinates of the AABB
inline void AABB::setMin(const Vector3& min) {
	m_minCoordinates = min;
}

// Return the maximum coordinates of the AABB
inline const Vector3& AABB::getMax() const {
	return m_maxCoordinates;
}

// Set the maximum coordinates of the AABB
inline void AABB::setMax(const Vector3& max) {
	m_maxCoordinates = max;
}

// Return the size of the AABB in the three dimension x, y and z
inline Vector3 AABB::getExtent() const {
  return  m_maxCoordinates - m_minCoordinates;
}

// Inflate each side of the AABB by a given size
inline void AABB::inflate(float dx, float dy, float dz) {
	m_maxCoordinates += Vector3(dx, dy, dz);
	m_minCoordinates -= Vector3(dx, dy, dz);
}

// Return true if the current AABB is overlapping with the AABB in argument.
/// Two AABBs overlap if they overlap in the three x, y and z axis at the same time
inline bool AABB::testCollision(const AABB& aabb) const {
	if (m_maxCoordinates.x < aabb.m_minCoordinates.x ||
		aabb.m_maxCoordinates.x < m_minCoordinates.x) return false;
	if (m_maxCoordinates.y < aabb.m_minCoordinates.y ||
		aabb.m_maxCoordinates.y < m_minCoordinates.y) return false;
	if (m_maxCoordinates.z < aabb.m_minCoordinates.z||
		aabb.m_maxCoordinates.z < m_minCoordinates.z) return false;
	return true;
}

// Return the volume of the AABB
inline float AABB::getVolume() const {
	const Vector3 diff = m_maxCoordinates - m_minCoordinates;
	return (diff.x * diff.y * diff.z);
}

// Return true if the AABB of a triangle int32_tersects the AABB
inline bool AABB::testCollisionTriangleAABB(const Vector3* trianglePoints) const {

	if (min3(trianglePoints[0].x, trianglePoints[1].x, trianglePoints[2].x) > m_maxCoordinates.x) return false;
	if (min3(trianglePoints[0].y, trianglePoints[1].y, trianglePoints[2].y) > m_maxCoordinates.y) return false;
	if (min3(trianglePoints[0].z, trianglePoints[1].z, trianglePoints[2].z) > m_maxCoordinates.z) return false;

	if (max3(trianglePoints[0].x, trianglePoints[1].x, trianglePoints[2].x) < m_minCoordinates.x) return false;
	if (max3(trianglePoints[0].y, trianglePoints[1].y, trianglePoints[2].y) < m_minCoordinates.y) return false;
	if (max3(trianglePoints[0].z, trianglePoints[1].z, trianglePoints[2].z) < m_minCoordinates.z) return false;

	return true;
}

// Return true if a point is inside the AABB
inline bool AABB::contains(const Vector3& point) const {

	return (point.x >= m_minCoordinates.x - MACHINE_EPSILON && point.x <= m_maxCoordinates.x + MACHINE_EPSILON &&
			point.y >= m_minCoordinates.y - MACHINE_EPSILON && point.y <= m_maxCoordinates.y + MACHINE_EPSILON &&
			point.z >= m_minCoordinates.z - MACHINE_EPSILON && point.z <= m_maxCoordinates.z + MACHINE_EPSILON);
}

// Assignment operator
inline AABB& AABB::operator=(const AABB& aabb) {
	if (this != &aabb) {
		m_minCoordinates = aabb.m_minCoordinates;
		m_maxCoordinates = aabb.m_maxCoordinates;
	}
	return *this;
}

}
