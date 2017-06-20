/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once
#include <ephysics/mathematics/mathematics.hpp>
namespace ephysics {
	/**
	 * @brief Represents a bounding volume of type "Axis Aligned
	 * Bounding Box". It's a box where all the edges are always aligned
	 * with the world coordinate system. The AABB is defined by the
	 * minimum and maximum world coordinates of the three axis.
	 */
	class AABB {
		private :
			/// Minimum world coordinates of the AABB on the x,y and z axis
			vec3 m_minCoordinates;
			/// Maximum world coordinates of the AABB on the x,y and z axis
			vec3 m_maxCoordinates;
		public :
			/**
			 * @brief default contructor
			 */
			AABB();
			/**
			 * @brief contructor Whit sizes
			 * @param[in] _minCoordinates Minimum coordinates
			 * @param[in] _maxCoordinates Maximum coordinates
			 */
			AABB(const vec3& _minCoordinates, const vec3& _maxCoordinates);
			/**
			 * @brief Copy-contructor
			 * @param[in] _aabb the object to copy
			 */
			AABB(const AABB& _aabb);
			/**
			 * @brief Get the center point of the AABB box
			 * @return The 3D position of the center
			 */
			vec3 getCenter() const {
				return (m_minCoordinates + m_maxCoordinates) * 0.5f;
			}
			/**
			 * @brief Get the minimum coordinates of the AABB
			 * @return The 3d minimum coordonates
			 */
			const vec3& getMin() const {
				return m_minCoordinates;
			}
			/**
			 * @brief Set the minimum coordinates of the AABB
			 * @param[in] _min The 3d minimum coordonates
			 */
			void setMin(const vec3& _min) {
				m_minCoordinates = _min;
			}
			/**
			 * @brief Return the maximum coordinates of the AABB
			 * @return The 3d maximum coordonates
			 */
			const vec3& getMax() const {
				return m_maxCoordinates;
			}
			/**
			 * @brief Set the maximum coordinates of the AABB
			 * @param[in] _max The 3d maximum coordonates
			 */
			void setMax(const vec3& _max) {
				m_maxCoordinates = _max;
			}
			/**
			 * @brief Get the size of the AABB in the three dimension x, y and z
			 * @return the AABB 3D size
			 */
			vec3 getExtent() const;
			/**
			 * @brief Inflate each side of the AABB by a given size
			 * @param[in] _dx Inflate X size
			 * @param[in] _dy Inflate Y size
			 * @param[in] _dz Inflate Z size
			 */
			void inflate(float _dx, float _dy, float _dz);
			/**
			 * @brief Return true if the current AABB is overlapping with the AABB in argument
			 * Two AABBs overlap if they overlap in the three x, y and z axis at the same time
			 * @param[in] _aabb Other AABB box to check.
			 * @return true Collision detected
			 * @return false Not collide
			 */
			bool testCollision(const AABB& _aabb) const;
			/**
			 * @brief Get the volume of the AABB
			 * @return The 3D volume.
			 */
			float getVolume() const;
			/**
			 * @brief Merge the AABB in parameter with the current one
			 * @param[in] _aabb Other AABB box to merge.
			 */
			void mergeWithAABB(const AABB& _aabb);
			/**
			 * @brief Replace the current AABB with a new AABB that is the union of two AABBs in parameters
			 * @param[in] _aabb1 first AABB box to merge with _aabb2.
			 * @param[in] _aabb2 second AABB box to merge with _aabb1.
			 */
			void mergeTwoAABBs(const AABB& _aabb1, const AABB& _aabb2);
			/**
			 * @brief Return true if the current AABB contains the AABB given in parameter
			 * @param[in] _aabb AABB box that is contains in the current.
			 * @return true The parameter in contained inside
			 */
			bool contains(const AABB& _aabb) const;
			/**
			 * @brief Return true if a point is inside the AABB
			 * @param[in] _point Point to check.
			 * @return true The point in contained inside
			 */
			bool contains(const vec3& _point) const;
			/**
			 * @brief check if the AABB of a triangle intersects the AABB
			 * @param[in] _trianglePoints List of 3 point od a triangle
			 * @return true The triangle is contained in the Box
			 */
			bool testCollisionTriangleAABB(const vec3* _trianglePoints) const;
			/**
			 * @brief check if the ray intersects the AABB
			 * This method use the line vs AABB raycasting technique described in
			 * Real-time Collision Detection by Christer Ericson.
			 * @param[in] _ray Ray to test
			 * @return true The raytest intersect the AABB box
			 */
			bool testRayIntersect(const Ray& _ray) const;
			/**
			 * @brief Create and return an AABB for a triangle
			 * @param[in] _trianglePoints List of 3 point od a triangle
			 * @return An AABB box
			 */
			static AABB createAABBForTriangle(const vec3* _trianglePoints);
			/**
			 * @brief Assignment operator
			 * @param[in] _aabb The other box to compare
			 * @return reference on this
			 */
			AABB& operator=(const AABB& _aabb);
			friend class DynamicAABBTree;
	};


}