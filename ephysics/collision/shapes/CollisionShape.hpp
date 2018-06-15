/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#pragma once

#include <etk/typeInfo.hpp>
#include <etk/math/Vector3D.hpp>
#include <etk/math/Matrix3x3.hpp>
#include <ephysics/mathematics/Ray.hpp>
#include <ephysics/collision/shapes/AABB.hpp>
#include <ephysics/collision/RaycastInfo.hpp>

namespace ephysics {
enum CollisionShapeType {TRIANGLE, BOX, SPHERE, CONE, CYLINDER,
						 CAPSULE, CONVEX_MESH, CONCAVE_MESH, HEIGHTFIELD};
const int32_t NB_COLLISION_SHAPE_TYPES = 9;

class ProxyShape;
class CollisionBody;

/**
 * This abstract class represents the collision shape associated with a
 * body that is used during the narrow-phase collision detection.
 */
class CollisionShape {
	public :
		/// Constructor
		CollisionShape(CollisionShapeType _type);
		/// DELETE copy-constructor
		CollisionShape(const CollisionShape& shape) = delete;
		/// DELETE assignment operator
		CollisionShape& operator=(const CollisionShape& shape) = delete;
		/// Virtualize destructor
		virtual ~CollisionShape() {};
		/**
		 * @brief Get the type of the collision shapes
		 * @return The type of the collision shape (box, sphere, cylinder, ...)
		 */
		CollisionShapeType getType() const {
			return m_type;
		}
		/**
		 * @brief Check if the shape is convex
		 * @return true If the collision shape is convex
		 * @return false If it is concave
		 */
		virtual bool isConvex() const = 0;
		/**
		 * @brief Get the local bounds of the shape in x, y and z directions.
		 * This method is used to compute the AABB of the box
		 * @param _min The minimum bounds of the shape in local-space coordinates
		 * @param _max The maximum bounds of the shape in local-space coordinates
		 */
		virtual void getLocalBounds(vec3& _min, vec3& _max) const=0;
		/// Return the scaling vector of the collision shape
		vec3 getScaling() const {
			return m_scaling;
		}
		/**
		 * @brief Set the scaling vector of the collision shape
		 */
		virtual void setLocalScaling(const vec3& _scaling) {
			m_scaling = _scaling;
		}
		/**
		 * @brief Compute the local inertia tensor of the sphere
		 * @param[out] _tensor The 3x3 inertia tensor matrix of the shape in local-space coordinates
		 * @param[in] _mass Mass to use to compute the inertia tensor of the collision shape
		 */
		virtual void computeLocalInertiaTensor(etk::Matrix3x3& _tensor, float _mass) const=0;
		/**
		 * @brief Update the AABB of a body using its collision shape
		 * @param[out] _aabb The axis-aligned bounding box (AABB) of the collision shape computed in world-space coordinates
		 * @param[in] _transform etk::Transform3D used to compute the AABB of the collision shape
		 */
		virtual void computeAABB(AABB& _aabb, const etk::Transform3D& _transform) const;
		/**
		 * @brief Check if the shape is convex
		 * @param[in] _shapeType shape type
		 * @return true If the collision shape is convex
		 * @return false If it is concave
		 */
		static bool isConvex(CollisionShapeType _shapeType) {
			return    _shapeType != CONCAVE_MESH
			       && _shapeType != HEIGHTFIELD;
		}
		/**
		 * @brief Get the maximum number of contact
		 * @return The maximum number of contact manifolds in an overlapping pair given two shape types
		 */
		static int32_t computeNbMaxContactManifolds(CollisionShapeType _shapeType1,
		                                            CollisionShapeType _shapeType2);
		friend class ProxyShape;
		friend class CollisionWorld;
	protected :
		CollisionShapeType m_type; //!< Type of the collision shape
		vec3 m_scaling; //!< Scaling vector of the collision shape
		/// Return true if a point is inside the collision shape
		virtual bool testPointInside(const vec3& worldPoint, ProxyShape* proxyShape) const = 0;
		/// Raycast method with feedback information
		virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const = 0;
		/// Return the number of bytes used by the collision shape
		virtual size_t getSizeInBytes() const = 0;
};



}

