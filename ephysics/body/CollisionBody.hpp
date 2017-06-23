/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <stdexcept>
#include <cassert>
#include <ephysics/body/Body.hpp>
#include <etk/math/Transform3D.hpp>
#include <ephysics/collision/shapes/AABB.hpp>
#include <ephysics/collision/shapes/CollisionShape.hpp>
#include <ephysics/collision/RaycastInfo.hpp>
#include <ephysics/memory/MemoryAllocator.hpp>
#include <ephysics/configuration.hpp>

namespace ephysics {
	struct ContactManifoldListElement;
	class ProxyShape;
	class CollisionWorld;
	/**
	 * @brief Define the type of the body
	 */
	enum BodyType {
		STATIC, //!< A static body has infinite mass, zero velocity but the position can be changed manually. A static body does not collide with other static or kinematic bodies.
		KINEMATIC, //!< A kinematic body has infinite mass, the velocity can be changed manually and its position is computed by the physics engine. A kinematic body does not collide with other static or kinematic bodies.
		DYNAMIC //!< A dynamic body has non-zero mass, non-zero velocity determined by forces and its position is determined by the physics engine. A dynamic body can collide with other dynamic, static or kinematic bodies.
	};
	/**
	 * @brief This class represents a body that is able to collide with others bodies. This class inherits from the Body class.
	 */
	class CollisionBody : public Body {
		protected :
			BodyType m_type; //!< Type of body (static, kinematic or dynamic)
			etk::Transform3D m_transform; //!< Position and orientation of the body
			ProxyShape* m_proxyCollisionShapes; //!< First element of the linked list of proxy collision shapes of this body
			uint32_t m_numberCollisionShapes; //!< Number of collision shapes
			ContactManifoldListElement* m_contactManifoldsList; //!< First element of the linked list of contact manifolds involving this body
			CollisionWorld& m_world; //!< Reference to the world the body belongs to
			/// Private copy-constructor
			CollisionBody(const CollisionBody& _body) = delete;
			/// Private assignment operator
			CollisionBody& operator=(const CollisionBody& _body) = delete;
			/**
			 * @brief Reset the contact manifold lists
			 */
			void resetContactManifoldsList();
			/**
			 * @brief Remove all the collision shapes
			 */
			void removeAllCollisionShapes();
			/**
			 * @brief Update the broad-phase state for this body (because it has moved for instance)
			 */
			virtual void updateBroadPhaseState() const;
			/**
			 * @brief Update the broad-phase state of a proxy collision shape of the body
			 */
			void updateProxyShapeInBroadPhase(ProxyShape* _proxyShape, bool _forceReinsert = false) const;
			/**
			 * @brief Ask the broad-phase to test again the collision shapes of the body for collision (as if the body has moved).
			 */
			void askForBroadPhaseCollisionCheck() const;
			/**
			 * @brief Reset the m_isAlreadyInIsland variable of the body and contact manifolds.
			 * This method also returns the number of contact manifolds of the body.
			 */
			int32_t resetIsAlreadyInIslandAndCountManifolds();
		public :
			/**
			 * @brief Constructor
			 * @param[in] _transform The transform of the body
			 * @param[in] _world The physics world where the body is created
			 * @param[in] _id ID of the body
			 */
			CollisionBody(const etk::Transform3D& _transform, CollisionWorld& _world, bodyindex _id);
			/**
			 * @brief Destructor
			 */
			virtual ~CollisionBody();
			/**
			 * @brief Return the type of the body
			 * @return the type of the body (STATIC, KINEMATIC, DYNAMIC)
			 */
			BodyType getType() const {
				return m_type;
			}
			/**
			 * @brief Set the type of the body
			 * @param[in] type The type of the body (STATIC, KINEMATIC, DYNAMIC)
			 */
			virtual void setType(BodyType _type);
			/**
			 * @brief Set whether or not the body is active
			 * @param[in] _isActive True if you want to activate the body
			 */
			virtual void setIsActive(bool _isActive);
			/**
			 * @brief Return the current position and orientation
			 * @return The current transformation of the body that transforms the local-space of the body int32_to world-space
			 */
			const etk::Transform3D& getTransform() const {
				return m_transform;
			}
			/**
			 * @brief Set the current position and orientation
			 * @param transform The transformation of the body that transforms the local-space of the body int32_to world-space
			 */
			void setTransform(const etk::Transform3D& _transform) {
				m_transform = _transform;
				updateBroadPhaseState();
			}
			/**
			 * @brief Add a collision shape to the body. Note that you can share a collision shape between several bodies using the same collision shape instance to
			 * when you add the shape to the different bodies. Do not forget to delete the collision shape you have created at the end of your program.
			 * 
			 * This method will return a pointer to a new proxy shape. A proxy shape is an object that links a collision shape and a given body. You can use the
			 * returned proxy shape to get and set information about the corresponding collision shape for that body.
			 * @param[in] collisionShape A pointer to the collision shape you want to add to the body
			 * @param[in] transform The transformation of the collision shape that transforms the local-space of the collision shape int32_to the local-space of the body
			 * @return A pointer to the proxy shape that has been created to link the body to the new collision shape you have added.
			 */
			ProxyShape* addCollisionShape(CollisionShape* _collisionShape, const etk::Transform3D& _transform);
			/**
			 * @brief Remove a collision shape from the body
			 * To remove a collision shape, you need to specify the pointer to the proxy shape that has been returned when you have added the collision shape to the body
			 * @param[in] _proxyShape The pointer of the proxy shape you want to remove
			 */
			virtual void removeCollisionShape(const ProxyShape* _proxyShape);
			/**
			 * @brief Get the first element of the linked list of contact manifolds involving this body
			 * @return A pointer to the first element of the linked-list with the contact manifolds of this body
			 */
			const ContactManifoldListElement* getContactManifoldsList() const {
				return m_contactManifoldsList;
			}
			/**
			 * @brief Return true if a point is inside the collision body
			 * This method returns true if a point is inside any collision shape of the body
			 * @param[in] _worldPoint The point to test (in world-space coordinates)
			 * @return True if the point is inside the body
			 */
			bool testPointInside(const vec3& _worldPoint) const;
			/**
			 * @brief Raycast method with feedback information
			 * The method returns the closest hit among all the collision shapes of the body
			 * @param[in] _ray The ray used to raycast agains the body
			 * @param[out] _raycastInfo Structure that contains the result of the raycasting (valid only if the method returned true)
			 * @return True if the ray hit the body and false otherwise
			 */
			bool raycast(const Ray& _ray, RaycastInfo& _raycastInfo);
			/**
			 * @brief Compute and return the AABB of the body by merging all proxy shapes AABBs
			 * @return The axis-aligned bounding box (AABB) of the body in world-space coordinates
			 */
			AABB getAABB() const;
			/**
			 * @brief Get the linked list of proxy shapes of that body
			 * @return The pointer of the first proxy shape of the linked-list of all the
			 *		 proxy shapes of the body
			 */
			ProxyShape* getProxyShapesList() {
				return m_proxyCollisionShapes;
			}
			/**
			 * @brief Get the linked list of proxy shapes of that body
			 * @return The pointer of the first proxy shape of the linked-list of all the proxy shapes of the body
			 */
			const ProxyShape* getProxyShapesList() const {
				return m_proxyCollisionShapes;
			}
			/**
			 * @brief Get the world-space coordinates of a point given the local-space coordinates of the body
			 * @param[in] _localPoint A point in the local-space coordinates of the body
			 * @return The point in world-space coordinates
			 */
			vec3 getWorldPoint(const vec3& _localPoint) const {
				return m_transform * _localPoint;
			}
			/**
			 * @brief Get the world-space vector of a vector given in local-space coordinates of the body
			 * @param[in] _localVector A vector in the local-space coordinates of the body
			 * @return The vector in world-space coordinates
			 */
			vec3 getWorldVector(const vec3& _localVector) const {
				return m_transform.getOrientation() * _localVector;
			}
			/**
			 * @brief Get the body local-space coordinates of a point given in the world-space coordinates
			 * @param[in] _worldPoint A point in world-space coordinates
			 * @return The point in the local-space coordinates of the body
			 */
			vec3 getLocalPoint(const vec3& _worldPoint) const {
				return m_transform.getInverse() * _worldPoint;
			}
			/**
			 * @brief Get the body local-space coordinates of a vector given in the world-space coordinates
			 * @param[in] _worldVector A vector in world-space coordinates
			 * @return The vector in the local-space coordinates of the body
			 */
			vec3 getLocalVector(const vec3& _worldVector) const {
				return m_transform.getOrientation().getInverse() * _worldVector;
			}
			friend class CollisionWorld;
			friend class DynamicsWorld;
			friend class CollisionDetection;
			friend class BroadPhaseAlgorithm;
			friend class ConvexMeshShape;
			friend class ProxyShape;
	};
}

