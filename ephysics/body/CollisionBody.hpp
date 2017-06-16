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

/// Namespace ephysics
namespace ephysics {

// Class declarations
struct ContactManifoldListElement;
class ProxyShape;
class CollisionWorld;

/// Enumeration for the type of a body
/// STATIC : A static body has infinite mass, zero velocity but the position can be
///		  changed manually. A static body does not collide with other static or kinematic bodies.
/// KINEMATIC : A kinematic body has infinite mass, the velocity can be changed manually and its
///			 position is computed by the physics engine. A kinematic body does not collide with
///			 other static or kinematic bodies.
/// DYNAMIC : A dynamic body has non-zero mass, non-zero velocity determined by forces and its
///		   position is determined by the physics engine. A dynamic body can collide with other
///		   dynamic, static or kinematic bodies.
enum BodyType {STATIC, KINEMATIC, DYNAMIC};

// Class CollisionBody
/**
 * This class represents a body that is able to collide with others
 * bodies. This class inherits from the Body class.
 */
class CollisionBody : public Body {

	protected :

		// -------------------- Attributes -------------------- //

		/// Type of body (static, kinematic or dynamic)
		BodyType m_type;

		/// Position and orientation of the body
		etk::Transform3D m_transform;

		/// First element of the linked list of proxy collision shapes of this body
		ProxyShape* m_proxyCollisionShapes;

		/// Number of collision shapes
		uint32_t m_numberCollisionShapes;

		/// First element of the linked list of contact manifolds involving this body
		ContactManifoldListElement* m_contactManifoldsList;

		/// Reference to the world the body belongs to
		CollisionWorld& m_world;

		// -------------------- Methods -------------------- //

		/// Private copy-constructor
		CollisionBody(const CollisionBody& body);

		/// Private assignment operator
		CollisionBody& operator=(const CollisionBody& body);

		/// Reset the contact manifold lists
		void resetContactManifoldsList();

		/// Remove all the collision shapes
		void removeAllCollisionShapes();

		/// Update the broad-phase state for this body (because it has moved for instance)
		virtual void updateBroadPhaseState() const;

		/// Update the broad-phase state of a proxy collision shape of the body
		void updateProxyShapeInBroadPhase(ProxyShape* proxyShape, bool forceReinsert = false) const;

		/// Ask the broad-phase to test again the collision shapes of the body for collision
		/// (as if the body has moved).
		void askForBroadPhaseCollisionCheck() const;

		/// Reset the m_isAlreadyInIsland variable of the body and contact manifolds
		int32_t resetIsAlreadyInIslandAndCountManifolds();

	public :

		// -------------------- Methods -------------------- //

		/// Constructor
		CollisionBody(const etk::Transform3D& transform, CollisionWorld& world, bodyindex id);

		/// Destructor
		virtual ~CollisionBody();

		/// Return the type of the body
		BodyType getType() const;
		/**
		 * @brief Set the type of the body
		 * The type of the body can either STATIC, KINEMATIC or DYNAMIC as described bellow:
		 * STATIC : A static body has infinite mass, zero velocity but the position can be
		 *          changed manually. A static body does not collide with other static or kinematic bodies.
		 * KINEMATIC : A kinematic body has infinite mass, the velocity can be changed manually and its
		 *             position is computed by the physics engine. A kinematic body does not collide with
		 *             other static or kinematic bodies.
		 * DYNAMIC : A dynamic body has non-zero mass, non-zero velocity determined by forces and its
		 *           position is determined by the physics engine. A dynamic body can collide with other
		 *           dynamic, static or kinematic bodies.
		 * @param[in] type The type of the body (STATIC, KINEMATIC, DYNAMIC)
		 */
		virtual void setType(BodyType _type);

		/// Set whether or not the body is active
		virtual void setIsActive(bool isActive);

		/// Return the current position and orientation
		const etk::Transform3D& getTransform() const;

		/// Set the current position and orientation
		virtual void setTransform(const etk::Transform3D& transform);

		/// Add a collision shape to the body.
		virtual ProxyShape* addCollisionShape(CollisionShape* collisionShape,
											  const etk::Transform3D& transform);

		/// Remove a collision shape from the body
		virtual void removeCollisionShape(const ProxyShape* proxyShape);

		/// Return the first element of the linked list of contact manifolds involving this body
		const ContactManifoldListElement* getContactManifoldsList() const;

		/// Return true if a point is inside the collision body
		bool testPointInside(const vec3& worldPoint) const;

		/// Raycast method with feedback information
		bool raycast(const Ray& ray, RaycastInfo& raycastInfo);

		/// Compute and return the AABB of the body by merging all proxy shapes AABBs
		AABB getAABB() const;

		/// Return the linked list of proxy shapes of that body
		ProxyShape* getProxyShapesList();

		/// Return the linked list of proxy shapes of that body
		const ProxyShape* getProxyShapesList() const;

		/// Return the world-space coordinates of a point given the local-space coordinates of the body
		vec3 getWorldPoint(const vec3& localPoint) const;

		/// Return the world-space vector of a vector given in local-space coordinates of the body
		vec3 getWorldVector(const vec3& localVector) const;

		/// Return the body local-space coordinates of a point given in the world-space coordinates
		vec3 getLocalPoint(const vec3& worldPoint) const;

		/// Return the body local-space coordinates of a vector given in the world-space coordinates
		vec3 getLocalVector(const vec3& worldVector) const;

		// -------------------- Friendship -------------------- //

		friend class CollisionWorld;
		friend class DynamicsWorld;
		friend class CollisionDetection;
		friend class BroadPhaseAlgorithm;
		friend class ConvexMeshShape;
		friend class ProxyShape;
};

// Return the type of the body
/**
 * @return the type of the body (STATIC, KINEMATIC, DYNAMIC)
 */
inline BodyType CollisionBody::getType() const {
	return m_type;
}


// Return the current position and orientation
/**
 * @return The current transformation of the body that transforms the local-space
 *		 of the body int32_to world-space
 */
inline const etk::Transform3D& CollisionBody::getTransform() const {
	return m_transform;
}

// Set the current position and orientation
/**
 * @param transform The transformation of the body that transforms the local-space
 *				  of the body int32_to world-space
 */
inline void CollisionBody::setTransform(const etk::Transform3D& transform) {

	// Update the transform of the body
	m_transform = transform;

	// Update the broad-phase state of the body
	updateBroadPhaseState();
}

// Return the first element of the linked list of contact manifolds involving this body
/**
 * @return A pointer to the first element of the linked-list with the contact
 *		 manifolds of this body
 */
inline const ContactManifoldListElement* CollisionBody::getContactManifoldsList() const {
	return m_contactManifoldsList;
}

// Return the linked list of proxy shapes of that body
/**
* @return The pointer of the first proxy shape of the linked-list of all the
*		 proxy shapes of the body
*/
inline ProxyShape* CollisionBody::getProxyShapesList() {
	return m_proxyCollisionShapes;
}

// Return the linked list of proxy shapes of that body
/**
* @return The pointer of the first proxy shape of the linked-list of all the
*		 proxy shapes of the body
*/
inline const ProxyShape* CollisionBody::getProxyShapesList() const {
	return m_proxyCollisionShapes;
}

// Return the world-space coordinates of a point given the local-space coordinates of the body
/**
* @param localPoint A point in the local-space coordinates of the body
* @return The point in world-space coordinates
*/
inline vec3 CollisionBody::getWorldPoint(const vec3& localPoint) const {
	return m_transform * localPoint;
}

// Return the world-space vector of a vector given in local-space coordinates of the body
/**
* @param localVector A vector in the local-space coordinates of the body
* @return The vector in world-space coordinates
*/
inline vec3 CollisionBody::getWorldVector(const vec3& localVector) const {
	return m_transform.getOrientation() * localVector;
}

// Return the body local-space coordinates of a point given in the world-space coordinates
/**
* @param worldPoint A point in world-space coordinates
* @return The point in the local-space coordinates of the body
*/
inline vec3 CollisionBody::getLocalPoint(const vec3& worldPoint) const {
	return m_transform.getInverse() * worldPoint;
}

// Return the body local-space coordinates of a vector given in the world-space coordinates
/**
* @param worldVector A vector in world-space coordinates
* @return The vector in the local-space coordinates of the body
*/
inline vec3 CollisionBody::getLocalVector(const vec3& worldVector) const {
	return m_transform.getOrientation().getInverse() * worldVector;
}

}
