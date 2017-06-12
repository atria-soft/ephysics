/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/body/CollisionBody.h>
#include <ephysics/collision/shapes/CollisionShape.h>

namespace  reactphysics3d {

// Class ProxyShape
/**
 * The CollisionShape instances are supposed to be unique for memory optimization. For instance,
 * consider two rigid bodies with the same sphere collision shape. In this situation, we will have
 * a unique instance of SphereShape but we need to differentiate between the two instances during
 * the collision detection. They do not have the same position in the world and they do not
 * belong to the same rigid body. The ProxyShape class is used for that purpose by attaching a
 * rigid body with one of its collision shape. A body can have multiple proxy shapes (one for
 * each collision shape attached to the body).
 */
class ProxyShape {

	protected:

		// -------------------- Attributes -------------------- //

		/// Pointer to the parent body
		CollisionBody* m_body;

		/// Internal collision shape
		CollisionShape* m_collisionShape;

		/// Local-space to parent body-space transform (does not change over time)
		Transform m_localToBodyTransform;

		/// Mass (in kilogramms) of the corresponding collision shape
		float m_mass;

		/// Pointer to the next proxy shape of the body (linked list)
		ProxyShape* m_next;

		/// Broad-phase ID (node ID in the dynamic AABB tree)
		int32_t m_broadPhaseID;

		/// Cached collision data
		void* m_cachedCollisionData;

		/// Pointer to user data
		void* m_userData;

		/// Bits used to define the collision category of this shape.
		/// You can set a single bit to one to define a category value for this
		/// shape. This value is one (0x0001) by default. This variable can be used
		/// together with the m_collideWithMaskBits variable so that given
		/// categories of shapes collide with each other and do not collide with
		/// other categories.
		unsigned short m_collisionCategoryBits;

		/// Bits mask used to state which collision categories this shape can
		/// collide with. This value is 0xFFFF by default. It means that this
		/// proxy shape will collide with every collision categories by default.
		unsigned short m_collideWithMaskBits;

		// -------------------- Methods -------------------- //

		/// Private copy-constructor
		ProxyShape(const ProxyShape& proxyShape);

		/// Private assignment operator
		ProxyShape& operator=(const ProxyShape& proxyShape);

	public:

		// -------------------- Methods -------------------- //

		/// Constructor
		ProxyShape(CollisionBody* body, CollisionShape* shape,
				   const Transform& transform, float mass);

		/// Destructor
		virtual ~ProxyShape();

		/// Return the collision shape
		const CollisionShape* getCollisionShape() const;

		/// Return the parent body
		CollisionBody* getBody() const;

		/// Return the mass of the collision shape
		float getMass() const;

		/// Return a pointer to the user data attached to this body
		void* getUserData() const;

		/// Attach user data to this body
		void setUserData(void* userData);

		/// Return the local to parent body transform
		const Transform& getLocalToBodyTransform() const;

		/// Set the local to parent body transform
		void setLocalToBodyTransform(const Transform& transform);

		/// Return the local to world transform
		const Transform getLocalToWorldTransform() const;

		/// Return true if a point is inside the collision shape
		bool testPointInside(const Vector3& worldPoint);

		/// Raycast method with feedback information
		bool raycast(const Ray& ray, RaycastInfo& raycastInfo);

		/// Return the collision bits mask
		unsigned short getCollideWithMaskBits() const;

		/// Set the collision bits mask
		void setCollideWithMaskBits(unsigned short collideWithMaskBits);

		/// Return the collision category bits
		unsigned short getCollisionCategoryBits() const;

		/// Set the collision category bits
		void setCollisionCategoryBits(unsigned short collisionCategoryBits);

		/// Return the next proxy shape in the linked list of proxy shapes
		ProxyShape* getNext();

		/// Return the next proxy shape in the linked list of proxy shapes
		const ProxyShape* getNext() const;

		/// Return the pointer to the cached collision data
		void** getCachedCollisionData();

		/// Return the local scaling vector of the collision shape
		Vector3 getLocalScaling() const;

		/// Set the local scaling vector of the collision shape
		virtual void setLocalScaling(const Vector3& scaling);

		// -------------------- Friendship -------------------- //

		friend class OverlappingPair;
		friend class CollisionBody;
		friend class RigidBody;
		friend class BroadPhaseAlgorithm;
		friend class DynamicAABBTree;
		friend class CollisionDetection;
		friend class CollisionWorld;
		friend class DynamicsWorld;
		friend class EPAAlgorithm;
		friend class GJKAlgorithm;
		friend class ConvexMeshShape;

};

// Return the pointer to the cached collision data
inline void** ProxyShape::getCachedCollisionData()  {
	return &m_cachedCollisionData;
}

// Return the collision shape
/**
 * @return Pointer to the int32_ternal collision shape
 */
inline const CollisionShape* ProxyShape::getCollisionShape() const {
	return m_collisionShape;
}

// Return the parent body
/**
 * @return Pointer to the parent body
 */
inline CollisionBody* ProxyShape::getBody() const {
	return m_body;
}

// Return the mass of the collision shape
/**
 * @return Mass of the collision shape (in kilograms)
 */
inline float ProxyShape::getMass() const {
	return m_mass;
}

// Return a pointer to the user data attached to this body
/**
 * @return A pointer to the user data stored int32_to the proxy shape
 */
inline void* ProxyShape::getUserData() const {
	return m_userData;
}

// Attach user data to this body
/**
 * @param userData Pointer to the user data you want to store within the proxy shape
 */
inline void ProxyShape::setUserData(void* userData) {
	m_userData = userData;
}

// Return the local to parent body transform
/**
 * @return The transformation that transforms the local-space of the collision shape
 *		 to the local-space of the parent body
 */
inline const Transform& ProxyShape::getLocalToBodyTransform() const {
	return m_localToBodyTransform;
}

// Set the local to parent body transform
inline void ProxyShape::setLocalToBodyTransform(const Transform& transform) {

	m_localToBodyTransform = transform;

	m_body->setIsSleeping(false);

	// Notify the body that the proxy shape has to be updated in the broad-phase
	m_body->updateProxyShapeInBroadPhase(this, true);
}

// Return the local to world transform
/**
 * @return The transformation that transforms the local-space of the collision
 *		 shape to the world-space
 */
inline const Transform ProxyShape::getLocalToWorldTransform() const {
	return m_body->m_transform * m_localToBodyTransform;
}

// Return the next proxy shape in the linked list of proxy shapes
/**
 * @return Pointer to the next proxy shape in the linked list of proxy shapes
 */
inline ProxyShape* ProxyShape::getNext() {
	return m_next;
}

// Return the next proxy shape in the linked list of proxy shapes
/**
 * @return Pointer to the next proxy shape in the linked list of proxy shapes
 */
inline const ProxyShape* ProxyShape::getNext() const {
	return m_next;
}

// Return the collision category bits
/**
 * @return The collision category bits mask of the proxy shape
 */
inline unsigned short ProxyShape::getCollisionCategoryBits() const {
	return m_collisionCategoryBits;
}

// Set the collision category bits
/**
 * @param collisionCategoryBits The collision category bits mask of the proxy shape
 */
inline void ProxyShape::setCollisionCategoryBits(unsigned short collisionCategoryBits) {
	m_collisionCategoryBits = collisionCategoryBits;
}

// Return the collision bits mask
/**
 * @return The bits mask that specifies with which collision category this shape will collide
 */
inline unsigned short ProxyShape::getCollideWithMaskBits() const {
	return m_collideWithMaskBits;
}

// Set the collision bits mask
/**
 * @param collideWithMaskBits The bits mask that specifies with which collision category this shape will collide
 */
inline void ProxyShape::setCollideWithMaskBits(unsigned short collideWithMaskBits) {
	m_collideWithMaskBits = collideWithMaskBits;
}

// Return the local scaling vector of the collision shape
/**
 * @return The local scaling vector
 */
inline Vector3 ProxyShape::getLocalScaling() const {
	return m_collisionShape->getScaling();
}

// Set the local scaling vector of the collision shape
/**
 * @param scaling The new local scaling vector
 */
inline void ProxyShape::setLocalScaling(const Vector3& scaling) {

	// Set the local scaling of the collision shape
	m_collisionShape->setLocalScaling(scaling);

	m_body->setIsSleeping(false);

	// Notify the body that the proxy shape has to be updated in the broad-phase
	m_body->updateProxyShapeInBroadPhase(this, true);
}

}

