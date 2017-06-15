/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <vector>
#include <set>
#include <list>
#include <algorithm>
#include <ephysics/mathematics/mathematics.h>
#include <ephysics/engine/Profiler.h>
#include <ephysics/body/CollisionBody.h>
#include <ephysics/collision/RaycastInfo.h>
#include <ephysics/engine/OverlappingPair.h>
#include <ephysics/collision/CollisionDetection.h>
#include <ephysics/constraint/Joint.h>
#include <ephysics/constraint/ContactPoint.h>
#include <ephysics/memory/MemoryAllocator.h>
#include <ephysics/engine/EventListener.h>

/// Namespace reactphysics3d
namespace reactphysics3d {

// Declarations
class CollisionCallback;

// Class CollisionWorld
/**
 * This class represent a world where it is possible to move bodies
 * by hand and to test collision between each other. In this kind of
 * world, the bodies movement is not computed using the laws of physics.
 */
class CollisionWorld {

	protected :

		// -------------------- Attributes -------------------- //

		/// Reference to the collision detection
		CollisionDetection m_collisionDetection;

		/// All the bodies (rigid and soft) of the world
		std::set<CollisionBody*> m_bodies;

		/// Current body ID
		bodyindex m_currentBodyID;

		/// List of free ID for rigid bodies
		std::vector<uint64_t> m_freeBodiesIDs;

		/// Memory allocator
		MemoryAllocator m_memoryAllocator;

		/// Pointer to an event listener object
		EventListener* m_eventListener;

		// -------------------- Methods -------------------- //

		/// Private copy-constructor
		CollisionWorld(const CollisionWorld& world);

		/// Private assignment operator
		CollisionWorld& operator=(const CollisionWorld& world);

		/// Return the next available body ID
		bodyindex computeNextAvailableBodyID();

		/// Reset all the contact manifolds linked list of each body
		void resetContactManifoldListsOfBodies();

	public :

		// -------------------- Methods -------------------- //

		/// Constructor
		CollisionWorld();

		/// Destructor
		virtual ~CollisionWorld();

		/// Return an iterator to the beginning of the bodies of the physics world
		std::set<CollisionBody*>::iterator getBodiesBeginIterator();

		/// Return an iterator to the end of the bodies of the physics world
		std::set<CollisionBody*>::iterator getBodiesEndIterator();

		/// Create a collision body
		CollisionBody* createCollisionBody(const etk::Transform3D& transform);

		/// Destroy a collision body
		void destroyCollisionBody(CollisionBody* collisionBody);

		/// Set the collision dispatch configuration
		void setCollisionDispatch(CollisionDispatch* collisionDispatch);

		/// Ray cast method
		void raycast(const Ray& ray, RaycastCallback* raycastCallback,
					 unsigned short raycastWithCategoryMaskBits = 0xFFFF) const;

		/// Test if the AABBs of two bodies overlap
		bool testAABBOverlap(const CollisionBody* body1,
							 const CollisionBody* body2) const;

		/// Test if the AABBs of two proxy shapes overlap
		bool testAABBOverlap(const ProxyShape* shape1,
							 const ProxyShape* shape2) const;

		/// Test and report collisions between a given shape and all the others
		/// shapes of the world
		virtual void testCollision(const ProxyShape* shape,
								   CollisionCallback* callback);

		/// Test and report collisions between two given shapes
		virtual void testCollision(const ProxyShape* shape1,
								   const ProxyShape* shape2,
								   CollisionCallback* callback);

		/// Test and report collisions between a body and all the others bodies of the
		/// world
		virtual void testCollision(const CollisionBody* body,
								   CollisionCallback* callback);

		/// Test and report collisions between two bodies
		virtual void testCollision(const CollisionBody* body1,
								   const CollisionBody* body2,
								   CollisionCallback* callback);

		/// Test and report collisions between all shapes of the world
		virtual void testCollision(CollisionCallback* callback);

		// -------------------- Friendship -------------------- //

		friend class CollisionDetection;
		friend class CollisionBody;
		friend class RigidBody;
		friend class ConvexMeshShape;
};

// Return an iterator to the beginning of the bodies of the physics world
/**
 * @return An starting iterator to the set of bodies of the world
 */
inline std::set<CollisionBody*>::iterator CollisionWorld::getBodiesBeginIterator() {
	return m_bodies.begin();
}

// Return an iterator to the end of the bodies of the physics world
/**
 * @return An ending iterator to the set of bodies of the world
 */
inline std::set<CollisionBody*>::iterator CollisionWorld::getBodiesEndIterator() {
	return m_bodies.end();
}

// Set the collision dispatch configuration
/// This can be used to replace default collision detection algorithms by your
/// custom algorithm for instance.
/**
 * @param CollisionDispatch Pointer to a collision dispatch object describing
 * which collision detection algorithm to use for two given collision shapes
 */
inline void CollisionWorld::setCollisionDispatch(CollisionDispatch* collisionDispatch) {
	m_collisionDetection.setCollisionDispatch(collisionDispatch);
}

// Ray cast method
/**
 * @param ray Ray to use for raycasting
 * @param raycastCallback Pointer to the class with the callback method
 * @param raycastWithCategoryMaskBits Bits mask corresponding to the category of
 *									bodies to be raycasted
 */
inline void CollisionWorld::raycast(const Ray& ray,
									RaycastCallback* raycastCallback,
									unsigned short raycastWithCategoryMaskBits) const {
	m_collisionDetection.raycast(raycastCallback, ray, raycastWithCategoryMaskBits);
}

// Test if the AABBs of two proxy shapes overlap
/**
 * @param shape1 Pointer to the first proxy shape to test
 * @param shape2 Pointer to the second proxy shape to test
 * @return
 */
inline bool CollisionWorld::testAABBOverlap(const ProxyShape* shape1,
											const ProxyShape* shape2) const {

	return m_collisionDetection.testAABBOverlap(shape1, shape2);
}

// Class CollisionCallback
/**
 * This class can be used to register a callback for collision test queries.
 * You should implement your own class inherited from this one and implement
 * the notifyRaycastHit() method. This method will be called for each ProxyShape
 * that is hit by the ray.
 */
class CollisionCallback {

	public:

		/// Destructor
		virtual ~CollisionCallback() {

		}

		/// This method will be called for contact
		virtual void notifyContact(const ContactPointInfo& contactPointInfo)=0;
};

}
