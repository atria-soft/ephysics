/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

#include <vector>
#include <set>
#include <list>
#include <algorithm>
#include <ephysics/mathematics/mathematics.hpp>
#include <ephysics/engine/Profiler.hpp>
#include <ephysics/body/CollisionBody.hpp>
#include <ephysics/collision/RaycastInfo.hpp>
#include <ephysics/engine/OverlappingPair.hpp>
#include <ephysics/collision/CollisionDetection.hpp>
#include <ephysics/constraint/Joint.hpp>
#include <ephysics/constraint/ContactPoint.hpp>
#include <ephysics/memory/MemoryAllocator.hpp>
#include <ephysics/engine/EventListener.hpp>

namespace ephysics {
	class CollisionCallback;
	/**
	 * @brief This class represent a world where it is possible to move bodies
	 * by hand and to test collision between each other. In this kind of
	 * world, the bodies movement is not computed using the laws of physics.
	 */
	class CollisionWorld {
		protected :
			CollisionDetection m_collisionDetection; //!< Reference to the collision detection
			std::set<CollisionBody*> m_bodies; //!< All the bodies (rigid and soft) of the world
			bodyindex m_currentBodyID; //!< Current body ID
			std::vector<uint64_t> m_freeBodiesIDs; //!< List of free ID for rigid bodies
			MemoryAllocator m_memoryAllocator; //!< Memory allocator
			EventListener* m_eventListener; //!< Pointer to an event listener object
			/// Private copy-constructor
			CollisionWorld(const CollisionWorld& world);
			/// Private assignment operator
			CollisionWorld& operator=(const CollisionWorld& world);
			/// Return the next available body ID
			bodyindex computeNextAvailableBodyID();
			/// Reset all the contact manifolds linked list of each body
			void resetContactManifoldListsOfBodies();
		public :
			/// Constructor
			CollisionWorld();
			/// Destructor
			virtual ~CollisionWorld();
			/**
			 * @brief Get an iterator to the beginning of the bodies of the physics world
			 * @return An starting iterator to the set of bodies of the world
			 */
			std::set<CollisionBody*>::iterator getBodiesBeginIterator() {
				return m_bodies.begin();
			}
			/**
			 * @brief Get an iterator to the end of the bodies of the physics world
			 * @return An ending iterator to the set of bodies of the world
			 */
			std::set<CollisionBody*>::iterator getBodiesEndIterator() {
				return m_bodies.end();
			}
			/// Create a collision body
			CollisionBody* createCollisionBody(const etk::Transform3D& transform);
			/// Destroy a collision body
			void destroyCollisionBody(CollisionBody* collisionBody);
			/**
			 * @brief Set the collision dispatch configuration
			 * This can be used to replace default collision detection algorithms by your
			 * custom algorithm for instance.
			 * @param[in] _CollisionDispatch Pointer to a collision dispatch object describing
			 * which collision detection algorithm to use for two given collision shapes
			 */
			void setCollisionDispatch(CollisionDispatch* _collisionDispatch) {
				m_collisionDetection.setCollisionDispatch(_collisionDispatch);
			}
			/**
			 * @brief Ray cast method
			 * @param _ray Ray to use for raycasting
			 * @param _raycastCallback Pointer to the class with the callback method
			 * @param _raycastWithCategoryMaskBits Bits mask corresponding to the category of bodies to be raycasted
			 */
			void raycast(const Ray& _ray,
			             RaycastCallback* _raycastCallback,
			             unsigned short _raycastWithCategoryMaskBits = 0xFFFF) const {
				m_collisionDetection.raycast(_raycastCallback, _ray, _raycastWithCategoryMaskBits);
			}
			/// Test if the AABBs of two bodies overlap
			bool testAABBOverlap(const CollisionBody* _body1,
			                     const CollisionBody* _body2) const;
			/**
			 * @brief Test if the AABBs of two proxy shapes overlap
			 * @param _shape1 Pointer to the first proxy shape to test
			 * @param _shape2 Pointer to the second proxy shape to test
			 */
			bool testAABBOverlap(const ProxyShape* _shape1,
			                     const ProxyShape* _shape2) const {
				return m_collisionDetection.testAABBOverlap(_shape1, _shape2);
			}
			/// Test and report collisions between a given shape and all the others
			/// shapes of the world
			virtual void testCollision(const ProxyShape* _shape,
			                           CollisionCallback* _callback);
			/// Test and report collisions between two given shapes
			virtual void testCollision(const ProxyShape* _shape1,
			                           const ProxyShape* _shape2,
			                           CollisionCallback* _callback);
			/// Test and report collisions between a body and all the others bodies of the
			/// world
			virtual void testCollision(const CollisionBody* _body,
			                           CollisionCallback* _callback);
			/// Test and report collisions between two bodies
			virtual void testCollision(const CollisionBody* _body1,
			                           const CollisionBody* _body2,
			                           CollisionCallback* _callback);
			/// Test and report collisions between all shapes of the world
			virtual void testCollision(CollisionCallback* _callback);
			friend class CollisionDetection;
			friend class CollisionBody;
			friend class RigidBody;
			friend class ConvexMeshShape;
	};
	
	/**
	 * @brief This class can be used to register a callback for collision test queries.
	 * You should implement your own class inherited from this one and implement
	 * the notifyRaycastHit() method. This method will be called for each ProxyShape
	 * that is hit by the ray.
	 */
	class CollisionCallback {
		public:
			/**
			 * @brief Virtualisation of the destructor.
			 */
			virtual ~CollisionCallback() = default;
			/**
			 * @brief This method will be called for contact.
			 * @param[in] _contactPointInfo Contact information property.
			 */
			virtual void notifyContact(const ContactPointInfo& _contactPointInfo)=0;
	};
}
