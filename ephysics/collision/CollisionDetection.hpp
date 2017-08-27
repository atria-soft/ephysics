/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

#include <ephysics/body/CollisionBody.hpp>
#include <ephysics/collision/broadphase/BroadPhaseAlgorithm.hpp>
#include <ephysics/engine/OverlappingPair.hpp>
#include <ephysics/engine/EventListener.hpp>
#include <ephysics/collision/narrowphase/DefaultCollisionDispatch.hpp>
#include <ephysics/memory/MemoryAllocator.hpp>
#include <ephysics/constraint/ContactPoint.hpp>
#include <etk/Vector.hpp>
#include <etk/Map.hpp>
#include <set>
#include <utility>

namespace ephysics {

	class BroadPhaseAlgorithm;
	class CollisionWorld;
	class CollisionCallback;
	
	class TestCollisionBetweenShapesCallback : public NarrowPhaseCallback {
		private:
			CollisionCallback* m_collisionCallback; //!< 
		public:
			// Constructor
			TestCollisionBetweenShapesCallback(CollisionCallback* _callback):
			  m_collisionCallback(_callback) {
				
			}
	
			// Called by a narrow-phase collision algorithm when a new contact has been found
			virtual void notifyContact(OverlappingPair* _overlappingPair,
			                           const ContactPointInfo& _contactInfo);
	};
	
	/**
	 * @brief It computes the collision detection algorithms. We first
	 * perform a broad-phase algorithm to know which pairs of bodies can
	 * collide and then we run a narrow-phase algorithm to compute the
	 * collision contacts between bodies.
	 */
	class CollisionDetection : public NarrowPhaseCallback {
		private :
			CollisionDispatch* m_collisionDispatch; //!< Collision Detection Dispatch configuration
			DefaultCollisionDispatch m_defaultCollisionDispatch; //!< Default collision dispatch configuration
			NarrowPhaseAlgorithm* m_collisionMatrix[NB_COLLISION_SHAPE_TYPES][NB_COLLISION_SHAPE_TYPES]; //!< Collision detection matrix (algorithms to use)
			MemoryAllocator& m_memoryAllocator; //!< Reference to the memory allocator
			CollisionWorld* m_world; //!< Pointer to the physics world
			etk::Map<overlappingpairid, OverlappingPair*> m_overlappingPairs; //!< Broad-phase overlapping pairs
			etk::Map<overlappingpairid, OverlappingPair*> m_contactOverlappingPairs; //!< Overlapping pairs in contact (during the current Narrow-phase collision detection)
			BroadPhaseAlgorithm m_broadPhaseAlgorithm; //!< Broad-phase algorithm
			// TODO : Delete this
			GJKAlgorithm m_narrowPhaseGJKAlgorithm; //!< Narrow-phase GJK algorithm
			std::set<bodyindexpair> m_noCollisionPairs; //!< Set of pair of bodies that cannot collide between each other
			bool m_isCollisionShapesAdded; //!< True if some collision shapes have been added previously
			/// Private copy-constructor
			CollisionDetection(const CollisionDetection& _collisionDetection);
			/// Private assignment operator
			CollisionDetection& operator=(const CollisionDetection& _collisionDetection);
			/// Compute the broad-phase collision detection
			void computeBroadPhase();
			/// Compute the narrow-phase collision detection
			void computeNarrowPhase();
			/// Add a contact manifold to the linked list of contact manifolds of the two bodies
			/// involed in the corresponding contact.
			void addContactManifoldToBody(OverlappingPair* _pair);
			/// Delete all the contact points in the currently overlapping pairs
			void clearContactPoints();
			/// Fill-in the collision detection matrix
			void fillInCollisionMatrix();
			/// Add all the contact manifold of colliding pairs to their bodies
			void addAllContactManifoldsToBodies();
		public :
			/// Constructor
			CollisionDetection(CollisionWorld* _world, MemoryAllocator& _memoryAllocator);
			/// Destructor
			~CollisionDetection();
			/// Set the collision dispatch configuration
			void setCollisionDispatch(CollisionDispatch* _collisionDispatch);
			/// Return the Narrow-phase collision detection algorithm to use between two types of shapes
			NarrowPhaseAlgorithm* getCollisionAlgorithm(CollisionShapeType _shape1Type,
			                                            CollisionShapeType _shape2Type) const;
			/// Add a proxy collision shape to the collision detection
			void addProxyCollisionShape(ProxyShape* _proxyShape, const AABB& _aabb);
			/// Remove a proxy collision shape from the collision detection
			void removeProxyCollisionShape(ProxyShape* _proxyShape);
			/// Update a proxy collision shape (that has moved for instance)
			void updateProxyCollisionShape(ProxyShape* _shape,
			                               const AABB& _aabb,
			                               const vec3& _displacement = vec3(0, 0, 0),
			                               bool _forceReinsert = false);
			/// Add a pair of bodies that cannot collide with each other
			void addNoCollisionPair(CollisionBody* _body1, CollisionBody* _body2);
			/// Remove a pair of bodies that cannot collide with each other
			void removeNoCollisionPair(CollisionBody* _body1, CollisionBody* _body2);
			/// Ask for a collision shape to be tested again during broad-phase.
			void askForBroadPhaseCollisionCheck(ProxyShape* _shape);
			/// Compute the collision detection
			void computeCollisionDetection();
			/// Compute the collision detection
			void testCollisionBetweenShapes(CollisionCallback* _callback,
			                                const std::set<uint32_t>& _shapes1,
			                                const std::set<uint32_t>& _shapes2);
			/// Report collision between two sets of shapes
			void reportCollisionBetweenShapes(CollisionCallback* _callback,
			                                  const std::set<uint32_t>& _shapes1,
			                                  const std::set<uint32_t>& _shapes2) ;
			/// Ray casting method
			void raycast(RaycastCallback* _raycastCallback,
			             const Ray& _ray,
			             unsigned short _raycastWithCategoryMaskBits) const;
			/// Test if the AABBs of two bodies overlap
			bool testAABBOverlap(const CollisionBody* _body1,
			                     const CollisionBody* _body2) const;
			/// Test if the AABBs of two proxy shapes overlap
			bool testAABBOverlap(const ProxyShape* _shape1,
			                     const ProxyShape* _shape2) const;
			/// Allow the broadphase to notify the collision detection about an overlapping pair.
			void broadPhaseNotifyOverlappingPair(ProxyShape* _shape1, ProxyShape* _shape2);
			/// Compute the narrow-phase collision detection
			void computeNarrowPhaseBetweenShapes(CollisionCallback* _callback,
			                                     const std::set<uint32_t>& _shapes1,
			                                     const std::set<uint32_t>& _shapes2);
			/// Return a pointer to the world
			CollisionWorld* getWorld();
			/// Return the world event listener
			EventListener* getWorldEventListener();
			/// Return a reference to the world memory allocator
			MemoryAllocator& getWorldMemoryAllocator();
			/// Called by a narrow-phase collision algorithm when a new contact has been found
			virtual void notifyContact(OverlappingPair* _overlappingPair, const ContactPointInfo& _contactInfo) override;
			/// Create a new contact
			void createContact(OverlappingPair* _overlappingPair, const ContactPointInfo& _contactInfo);
			friend class DynamicsWorld;
			friend class ConvexMeshShape;
	};

}
