/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

#include <etk/Vector.hpp>
#include <ephysics/body/CollisionBody.hpp>
#include <ephysics/collision/ProxyShape.hpp>
#include <ephysics/collision/broadphase/DynamicAABBTree.hpp>
#include <ephysics/engine/Profiler.hpp>

namespace ephysics {

	class CollisionDetection;
	class BroadPhaseAlgorithm;
	
	/**
	 * @brief It represent a potential overlapping pair during the
	 * broad-phase collision detection.
	 */
	struct BroadPhasePair {
		int32_t collisionShape1ID; //!< Broad-phase ID of the first collision shape
		int32_t collisionShape2ID; //!< Broad-phase ID of the second collision shape
		/**
		 * @brief Method used to compare two pairs for sorting algorithm
		 * @param[in] _pair1 first pair of element
		 * @param[in] _pair2 Second pair of element
		 * @return _pair1 is smaller than _pair2
		 */
		static bool smallerThan(const BroadPhasePair& _pair1, const BroadPhasePair& _pair2) {
			if (_pair1.collisionShape1ID < _pair2.collisionShape1ID) return true;
			if (_pair1.collisionShape1ID == _pair2.collisionShape1ID) {
				return _pair1.collisionShape2ID < _pair2.collisionShape2ID;
			}
			return false;
		}
	};
	
	class AABBOverlapCallback : public DynamicAABBTreeOverlapCallback {
		private:
			BroadPhaseAlgorithm& m_broadPhaseAlgorithm;
			int32_t m_referenceNodeId;
		public:
			// Constructor
			AABBOverlapCallback(BroadPhaseAlgorithm& _broadPhaseAlgo, int32_t _referenceNodeId):
			  m_broadPhaseAlgorithm(_broadPhaseAlgo),
			  m_referenceNodeId(_referenceNodeId) {
				
			}
			// Called when a overlapping node has been found during the call to
			// DynamicAABBTree:reportAllShapesOverlappingWithAABB()
			virtual void notifyOverlappingNode(int32_t nodeId);
	};
	
	/**
	 * Callback called when the AABB of a leaf node is hit by a ray the
	 * broad-phase Dynamic AABB Tree.
	 */
	class BroadPhaseRaycastCallback : public DynamicAABBTreeRaycastCallback {
		private :
			const DynamicAABBTree& m_dynamicAABBTree;
			unsigned short m_raycastWithCategoryMaskBits;
			RaycastTest& m_raycastTest;
		public:
			// Constructor
			BroadPhaseRaycastCallback(const DynamicAABBTree& _dynamicAABBTree,
			                          unsigned short _raycastWithCategoryMaskBits,
			                          RaycastTest& _raycastTest):
			  m_dynamicAABBTree(_dynamicAABBTree),
			  m_raycastWithCategoryMaskBits(_raycastWithCategoryMaskBits),
			  m_raycastTest(_raycastTest) {
				
			}
			// Called for a broad-phase shape that has to be tested for raycast
			virtual float raycastBroadPhaseShape(int32_t _nodeId, const Ray& _ray);
	};
	
	/**
	 * @brief It represents the broad-phase collision detection. The
	 * goal of the broad-phase collision detection is to compute the pairs of proxy shapes
	 * that have their AABBs overlapping. Only those pairs of bodies will be tested
	 * later for collision during the narrow-phase collision detection. A dynamic AABB
	 * tree data structure is used for fast broad-phase collision detection.
	 */
	class BroadPhaseAlgorithm {
		protected :
			DynamicAABBTree m_dynamicAABBTree; //!< Dynamic AABB tree
			int32_t* m_movedShapes; //!< Array with the broad-phase IDs of all collision shapes that have moved (or have been created) during the last simulation step. Those are the shapes that need to be tested for overlapping in the next simulation step.
			uint32_t m_numberMovedShapes; //!< Number of collision shapes in the array of shapes that have moved during the last simulation step.
			uint32_t m_numberAllocatedMovedShapes; //!< Number of allocated elements for the array of shapes that have moved during the last simulation step.
			uint32_t m_numberNonUsedMovedShapes; //!< Number of non-used elements in the array of shapes that have moved during the last simulation step.
			BroadPhasePair* m_potentialPairs; //!< Temporary array of potential overlapping pairs (with potential duplicates)
			uint32_t m_numberPotentialPairs; //!< Number of potential overlapping pairs
			uint32_t m_numberAllocatedPotentialPairs; //!< Number of allocated elements for the array of potential overlapping pairs
			CollisionDetection& m_collisionDetection; //!< Reference to the collision detection object
			/// Private copy-constructor
			BroadPhaseAlgorithm(const BroadPhaseAlgorithm& algorithm);
			/// Private assignment operator
			BroadPhaseAlgorithm& operator=(const BroadPhaseAlgorithm& algorithm);
		public :
			/// Constructor
			BroadPhaseAlgorithm(CollisionDetection& _collisionDetection);
			/// Destructor
			virtual ~BroadPhaseAlgorithm();
			/// Add a proxy collision shape int32_to the broad-phase collision detection
			void addProxyCollisionShape(ProxyShape* _proxyShape, const AABB& _aabb);
			/// Remove a proxy collision shape from the broad-phase collision detection
			void removeProxyCollisionShape(ProxyShape* _proxyShape);
			/// Notify the broad-phase that a collision shape has moved and need to be updated
			void updateProxyCollisionShape(ProxyShape* _proxyShape,
			                               const AABB& _aabb,
			                               const vec3& _displacement,
			                               bool _forceReinsert = false);
			/// Add a collision shape in the array of shapes that have moved in the last simulation step
			/// and that need to be tested again for broad-phase overlapping.
			void addMovedCollisionShape(int32_t _broadPhaseID);
			/// Remove a collision shape from the array of shapes that have moved in the last simulation
			/// step and that need to be tested again for broad-phase overlapping.
			void removeMovedCollisionShape(int32_t _broadPhaseID);
			/// Notify the broad-phase about a potential overlapping pair in the dynamic AABB tree
			void notifyOverlappingNodes(int32_t _broadPhaseId1, int32_t _broadPhaseId2);
			/// Compute all the overlapping pairs of collision shapes
			void computeOverlappingPairs();
			/// Return true if the two broad-phase collision shapes are overlapping
			bool testOverlappingShapes(const ProxyShape* _shape1, const ProxyShape* _shape2) const;
			/// Ray casting method
			void raycast(const Ray& _ray,
			             RaycastTest& _raycastTest,
			             unsigned short _raycastWithCategoryMaskBits) const;
	};

}

