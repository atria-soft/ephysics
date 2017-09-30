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
	
	// TODO : remove this as callback ... DynamicAABBTreeOverlapCallback {
	/**
	 * Callback called when the AABB of a leaf node is hit by a ray the
	 * broad-phase Dynamic AABB Tree.
	 */
	class BroadPhaseRaycastCallback {
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
			float operator()(int32_t _nodeId, const Ray& _ray);
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
			etk::Vector<etk::Pair<int32_t,int32_t>> m_potentialPairs; //!< Temporary array of potential overlapping pairs (with potential duplicates)
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

