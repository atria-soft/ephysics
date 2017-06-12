/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <vector>
#include <ephysics/body/CollisionBody.h>
#include <ephysics/collision/ProxyShape.h>
#include <ephysics/collision/broadphase/DynamicAABBTree.h>
#include <ephysics/engine/Profiler.h>

/// Namespace ReactPhysics3D
namespace reactphysics3d {

// Declarations
class CollisionDetection;
class BroadPhaseAlgorithm;

// Structure BroadPhasePair
/**
 * This structure represent a potential overlapping pair during the
 * broad-phase collision detection.
 */
struct BroadPhasePair {

	// -------------------- Attributes -------------------- //

	/// Broad-phase ID of the first collision shape
	int32_t collisionShape1ID;

	/// Broad-phase ID of the second collision shape
	int32_t collisionShape2ID;

	// -------------------- Methods -------------------- //

	/// Method used to compare two pairs for sorting algorithm
	static bool smallerThan(const BroadPhasePair& pair1, const BroadPhasePair& pair2);
};

// class AABBOverlapCallback
class AABBOverlapCallback : public DynamicAABBTreeOverlapCallback {

	private:

		BroadPhaseAlgorithm& m_broadPhaseAlgorithm;

		int32_t m_referenceNodeId;

	public:

		// Constructor
		AABBOverlapCallback(BroadPhaseAlgorithm& broadPhaseAlgo, int32_t referenceNodeId)
			 : m_broadPhaseAlgorithm(broadPhaseAlgo), m_referenceNodeId(referenceNodeId) {

		}

		// Called when a overlapping node has been found during the call to
		// DynamicAABBTree:reportAllShapesOverlappingWithAABB()
		virtual void notifyOverlappingNode(int32_t nodeId);

};

// Class BroadPhaseRaycastCallback
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
		BroadPhaseRaycastCallback(const DynamicAABBTree& dynamicAABBTree, unsigned short raycastWithCategoryMaskBits,
								  RaycastTest& raycastTest)
			: m_dynamicAABBTree(dynamicAABBTree), m_raycastWithCategoryMaskBits(raycastWithCategoryMaskBits),
			  m_raycastTest(raycastTest) {

		}

		// Called for a broad-phase shape that has to be tested for raycast
		virtual float raycastBroadPhaseShape(int32_t nodeId, const Ray& ray);

};

// Class BroadPhaseAlgorithm
/**
 * This class represents the broad-phase collision detection. The
 * goal of the broad-phase collision detection is to compute the pairs of proxy shapes
 * that have their AABBs overlapping. Only those pairs of bodies will be tested
 * later for collision during the narrow-phase collision detection. A dynamic AABB
 * tree data structure is used for fast broad-phase collision detection.
 */
class BroadPhaseAlgorithm {

	protected :

		// -------------------- Attributes -------------------- //

		/// Dynamic AABB tree
		DynamicAABBTree m_dynamicAABBTree;

		/// Array with the broad-phase IDs of all collision shapes that have moved (or have been
		/// created) during the last simulation step. Those are the shapes that need to be tested
		/// for overlapping in the next simulation step.
		int32_t* m_movedShapes;

		/// Number of collision shapes in the array of shapes that have moved during the last
		/// simulation step.
		uint32_t m_numberMovedShapes;

		/// Number of allocated elements for the array of shapes that have moved during the last
		/// simulation step.
		uint32_t m_numberAllocatedMovedShapes;

		/// Number of non-used elements in the array of shapes that have moved during the last
		/// simulation step.
		uint32_t m_numberNonUsedMovedShapes;

		/// Temporary array of potential overlapping pairs (with potential duplicates)
		BroadPhasePair* m_potentialPairs;

		/// Number of potential overlapping pairs
		uint32_t m_numberPotentialPairs;

		/// Number of allocated elements for the array of potential overlapping pairs
		uint32_t m_numberAllocatedPotentialPairs;

		/// Reference to the collision detection object
		CollisionDetection& m_collisionDetection;
		
		// -------------------- Methods -------------------- //

		/// Private copy-constructor
		BroadPhaseAlgorithm(const BroadPhaseAlgorithm& algorithm);

		/// Private assignment operator
		BroadPhaseAlgorithm& operator=(const BroadPhaseAlgorithm& algorithm);

	public :

		// -------------------- Methods -------------------- //

		/// Constructor
		BroadPhaseAlgorithm(CollisionDetection& collisionDetection);

		/// Destructor
		virtual ~BroadPhaseAlgorithm();
		
		/// Add a proxy collision shape int32_to the broad-phase collision detection
		void addProxyCollisionShape(ProxyShape* proxyShape, const AABB& aabb);

		/// Remove a proxy collision shape from the broad-phase collision detection
		void removeProxyCollisionShape(ProxyShape* proxyShape);

		/// Notify the broad-phase that a collision shape has moved and need to be updated
		void updateProxyCollisionShape(ProxyShape* proxyShape, const AABB& aabb,
									   const Vector3& displacement, bool forceReinsert = false);

		/// Add a collision shape in the array of shapes that have moved in the last simulation step
		/// and that need to be tested again for broad-phase overlapping.
		void addMovedCollisionShape(int32_t broadPhaseID);

		/// Remove a collision shape from the array of shapes that have moved in the last simulation
		/// step and that need to be tested again for broad-phase overlapping.
		void removeMovedCollisionShape(int32_t broadPhaseID);

		/// Notify the broad-phase about a potential overlapping pair in the dynamic AABB tree
		void notifyOverlappingNodes(int32_t broadPhaseId1, int32_t broadPhaseId2);

		/// Compute all the overlapping pairs of collision shapes
		void computeOverlappingPairs();

		/// Return true if the two broad-phase collision shapes are overlapping
		bool testOverlappingShapes(const ProxyShape* shape1, const ProxyShape* shape2) const;

		/// Ray casting method
		void raycast(const Ray& ray, RaycastTest& raycastTest,
					 unsigned short raycastWithCategoryMaskBits) const;
};

// Method used to compare two pairs for sorting algorithm
inline bool BroadPhasePair::smallerThan(const BroadPhasePair& pair1, const BroadPhasePair& pair2) {

	if (pair1.collisionShape1ID < pair2.collisionShape1ID) return true;
	if (pair1.collisionShape1ID == pair2.collisionShape1ID) {
		return pair1.collisionShape2ID < pair2.collisionShape2ID;
	}
	return false;
}

// Return true if the two broad-phase collision shapes are overlapping
inline bool BroadPhaseAlgorithm::testOverlappingShapes(const ProxyShape* shape1,
													   const ProxyShape* shape2) const {
	// Get the two AABBs of the collision shapes
	const AABB& aabb1 = m_dynamicAABBTree.getFatAABB(shape1->m_broadPhaseID);
	const AABB& aabb2 = m_dynamicAABBTree.getFatAABB(shape2->m_broadPhaseID);

	// Check if the two AABBs are overlapping
	return aabb1.testCollision(aabb2);
}

// Ray casting method
inline void BroadPhaseAlgorithm::raycast(const Ray& ray, RaycastTest& raycastTest,
										 unsigned short raycastWithCategoryMaskBits) const {

	PROFILE("BroadPhaseAlgorithm::raycast()");

	BroadPhaseRaycastCallback broadPhaseRaycastCallback(m_dynamicAABBTree, raycastWithCategoryMaskBits, raycastTest);

	m_dynamicAABBTree.raycast(ray, broadPhaseRaycastCallback);
}

}



