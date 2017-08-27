/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/broadphase/BroadPhaseAlgorithm.hpp>
#include <ephysics/collision/CollisionDetection.hpp>
#include <ephysics/engine/Profiler.hpp>

using namespace ephysics;

BroadPhaseAlgorithm::BroadPhaseAlgorithm(CollisionDetection& collisionDetection)
					:m_dynamicAABBTree(DYNAMIC_TREE_AABB_GAP), m_numberMovedShapes(0), m_numberAllocatedMovedShapes(8),
					 m_numberNonUsedMovedShapes(0), m_numberPotentialPairs(0), m_numberAllocatedPotentialPairs(8),
					 m_collisionDetection(collisionDetection) {

	// Allocate memory for the array of non-static proxy shapes IDs
	m_movedShapes = (int32_t*) malloc(m_numberAllocatedMovedShapes * sizeof(int32_t));
	assert(m_movedShapes != NULL);

	// Allocate memory for the array of potential overlapping pairs
	m_potentialPairs = (BroadPhasePair*) malloc(m_numberAllocatedPotentialPairs * sizeof(BroadPhasePair));
	assert(m_potentialPairs != NULL);
}

BroadPhaseAlgorithm::~BroadPhaseAlgorithm() {

	// Release the memory for the array of non-static proxy shapes IDs
	free(m_movedShapes);

	// Release the memory for the array of potential overlapping pairs
	free(m_potentialPairs);
}

void BroadPhaseAlgorithm::addMovedCollisionShape(int32_t broadPhaseID) {

	// Allocate more elements in the array of shapes that have moved if necessary
	if (m_numberAllocatedMovedShapes == m_numberMovedShapes) {
		m_numberAllocatedMovedShapes *= 2;
		int32_t* oldArray = m_movedShapes;
		m_movedShapes = (int32_t*) malloc(m_numberAllocatedMovedShapes * sizeof(int32_t));
		assert(m_movedShapes != NULL);
		memcpy(m_movedShapes, oldArray, m_numberMovedShapes * sizeof(int32_t));
		free(oldArray);
	}

	// Store the broad-phase ID int32_to the array of shapes that have moved
	assert(m_numberMovedShapes < m_numberAllocatedMovedShapes);
	assert(m_movedShapes != NULL);
	m_movedShapes[m_numberMovedShapes] = broadPhaseID;
	m_numberMovedShapes++;
}

void BroadPhaseAlgorithm::removeMovedCollisionShape(int32_t broadPhaseID) {

	assert(m_numberNonUsedMovedShapes <= m_numberMovedShapes);

	// If less than the quarter of allocated elements of the non-static shapes IDs array
	// are used, we release some allocated memory
	if ((m_numberMovedShapes - m_numberNonUsedMovedShapes) < m_numberAllocatedMovedShapes / 4 &&
			m_numberAllocatedMovedShapes > 8) {

		m_numberAllocatedMovedShapes /= 2;
		int32_t* oldArray = m_movedShapes;
		m_movedShapes = (int32_t*) malloc(m_numberAllocatedMovedShapes * sizeof(int32_t));
		assert(m_movedShapes != NULL);
		uint32_t nbElements = 0;
		for (uint32_t i=0; i<m_numberMovedShapes; i++) {
			if (oldArray[i] != -1) {
				m_movedShapes[nbElements] = oldArray[i];
				nbElements++;
			}
		}
		m_numberMovedShapes = nbElements;
		m_numberNonUsedMovedShapes = 0;
		free(oldArray);
	}

	// Remove the broad-phase ID from the array
	for (uint32_t i=0; i<m_numberMovedShapes; i++) {
		if (m_movedShapes[i] == broadPhaseID) {
			m_movedShapes[i] = -1;
			m_numberNonUsedMovedShapes++;
			break;
		}
	}
}

void BroadPhaseAlgorithm::addProxyCollisionShape(ProxyShape* proxyShape, const AABB& aabb) {

	// Add the collision shape int32_to the dynamic AABB tree and get its broad-phase ID
	int32_t nodeId = m_dynamicAABBTree.addObject(aabb, proxyShape);

	// Set the broad-phase ID of the proxy shape
	proxyShape->m_broadPhaseID = nodeId;

	// Add the collision shape int32_to the array of bodies that have moved (or have been created)
	// during the last simulation step
	addMovedCollisionShape(proxyShape->m_broadPhaseID);
}

void BroadPhaseAlgorithm::removeProxyCollisionShape(ProxyShape* proxyShape) {

	int32_t broadPhaseID = proxyShape->m_broadPhaseID;

	// Remove the collision shape from the dynamic AABB tree
	m_dynamicAABBTree.removeObject(broadPhaseID);

	// Remove the collision shape int32_to the array of shapes that have moved (or have been created)
	// during the last simulation step
	removeMovedCollisionShape(broadPhaseID);
}

void BroadPhaseAlgorithm::updateProxyCollisionShape(ProxyShape* _proxyShape,
                                                    const AABB& _aabb,
                                                    const vec3& _displacement,
                                                    bool _forceReinsert) {
	int32_t broadPhaseID = _proxyShape->m_broadPhaseID;
	assert(broadPhaseID >= 0);
	// Update the dynamic AABB tree according to the movement of the collision shape
	bool hasBeenReInserted = m_dynamicAABBTree.updateObject(broadPhaseID, _aabb, _displacement, _forceReinsert);
	// If the collision shape has moved out of its fat AABB (and therefore has been reinserted
	// int32_to the tree).
	if (hasBeenReInserted) {
		// Add the collision shape int32_to the array of shapes that have moved (or have been created)
		// during the last simulation step
		addMovedCollisionShape(broadPhaseID);
	}
}

void BroadPhaseAlgorithm::computeOverlappingPairs() {

	// Reset the potential overlapping pairs
	m_numberPotentialPairs = 0;

	// For all collision shapes that have moved (or have been created) during the
	// last simulation step
	for (uint32_t i=0; i<m_numberMovedShapes; i++) {
		int32_t shapeID = m_movedShapes[i];

		if (shapeID == -1) continue;

		AABBOverlapCallback callback(*this, shapeID);

		// Get the AABB of the shape
		const AABB& shapeAABB = m_dynamicAABBTree.getFatAABB(shapeID);

		// Ask the dynamic AABB tree to report all collision shapes that overlap with
		// this AABB. The method BroadPhase::notifiyOverlappingPair() will be called
		// by the dynamic AABB tree for each potential overlapping pair.
		m_dynamicAABBTree.reportAllShapesOverlappingWithAABB(shapeAABB, callback);
	}

	// Reset the array of collision shapes that have move (or have been created) during the
	// last simulation step
	m_numberMovedShapes = 0;

	// Sort the array of potential overlapping pairs in order to remove duplicate pairs
	std::sort(m_potentialPairs, m_potentialPairs + m_numberPotentialPairs, BroadPhasePair::smallerThan);

	// Check all the potential overlapping pairs avoiding duplicates to report unique
	// overlapping pairs
	uint32_t i=0;
	while (i < m_numberPotentialPairs) {

		// Get a potential overlapping pair
		BroadPhasePair* pair = m_potentialPairs + i;
		i++;

		assert(pair->collisionShape1ID != pair->collisionShape2ID);

		// Get the two collision shapes of the pair
		ProxyShape* shape1 = static_cast<ProxyShape*>(m_dynamicAABBTree.getNodeDataPointer(pair->collisionShape1ID));
		ProxyShape* shape2 = static_cast<ProxyShape*>(m_dynamicAABBTree.getNodeDataPointer(pair->collisionShape2ID));

		// Notify the collision detection about the overlapping pair
		m_collisionDetection.broadPhaseNotifyOverlappingPair(shape1, shape2);

		// Skip the duplicate overlapping pairs
		while (i < m_numberPotentialPairs) {

			// Get the next pair
			BroadPhasePair* nextPair = m_potentialPairs + i;

			// If the next pair is different from the previous one, we stop skipping pairs
			if (nextPair->collisionShape1ID != pair->collisionShape1ID ||
				nextPair->collisionShape2ID != pair->collisionShape2ID) {
				break;
			}
			i++;
		}
	}

	// If the number of potential overlapping pairs is less than the quarter of allocated
	// number of overlapping pairs
	if (m_numberPotentialPairs < m_numberAllocatedPotentialPairs / 4 && m_numberPotentialPairs > 8) {

		// Reduce the number of allocated potential overlapping pairs
		BroadPhasePair* oldPairs = m_potentialPairs;
		m_numberAllocatedPotentialPairs /= 2;
		m_potentialPairs = (BroadPhasePair*) malloc(m_numberAllocatedPotentialPairs * sizeof(BroadPhasePair));
		assert(m_potentialPairs);
		memcpy(m_potentialPairs, oldPairs, m_numberPotentialPairs * sizeof(BroadPhasePair));
		free(oldPairs);
	}
}

void BroadPhaseAlgorithm::notifyOverlappingNodes(int32_t node1ID, int32_t node2ID) {

	// If both the nodes are the same, we do not create store the overlapping pair
	if (node1ID == node2ID) return;

	// If we need to allocate more memory for the array of potential overlapping pairs
	if (m_numberPotentialPairs == m_numberAllocatedPotentialPairs) {

		// Allocate more memory for the array of potential pairs
		BroadPhasePair* oldPairs = m_potentialPairs;
		m_numberAllocatedPotentialPairs *= 2;
		m_potentialPairs = (BroadPhasePair*) malloc(m_numberAllocatedPotentialPairs * sizeof(BroadPhasePair));
		assert(m_potentialPairs);
		memcpy(m_potentialPairs, oldPairs, m_numberPotentialPairs * sizeof(BroadPhasePair));
		free(oldPairs);
	}

	// Add the new potential pair int32_to the array of potential overlapping pairs
	m_potentialPairs[m_numberPotentialPairs].collisionShape1ID = etk::min(node1ID, node2ID);
	m_potentialPairs[m_numberPotentialPairs].collisionShape2ID = etk::max(node1ID, node2ID);
	m_numberPotentialPairs++;
}

void AABBOverlapCallback::notifyOverlappingNode(int32_t nodeId) {

	m_broadPhaseAlgorithm.notifyOverlappingNodes(m_referenceNodeId, nodeId);
}

float BroadPhaseRaycastCallback::raycastBroadPhaseShape(int32_t nodeId, const Ray& ray) {

	float hitFraction = float(-1.0);

	// Get the proxy shape from the node
	ProxyShape* proxyShape = static_cast<ProxyShape*>(m_dynamicAABBTree.getNodeDataPointer(nodeId));

	// Check if the raycast filtering mask allows raycast against this shape
	if ((m_raycastWithCategoryMaskBits & proxyShape->getCollisionCategoryBits()) != 0) {

		// Ask the collision detection to perform a ray cast test against
		// the proxy shape of this node because the ray is overlapping
		// with the shape in the broad-phase
		hitFraction = m_raycastTest.raycastAgainstShape(proxyShape, ray);
	}

	return hitFraction;
}

bool BroadPhaseAlgorithm::testOverlappingShapes(const ProxyShape* _shape1,
                                                const ProxyShape* _shape2) const {
	// Get the two AABBs of the collision shapes
	const AABB& aabb1 = m_dynamicAABBTree.getFatAABB(_shape1->m_broadPhaseID);
	const AABB& aabb2 = m_dynamicAABBTree.getFatAABB(_shape2->m_broadPhaseID);

	// Check if the two AABBs are overlapping
	return aabb1.testCollision(aabb2);
}

void BroadPhaseAlgorithm::raycast(const Ray& _ray,
                                  RaycastTest& _raycastTest,
                                  unsigned short _raycastWithCategoryMaskBits) const {
	PROFILE("BroadPhaseAlgorithm::raycast()");
	BroadPhaseRaycastCallback broadPhaseRaycastCallback(m_dynamicAABBTree, _raycastWithCategoryMaskBits, _raycastTest);
	m_dynamicAABBTree.raycast(_ray, broadPhaseRaycastCallback);
}

