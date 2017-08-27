/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

#include <ephysics/collision/broadphase/DynamicAABBTree.hpp>
#include <ephysics/collision/broadphase/BroadPhaseAlgorithm.hpp>
#include <ephysics/memory/Stack.hpp>
#include <ephysics/engine/Profiler.hpp>
#include <ephysics/debug.hpp>

using namespace ephysics;

const int32_t TreeNode::NULL_TREE_NODE = -1;

DynamicAABBTree::DynamicAABBTree(float extraAABBGap) : m_extraAABBGap(extraAABBGap) {

	init();
}

DynamicAABBTree::~DynamicAABBTree() {

	free(m_nodes);
}

// Initialize the tree
void DynamicAABBTree::init() {

	m_rootNodeID = TreeNode::NULL_TREE_NODE;
	m_numberNodes = 0;
	m_numberAllocatedNodes = 8;

	// Allocate memory for the nodes of the tree
	m_nodes = (TreeNode*) malloc(m_numberAllocatedNodes * sizeof(TreeNode));
	assert(m_nodes);
	memset(m_nodes, 0, m_numberAllocatedNodes * sizeof(TreeNode));

	// Initialize the allocated nodes
	for (int32_t i=0; i<m_numberAllocatedNodes - 1; i++) {
		m_nodes[i].nextNodeID = i + 1;
		m_nodes[i].height = -1;
	}
	m_nodes[m_numberAllocatedNodes - 1].nextNodeID = TreeNode::NULL_TREE_NODE;
	m_nodes[m_numberAllocatedNodes - 1].height = -1;
	m_freeNodeID = 0;
}

// Clear all the nodes and reset the tree
void DynamicAABBTree::reset() {

	// Free the allocated memory for the nodes
	free(m_nodes);

	// Initialize the tree
	init();
}

// Allocate and return a new node in the tree
int32_t DynamicAABBTree::allocateNode() {
	// If there is no more allocated node to use
	if (m_freeNodeID == TreeNode::NULL_TREE_NODE) {
		assert(m_numberNodes == m_numberAllocatedNodes);
		// Allocate more nodes in the tree
		m_numberAllocatedNodes *= 2;
		TreeNode* oldNodes = m_nodes;
		m_nodes = (TreeNode*) malloc(m_numberAllocatedNodes * sizeof(TreeNode));
		assert(m_nodes);
		memcpy(m_nodes, oldNodes, m_numberNodes * sizeof(TreeNode));
		free(oldNodes);
		// Initialize the allocated nodes
		for (int32_t i=m_numberNodes; i<m_numberAllocatedNodes - 1; i++) {
			m_nodes[i].nextNodeID = i + 1;
			m_nodes[i].height = -1;
		}
		m_nodes[m_numberAllocatedNodes - 1].nextNodeID = TreeNode::NULL_TREE_NODE;
		m_nodes[m_numberAllocatedNodes - 1].height = -1;
		m_freeNodeID = m_numberNodes;
	}
	// Get the next free node
	int32_t freeNodeID = m_freeNodeID;
	m_freeNodeID = m_nodes[freeNodeID].nextNodeID;
	m_nodes[freeNodeID].parentID = TreeNode::NULL_TREE_NODE;
	m_nodes[freeNodeID].height = 0;
	m_numberNodes++;
	return freeNodeID;
}

// Release a node
void DynamicAABBTree::releaseNode(int32_t _nodeID) {

	assert(m_numberNodes > 0);
	assert(_nodeID >= 0 && _nodeID < m_numberAllocatedNodes);
	assert(m_nodes[_nodeID].height >= 0);
	m_nodes[_nodeID].nextNodeID = m_freeNodeID;
	m_nodes[_nodeID].height = -1;
	m_freeNodeID = _nodeID;
	m_numberNodes--;
}

// Internally add an object int32_to the tree
int32_t DynamicAABBTree::addObjectInternal(const AABB& aabb) {

	// Get the next available node (or allocate new ones if necessary)
	int32_t _nodeID = allocateNode();

	// Create the fat aabb to use in the tree
	const vec3 gap(m_extraAABBGap, m_extraAABBGap, m_extraAABBGap);
	m_nodes[_nodeID].aabb.setMin(aabb.getMin() - gap);
	m_nodes[_nodeID].aabb.setMax(aabb.getMax() + gap);

	// Set the height of the node in the tree
	m_nodes[_nodeID].height = 0;

	// Insert the new leaf node in the tree
	insertLeafNode(_nodeID);
	assert(m_nodes[_nodeID].isLeaf());

	assert(_nodeID >= 0);

	// Return the Id of the node
	return _nodeID;
}

// Remove an object from the tree
void DynamicAABBTree::removeObject(int32_t _nodeID) {

	assert(_nodeID >= 0 && _nodeID < m_numberAllocatedNodes);
	assert(m_nodes[_nodeID].isLeaf());

	// Remove the node from the tree
	removeLeafNode(_nodeID);
	releaseNode(_nodeID);
}


// Update the dynamic tree after an object has moved.
/// If the new AABB of the object that has moved is still inside its fat AABB, then
/// nothing is done. Otherwise, the corresponding node is removed and reinserted int32_to the tree.
/// The method returns true if the object has been reinserted int32_to the tree. The "displacement"
/// argument is the linear velocity of the AABB multiplied by the elapsed time between two
/// frames. If the "forceReinsert" parameter is true, we force a removal and reinsertion of the node
/// (this can be useful if the shape AABB has become much smaller than the previous one for instance).
bool DynamicAABBTree::updateObject(int32_t _nodeID, const AABB& _newAABB, const vec3& _displacement, bool _forceReinsert) {

	PROFILE("DynamicAABBTree::updateObject()");

	assert(_nodeID >= 0 && _nodeID < m_numberAllocatedNodes);
	assert(m_nodes[_nodeID].isLeaf());
	assert(m_nodes[_nodeID].height >= 0);

	EPHY_INFO(" compare : " << m_nodes[_nodeID].aabb.m_minCoordinates << " " << m_nodes[_nodeID].aabb.m_maxCoordinates);
	EPHY_INFO("         : " << _newAABB.m_minCoordinates << " " << _newAABB.m_maxCoordinates);
	// If the new AABB is still inside the fat AABB of the node
	if (    _forceReinsert == false
	     && m_nodes[_nodeID].aabb.contains(_newAABB)) {
		return false;
	}

	// If the new AABB is outside the fat AABB, we remove the corresponding node
	removeLeafNode(_nodeID);

	// Compute the fat AABB by inflating the AABB with a constant gap
	m_nodes[_nodeID].aabb = _newAABB;
	const vec3 gap(m_extraAABBGap, m_extraAABBGap, m_extraAABBGap);
	m_nodes[_nodeID].aabb.m_minCoordinates -= gap;
	m_nodes[_nodeID].aabb.m_maxCoordinates += gap;

	// Inflate the fat AABB in direction of the linear motion of the AABB
	if (_displacement.x() < 0.0f) {
		m_nodes[_nodeID].aabb.m_minCoordinates.setX(m_nodes[_nodeID].aabb.m_minCoordinates.x() + DYNAMIC_TREE_AABB_LIN_GAP_MULTIPLIER *_displacement.x());
	} else {
		m_nodes[_nodeID].aabb.m_maxCoordinates.setX(m_nodes[_nodeID].aabb.m_maxCoordinates.x() + DYNAMIC_TREE_AABB_LIN_GAP_MULTIPLIER *_displacement.x());
	}
	if (_displacement.y() < 0.0f) {
		m_nodes[_nodeID].aabb.m_minCoordinates.setY(m_nodes[_nodeID].aabb.m_minCoordinates.y() + DYNAMIC_TREE_AABB_LIN_GAP_MULTIPLIER *_displacement.y());
	} else {
		m_nodes[_nodeID].aabb.m_maxCoordinates.setY(m_nodes[_nodeID].aabb.m_maxCoordinates.y() + DYNAMIC_TREE_AABB_LIN_GAP_MULTIPLIER *_displacement.y());
	}
	if (_displacement.z() < 0.0f) {
		m_nodes[_nodeID].aabb.m_minCoordinates.setZ(m_nodes[_nodeID].aabb.m_minCoordinates.z() + DYNAMIC_TREE_AABB_LIN_GAP_MULTIPLIER *_displacement.z());
	} else {
		m_nodes[_nodeID].aabb.m_maxCoordinates.setZ(m_nodes[_nodeID].aabb.m_maxCoordinates.z() + DYNAMIC_TREE_AABB_LIN_GAP_MULTIPLIER *_displacement.z());
	}
	EPHY_ERROR(" compare : " << m_nodes[_nodeID].aabb.m_minCoordinates << " " << m_nodes[_nodeID].aabb.m_maxCoordinates);
	EPHY_ERROR("         : " << _newAABB.m_minCoordinates << " " << _newAABB.m_maxCoordinates);
	if (m_nodes[_nodeID].aabb.contains(_newAABB) == false) {
		//EPHY_CRITICAL("ERROR");
	}
	assert(m_nodes[_nodeID].aabb.contains(_newAABB));

	// Reinsert the node int32_to the tree
	insertLeafNode(_nodeID);

	return true;
}

// Insert a leaf node in the tree. The process of inserting a new leaf node
// in the dynamic tree is described in the book "Introduction to Game Physics
// with Box2D" by Ian Parberry.
void DynamicAABBTree::insertLeafNode(int32_t _nodeID) {

	// If the tree is empty
	if (m_rootNodeID == TreeNode::NULL_TREE_NODE) {
		m_rootNodeID = _nodeID;
		m_nodes[m_rootNodeID].parentID = TreeNode::NULL_TREE_NODE;
		return;
	}

	assert(m_rootNodeID != TreeNode::NULL_TREE_NODE);

	// Find the best sibling node for the new node
	AABB newNodeAABB = m_nodes[_nodeID].aabb;
	int32_t currentNodeID = m_rootNodeID;
	while (!m_nodes[currentNodeID].isLeaf()) {

		int32_t leftChild = m_nodes[currentNodeID].children[0];
		int32_t rightChild = m_nodes[currentNodeID].children[1];

		// Compute the merged AABB
		float volumeAABB = m_nodes[currentNodeID].aabb.getVolume();
		AABB mergedAABBs;
		mergedAABBs.mergeTwoAABBs(m_nodes[currentNodeID].aabb, newNodeAABB);
		float mergedVolume = mergedAABBs.getVolume();

		// Compute the cost of making the current node the sibbling of the new node
		float costS = float(2.0) * mergedVolume;

		// Compute the minimum cost of pushing the new node further down the tree (inheritance cost)
		float costI = float(2.0) * (mergedVolume - volumeAABB);

		// Compute the cost of descending int32_to the left child
		float costLeft;
		AABB currentAndLeftAABB;
		currentAndLeftAABB.mergeTwoAABBs(newNodeAABB, m_nodes[leftChild].aabb);
		if (m_nodes[leftChild].isLeaf()) {   // If the left child is a leaf
			costLeft = currentAndLeftAABB.getVolume() + costI;
		}
		else {
			float leftChildVolume = m_nodes[leftChild].aabb.getVolume();
			costLeft = costI + currentAndLeftAABB.getVolume() - leftChildVolume;
		}

		// Compute the cost of descending int32_to the right child
		float costRight;
		AABB currentAndRightAABB;
		currentAndRightAABB.mergeTwoAABBs(newNodeAABB, m_nodes[rightChild].aabb);
		if (m_nodes[rightChild].isLeaf()) {   // If the right child is a leaf
			costRight = currentAndRightAABB.getVolume() + costI;
		}
		else {
			float rightChildVolume = m_nodes[rightChild].aabb.getVolume();
			costRight = costI + currentAndRightAABB.getVolume() - rightChildVolume;
		}

		// If the cost of making the current node a sibbling of the new node is smaller than
		// the cost of going down int32_to the left or right child
		if (costS < costLeft && costS < costRight) break;

		// It is cheaper to go down int32_to a child of the current node, choose the best child
		if (costLeft < costRight) {
			currentNodeID = leftChild;
		}
		else {
			currentNodeID = rightChild;
		}
	}

	int32_t siblingNode = currentNodeID;

	// Create a new parent for the new node and the sibling node
	int32_t oldParentNode = m_nodes[siblingNode].parentID;
	int32_t newParentNode = allocateNode();
	m_nodes[newParentNode].parentID = oldParentNode;
	m_nodes[newParentNode].aabb.mergeTwoAABBs(m_nodes[siblingNode].aabb, newNodeAABB);
	m_nodes[newParentNode].height = m_nodes[siblingNode].height + 1;
	assert(m_nodes[newParentNode].height > 0);

	// If the sibling node was not the root node
	if (oldParentNode != TreeNode::NULL_TREE_NODE) {
		assert(!m_nodes[oldParentNode].isLeaf());
		if (m_nodes[oldParentNode].children[0] == siblingNode) {
			m_nodes[oldParentNode].children[0] = newParentNode;
		}
		else {
			m_nodes[oldParentNode].children[1] = newParentNode;
		}
		m_nodes[newParentNode].children[0] = siblingNode;
		m_nodes[newParentNode].children[1] = _nodeID;
		m_nodes[siblingNode].parentID = newParentNode;
		m_nodes[_nodeID].parentID = newParentNode;
	}
	else {  // If the sibling node was the root node
		m_nodes[newParentNode].children[0] = siblingNode;
		m_nodes[newParentNode].children[1] = _nodeID;
		m_nodes[siblingNode].parentID = newParentNode;
		m_nodes[_nodeID].parentID = newParentNode;
		m_rootNodeID = newParentNode;
	}

	// Move up in the tree to change the AABBs that have changed
	currentNodeID = m_nodes[_nodeID].parentID;
	assert(!m_nodes[currentNodeID].isLeaf());
	while (currentNodeID != TreeNode::NULL_TREE_NODE) {

		// Balance the sub-tree of the current node if it is not balanced
		currentNodeID = balanceSubTreeAtNode(currentNodeID);
		assert(m_nodes[_nodeID].isLeaf());

		assert(!m_nodes[currentNodeID].isLeaf());
		int32_t leftChild = m_nodes[currentNodeID].children[0];
		int32_t rightChild = m_nodes[currentNodeID].children[1];
		assert(leftChild != TreeNode::NULL_TREE_NODE);
		assert(rightChild != TreeNode::NULL_TREE_NODE);

		// Recompute the height of the node in the tree
		m_nodes[currentNodeID].height = etk::max(m_nodes[leftChild].height,
												m_nodes[rightChild].height) + 1;
		assert(m_nodes[currentNodeID].height > 0);

		// Recompute the AABB of the node
		m_nodes[currentNodeID].aabb.mergeTwoAABBs(m_nodes[leftChild].aabb, m_nodes[rightChild].aabb);

		currentNodeID = m_nodes[currentNodeID].parentID;
	}

	assert(m_nodes[_nodeID].isLeaf());
}

// Remove a leaf node from the tree
void DynamicAABBTree::removeLeafNode(int32_t _nodeID) {

	assert(_nodeID >= 0 && _nodeID < m_numberAllocatedNodes);
	assert(m_nodes[_nodeID].isLeaf());

	// If we are removing the root node (root node is a leaf in this case)
	if (m_rootNodeID == _nodeID) {
		m_rootNodeID = TreeNode::NULL_TREE_NODE;
		return;
	}

	int32_t parentNodeID = m_nodes[_nodeID].parentID;
	int32_t grandParentNodeID = m_nodes[parentNodeID].parentID;
	int32_t siblingNodeID;
	if (m_nodes[parentNodeID].children[0] == _nodeID) {
		siblingNodeID = m_nodes[parentNodeID].children[1];
	}
	else {
		siblingNodeID = m_nodes[parentNodeID].children[0];
	}

	// If the parent of the node to remove is not the root node
	if (grandParentNodeID != TreeNode::NULL_TREE_NODE) {

		// Destroy the parent node
		if (m_nodes[grandParentNodeID].children[0] == parentNodeID) {
			m_nodes[grandParentNodeID].children[0] = siblingNodeID;
		}
		else {
			assert(m_nodes[grandParentNodeID].children[1] == parentNodeID);
			m_nodes[grandParentNodeID].children[1] = siblingNodeID;
		}
		m_nodes[siblingNodeID].parentID = grandParentNodeID;
		releaseNode(parentNodeID);

		// Now, we need to recompute the AABBs of the node on the path back to the root
		// and make sure that the tree is still balanced
		int32_t currentNodeID = grandParentNodeID;
		while(currentNodeID != TreeNode::NULL_TREE_NODE) {

			// Balance the current sub-tree if necessary
			currentNodeID = balanceSubTreeAtNode(currentNodeID);

			assert(!m_nodes[currentNodeID].isLeaf());

			// Get the two children of the current node
			int32_t leftChildID = m_nodes[currentNodeID].children[0];
			int32_t rightChildID = m_nodes[currentNodeID].children[1];

			// Recompute the AABB and the height of the current node
			m_nodes[currentNodeID].aabb.mergeTwoAABBs(m_nodes[leftChildID].aabb,
													 m_nodes[rightChildID].aabb);
			m_nodes[currentNodeID].height = etk::max(m_nodes[leftChildID].height,
													m_nodes[rightChildID].height) + 1;
			assert(m_nodes[currentNodeID].height > 0);

			currentNodeID = m_nodes[currentNodeID].parentID;
		}
	}
	else { // If the parent of the node to remove is the root node

		// The sibling node becomes the new root node
		m_rootNodeID = siblingNodeID;
		m_nodes[siblingNodeID].parentID = TreeNode::NULL_TREE_NODE;
		releaseNode(parentNodeID);
	}
}

// Balance the sub-tree of a given node using left or right rotations.
/// The rotation schemes are described in the book "Introduction to Game Physics
/// with Box2D" by Ian Parberry. This method returns the new root node ID.
int32_t DynamicAABBTree::balanceSubTreeAtNode(int32_t _nodeID) {

	assert(_nodeID != TreeNode::NULL_TREE_NODE);

	TreeNode* nodeA = m_nodes + _nodeID;

	// If the node is a leaf or the height of A's sub-tree is less than 2
	if (nodeA->isLeaf() || nodeA->height < 2) {

		// Do not perform any rotation
		return _nodeID;
	}

	// Get the two children nodes
	int32_t nodeBID = nodeA->children[0];
	int32_t nodeCID = nodeA->children[1];
	assert(nodeBID >= 0 && nodeBID < m_numberAllocatedNodes);
	assert(nodeCID >= 0 && nodeCID < m_numberAllocatedNodes);
	TreeNode* nodeB = m_nodes + nodeBID;
	TreeNode* nodeC = m_nodes + nodeCID;

	// Compute the factor of the left and right sub-trees
	int32_t balanceFactor = nodeC->height - nodeB->height;

	// If the right node C is 2 higher than left node B
	if (balanceFactor > 1) {

		assert(!nodeC->isLeaf());

		int32_t nodeFID = nodeC->children[0];
		int32_t nodeGID = nodeC->children[1];
		assert(nodeFID >= 0 && nodeFID < m_numberAllocatedNodes);
		assert(nodeGID >= 0 && nodeGID < m_numberAllocatedNodes);
		TreeNode* nodeF = m_nodes + nodeFID;
		TreeNode* nodeG = m_nodes + nodeGID;

		nodeC->children[0] = _nodeID;
		nodeC->parentID = nodeA->parentID;
		nodeA->parentID = nodeCID;

		if (nodeC->parentID != TreeNode::NULL_TREE_NODE) {

			if (m_nodes[nodeC->parentID].children[0] == _nodeID) {
				m_nodes[nodeC->parentID].children[0] = nodeCID;
			}
			else {
				assert(m_nodes[nodeC->parentID].children[1] == _nodeID);
				m_nodes[nodeC->parentID].children[1] = nodeCID;
			}
		}
		else {
			m_rootNodeID = nodeCID;
		}

		assert(!nodeC->isLeaf());
		assert(!nodeA->isLeaf());

		// If the right node C was higher than left node B because of the F node
		if (nodeF->height > nodeG->height) {

			nodeC->children[1] = nodeFID;
			nodeA->children[1] = nodeGID;
			nodeG->parentID = _nodeID;

			// Recompute the AABB of node A and C
			nodeA->aabb.mergeTwoAABBs(nodeB->aabb, nodeG->aabb);
			nodeC->aabb.mergeTwoAABBs(nodeA->aabb, nodeF->aabb);

			// Recompute the height of node A and C
			nodeA->height = etk::max(nodeB->height, nodeG->height) + 1;
			nodeC->height = etk::max(nodeA->height, nodeF->height) + 1;
			assert(nodeA->height > 0);
			assert(nodeC->height > 0);
		}
		else {  // If the right node C was higher than left node B because of node G
			nodeC->children[1] = nodeGID;
			nodeA->children[1] = nodeFID;
			nodeF->parentID = _nodeID;

			// Recompute the AABB of node A and C
			nodeA->aabb.mergeTwoAABBs(nodeB->aabb, nodeF->aabb);
			nodeC->aabb.mergeTwoAABBs(nodeA->aabb, nodeG->aabb);

			// Recompute the height of node A and C
			nodeA->height = etk::max(nodeB->height, nodeF->height) + 1;
			nodeC->height = etk::max(nodeA->height, nodeG->height) + 1;
			assert(nodeA->height > 0);
			assert(nodeC->height > 0);
		}

		// Return the new root of the sub-tree
		return nodeCID;
	}

	// If the left node B is 2 higher than right node C
	if (balanceFactor < -1) {

		assert(!nodeB->isLeaf());

		int32_t nodeFID = nodeB->children[0];
		int32_t nodeGID = nodeB->children[1];
		assert(nodeFID >= 0 && nodeFID < m_numberAllocatedNodes);
		assert(nodeGID >= 0 && nodeGID < m_numberAllocatedNodes);
		TreeNode* nodeF = m_nodes + nodeFID;
		TreeNode* nodeG = m_nodes + nodeGID;

		nodeB->children[0] = _nodeID;
		nodeB->parentID = nodeA->parentID;
		nodeA->parentID = nodeBID;

		if (nodeB->parentID != TreeNode::NULL_TREE_NODE) {

			if (m_nodes[nodeB->parentID].children[0] == _nodeID) {
				m_nodes[nodeB->parentID].children[0] = nodeBID;
			}
			else {
				assert(m_nodes[nodeB->parentID].children[1] == _nodeID);
				m_nodes[nodeB->parentID].children[1] = nodeBID;
			}
		}
		else {
			m_rootNodeID = nodeBID;
		}

		assert(!nodeB->isLeaf());
		assert(!nodeA->isLeaf());

		// If the left node B was higher than right node C because of the F node
		if (nodeF->height > nodeG->height) {

			nodeB->children[1] = nodeFID;
			nodeA->children[0] = nodeGID;
			nodeG->parentID = _nodeID;

			// Recompute the AABB of node A and B
			nodeA->aabb.mergeTwoAABBs(nodeC->aabb, nodeG->aabb);
			nodeB->aabb.mergeTwoAABBs(nodeA->aabb, nodeF->aabb);

			// Recompute the height of node A and B
			nodeA->height = etk::max(nodeC->height, nodeG->height) + 1;
			nodeB->height = etk::max(nodeA->height, nodeF->height) + 1;
			assert(nodeA->height > 0);
			assert(nodeB->height > 0);
		}
		else {  // If the left node B was higher than right node C because of node G
			nodeB->children[1] = nodeGID;
			nodeA->children[0] = nodeFID;
			nodeF->parentID = _nodeID;

			// Recompute the AABB of node A and B
			nodeA->aabb.mergeTwoAABBs(nodeC->aabb, nodeF->aabb);
			nodeB->aabb.mergeTwoAABBs(nodeA->aabb, nodeG->aabb);

			// Recompute the height of node A and B
			nodeA->height = etk::max(nodeC->height, nodeF->height) + 1;
			nodeB->height = etk::max(nodeA->height, nodeG->height) + 1;
			assert(nodeA->height > 0);
			assert(nodeB->height > 0);
		}

		// Return the new root of the sub-tree
		return nodeBID;
	}

	// If the sub-tree is balanced, return the current root node
	return _nodeID;
}

/// Report all shapes overlapping with the AABB given in parameter.
void DynamicAABBTree::reportAllShapesOverlappingWithAABB(const AABB& aabb,
														 DynamicAABBTreeOverlapCallback& callback) const {

	// Create a stack with the nodes to visit
	Stack<int32_t, 64> stack;
	stack.push(m_rootNodeID);

	// While there are still nodes to visit
	while(stack.getNbElements() > 0) {

		// Get the next node ID to visit
		int32_t nodeIDToVisit = stack.pop();

		// Skip it if it is a null node
		if (nodeIDToVisit == TreeNode::NULL_TREE_NODE) continue;

		// Get the corresponding node
		const TreeNode* nodeToVisit = m_nodes + nodeIDToVisit;

		// If the AABB in parameter overlaps with the AABB of the node to visit
		if (aabb.testCollision(nodeToVisit->aabb)) {

			// If the node is a leaf
			if (nodeToVisit->isLeaf()) {

				// Notify the broad-phase about a new potential overlapping pair
				callback.notifyOverlappingNode(nodeIDToVisit);
			}
			else {  // If the node is not a leaf

				// We need to visit its children
				stack.push(nodeToVisit->children[0]);
				stack.push(nodeToVisit->children[1]);
			}
		}
	}
}

// Ray casting method
void DynamicAABBTree::raycast(const Ray& ray, DynamicAABBTreeRaycastCallback &callback) const {

	PROFILE("DynamicAABBTree::raycast()");

	float maxFraction = ray.maxFraction;

	Stack<int32_t, 128> stack;
	stack.push(m_rootNodeID);

	// Walk through the tree from the root looking for proxy shapes
	// that overlap with the ray AABB
	while (stack.getNbElements() > 0) {

		// Get the next node in the stack
		int32_t nodeID = stack.pop();

		// If it is a null node, skip it
		if (nodeID == TreeNode::NULL_TREE_NODE) continue;

		// Get the corresponding node
		const TreeNode* node = m_nodes + nodeID;

		Ray rayTemp(ray.point1, ray.point2, maxFraction);

		// Test if the ray int32_tersects with the current node AABB
		if (!node->aabb.testRayIntersect(rayTemp)) continue;

		// If the node is a leaf of the tree
		if (node->isLeaf()) {

			// Call the callback that will raycast again the broad-phase shape
			float hitFraction = callback.raycastBroadPhaseShape(nodeID, rayTemp);

			// If the user returned a hitFraction of zero, it means that
			// the raycasting should stop here
			if (hitFraction == 0.0f) {
				return;
			}

			// If the user returned a positive fraction
			if (hitFraction > 0.0f) {

				// We update the maxFraction value and the ray
				// AABB using the new maximum fraction
				if (hitFraction < maxFraction) {
					maxFraction = hitFraction;
				}
			}

			// If the user returned a negative fraction, we continue
			// the raycasting as if the proxy shape did not exist
		}
		else {  // If the node has children

			// Push its children in the stack of nodes to explore
			stack.push(node->children[0]);
			stack.push(node->children[1]);
		}
	}
}

// Return true if the node is a leaf of the tree
bool TreeNode::isLeaf() const {
	return (height == 0);
}

// Return the fat AABB corresponding to a given node ID
const AABB& DynamicAABBTree::getFatAABB(int32_t nodeID) const {
	assert(nodeID >= 0 && nodeID < m_numberAllocatedNodes);
	return m_nodes[nodeID].aabb;
}

// Return the pointer to the data array of a given leaf node of the tree
int32_t* DynamicAABBTree::getNodeDataInt(int32_t nodeID) const {
	assert(nodeID >= 0 && nodeID < m_numberAllocatedNodes);
	assert(m_nodes[nodeID].isLeaf());
	return m_nodes[nodeID].dataInt;
}

// Return the pointer to the data pointer of a given leaf node of the tree
void* DynamicAABBTree::getNodeDataPointer(int32_t nodeID) const {
	assert(nodeID >= 0 && nodeID < m_numberAllocatedNodes);
	assert(m_nodes[nodeID].isLeaf());
	return m_nodes[nodeID].dataPointer;
}

// Return the root AABB of the tree
AABB DynamicAABBTree::getRootAABB() const {
	return getFatAABB(m_rootNodeID);
}

// Add an object int32_to the tree. This method creates a new leaf node in the tree and
// returns the ID of the corresponding node.
int32_t DynamicAABBTree::addObject(const AABB& aabb, int32_t data1, int32_t data2) {

	int32_t nodeId = addObjectInternal(aabb);

	m_nodes[nodeId].dataInt[0] = data1;
	m_nodes[nodeId].dataInt[1] = data2;

	return nodeId;
}

// Add an object int32_to the tree. This method creates a new leaf node in the tree and
// returns the ID of the corresponding node.
int32_t DynamicAABBTree::addObject(const AABB& aabb, void* data) {

	int32_t nodeId = addObjectInternal(aabb);

	m_nodes[nodeId].dataPointer = data;

	return nodeId;
}


#ifndef NDEBUG

// Check if the tree structure is valid (for debugging purpose)
void DynamicAABBTree::check() const {

	// Recursively check each node
	checkNode(m_rootNodeID);

	int32_t nbFreeNodes = 0;
	int32_t freeNodeID = m_freeNodeID;

	// Check the free nodes
	while(freeNodeID != TreeNode::NULL_TREE_NODE) {
		assert(0 <= freeNodeID && freeNodeID < m_numberAllocatedNodes);
		freeNodeID = m_nodes[freeNodeID].nextNodeID;
		nbFreeNodes++;
	}

	assert(m_numberNodes + nbFreeNodes == m_numberAllocatedNodes);
}

// Check if the node structure is valid (for debugging purpose)
void DynamicAABBTree::checkNode(int32_t _nodeID) const {

	if (_nodeID == TreeNode::NULL_TREE_NODE) return;

	// If it is the root
	if (_nodeID == m_rootNodeID) {
		assert(m_nodes[_nodeID].parentID == TreeNode::NULL_TREE_NODE);
	}

	// Get the children nodes
	TreeNode* pNode = m_nodes + _nodeID;
	assert(!pNode->isLeaf());
	int32_t leftChild = pNode->children[0];
	int32_t rightChild = pNode->children[1];

	assert(pNode->height >= 0);
	assert(pNode->aabb.getVolume() > 0);

	// If the current node is a leaf
	if (pNode->isLeaf()) {

		// Check that there are no children
		assert(leftChild == TreeNode::NULL_TREE_NODE);
		assert(rightChild == TreeNode::NULL_TREE_NODE);
		assert(pNode->height == 0);
	}
	else {

		// Check that the children node IDs are valid
		assert(0 <= leftChild && leftChild < m_numberAllocatedNodes);
		assert(0 <= rightChild && rightChild < m_numberAllocatedNodes);

		// Check that the children nodes have the correct parent node
		assert(m_nodes[leftChild].parentID == _nodeID);
		assert(m_nodes[rightChild].parentID == _nodeID);

		// Check the height of node
		int32_t height = 1 + etk::max(m_nodes[leftChild].height, m_nodes[rightChild].height);
		assert(m_nodes[_nodeID].height == height);

		// Check the AABB of the node
		AABB aabb;
		aabb.mergeTwoAABBs(m_nodes[leftChild].aabb, m_nodes[rightChild].aabb);
		assert(aabb.getMin() == m_nodes[_nodeID].aabb.getMin());
		assert(aabb.getMax() == m_nodes[_nodeID].aabb.getMax());

		// Recursively check the children nodes
		checkNode(leftChild);
		checkNode(rightChild);
	}
}

// Compute the height of the tree
int32_t DynamicAABBTree::computeHeight() {
   return computeHeight(m_rootNodeID);
}

// Compute the height of a given node in the tree
int32_t DynamicAABBTree::computeHeight(int32_t _nodeID) {
	assert(_nodeID >= 0 && _nodeID < m_numberAllocatedNodes);
	TreeNode* node = m_nodes + _nodeID;

	// If the node is a leaf, its height is zero
	if (node->isLeaf()) {
		return 0;
	}

	// Compute the height of the left and right sub-tree
	int32_t leftHeight = computeHeight(node->children[0]);
	int32_t rightHeight = computeHeight(node->children[1]);

	// Return the height of the node
	return 1 + etk::max(leftHeight, rightHeight);
}

#endif
