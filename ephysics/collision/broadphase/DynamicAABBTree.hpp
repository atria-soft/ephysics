/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/configuration.hpp>
#include <ephysics/collision/shapes/AABB.hpp>
#include <ephysics/body/CollisionBody.hpp>

/// Namespace ReactPhysics3D
namespace ephysics {

// Declarations
class BroadPhaseAlgorithm;
class BroadPhaseRaycastTestCallback;
class DynamicAABBTreeOverlapCallback;
struct RaycastTest;

// Structure TreeNode
/**
 * This structure represents a node of the dynamic AABB tree.
 */
struct TreeNode {

	// -------------------- Constants -------------------- //

	/// Null tree node constant
	const static int32_t NULL_TREE_NODE;

	// -------------------- Attributes -------------------- //

	// A node is either in the tree (has a parent) or in the free nodes list
	// (has a next node)
	union {

		/// Parent node ID
		int32_t parentID;

		/// Next allocated node ID
		int32_t nextNodeID;
	};

	// A node is either a leaf (has data) or is an int32_ternal node (has children)
	union {

		/// Left and right child of the node (children[0] = left child)
		int32_t children[2];

		/// Two pieces of data stored at that node (in case the node is a leaf)
		union {
			int32_t dataInt[2];
			void* dataPointer;
		};
	};

	/// Height of the node in the tree
	int16_t height;

	/// Fat axis aligned bounding box (AABB) corresponding to the node
	AABB aabb;

	// -------------------- Methods -------------------- //

	/// Return true if the node is a leaf of the tree
	bool isLeaf() const;
};

// Class DynamicAABBTreeOverlapCallback
/**
 * Overlapping callback method that has to be used as parameter of the
 * reportAllShapesOverlappingWithNode() method.
 */
class DynamicAABBTreeOverlapCallback {

	public :
		virtual ~DynamicAABBTreeOverlapCallback() = default;

		// Called when a overlapping node has been found during the call to
		// DynamicAABBTree:reportAllShapesOverlappingWithAABB()
		virtual void notifyOverlappingNode(int32_t nodeId)=0;
};

// Class DynamicAABBTreeRaycastCallback
/**
 * Raycast callback in the Dynamic AABB Tree called when the AABB of a leaf
 * node is hit by the ray.
 */
class DynamicAABBTreeRaycastCallback {

	public:
		virtual ~DynamicAABBTreeRaycastCallback() = default;

		// Called when the AABB of a leaf node is hit by a ray
		virtual float raycastBroadPhaseShape(int32_t nodeId, const Ray& ray)=0;

};

// Class DynamicAABBTree
/**
 * This class implements a dynamic AABB tree that is used for broad-phase
 * collision detection. This data structure is inspired by Nathanael Presson's
 * dynamic tree implementation in BulletPhysics. The following implementation is
 * based on the one from Erin Catto in Box2D as described in the book
 * "Introduction to Game Physics with Box2D" by Ian Parberry.
 */
class DynamicAABBTree {

	private:

		// -------------------- Attributes -------------------- //

		/// Pointer to the memory location of the nodes of the tree
		TreeNode* m_nodes;

		/// ID of the root node of the tree
		int32_t m_rootNodeID;

		/// ID of the first node of the list of free (allocated) nodes in the tree that we can use
		int32_t m_freeNodeID;

		/// Number of allocated nodes in the tree
		int32_t m_numberAllocatedNodes;

		/// Number of nodes in the tree
		int32_t m_numberNodes;

		/// Extra AABB Gap used to allow the collision shape to move a little bit
		/// without triggering a large modification of the tree which can be costly
		float m_extraAABBGap;

		// -------------------- Methods -------------------- //

		/// Allocate and return a node to use in the tree
		int32_t allocateNode();

		/// Release a node
		void releaseNode(int32_t nodeID);

		/// Insert a leaf node in the tree
		void insertLeafNode(int32_t nodeID);

		/// Remove a leaf node from the tree
		void removeLeafNode(int32_t nodeID);

		/// Balance the sub-tree of a given node using left or right rotations.
		int32_t balanceSubTreeAtNode(int32_t nodeID);

		/// Compute the height of a given node in the tree
		int32_t computeHeight(int32_t nodeID);

		/// Internally add an object int32_to the tree
		int32_t addObjectInternal(const AABB& aabb);

		/// Initialize the tree
		void init();

#ifndef NDEBUG

		/// Check if the tree structure is valid (for debugging purpose)
		void check() const;

		/// Check if the node structure is valid (for debugging purpose)
		void checkNode(int32_t nodeID) const;

#endif

	public:

		// -------------------- Methods -------------------- //

		/// Constructor
		DynamicAABBTree(float extraAABBGap = 0.0f);

		/// Destructor
		virtual ~DynamicAABBTree();

		/// Add an object int32_to the tree (where node data are two int32_tegers)
		int32_t addObject(const AABB& aabb, int32_t data1, int32_t data2);

		/// Add an object int32_to the tree (where node data is a pointer)
		int32_t addObject(const AABB& aabb, void* data);

		/// Remove an object from the tree
		void removeObject(int32_t nodeID);

		/// Update the dynamic tree after an object has moved.
		bool updateObject(int32_t nodeID, const AABB& newAABB, const vec3& displacement, bool forceReinsert = false);

		/// Return the fat AABB corresponding to a given node ID
		const AABB& getFatAABB(int32_t nodeID) const;

		/// Return the pointer to the data array of a given leaf node of the tree
		int32_t* getNodeDataInt(int32_t nodeID) const;

		/// Return the data pointer of a given leaf node of the tree
		void* getNodeDataPointer(int32_t nodeID) const;

		/// Report all shapes overlapping with the AABB given in parameter.
		void reportAllShapesOverlappingWithAABB(const AABB& aabb,
												DynamicAABBTreeOverlapCallback& callback) const;

		/// Ray casting method
		void raycast(const Ray& ray, DynamicAABBTreeRaycastCallback& callback) const;

		/// Compute the height of the tree
		int32_t computeHeight();

		/// Return the root AABB of the tree
		AABB getRootAABB() const;

		/// Clear all the nodes and reset the tree
		void reset();
};

// Return true if the node is a leaf of the tree
inline bool TreeNode::isLeaf() const {
	return (height == 0);
}

// Return the fat AABB corresponding to a given node ID
inline const AABB& DynamicAABBTree::getFatAABB(int32_t nodeID) const {
	assert(nodeID >= 0 && nodeID < m_numberAllocatedNodes);
	return m_nodes[nodeID].aabb;
}

// Return the pointer to the data array of a given leaf node of the tree
inline int32_t* DynamicAABBTree::getNodeDataInt(int32_t nodeID) const {
	assert(nodeID >= 0 && nodeID < m_numberAllocatedNodes);
	assert(m_nodes[nodeID].isLeaf());
	return m_nodes[nodeID].dataInt;
}

// Return the pointer to the data pointer of a given leaf node of the tree
inline void* DynamicAABBTree::getNodeDataPointer(int32_t nodeID) const {
	assert(nodeID >= 0 && nodeID < m_numberAllocatedNodes);
	assert(m_nodes[nodeID].isLeaf());
	return m_nodes[nodeID].dataPointer;
}

// Return the root AABB of the tree
inline AABB DynamicAABBTree::getRootAABB() const {
	return getFatAABB(m_rootNodeID);
}

// Add an object int32_to the tree. This method creates a new leaf node in the tree and
// returns the ID of the corresponding node.
inline int32_t DynamicAABBTree::addObject(const AABB& aabb, int32_t data1, int32_t data2) {

	int32_t nodeId = addObjectInternal(aabb);

	m_nodes[nodeId].dataInt[0] = data1;
	m_nodes[nodeId].dataInt[1] = data2;

	return nodeId;
}

// Add an object int32_to the tree. This method creates a new leaf node in the tree and
// returns the ID of the corresponding node.
inline int32_t DynamicAABBTree::addObject(const AABB& aabb, void* data) {

	int32_t nodeId = addObjectInternal(aabb);

	m_nodes[nodeId].dataPointer = data;

	return nodeId;
}

}
