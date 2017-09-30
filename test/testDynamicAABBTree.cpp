/** @file
 * @author Edouard DUPIN
 * @copyright 2017, Edouard DUPIN, all right reserved
 * @license MPL v2.0 (see license file)
 */

#include <etest/etest.hpp>
#include <ephysics/ephysics.hpp>
#include <ephysics/collision/broadphase/DynamicAABBTree.hpp>
#include <etk/Vector.hpp>


class OverlapCallback {
	public :
		etk::Vector<int32_t> m_overlapNodes;
		// Called when a overlapping node has been found during the call to
		// DynamicAABBTree:reportAllShapesOverlappingWithAABB()
		void operator()(int32_t _nodeId) {
			m_overlapNodes.pushBack(_nodeId);
		}
		void reset() {
			m_overlapNodes.clear();
		}
		bool isOverlapping(int32_t _nodeId) const {
			return etk::isIn(_nodeId, m_overlapNodes);
		}
};

class DynamicTreeRaycastCallback {
	public:
		etk::Vector<int32_t> m_hitNodes;
		// Called when the AABB of a leaf node is hit by a ray
		float operator()(int32_t _nodeId, const ephysics::Ray& _ray) {
			m_hitNodes.pushBack(_nodeId);
			return 1.0;
		}
		void reset() {
			m_hitNodes.clear();
		}
		bool isHit(int32_t _nodeId) const {
			return etk::isIn(_nodeId, m_hitNodes);
		}
};

TEST(TestAABBTree, create) {
	// Dynamic AABB Tree
	ephysics::DynamicAABBTree tree;

	int32_t object1Data = 56;
	int32_t object2Data = 23;
	int32_t object3Data = 13;
	int32_t object4Data = 7;

	// First object
	ephysics::AABB aabb1 = ephysics::AABB(vec3(-6, 4, -3), vec3(4, 8, 3));
	int32_t object1Id = tree.addObject(aabb1, &object1Data);

	// Second object
	ephysics::AABB aabb2 = ephysics::AABB(vec3(5, 2, -3), vec3(10, 7, 3));
	int32_t object2Id = tree.addObject(aabb2, &object2Data);

	// Third object
	ephysics::AABB aabb3 = ephysics::AABB(vec3(-5, 1, -3), vec3(-2, 3, 3));
	int32_t object3Id = tree.addObject(aabb3, &object3Data);

	// Fourth object
	ephysics::AABB aabb4 = ephysics::AABB(vec3(0, -4, -3), vec3(3, -2, 3));
	int32_t object4Id = tree.addObject(aabb4, &object4Data);

	// ----------- Tests ----------- //

	// Test root AABB
	ephysics::AABB rootAABB = tree.getRootAABB();
	EXPECT_EQ(rootAABB.getMin().x(), -6);
	EXPECT_EQ(rootAABB.getMin().y(), -4);
	EXPECT_EQ(rootAABB.getMin().z(), -3);
	EXPECT_EQ(rootAABB.getMax().x(), 10);
	EXPECT_EQ(rootAABB.getMax().y(), 8);
	EXPECT_EQ(rootAABB.getMax().z(), 3);

	// Test data stored at the nodes of the tree
	EXPECT_EQ(*(int32_t*)(tree.getNodeDataPointer(object1Id)), object1Data);
	EXPECT_EQ(*(int32_t*)(tree.getNodeDataPointer(object2Id)), object2Data);
	EXPECT_EQ(*(int32_t*)(tree.getNodeDataPointer(object3Id)), object3Data);
	EXPECT_EQ(*(int32_t*)(tree.getNodeDataPointer(object4Id)), object4Data);
}

TEST(TestAABBTree, overlapping) {
	OverlapCallback overlapCallback;
	// Dynamic AABB Tree
	ephysics::DynamicAABBTree tree;

	int32_t object1Data = 56;
	int32_t object2Data = 23;
	int32_t object3Data = 13;
	int32_t object4Data = 7;

	// First object
	ephysics::AABB aabb1 = ephysics::AABB(vec3(-6, 4, -3), vec3(4, 8, 3));
	int32_t object1Id = tree.addObject(aabb1, &object1Data);

	// Second object
	ephysics::AABB aabb2 = ephysics::AABB(vec3(5, 2, -3), vec3(10, 7, 3));
	int32_t object2Id = tree.addObject(aabb2, &object2Data);

	// Third object
	ephysics::AABB aabb3 = ephysics::AABB(vec3(-5, 1, -3), vec3(-2, 3, 3));
	int32_t object3Id = tree.addObject(aabb3, &object3Data);

	// Fourth object
	ephysics::AABB aabb4 = ephysics::AABB(vec3(0, -4, -3), vec3(3, -2, 3));
	int32_t object4Id = tree.addObject(aabb4, &object4Data);

	// ---------- Tests ---------- //

	// AABB overlapping nothing
	overlapCallback.reset();
	tree.reportAllShapesOverlappingWithAABB(ephysics::AABB(vec3(-10, 12, -4), vec3(10, 50, 4)), [&](int32_t _nodeId) mutable { overlapCallback(_nodeId);});
	EXPECT_EQ(overlapCallback.isOverlapping(object1Id), false);
	EXPECT_EQ(overlapCallback.isOverlapping(object2Id), false);
	EXPECT_EQ(overlapCallback.isOverlapping(object3Id), false);
	EXPECT_EQ(overlapCallback.isOverlapping(object4Id), false);

	// AABB overlapping everything
	overlapCallback.reset();
	tree.reportAllShapesOverlappingWithAABB(ephysics::AABB(vec3(-15, -15, -4), vec3(15, 15, 4)), [&](int32_t _nodeId) mutable { overlapCallback(_nodeId);});
	EXPECT_EQ(overlapCallback.isOverlapping(object1Id), true);
	EXPECT_EQ(overlapCallback.isOverlapping(object2Id), true);
	EXPECT_EQ(overlapCallback.isOverlapping(object3Id), true);
	EXPECT_EQ(overlapCallback.isOverlapping(object4Id), true);

	// AABB overlapping object 1 and 3
	overlapCallback.reset();
	tree.reportAllShapesOverlappingWithAABB(ephysics::AABB(vec3(-4, 2, -4), vec3(-1, 7, 4)), [&](int32_t _nodeId) mutable { overlapCallback(_nodeId);});
	EXPECT_EQ(overlapCallback.isOverlapping(object1Id), true);
	EXPECT_EQ(overlapCallback.isOverlapping(object2Id), false);
	EXPECT_EQ(overlapCallback.isOverlapping(object3Id), true);
	EXPECT_EQ(overlapCallback.isOverlapping(object4Id), false);

	// AABB overlapping object 3 and 4
	overlapCallback.reset();
	tree.reportAllShapesOverlappingWithAABB(ephysics::AABB(vec3(-6, -5, -2), vec3(2, 2, 0)), [&](int32_t _nodeId) mutable { overlapCallback(_nodeId);});
	EXPECT_EQ(overlapCallback.isOverlapping(object1Id), false);
	EXPECT_EQ(overlapCallback.isOverlapping(object2Id), false);
	EXPECT_EQ(overlapCallback.isOverlapping(object3Id), true);
	EXPECT_EQ(overlapCallback.isOverlapping(object4Id), true);

	// AABB overlapping object 2
	overlapCallback.reset();
	tree.reportAllShapesOverlappingWithAABB(ephysics::AABB(vec3(5, -10, -2), vec3(7, 10, 9)), [&](int32_t _nodeId) mutable { overlapCallback(_nodeId);});
	EXPECT_EQ(overlapCallback.isOverlapping(object1Id), false);
	EXPECT_EQ(overlapCallback.isOverlapping(object2Id), true);
	EXPECT_EQ(overlapCallback.isOverlapping(object3Id), false);
	EXPECT_EQ(overlapCallback.isOverlapping(object4Id), false);

	// ---- Update the object AABBs with the initial AABBs (no reinsertion) ----- //

	tree.updateObject(object1Id, aabb1, vec3(0.0f,0.0f,0.0f));
	tree.updateObject(object2Id, aabb2, vec3(0.0f,0.0f,0.0f));
	tree.updateObject(object3Id, aabb3, vec3(0.0f,0.0f,0.0f));
	tree.updateObject(object4Id, aabb4, vec3(0.0f,0.0f,0.0f));

	// AABB overlapping nothing
	overlapCallback.reset();
	tree.reportAllShapesOverlappingWithAABB(ephysics::AABB(vec3(-10, 12, -4), vec3(10, 50, 4)), [&](int32_t _nodeId) mutable { overlapCallback(_nodeId);});
	EXPECT_EQ(overlapCallback.isOverlapping(object1Id), false);
	EXPECT_EQ(overlapCallback.isOverlapping(object2Id), false);
	EXPECT_EQ(overlapCallback.isOverlapping(object3Id), false);
	EXPECT_EQ(overlapCallback.isOverlapping(object4Id), false);

	// AABB overlapping everything
	overlapCallback.reset();
	tree.reportAllShapesOverlappingWithAABB(ephysics::AABB(vec3(-15, -15, -4), vec3(15, 15, 4)), [&](int32_t _nodeId) mutable { overlapCallback(_nodeId);});
	EXPECT_EQ(overlapCallback.isOverlapping(object1Id), true);
	EXPECT_EQ(overlapCallback.isOverlapping(object2Id), true);
	EXPECT_EQ(overlapCallback.isOverlapping(object3Id), true);
	EXPECT_EQ(overlapCallback.isOverlapping(object4Id), true);

	// AABB overlapping object 1 and 3
	overlapCallback.reset();
	tree.reportAllShapesOverlappingWithAABB(ephysics::AABB(vec3(-4, 2, -4), vec3(-1, 7, 4)), [&](int32_t _nodeId) mutable { overlapCallback(_nodeId);});
	EXPECT_EQ(overlapCallback.isOverlapping(object1Id), true);
	EXPECT_EQ(overlapCallback.isOverlapping(object2Id), false);
	EXPECT_EQ(overlapCallback.isOverlapping(object3Id), true);
	EXPECT_EQ(overlapCallback.isOverlapping(object4Id), false);

	// AABB overlapping object 3 and 4
	overlapCallback.reset();
	tree.reportAllShapesOverlappingWithAABB(ephysics::AABB(vec3(-6, -5, -2), vec3(2, 2, 0)), [&](int32_t _nodeId) mutable { overlapCallback(_nodeId);});
	EXPECT_EQ(overlapCallback.isOverlapping(object1Id), false);
	EXPECT_EQ(overlapCallback.isOverlapping(object2Id), false);
	EXPECT_EQ(overlapCallback.isOverlapping(object3Id), true);
	EXPECT_EQ(overlapCallback.isOverlapping(object4Id), true);

	// AABB overlapping object 2
	overlapCallback.reset();
	tree.reportAllShapesOverlappingWithAABB(ephysics::AABB(vec3(5, -10, -2), vec3(7, 10, 9)), [&](int32_t _nodeId) mutable { overlapCallback(_nodeId);});
	EXPECT_EQ(overlapCallback.isOverlapping(object1Id), false);
	EXPECT_EQ(overlapCallback.isOverlapping(object2Id), true);
	EXPECT_EQ(overlapCallback.isOverlapping(object3Id), false);
	EXPECT_EQ(overlapCallback.isOverlapping(object4Id), false);

	// ---- Update the object AABBs with the initial AABBs (with reinsertion) ----- //

	tree.updateObject(object1Id, aabb1, vec3(0.0f,0.0f,0.0f));
	tree.updateObject(object2Id, aabb2, vec3(0.0f,0.0f,0.0f));
	tree.updateObject(object3Id, aabb3, vec3(0.0f,0.0f,0.0f));
	tree.updateObject(object4Id, aabb4, vec3(0.0f,0.0f,0.0f));

	// AABB overlapping nothing
	overlapCallback.reset();
	tree.reportAllShapesOverlappingWithAABB(ephysics::AABB(vec3(-10, 12, -4), vec3(10, 50, 4)), [&](int32_t _nodeId) mutable { overlapCallback(_nodeId);});
	EXPECT_EQ(overlapCallback.isOverlapping(object1Id), false);
	EXPECT_EQ(overlapCallback.isOverlapping(object2Id), false);
	EXPECT_EQ(overlapCallback.isOverlapping(object3Id), false);
	EXPECT_EQ(overlapCallback.isOverlapping(object4Id), false);

	// AABB overlapping everything
	overlapCallback.reset();
	tree.reportAllShapesOverlappingWithAABB(ephysics::AABB(vec3(-15, -15, -4), vec3(15, 15, 4)), [&](int32_t _nodeId) mutable { overlapCallback(_nodeId);});
	EXPECT_EQ(overlapCallback.isOverlapping(object1Id), true);
	EXPECT_EQ(overlapCallback.isOverlapping(object2Id), true);
	EXPECT_EQ(overlapCallback.isOverlapping(object3Id), true);
	EXPECT_EQ(overlapCallback.isOverlapping(object4Id), true);

	// AABB overlapping object 1 and 3
	overlapCallback.reset();
	tree.reportAllShapesOverlappingWithAABB(ephysics::AABB(vec3(-4, 2, -4), vec3(-1, 7, 4)), [&](int32_t _nodeId) mutable { overlapCallback(_nodeId);});
	EXPECT_EQ(overlapCallback.isOverlapping(object1Id), true);
	EXPECT_EQ(overlapCallback.isOverlapping(object2Id), false);
	EXPECT_EQ(overlapCallback.isOverlapping(object3Id), true);
	EXPECT_EQ(overlapCallback.isOverlapping(object4Id), false);

	// AABB overlapping object 3 and 4
	overlapCallback.reset();
	tree.reportAllShapesOverlappingWithAABB(ephysics::AABB(vec3(-6, -5, -2), vec3(2, 2, 0)), [&](int32_t _nodeId) mutable { overlapCallback(_nodeId);});
	EXPECT_EQ(overlapCallback.isOverlapping(object1Id), false);
	EXPECT_EQ(overlapCallback.isOverlapping(object2Id), false);
	EXPECT_EQ(overlapCallback.isOverlapping(object3Id), true);
	EXPECT_EQ(overlapCallback.isOverlapping(object4Id), true);

	// AABB overlapping object 2
	overlapCallback.reset();
	tree.reportAllShapesOverlappingWithAABB(ephysics::AABB(vec3(5, -10, -2), vec3(7, 10, 9)), [&](int32_t _nodeId) mutable { overlapCallback(_nodeId);});
	EXPECT_EQ(overlapCallback.isOverlapping(object1Id), false);
	EXPECT_EQ(overlapCallback.isOverlapping(object2Id), true);
	EXPECT_EQ(overlapCallback.isOverlapping(object3Id), false);
	EXPECT_EQ(overlapCallback.isOverlapping(object4Id), false);

	// ---- Move objects 2 and 3 ----- //

	ephysics::AABB newAABB2(vec3(-7, 10, -3), vec3(1, 13, 3));
	tree.updateObject(object2Id, newAABB2, vec3(0.0f,0.0f,0.0f));

	ephysics::AABB newAABB3(vec3(7, -6, -3), vec3(9, 1, 3));
	tree.updateObject(object3Id, newAABB3, vec3(0.0f,0.0f,0.0f));

	// AABB overlapping object 3
	overlapCallback.reset();
	tree.reportAllShapesOverlappingWithAABB(ephysics::AABB(vec3(6, -10, -2), vec3(8, 5, 3)), [&](int32_t _nodeId) mutable { overlapCallback(_nodeId);});
	EXPECT_EQ(overlapCallback.isOverlapping(object1Id), false);
	EXPECT_EQ(overlapCallback.isOverlapping(object2Id), false);
	EXPECT_EQ(overlapCallback.isOverlapping(object3Id), true);
	EXPECT_EQ(overlapCallback.isOverlapping(object4Id), false);

	// AABB overlapping objects 1, 2
	overlapCallback.reset();
	tree.reportAllShapesOverlappingWithAABB(ephysics::AABB(vec3(-8, 5, -3), vec3(-2, 11, 3)), [&](int32_t _nodeId) mutable { overlapCallback(_nodeId);});
	EXPECT_EQ(overlapCallback.isOverlapping(object1Id), true);
	EXPECT_EQ(overlapCallback.isOverlapping(object2Id), true);
	EXPECT_EQ(overlapCallback.isOverlapping(object3Id), false);
	EXPECT_EQ(overlapCallback.isOverlapping(object4Id), false);

}

TEST(TestAABBTree, raycast) {
	DynamicTreeRaycastCallback raycastCallback;
	ephysics::DynamicAABBTree tree;

	int32_t object1Data = 56;
	int32_t object2Data = 23;
	int32_t object3Data = 13;
	int32_t object4Data = 7;

	// First object
	ephysics::AABB aabb1 = ephysics::AABB(vec3(-6, 4, -3), vec3(4, 8, 3));
	int32_t object1Id = tree.addObject(aabb1, &object1Data);

	// Second object
	ephysics::AABB aabb2 = ephysics::AABB(vec3(5, 2, -3), vec3(10, 7, 3));
	int32_t object2Id = tree.addObject(aabb2, &object2Data);

	// Third object
	ephysics::AABB aabb3 = ephysics::AABB(vec3(-5, 1, -3), vec3(-2, 3, 3));
	int32_t object3Id = tree.addObject(aabb3, &object3Data);

	// Fourth object
	ephysics::AABB aabb4 = ephysics::AABB(vec3(0, -4, -3), vec3(3, -2, 3));
	int32_t object4Id = tree.addObject(aabb4, &object4Data);

	// ---------- Tests ---------- //

	// Ray with no hits
	raycastCallback.reset();
	ephysics::Ray ray1(vec3(4.5, -10, -5), vec3(4.5, 10, -5));
	tree.raycast(ray1, [&](int32_t _nodeId, const ephysics::Ray& _ray) mutable { return raycastCallback(_nodeId, _ray);});
	EXPECT_EQ(raycastCallback.isHit(object1Id), false);
	EXPECT_EQ(raycastCallback.isHit(object2Id), false);
	EXPECT_EQ(raycastCallback.isHit(object3Id), false);
	EXPECT_EQ(raycastCallback.isHit(object4Id), false);

	// Ray that hits object 1
	raycastCallback.reset();
	ephysics::Ray ray2(vec3(-1, -20, -2), vec3(-1, 20, -2));
	tree.raycast(ray2, [&](int32_t _nodeId, const ephysics::Ray& _ray) mutable { return raycastCallback(_nodeId, _ray);});
	EXPECT_EQ(raycastCallback.isHit(object1Id), true);
	EXPECT_EQ(raycastCallback.isHit(object2Id), false);
	EXPECT_EQ(raycastCallback.isHit(object3Id), false);
	EXPECT_EQ(raycastCallback.isHit(object4Id), false);

	// Ray that hits object 1 and 2
	raycastCallback.reset();
	ephysics::Ray ray3(vec3(-7, 6, -2), vec3(8, 6, -2));
	tree.raycast(ray3, [&](int32_t _nodeId, const ephysics::Ray& _ray) mutable { return raycastCallback(_nodeId, _ray);});
	EXPECT_EQ(raycastCallback.isHit(object1Id), true);
	EXPECT_EQ(raycastCallback.isHit(object2Id), true);
	EXPECT_EQ(raycastCallback.isHit(object3Id), false);
	EXPECT_EQ(raycastCallback.isHit(object4Id), false);

	// Ray that hits object 3
	raycastCallback.reset();
	ephysics::Ray ray4(vec3(-7, 2, 0), vec3(-1, 2, 0));
	tree.raycast(ray4, [&](int32_t _nodeId, const ephysics::Ray& _ray) mutable { return raycastCallback(_nodeId, _ray);});
	EXPECT_EQ(raycastCallback.isHit(object1Id), false);
	EXPECT_EQ(raycastCallback.isHit(object2Id), false);
	EXPECT_EQ(raycastCallback.isHit(object3Id), true);
	EXPECT_EQ(raycastCallback.isHit(object4Id), false);

	// ---- Update the object AABBs with the initial AABBs (no reinsertion) ----- //

	tree.updateObject(object1Id, aabb1, vec3(0.0f,0.0f,0.0f));
	tree.updateObject(object2Id, aabb2, vec3(0.0f,0.0f,0.0f));
	tree.updateObject(object3Id, aabb3, vec3(0.0f,0.0f,0.0f));
	tree.updateObject(object4Id, aabb4, vec3(0.0f,0.0f,0.0f));

	// Ray with no hits
	raycastCallback.reset();
	tree.raycast(ray1, [&](int32_t _nodeId, const ephysics::Ray& _ray) mutable { return raycastCallback(_nodeId, _ray);});
	EXPECT_EQ(raycastCallback.isHit(object1Id), false);
	EXPECT_EQ(raycastCallback.isHit(object2Id), false);
	EXPECT_EQ(raycastCallback.isHit(object3Id), false);
	EXPECT_EQ(raycastCallback.isHit(object4Id), false);

	// Ray that hits object 1
	raycastCallback.reset();
	tree.raycast(ray2, [&](int32_t _nodeId, const ephysics::Ray& _ray) mutable { return raycastCallback(_nodeId, _ray);});
	EXPECT_EQ(raycastCallback.isHit(object1Id), true);
	EXPECT_EQ(raycastCallback.isHit(object2Id), false);
	EXPECT_EQ(raycastCallback.isHit(object3Id), false);
	EXPECT_EQ(raycastCallback.isHit(object4Id), false);

	// Ray that hits object 1 and 2
	raycastCallback.reset();
	tree.raycast(ray3, [&](int32_t _nodeId, const ephysics::Ray& _ray) mutable { return raycastCallback(_nodeId, _ray);});
	EXPECT_EQ(raycastCallback.isHit(object1Id), true);
	EXPECT_EQ(raycastCallback.isHit(object2Id), true);
	EXPECT_EQ(raycastCallback.isHit(object3Id), false);
	EXPECT_EQ(raycastCallback.isHit(object4Id), false);

	// Ray that hits object 3
	raycastCallback.reset();
	tree.raycast(ray4, [&](int32_t _nodeId, const ephysics::Ray& _ray) mutable { return raycastCallback(_nodeId, _ray);});
	EXPECT_EQ(raycastCallback.isHit(object1Id), false);
	EXPECT_EQ(raycastCallback.isHit(object2Id), false);
	EXPECT_EQ(raycastCallback.isHit(object3Id), true);
	EXPECT_EQ(raycastCallback.isHit(object4Id), false);

	// ---- Update the object AABBs with the initial AABBs (with reinsertion) ----- //

	tree.updateObject(object1Id, aabb1, vec3(0.0f,0.0f,0.0f));
	tree.updateObject(object2Id, aabb2, vec3(0.0f,0.0f,0.0f));
	tree.updateObject(object3Id, aabb3, vec3(0.0f,0.0f,0.0f));
	tree.updateObject(object4Id, aabb4, vec3(0.0f,0.0f,0.0f));

	// Ray with no hits
	raycastCallback.reset();
	tree.raycast(ray1, [&](int32_t _nodeId, const ephysics::Ray& _ray) mutable { return raycastCallback(_nodeId, _ray);});
	EXPECT_EQ(raycastCallback.isHit(object1Id), false);
	EXPECT_EQ(raycastCallback.isHit(object2Id), false);
	EXPECT_EQ(raycastCallback.isHit(object3Id), false);
	EXPECT_EQ(raycastCallback.isHit(object4Id), false);

	// Ray that hits object 1
	raycastCallback.reset();
	tree.raycast(ray2, [&](int32_t _nodeId, const ephysics::Ray& _ray) mutable { return raycastCallback(_nodeId, _ray);});
	EXPECT_EQ(raycastCallback.isHit(object1Id), true);
	EXPECT_EQ(raycastCallback.isHit(object2Id), false);
	EXPECT_EQ(raycastCallback.isHit(object3Id), false);
	EXPECT_EQ(raycastCallback.isHit(object4Id), false);

	// Ray that hits object 1 and 2
	raycastCallback.reset();
	tree.raycast(ray3, [&](int32_t _nodeId, const ephysics::Ray& _ray) mutable { return raycastCallback(_nodeId, _ray);});
	EXPECT_EQ(raycastCallback.isHit(object1Id), true);
	EXPECT_EQ(raycastCallback.isHit(object2Id), true);
	EXPECT_EQ(raycastCallback.isHit(object3Id), false);
	EXPECT_EQ(raycastCallback.isHit(object4Id), false);

	// Ray that hits object 3
	raycastCallback.reset();
	tree.raycast(ray4, [&](int32_t _nodeId, const ephysics::Ray& _ray) mutable { return raycastCallback(_nodeId, _ray);});
	EXPECT_EQ(raycastCallback.isHit(object1Id), false);
	EXPECT_EQ(raycastCallback.isHit(object2Id), false);
	EXPECT_EQ(raycastCallback.isHit(object3Id), true);
	EXPECT_EQ(raycastCallback.isHit(object4Id), false);

	// ---- Move objects 2 and 3 ----- //

	ephysics::AABB newAABB2(vec3(-7, 10, -3), vec3(1, 13, 3));
	tree.updateObject(object2Id, newAABB2, vec3(0.0f,0.0f,0.0f));

	ephysics::AABB newAABB3(vec3(7, -6, -3), vec3(9, 1, 3));
	tree.updateObject(object3Id, newAABB3, vec3(0.0f,0.0f,0.0f));

	// Ray that hits object 1, 2
	ephysics::Ray ray5(vec3(-4, -5, 0), vec3(-4, 12, 0));
	raycastCallback.reset();
	tree.raycast(ray5, [&](int32_t _nodeId, const ephysics::Ray& _ray) mutable { return raycastCallback(_nodeId, _ray);});
	EXPECT_EQ(raycastCallback.isHit(object1Id), true);
	EXPECT_EQ(raycastCallback.isHit(object2Id), true);
	EXPECT_EQ(raycastCallback.isHit(object3Id), false);
	EXPECT_EQ(raycastCallback.isHit(object4Id), false);

	// Ray that hits object 3 and 4
	ephysics::Ray ray6(vec3(11, -3, 1), vec3(-2, -3, 1));
	raycastCallback.reset();
	tree.raycast(ray6, [&](int32_t _nodeId, const ephysics::Ray& _ray) mutable { return raycastCallback(_nodeId, _ray);});
	EXPECT_EQ(raycastCallback.isHit(object1Id), false);
	EXPECT_EQ(raycastCallback.isHit(object2Id), false);
	EXPECT_EQ(raycastCallback.isHit(object3Id), true);
	EXPECT_EQ(raycastCallback.isHit(object4Id), true);
}

