/********************************************************************************
* ReactPhysics3D physics library, http://www.ephysics.com				 *
* Copyright (c) 2010-2016 Daniel Chappuis									   *
*********************************************************************************
*																			   *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.														 *
*																			   *
* Permission is granted to anyone to use this software for any purpose,		 *
* including commercial applications, and to alter it and redistribute it		*
* freely, subject to the following restrictions:								*
*																			   *
* 1. The origin of this software must not be misrepresented; you must not claim *
*	that you wrote the original software. If you use this software in a		*
*	product, an acknowledgment in the product documentation would be		   *
*	appreciated but is not required.										   *
*																			   *
* 2. Altered source versions must be plainly marked as such, and must not be	*
*	misrepresented as being the original software.							 *
*																			   *
* 3. This notice may not be removed or altered from any source distribution.	*
*																			   *
********************************************************************************/

#ifndef TEST_DYNAMIC_AABB_TREE_H
#define TEST_DYNAMIC_AABB_TREE_H

// Libraries
#include <test/Test.hpp>
#include <ephysics/collision/broadphase/DynamicAABBTree.hpp>
#include <vector>

/// Reactphysics3D namespace
namespace ephysics {

class OverlapCallback : public DynamicAABBTreeOverlapCallback {

	public :

		std::vector<int32_t> mOverlapNodes;

		// Called when a overlapping node has been found during the call to
		// DynamicAABBTree:reportAllShapesOverlappingWithAABB()
		virtual void notifyOverlappingNode(int32_t nodeId) {
			mOverlapNodes.push_back(nodeId);
		}

		void reset() {
			mOverlapNodes.clear();
		}

		bool isOverlapping(int32_t nodeId) const {
			return std::find(mOverlapNodes.begin(), mOverlapNodes.end(), nodeId) != mOverlapNodes.end();
		}
};

class DynamicTreeRaycastCallback : public DynamicAABBTreeRaycastCallback {

	public:

		std::vector<int32_t> mHitNodes;

		// Called when the AABB of a leaf node is hit by a ray
		virtual float raycastBroadPhaseShape(int32_t nodeId, const Ray& ray) {
			mHitNodes.push_back(nodeId);
			return 1.0;
		}

		void reset() {
			mHitNodes.clear();
		}

		bool isHit(int32_t nodeId) const {
			return std::find(mHitNodes.begin(), mHitNodes.end(), nodeId) != mHitNodes.end();
		}
};

// Class TestDynamicAABBTree
/**
 * Unit test for the dynamic AABB tree
 */
class TestDynamicAABBTree : public Test {

	private :

		// ---------- Atributes ---------- //

		OverlapCallback mOverlapCallback;
		DynamicTreeRaycastCallback m_raycastCallback;



	public :

		// ---------- Methods ---------- //

		/// Constructor
		TestDynamicAABBTree(const std::string& name): Test(name)  {


		}

		/// Run the tests
		void run() {

			testBasicsMethods();
			testOverlapping();
			testRaycast();

		}

		void testBasicsMethods() {

			// ------------ Create tree ---------- //

			// Dynamic AABB Tree
			DynamicAABBTree tree;

			int32_t object1Data = 56;
			int32_t object2Data = 23;
			int32_t object3Data = 13;
			int32_t object4Data = 7;

			// First object
			AABB aabb1 = AABB(vec3(-6, 4, -3), vec3(4, 8, 3));
			int32_t object1Id = tree.addObject(aabb1, &object1Data);

			// Second object
			AABB aabb2 = AABB(vec3(5, 2, -3), vec3(10, 7, 3));
			int32_t object2Id = tree.addObject(aabb2, &object2Data);

			// Third object
			AABB aabb3 = AABB(vec3(-5, 1, -3), vec3(-2, 3, 3));
			int32_t object3Id = tree.addObject(aabb3, &object3Data);

			// Fourth object
			AABB aabb4 = AABB(vec3(0, -4, -3), vec3(3, -2, 3));
			int32_t object4Id = tree.addObject(aabb4, &object4Data);

			// ----------- Tests ----------- //

			// Test root AABB
			AABB rootAABB = tree.getRootAABB();
			test(rootAABB.getMin().x() == -6);
			test(rootAABB.getMin().y() == -4);
			test(rootAABB.getMin().z() == -3);
			test(rootAABB.getMax().x() == 10);
			test(rootAABB.getMax().y() == 8);
			test(rootAABB.getMax().z() == 3);

			// Test data stored at the nodes of the tree
			test(*(int32_t*)(tree.getNodeDataPointer(object1Id)) == object1Data);
			test(*(int32_t*)(tree.getNodeDataPointer(object2Id)) == object2Data);
			test(*(int32_t*)(tree.getNodeDataPointer(object3Id)) == object3Data);
			test(*(int32_t*)(tree.getNodeDataPointer(object4Id)) == object4Data);
		}

		void testOverlapping() {

			// ------------- Create tree ----------- //

			// Dynamic AABB Tree
			DynamicAABBTree tree;

			int32_t object1Data = 56;
			int32_t object2Data = 23;
			int32_t object3Data = 13;
			int32_t object4Data = 7;

			// First object
			AABB aabb1 = AABB(vec3(-6, 4, -3), vec3(4, 8, 3));
			int32_t object1Id = tree.addObject(aabb1, &object1Data);

			// Second object
			AABB aabb2 = AABB(vec3(5, 2, -3), vec3(10, 7, 3));
			int32_t object2Id = tree.addObject(aabb2, &object2Data);

			// Third object
			AABB aabb3 = AABB(vec3(-5, 1, -3), vec3(-2, 3, 3));
			int32_t object3Id = tree.addObject(aabb3, &object3Data);

			// Fourth object
			AABB aabb4 = AABB(vec3(0, -4, -3), vec3(3, -2, 3));
			int32_t object4Id = tree.addObject(aabb4, &object4Data);

			// ---------- Tests ---------- //

			// AABB overlapping nothing
			mOverlapCallback.reset();
			tree.reportAllShapesOverlappingWithAABB(AABB(vec3(-10, 12, -4), vec3(10, 50, 4)), mOverlapCallback);
			test(!mOverlapCallback.isOverlapping(object1Id));
			test(!mOverlapCallback.isOverlapping(object2Id));
			test(!mOverlapCallback.isOverlapping(object3Id));
			test(!mOverlapCallback.isOverlapping(object4Id));

			// AABB overlapping everything
			mOverlapCallback.reset();
			tree.reportAllShapesOverlappingWithAABB(AABB(vec3(-15, -15, -4), vec3(15, 15, 4)), mOverlapCallback);
			test(mOverlapCallback.isOverlapping(object1Id));
			test(mOverlapCallback.isOverlapping(object2Id));
			test(mOverlapCallback.isOverlapping(object3Id));
			test(mOverlapCallback.isOverlapping(object4Id));

			// AABB overlapping object 1 and 3
			mOverlapCallback.reset();
			tree.reportAllShapesOverlappingWithAABB(AABB(vec3(-4, 2, -4), vec3(-1, 7, 4)), mOverlapCallback);
			test(mOverlapCallback.isOverlapping(object1Id));
			test(!mOverlapCallback.isOverlapping(object2Id));
			test(mOverlapCallback.isOverlapping(object3Id));
			test(!mOverlapCallback.isOverlapping(object4Id));

			// AABB overlapping object 3 and 4
			mOverlapCallback.reset();
			tree.reportAllShapesOverlappingWithAABB(AABB(vec3(-6, -5, -2), vec3(2, 2, 0)), mOverlapCallback);
			test(!mOverlapCallback.isOverlapping(object1Id));
			test(!mOverlapCallback.isOverlapping(object2Id));
			test(mOverlapCallback.isOverlapping(object3Id));
			test(mOverlapCallback.isOverlapping(object4Id));

			// AABB overlapping object 2
			mOverlapCallback.reset();
			tree.reportAllShapesOverlappingWithAABB(AABB(vec3(5, -10, -2), vec3(7, 10, 9)), mOverlapCallback);
			test(!mOverlapCallback.isOverlapping(object1Id));
			test(mOverlapCallback.isOverlapping(object2Id));
			test(!mOverlapCallback.isOverlapping(object3Id));
			test(!mOverlapCallback.isOverlapping(object4Id));

			// ---- Update the object AABBs with the initial AABBs (no reinsertion) ----- //

			tree.updateObject(object1Id, aabb1, vec3(0.0f,0.0f,0.0f)(), false);
			tree.updateObject(object2Id, aabb2, vec3(0.0f,0.0f,0.0f)(), false);
			tree.updateObject(object3Id, aabb3, vec3(0.0f,0.0f,0.0f)(), false);
			tree.updateObject(object4Id, aabb4, vec3(0.0f,0.0f,0.0f)(), false);

			// AABB overlapping nothing
			mOverlapCallback.reset();
			tree.reportAllShapesOverlappingWithAABB(AABB(vec3(-10, 12, -4), vec3(10, 50, 4)), mOverlapCallback);
			test(!mOverlapCallback.isOverlapping(object1Id));
			test(!mOverlapCallback.isOverlapping(object2Id));
			test(!mOverlapCallback.isOverlapping(object3Id));
			test(!mOverlapCallback.isOverlapping(object4Id));

			// AABB overlapping everything
			mOverlapCallback.reset();
			tree.reportAllShapesOverlappingWithAABB(AABB(vec3(-15, -15, -4), vec3(15, 15, 4)), mOverlapCallback);
			test(mOverlapCallback.isOverlapping(object1Id));
			test(mOverlapCallback.isOverlapping(object2Id));
			test(mOverlapCallback.isOverlapping(object3Id));
			test(mOverlapCallback.isOverlapping(object4Id));

			// AABB overlapping object 1 and 3
			mOverlapCallback.reset();
			tree.reportAllShapesOverlappingWithAABB(AABB(vec3(-4, 2, -4), vec3(-1, 7, 4)), mOverlapCallback);
			test(mOverlapCallback.isOverlapping(object1Id));
			test(!mOverlapCallback.isOverlapping(object2Id));
			test(mOverlapCallback.isOverlapping(object3Id));
			test(!mOverlapCallback.isOverlapping(object4Id));

			// AABB overlapping object 3 and 4
			mOverlapCallback.reset();
			tree.reportAllShapesOverlappingWithAABB(AABB(vec3(-6, -5, -2), vec3(2, 2, 0)), mOverlapCallback);
			test(!mOverlapCallback.isOverlapping(object1Id));
			test(!mOverlapCallback.isOverlapping(object2Id));
			test(mOverlapCallback.isOverlapping(object3Id));
			test(mOverlapCallback.isOverlapping(object4Id));

			// AABB overlapping object 2
			mOverlapCallback.reset();
			tree.reportAllShapesOverlappingWithAABB(AABB(vec3(5, -10, -2), vec3(7, 10, 9)), mOverlapCallback);
			test(!mOverlapCallback.isOverlapping(object1Id));
			test(mOverlapCallback.isOverlapping(object2Id));
			test(!mOverlapCallback.isOverlapping(object3Id));
			test(!mOverlapCallback.isOverlapping(object4Id));

			// ---- Update the object AABBs with the initial AABBs (with reinsertion) ----- //

			tree.updateObject(object1Id, aabb1, vec3(0.0f,0.0f,0.0f)(), true);
			tree.updateObject(object2Id, aabb2, vec3(0.0f,0.0f,0.0f)(), true);
			tree.updateObject(object3Id, aabb3, vec3(0.0f,0.0f,0.0f)(), true);
			tree.updateObject(object4Id, aabb4, vec3(0.0f,0.0f,0.0f)(), true);

			// AABB overlapping nothing
			mOverlapCallback.reset();
			tree.reportAllShapesOverlappingWithAABB(AABB(vec3(-10, 12, -4), vec3(10, 50, 4)), mOverlapCallback);
			test(!mOverlapCallback.isOverlapping(object1Id));
			test(!mOverlapCallback.isOverlapping(object2Id));
			test(!mOverlapCallback.isOverlapping(object3Id));
			test(!mOverlapCallback.isOverlapping(object4Id));

			// AABB overlapping everything
			mOverlapCallback.reset();
			tree.reportAllShapesOverlappingWithAABB(AABB(vec3(-15, -15, -4), vec3(15, 15, 4)), mOverlapCallback);
			test(mOverlapCallback.isOverlapping(object1Id));
			test(mOverlapCallback.isOverlapping(object2Id));
			test(mOverlapCallback.isOverlapping(object3Id));
			test(mOverlapCallback.isOverlapping(object4Id));

			// AABB overlapping object 1 and 3
			mOverlapCallback.reset();
			tree.reportAllShapesOverlappingWithAABB(AABB(vec3(-4, 2, -4), vec3(-1, 7, 4)), mOverlapCallback);
			test(mOverlapCallback.isOverlapping(object1Id));
			test(!mOverlapCallback.isOverlapping(object2Id));
			test(mOverlapCallback.isOverlapping(object3Id));
			test(!mOverlapCallback.isOverlapping(object4Id));

			// AABB overlapping object 3 and 4
			mOverlapCallback.reset();
			tree.reportAllShapesOverlappingWithAABB(AABB(vec3(-6, -5, -2), vec3(2, 2, 0)), mOverlapCallback);
			test(!mOverlapCallback.isOverlapping(object1Id));
			test(!mOverlapCallback.isOverlapping(object2Id));
			test(mOverlapCallback.isOverlapping(object3Id));
			test(mOverlapCallback.isOverlapping(object4Id));

			// AABB overlapping object 2
			mOverlapCallback.reset();
			tree.reportAllShapesOverlappingWithAABB(AABB(vec3(5, -10, -2), vec3(7, 10, 9)), mOverlapCallback);
			test(!mOverlapCallback.isOverlapping(object1Id));
			test(mOverlapCallback.isOverlapping(object2Id));
			test(!mOverlapCallback.isOverlapping(object3Id));
			test(!mOverlapCallback.isOverlapping(object4Id));

			// ---- Move objects 2 and 3 ----- //

			AABB newAABB2(vec3(-7, 10, -3), vec3(1, 13, 3));
			tree.updateObject(object2Id, newAABB2, vec3(0.0f,0.0f,0.0f)());

			AABB newAABB3(vec3(7, -6, -3), vec3(9, 1, 3));
			tree.updateObject(object3Id, newAABB3, vec3(0.0f,0.0f,0.0f)());

			// AABB overlapping object 3
			mOverlapCallback.reset();
			tree.reportAllShapesOverlappingWithAABB(AABB(vec3(6, -10, -2), vec3(8, 5, 3)), mOverlapCallback);
			test(!mOverlapCallback.isOverlapping(object1Id));
			test(!mOverlapCallback.isOverlapping(object2Id));
			test(mOverlapCallback.isOverlapping(object3Id));
			test(!mOverlapCallback.isOverlapping(object4Id));

			// AABB overlapping objects 1, 2
			mOverlapCallback.reset();
			tree.reportAllShapesOverlappingWithAABB(AABB(vec3(-8, 5, -3), vec3(-2, 11, 3)), mOverlapCallback);
			test(mOverlapCallback.isOverlapping(object1Id));
			test(mOverlapCallback.isOverlapping(object2Id));
			test(!mOverlapCallback.isOverlapping(object3Id));
			test(!mOverlapCallback.isOverlapping(object4Id));

		}

		void testRaycast() {

			// ------------- Create tree ----------- //

			// Dynamic AABB Tree
			DynamicAABBTree tree;

			int32_t object1Data = 56;
			int32_t object2Data = 23;
			int32_t object3Data = 13;
			int32_t object4Data = 7;

			// First object
			AABB aabb1 = AABB(vec3(-6, 4, -3), vec3(4, 8, 3));
			int32_t object1Id = tree.addObject(aabb1, &object1Data);

			// Second object
			AABB aabb2 = AABB(vec3(5, 2, -3), vec3(10, 7, 3));
			int32_t object2Id = tree.addObject(aabb2, &object2Data);

			// Third object
			AABB aabb3 = AABB(vec3(-5, 1, -3), vec3(-2, 3, 3));
			int32_t object3Id = tree.addObject(aabb3, &object3Data);

			// Fourth object
			AABB aabb4 = AABB(vec3(0, -4, -3), vec3(3, -2, 3));
			int32_t object4Id = tree.addObject(aabb4, &object4Data);

			// ---------- Tests ---------- //

			// Ray with no hits
			m_raycastCallback.reset();
			Ray ray1(vec3(4.5, -10, -5), vec3(4.5, 10, -5));
			tree.raycast(ray1, m_raycastCallback);
			test(!m_raycastCallback.isHit(object1Id));
			test(!m_raycastCallback.isHit(object2Id));
			test(!m_raycastCallback.isHit(object3Id));
			test(!m_raycastCallback.isHit(object4Id));

			// Ray that hits object 1
			m_raycastCallback.reset();
			Ray ray2(vec3(-1, -20, -2), vec3(-1, 20, -2));
			tree.raycast(ray2, m_raycastCallback);
			test(m_raycastCallback.isHit(object1Id));
			test(!m_raycastCallback.isHit(object2Id));
			test(!m_raycastCallback.isHit(object3Id));
			test(!m_raycastCallback.isHit(object4Id));

			// Ray that hits object 1 and 2
			m_raycastCallback.reset();
			Ray ray3(vec3(-7, 6, -2), vec3(8, 6, -2));
			tree.raycast(ray3, m_raycastCallback);
			test(m_raycastCallback.isHit(object1Id));
			test(m_raycastCallback.isHit(object2Id));
			test(!m_raycastCallback.isHit(object3Id));
			test(!m_raycastCallback.isHit(object4Id));

			// Ray that hits object 3
			m_raycastCallback.reset();
			Ray ray4(vec3(-7, 2, 0), vec3(-1, 2, 0));
			tree.raycast(ray4, m_raycastCallback);
			test(!m_raycastCallback.isHit(object1Id));
			test(!m_raycastCallback.isHit(object2Id));
			test(m_raycastCallback.isHit(object3Id));
			test(!m_raycastCallback.isHit(object4Id));

			// ---- Update the object AABBs with the initial AABBs (no reinsertion) ----- //

			tree.updateObject(object1Id, aabb1, vec3(0.0f,0.0f,0.0f)(), false);
			tree.updateObject(object2Id, aabb2, vec3(0.0f,0.0f,0.0f)(), false);
			tree.updateObject(object3Id, aabb3, vec3(0.0f,0.0f,0.0f)(), false);
			tree.updateObject(object4Id, aabb4, vec3(0.0f,0.0f,0.0f)(), false);

			// Ray with no hits
			m_raycastCallback.reset();
			tree.raycast(ray1, m_raycastCallback);
			test(!m_raycastCallback.isHit(object1Id));
			test(!m_raycastCallback.isHit(object2Id));
			test(!m_raycastCallback.isHit(object3Id));
			test(!m_raycastCallback.isHit(object4Id));

			// Ray that hits object 1
			m_raycastCallback.reset();
			tree.raycast(ray2, m_raycastCallback);
			test(m_raycastCallback.isHit(object1Id));
			test(!m_raycastCallback.isHit(object2Id));
			test(!m_raycastCallback.isHit(object3Id));
			test(!m_raycastCallback.isHit(object4Id));

			// Ray that hits object 1 and 2
			m_raycastCallback.reset();
			tree.raycast(ray3, m_raycastCallback);
			test(m_raycastCallback.isHit(object1Id));
			test(m_raycastCallback.isHit(object2Id));
			test(!m_raycastCallback.isHit(object3Id));
			test(!m_raycastCallback.isHit(object4Id));

			// Ray that hits object 3
			m_raycastCallback.reset();
			tree.raycast(ray4, m_raycastCallback);
			test(!m_raycastCallback.isHit(object1Id));
			test(!m_raycastCallback.isHit(object2Id));
			test(m_raycastCallback.isHit(object3Id));
			test(!m_raycastCallback.isHit(object4Id));

			// ---- Update the object AABBs with the initial AABBs (with reinsertion) ----- //

			tree.updateObject(object1Id, aabb1, vec3(0.0f,0.0f,0.0f)(), true);
			tree.updateObject(object2Id, aabb2, vec3(0.0f,0.0f,0.0f)(), true);
			tree.updateObject(object3Id, aabb3, vec3(0.0f,0.0f,0.0f)(), true);
			tree.updateObject(object4Id, aabb4, vec3(0.0f,0.0f,0.0f)(), true);

			// Ray with no hits
			m_raycastCallback.reset();
			tree.raycast(ray1, m_raycastCallback);
			test(!m_raycastCallback.isHit(object1Id));
			test(!m_raycastCallback.isHit(object2Id));
			test(!m_raycastCallback.isHit(object3Id));
			test(!m_raycastCallback.isHit(object4Id));

			// Ray that hits object 1
			m_raycastCallback.reset();
			tree.raycast(ray2, m_raycastCallback);
			test(m_raycastCallback.isHit(object1Id));
			test(!m_raycastCallback.isHit(object2Id));
			test(!m_raycastCallback.isHit(object3Id));
			test(!m_raycastCallback.isHit(object4Id));

			// Ray that hits object 1 and 2
			m_raycastCallback.reset();
			tree.raycast(ray3, m_raycastCallback);
			test(m_raycastCallback.isHit(object1Id));
			test(m_raycastCallback.isHit(object2Id));
			test(!m_raycastCallback.isHit(object3Id));
			test(!m_raycastCallback.isHit(object4Id));

			// Ray that hits object 3
			m_raycastCallback.reset();
			tree.raycast(ray4, m_raycastCallback);
			test(!m_raycastCallback.isHit(object1Id));
			test(!m_raycastCallback.isHit(object2Id));
			test(m_raycastCallback.isHit(object3Id));
			test(!m_raycastCallback.isHit(object4Id));

			// ---- Move objects 2 and 3 ----- //

			AABB newAABB2(vec3(-7, 10, -3), vec3(1, 13, 3));
			tree.updateObject(object2Id, newAABB2, vec3(0.0f,0.0f,0.0f)());

			AABB newAABB3(vec3(7, -6, -3), vec3(9, 1, 3));
			tree.updateObject(object3Id, newAABB3, vec3(0.0f,0.0f,0.0f)());

			// Ray that hits object 1, 2
			Ray ray5(vec3(-4, -5, 0), vec3(-4, 12, 0));
			m_raycastCallback.reset();
			tree.raycast(ray5, m_raycastCallback);
			test(m_raycastCallback.isHit(object1Id));
			test(m_raycastCallback.isHit(object2Id));
			test(!m_raycastCallback.isHit(object3Id));
			test(!m_raycastCallback.isHit(object4Id));

			// Ray that hits object 3 and 4
			Ray ray6(vec3(11, -3, 1), vec3(-2, -3, 1));
			m_raycastCallback.reset();
			tree.raycast(ray6, m_raycastCallback);
			test(!m_raycastCallback.isHit(object1Id));
			test(!m_raycastCallback.isHit(object2Id));
			test(m_raycastCallback.isHit(object3Id));
			test(m_raycastCallback.isHit(object4Id));
		}
 };

}

#endif