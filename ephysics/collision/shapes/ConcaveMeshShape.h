/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/collision/shapes/ConcaveShape.h>
#include <ephysics/collision/broadphase/DynamicAABBTree.h>
#include <ephysics/collision/TriangleMesh.h>
#include <ephysics/collision/shapes/TriangleShape.h>
#include <ephysics/engine/Profiler.h>

namespace reactphysics3d {

class ConcaveMeshShape;

// class ConvexTriangleAABBOverlapCallback
class ConvexTriangleAABBOverlapCallback : public DynamicAABBTreeOverlapCallback {

	private:

		TriangleCallback& m_triangleTestCallback;

		// Reference to the concave mesh shape
		const ConcaveMeshShape& m_concaveMeshShape;

		// Reference to the Dynamic AABB tree
		const DynamicAABBTree& m_dynamicAABBTree;

	public:

		// Constructor
		ConvexTriangleAABBOverlapCallback(TriangleCallback& triangleCallback, const ConcaveMeshShape& concaveShape,
										  const DynamicAABBTree& dynamicAABBTree)
		  : m_triangleTestCallback(triangleCallback), m_concaveMeshShape(concaveShape), m_dynamicAABBTree(dynamicAABBTree) {

		}

		// Called when a overlapping node has been found during the call to
		// DynamicAABBTree:reportAllShapesOverlappingWithAABB()
		virtual void notifyOverlappingNode(int32_t nodeId);

};

/// Class ConcaveMeshRaycastCallback
class ConcaveMeshRaycastCallback : public DynamicAABBTreeRaycastCallback {

	private :

		std::vector<int32_t> m_hitAABBNodes;
		const DynamicAABBTree& m_dynamicAABBTree;
		const ConcaveMeshShape& m_concaveMeshShape;
		ProxyShape* m_proxyShape;
		RaycastInfo& m_raycastInfo;
		const Ray& m_ray;
		bool mIsHit;

	public:

		// Constructor
		ConcaveMeshRaycastCallback(const DynamicAABBTree& dynamicAABBTree, const ConcaveMeshShape& concaveMeshShape,
								   ProxyShape* proxyShape, RaycastInfo& raycastInfo, const Ray& ray)
			: m_dynamicAABBTree(dynamicAABBTree), m_concaveMeshShape(concaveMeshShape), m_proxyShape(proxyShape),
			  m_raycastInfo(raycastInfo), m_ray(ray), mIsHit(false) {

		}

		/// Collect all the AABB nodes that are hit by the ray in the Dynamic AABB Tree
		virtual float raycastBroadPhaseShape(int32_t nodeId, const Ray& ray);

		/// Raycast all collision shapes that have been collected
		void raycastTriangles();

		/// Return true if a raycast hit has been found
		bool getIsHit() const {
			return mIsHit;
		}
};

// Class ConcaveMeshShape
/**
 * This class represents a static concave mesh shape. Note that collision detection
 * with a concave mesh shape can be very expensive. You should use only use
 * this shape for a static mesh.
 */
class ConcaveMeshShape : public ConcaveShape {

	protected:

		// -------------------- Attributes -------------------- //

		/// Triangle mesh
		TriangleMesh* m_triangleMesh;

		/// Dynamic AABB tree to accelerate collision with the triangles
		DynamicAABBTree m_dynamicAABBTree;

		// -------------------- Methods -------------------- //

		/// Private copy-constructor
		ConcaveMeshShape(const ConcaveMeshShape& shape);

		/// Private assignment operator
		ConcaveMeshShape& operator=(const ConcaveMeshShape& shape);

		/// Raycast method with feedback information
		virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const;

		/// Return the number of bytes used by the collision shape
		virtual size_t getSizeInBytes() const;

		/// Insert all the triangles int32_to the dynamic AABB tree
		void initBVHTree();

		/// Return the three vertices coordinates (in the array outTriangleVertices) of a triangle
		/// given the start vertex index pointer of the triangle.
		void getTriangleVerticesWithIndexPointer(int32_t subPart, int32_t triangleIndex,
												 Vector3* outTriangleVertices) const;

	public:

		/// Constructor
		ConcaveMeshShape(TriangleMesh* triangleMesh);

		/// Destructor
		~ConcaveMeshShape();

		/// Return the local bounds of the shape in x, y and z directions.
		virtual void getLocalBounds(Vector3& min, Vector3& max) const;

		/// Set the local scaling vector of the collision shape
		virtual void setLocalScaling(const Vector3& scaling);

		/// Return the local inertia tensor of the collision shape
		virtual void computeLocalInertiaTensor(Matrix3x3& tensor, float mass) const;

		/// Use a callback method on all triangles of the concave shape inside a given AABB
		virtual void testAllTriangles(TriangleCallback& callback, const AABB& localAABB) const;

		// ---------- Friendship ----------- //

		friend class ConvexTriangleAABBOverlapCallback;
		friend class ConcaveMeshRaycastCallback;
};

// Return the number of bytes used by the collision shape
inline size_t ConcaveMeshShape::getSizeInBytes() const {
	return sizeof(ConcaveMeshShape);
}

// Return the local bounds of the shape in x, y and z directions.
// This method is used to compute the AABB of the box
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
inline void ConcaveMeshShape::getLocalBounds(Vector3& min, Vector3& max) const {

	// Get the AABB of the whole tree
	AABB treeAABB = m_dynamicAABBTree.getRootAABB();

	min = treeAABB.getMin();
	max = treeAABB.getMax();
}

// Set the local scaling vector of the collision shape
inline void ConcaveMeshShape::setLocalScaling(const Vector3& scaling) {

	CollisionShape::setLocalScaling(scaling);

	// Reset the Dynamic AABB Tree
	m_dynamicAABBTree.reset();

	// Rebuild Dynamic AABB Tree here
	initBVHTree();
}

// Return the local inertia tensor of the shape
/**
 * @param[out] tensor The 3x3 inertia tensor matrix of the shape in local-space
 *					coordinates
 * @param mass Mass to use to compute the inertia tensor of the collision shape
 */
inline void ConcaveMeshShape::computeLocalInertiaTensor(Matrix3x3& tensor, float mass) const {

	// Default inertia tensor
	// Note that this is not very realistic for a concave triangle mesh.
	// However, in most cases, it will only be used static bodies and therefore,
	// the inertia tensor is not used.
	tensor.setAllValues(mass, 0, 0,
						0, mass, 0,
						0, 0, mass);
}

// Called when a overlapping node has been found during the call to
// DynamicAABBTree:reportAllShapesOverlappingWithAABB()
inline void ConvexTriangleAABBOverlapCallback::notifyOverlappingNode(int32_t nodeId) {

	// Get the node data (triangle index and mesh subpart index)
	int32_t* data = m_dynamicAABBTree.getNodeDataInt(nodeId);

	// Get the triangle vertices for this node from the concave mesh shape
	Vector3 trianglePoints[3];
	m_concaveMeshShape.getTriangleVerticesWithIndexPointer(data[0], data[1], trianglePoints);

	// Call the callback to test narrow-phase collision with this triangle
	m_triangleTestCallback.testTriangle(trianglePoints);
}

}
