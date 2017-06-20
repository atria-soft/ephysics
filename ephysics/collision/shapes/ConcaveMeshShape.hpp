/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

#include <ephysics/collision/shapes/ConcaveShape.hpp>
#include <ephysics/collision/broadphase/DynamicAABBTree.hpp>
#include <ephysics/collision/TriangleMesh.hpp>
#include <ephysics/collision/shapes/TriangleShape.hpp>
#include <ephysics/engine/Profiler.hpp>

namespace ephysics {

	class ConcaveMeshShape;
	class ConvexTriangleAABBOverlapCallback : public DynamicAABBTreeOverlapCallback {
		private:
			TriangleCallback& m_triangleTestCallback; //!< 
			const ConcaveMeshShape& m_concaveMeshShape; //!< Reference to the concave mesh shape
			const DynamicAABBTree& m_dynamicAABBTree; //!< Reference to the Dynamic AABB tree
		public:
			// Constructor
			ConvexTriangleAABBOverlapCallback(TriangleCallback& _triangleCallback,
			                                  const ConcaveMeshShape& _concaveShape,
			                                  const DynamicAABBTree& _dynamicAABBTree):
			  m_triangleTestCallback(_triangleCallback),
			  m_concaveMeshShape(_concaveShape),
			  m_dynamicAABBTree(_dynamicAABBTree) {
				
			}
			// Called when a overlapping node has been found during the call to
			// DynamicAABBTree:reportAllShapesOverlappingWithAABB()
			virtual void notifyOverlappingNode(int32_t _nodeId);
	
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
			ConcaveMeshRaycastCallback(const DynamicAABBTree& _dynamicAABBTree,
			                           const ConcaveMeshShape& _concaveMeshShape,
			                           ProxyShape* _proxyShape,
			                           RaycastInfo& _raycastInfo,
			                           const Ray& _ray):
			  m_dynamicAABBTree(_dynamicAABBTree),
			  m_concaveMeshShape(_concaveMeshShape),
			  m_proxyShape(_proxyShape),
			  m_raycastInfo(_raycastInfo),
			  m_ray(_ray),
			  mIsHit(false) {
				
			}
			/// Collect all the AABB nodes that are hit by the ray in the Dynamic AABB Tree
			virtual float raycastBroadPhaseShape(int32_t _nodeId, const Ray& _ray);
			/// Raycast all collision shapes that have been collected
			void raycastTriangles();
			/// Return true if a raycast hit has been found
			bool getIsHit() const {
				return mIsHit;
			}
	};
	/**
	 * @brief Represents a static concave mesh shape. Note that collision detection
	 * with a concave mesh shape can be very expensive. You should use only use
	 * this shape for a static mesh.
	 */
	class ConcaveMeshShape : public ConcaveShape {
		protected:
			TriangleMesh* m_triangleMesh; //!< Triangle mesh
			DynamicAABBTree m_dynamicAABBTree; //!< Dynamic AABB tree to accelerate collision with the triangles
			/// Private copy-constructor
			ConcaveMeshShape(const ConcaveMeshShape& _shape) = delete;
			/// Private assignment operator
			ConcaveMeshShape& operator=(const ConcaveMeshShape& _shape) = delete;
			/// Raycast method with feedback information
			virtual bool raycast(const Ray& _ray, RaycastInfo& _raycastInfo, ProxyShape* _proxyShape) const;
			/// Return the number of bytes used by the collision shape
			virtual size_t getSizeInBytes() const;
			/// Insert all the triangles int32_to the dynamic AABB tree
			void initBVHTree();
			/// Return the three vertices coordinates (in the array outTriangleVertices) of a triangle
			/// given the start vertex index pointer of the triangle.
			void getTriangleVerticesWithIndexPointer(int32_t _subPart,
			                                         int32_t _triangleIndex,
			                                         vec3* _outTriangleVertices) const;
		public:
			/// Constructor
			ConcaveMeshShape(TriangleMesh* triangleMesh);
			/// Destructor
			~ConcaveMeshShape();
			/// Return the local bounds of the shape in x, y and z directions.
			virtual void getLocalBounds(vec3& min, vec3& max) const;
			/// Set the local scaling vector of the collision shape
			virtual void setLocalScaling(const vec3& scaling);
			/// Return the local inertia tensor of the collision shape
			virtual void computeLocalInertiaTensor(etk::Matrix3x3& tensor, float mass) const;
			/// Use a callback method on all triangles of the concave shape inside a given AABB
			virtual void testAllTriangles(TriangleCallback& callback, const AABB& localAABB) const;
			friend class ConvexTriangleAABBOverlapCallback;
			friend class ConcaveMeshRaycastCallback;
	};

// Return the number of bytes used by the collision shape
size_t ConcaveMeshShape::getSizeInBytes() const {
	return sizeof(ConcaveMeshShape);
}

// Return the local bounds of the shape in x, y and z directions.
// This method is used to compute the AABB of the box
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
void ConcaveMeshShape::getLocalBounds(vec3& min, vec3& max) const {

	// Get the AABB of the whole tree
	AABB treeAABB = m_dynamicAABBTree.getRootAABB();

	min = treeAABB.getMin();
	max = treeAABB.getMax();
}

// Set the local scaling vector of the collision shape
void ConcaveMeshShape::setLocalScaling(const vec3& scaling) {

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
void ConcaveMeshShape::computeLocalInertiaTensor(etk::Matrix3x3& _tensor, float _mass) const {

	// Default inertia tensor
	// Note that this is not very realistic for a concave triangle mesh.
	// However, in most cases, it will only be used static bodies and therefore,
	// the inertia tensor is not used.
	tensor.setValue(mass, 0, 0,
						0, mass, 0,
						0, 0, mass);
}

// Called when a overlapping node has been found during the call to
// DynamicAABBTree:reportAllShapesOverlappingWithAABB()
void ConvexTriangleAABBOverlapCallback::notifyOverlappingNode(int32_t _nodeId) {

	// Get the node data (triangle index and mesh subpart index)
	int32_t* data = m_dynamicAABBTree.getNodeDataInt(nodeId);

	// Get the triangle vertices for this node from the concave mesh shape
	vec3 trianglePoints[3];
	m_concaveMeshShape.getTriangleVerticesWithIndexPointer(data[0], data[1], trianglePoints);

	// Call the callback to test narrow-phase collision with this triangle
	m_triangleTestCallback.testTriangle(trianglePoints);
}

}
