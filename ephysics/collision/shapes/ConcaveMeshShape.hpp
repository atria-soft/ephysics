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
	
	class ConcaveMeshRaycastCallback : public DynamicAABBTreeRaycastCallback {
		private :
			etk::Vector<int32_t> m_hitAABBNodes;
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
			virtual bool raycast(const Ray& _ray, RaycastInfo& _raycastInfo, ProxyShape* _proxyShape) const override;
			virtual size_t getSizeInBytes() const override;
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
			virtual void getLocalBounds(vec3& min, vec3& max) const override;
			virtual void setLocalScaling(const vec3& scaling) override;
			virtual void computeLocalInertiaTensor(etk::Matrix3x3& tensor, float mass) const override;
			/// Use a callback method on all triangles of the concave shape inside a given AABB
			virtual void testAllTriangles(TriangleCallback& callback, const AABB& localAABB) const override;
			friend class ConvexTriangleAABBOverlapCallback;
			friend class ConcaveMeshRaycastCallback;
	};

}
