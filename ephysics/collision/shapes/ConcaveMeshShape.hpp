/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#pragma once

#include <ephysics/collision/shapes/ConcaveShape.hpp>
#include <ephysics/collision/broadphase/DynamicAABBTree.hpp>
#include <ephysics/collision/TriangleMesh.hpp>
#include <ephysics/collision/shapes/TriangleShape.hpp>
#include <ephysics/engine/Profiler.hpp>

namespace ephysics {
	class ConcaveMeshShape;
	class ConcaveMeshRaycastCallback {
		private:
			etk::Vector<int32_t> m_hitAABBNodes;
			const DynamicAABBTree& m_dynamicAABBTree;
			const ConcaveMeshShape& m_concaveMeshShape;
			ProxyShape* m_proxyShape;
			RaycastInfo& m_raycastInfo;
			const Ray& m_ray;
			bool m_isHit;
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
			  m_isHit(false) {
				
			}
			/// Collect all the AABB nodes that are hit by the ray in the Dynamic AABB Tree
			float operator()(int32_t _nodeId, const ephysics::Ray& _ray);
			/// Raycast all collision shapes that have been collected
			void raycastTriangles();
			/// Return true if a raycast hit has been found
			bool getIsHit() const {
				return m_isHit;
			}
	};
	/**
	 * @brief Represents a static concave mesh shape. Note that collision detection
	 * with a concave mesh shape can be very expensive. You should use only use
	 * this shape for a static mesh.
	 */
	class ConcaveMeshShape : public ConcaveShape {
		public:
			/// Constructor
			ConcaveMeshShape(TriangleMesh* _triangleMesh);
			/// DELETE copy-constructor
			ConcaveMeshShape(const ConcaveMeshShape& _shape) = delete;
			/// DELETE assignment operator
			ConcaveMeshShape& operator=(const ConcaveMeshShape& _shape) = delete;
			virtual void getLocalBounds(vec3& _min, vec3& _max) const override;
			virtual void setLocalScaling(const vec3& _scaling) override;
			virtual void computeLocalInertiaTensor(etk::Matrix3x3& _tensor, float _mass) const override;
			virtual void testAllTriangles(TriangleCallback& _callback, const AABB& _localAABB) const override;
			friend class ConvexTriangleAABBOverlapCallback;
			friend class ConcaveMeshRaycastCallback;
		protected:
			TriangleMesh* m_triangleMesh; //!< Triangle mesh
			DynamicAABBTree m_dynamicAABBTree; //!< Dynamic AABB tree to accelerate collision with the triangles
			virtual bool raycast(const Ray& _ray, RaycastInfo& _raycastInfo, ProxyShape* _proxyShape) const override;
			virtual size_t getSizeInBytes() const override;
			/// Insert all the triangles int32_to the dynamic AABB tree
			void initBVHTree();
			/// Return the three vertices coordinates (in the array outTriangleVertices) of a triangle
			/// given the start vertex index pointer of the triangle.
			void getTriangleVerticesWithIndexPointer(int32_t _subPart,
			                                         int32_t _triangleIndex,
			                                         vec3* _outTriangleVertices) const;
	};

}
