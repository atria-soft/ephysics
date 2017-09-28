/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

#include <ephysics/collision/shapes/ConvexShape.hpp>
#include <ephysics/engine/CollisionWorld.hpp>
#include <ephysics/mathematics/mathematics.hpp>
#include <ephysics/collision/TriangleMesh.hpp>
#include <ephysics/collision/narrowphase/GJK/GJKAlgorithm.hpp>
#include <etk/Vector.hpp>
#include <etk/Map.hpp>

namespace ephysics {
	class CollisionWorld;
	/**
	 * @brief It represents a convex mesh shape. In order to create a convex mesh shape, you
	 * need to indicate the local-space position of the mesh vertices. You do it either by
	 * passing a vertices array to the constructor or using the addVertex() method. Make sure
	 * that the set of vertices that you use to create the shape are indeed part of a convex
	 * mesh. The center of mass of the shape will be at the origin of the local-space geometry
	 * that you use to create the mesh. The method used for collision detection with a convex
	 * mesh shape has an O(n) running time with "n" beeing the number of vertices in the mesh.
	 * Therefore, you should try not to use too many vertices. However, it is possible to speed
	 * up the collision detection by using the edges information of your mesh. The running time
	 * of the collision detection that uses the edges is almost O(1) constant time at the cost
	 * of additional memory used to store the vertices. You can indicate edges information
	 * with the addEdge() method. Then, you must use the setIsEdgesInformationUsed(true) method
	 * in order to use the edges information for collision detection.
	 */
	class ConvexMeshShape : public ConvexShape {
		protected :
			etk::Vector<vec3> m_vertices; //!< Array with the vertices of the mesh
			uint32_t m_numberVertices; //!< Number of vertices in the mesh
			vec3 m_minBounds; //!< Mesh minimum bounds in the three local x, y and z directions
			vec3 m_maxBounds; //!< Mesh maximum bounds in the three local x, y and z directions
			bool m_isEdgesInformationUsed; //!< True if the shape contains the edges of the convex mesh in order to make the collision detection faster
			etk::Map<uint32_t, etk::Set<uint32_t> > m_edgesAdjacencyList; //!< Adjacency list representing the edges of the mesh
			/// Private copy-constructor
			ConvexMeshShape(const ConvexMeshShape& _shape);
			/// Private assignment operator
			ConvexMeshShape& operator=(const ConvexMeshShape& _shape);
			/// Recompute the bounds of the mesh
			void recalculateBounds();
			void setLocalScaling(const vec3& _scaling) override;
			vec3 getLocalSupportPointWithoutMargin(const vec3& _direction, void** _cachedCollisionData) const override;
			bool testPointInside(const vec3& _localPoint, ProxyShape* _proxyShape) const override;
			bool raycast(const Ray& _ray, RaycastInfo& _raycastInfo, ProxyShape* _proxyShape) const override;
			size_t getSizeInBytes() const override;
		public :
			/**
			 * @brief  Constructor to initialize with an array of 3D vertices.
			 * This method creates an int32_ternal copy of the input vertices.
			 * @param[in] _arrayVertices Array with the vertices of the convex mesh
			 * @param[in] _nbVertices Number of vertices in the convex mesh
			 * @param[in] _stride Stride between the beginning of two elements in the vertices array
			 * @param[in] _margin Collision margin (in meters) around the collision shape
			 */
			ConvexMeshShape(const float* _arrayVertices,
			                uint32_t _nbVertices,
			                int32_t _stride,
			                float _margin = OBJECT_MARGIN);
			/**
			 * @brief Constructor to initialize with a triangle mesh
			 * This method creates an internal copy of the input vertices.
			 * @param _triangleVertexArray Array with the vertices and indices of the vertices and triangles of the mesh
			 * @param _isEdgesInformationUsed True if you want to use edges information for collision detection (faster but requires more memory)
			 * @param _margin Collision margin (in meters) around the collision shape
			 */
			ConvexMeshShape(TriangleVertexArray* _triangleVertexArray,
			                bool _isEdgesInformationUsed = true,
			                float _margin = OBJECT_MARGIN);
			/// Constructor.
			ConvexMeshShape(float _margin = OBJECT_MARGIN);
			void getLocalBounds(vec3& _min, vec3& _max) const override;
			void computeLocalInertiaTensor(etk::Matrix3x3& _tensor, float _mass) const override;
			/**
			 * @brief Add a vertex int32_to the convex mesh
			 * @param vertex Vertex to be added
			 */
			void addVertex(const vec3& _vertex);
			/**
			 * @brief Add an edge int32_to the convex mesh by specifying the two vertex indices of the edge.
			 * Note that the vertex indices start at zero and need to correspond to the order of
			 * the vertices in the vertices array in the constructor or the order of the calls
			 * of the addVertex() methods that you use to add vertices int32_to the convex mesh.
			 * @param[in] _v1 Index of the first vertex of the edge to add
			 * @param[in] _v2 Index of the second vertex of the edge to add
			 */
			void addEdge(uint32_t _v1, uint32_t _v2);
			/**
			 * @brief Return true if the edges information is used to speed up the collision detection
			 * @return True if the edges information is used and false otherwise
			 */
			bool isEdgesInformationUsed() const;
			/**
			 * @brief Set the variable to know if the edges information is used to speed up the
			 * collision detection
			 * @param[in] isEdgesUsed True if you want to use the edges information to speed up the collision detection with the convex mesh shape
			 */
			void setIsEdgesInformationUsed(bool _isEdgesUsed);
	};
}


