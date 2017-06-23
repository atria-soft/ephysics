/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

#include <ephysics/configuration.hpp>
#include <etk/math/Vector3D.hpp>

namespace ephysics {
	class Triangle {
		public:
			vec3 value[3];
			vec3& operator[] (size_t _id) {
				return value[_id];
			}
	};
	/**
	 * This class is used to describe the vertices and faces of a triangular mesh.
	 * A TriangleVertexArray represents a continuous array of vertices and indexes
	 * of a triangular mesh. When you create a TriangleVertexArray, no data is copied
	 * int32_to the array. It only stores pointer to the data. The purpose is to allow
	 * the user to share vertices data between the physics engine and the rendering
	 * part. Therefore, make sure that the data pointed by a TriangleVertexArray
	 * remains valid during the TriangleVertexArray life.
	 */
	class TriangleVertexArray {
		protected:
			std::vector<vec3> m_vertices; //!< Vertice list
			std::vector<size_t> m_triangles; //!< List of triangle (3 pos for each triangle)
		public:
			/**
			 * @brief Constructor
			 * @param[in] _vertices List Of all vertices
			 * @param[in] _triangles List of all linked points
			 */
			TriangleVertexArray(const std::vector<vec3>& _vertices,
			                    std::vector<size_t> _triangles);
			/**
			 * @brief Get the number of vertices
			 * @return Number of vertices
			 */
			size_t getNbVertices() const;
			/**
			 * @brief Get the number of triangle
			 * @return Number of triangles
			 */
			size_t getNbTriangles() const;
			/**
			 * @brief Get The table of the vertices
			 * @return reference on the vertices
			 */
			const std::vector<vec3>& getVertices() const;
			/**
			 * @brief Get The table of the triangle indice
			 * @return reference on the triangle indice
			 */
			const std::vector<size_t>& getIndices() const;
			/**
			 * @brief Get a triangle at the specific ID
			 * @return Buffer of 3 points
			 */
			ephysics::Triangle getTriangle(size_t _id) const;
	};


}


