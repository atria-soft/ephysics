/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#pragma once
#include <etk/Vector.hpp>
#include <ephysics/collision/TriangleVertexArray.hpp>

namespace ephysics {
	/**
	 * @brief Represents a mesh made of triangles. A TriangleMesh contains
	 * one or several parts. Each part is a set of triangles represented in a
	 * TriangleVertexArray object describing all the triangles vertices of the part.
	 * A TriangleMesh object is used to create a ConcaveMeshShape from a triangle
	 * mesh for instance.
	 */
	class TriangleMesh {
		protected:
			etk::Vector<TriangleVertexArray*> m_triangleArrays; //!< All the triangle arrays of the mesh (one triangle array per part)
		public:
			/**
			 * @brief Constructor
			 */
			TriangleMesh();
			/**
			 * @brief Virtualisation of Destructor
			 */
			virtual ~TriangleMesh() = default;
			/**
			 * @brief Add a subpart of the mesh
			 */
			void addSubpart(TriangleVertexArray* _triangleVertexArray) {
				m_triangleArrays.pushBack(_triangleVertexArray );
			}
			/**
			 * @brief Get a pointer to a given subpart (triangle vertex array) of the mesh
			 */
			TriangleVertexArray* getSubpart(uint32_t _indexSubpart) const {
				assert(_indexSubpart < m_triangleArrays.size());
				return m_triangleArrays[_indexSubpart];
			}
			/**
			 * @brief Get the number of subparts of the mesh
			 */
			uint32_t getNbSubparts() const {
				return m_triangleArrays.size();
			}
	};
}


