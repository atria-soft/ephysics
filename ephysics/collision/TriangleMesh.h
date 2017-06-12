/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <vector>
#include <cassert>
#include <ephysics/collision/TriangleVertexArray.h>

namespace reactphysics3d {

// Class TriangleMesh
/**
 * This class represents a mesh made of triangles. A TriangleMesh contains
 * one or several parts. Each part is a set of triangles represented in a
 * TriangleVertexArray object describing all the triangles vertices of the part.
 * A TriangleMesh object is used to create a ConcaveMeshShape from a triangle
 * mesh for instance.
 */
class TriangleMesh {

	protected:

		/// All the triangle arrays of the mesh (one triangle array per part)
		std::vector<TriangleVertexArray*> m_triangleArrays;

	public:

		/// Constructor
		TriangleMesh();

		/// Destructor
		virtual ~TriangleMesh();

		/// Add a subpart of the mesh
		void addSubpart(TriangleVertexArray* triangleVertexArray);

		/// Return a pointer to a given subpart (triangle vertex array) of the mesh
		TriangleVertexArray* getSubpart(uint32_t indexSubpart) const;

		/// Return the number of subparts of the mesh
		uint32_t getNbSubparts() const;
};

// Add a subpart of the mesh
inline void TriangleMesh::addSubpart(TriangleVertexArray* triangleVertexArray) {
	m_triangleArrays.push_back(triangleVertexArray );
}

// Return a pointer to a given subpart (triangle vertex array) of the mesh
inline TriangleVertexArray* TriangleMesh::getSubpart(uint32_t indexSubpart) const {
   assert(indexSubpart < m_triangleArrays.size());
   return m_triangleArrays[indexSubpart];
}

// Return the number of subparts of the mesh
inline uint32_t TriangleMesh::getNbSubparts() const {
	return m_triangleArrays.size();
}

}


