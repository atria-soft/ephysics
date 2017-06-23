/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/TriangleVertexArray.hpp>


ephysics::TriangleVertexArray::TriangleVertexArray(const std::vector<vec3>& _vertices, std::vector<size_t> _triangles):
  m_vertices(_vertices),
  m_triangles(_triangles) {
	
}

size_t ephysics::TriangleVertexArray::getNbVertices() const {
	return m_vertices.size();
}

size_t ephysics::TriangleVertexArray::getNbTriangles() const {
	return m_triangles.size()/3;
}

const std::vector<vec3>& ephysics::TriangleVertexArray::getVertices() const {
	return m_vertices;
}

const std::vector<size_t>& ephysics::TriangleVertexArray::getIndices() const{
	return m_triangles;
}

ephysics::Triangle ephysics::TriangleVertexArray::getTriangle(size_t _id) const {
	ephysics::Triangle out;
	out[0] = m_vertices[m_triangles[_id*3]];
	out[1] = m_vertices[m_triangles[_id*3+1]];
	out[2] = m_vertices[m_triangles[_id*3+2]];
	return out;
}