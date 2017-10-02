/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#include <ephysics/collision/TriangleVertexArray.hpp>


ephysics::TriangleVertexArray::TriangleVertexArray(const etk::Vector<vec3>& _vertices, const etk::Vector<uint32_t>& _triangles):
  m_vertices(_vertices),
  m_triangles(_triangles) {
	
}

size_t ephysics::TriangleVertexArray::getNbVertices() const {
	return m_vertices.size();
}

size_t ephysics::TriangleVertexArray::getNbTriangles() const {
	return m_triangles.size()/3;
}

const etk::Vector<vec3>& ephysics::TriangleVertexArray::getVertices() const {
	return m_vertices;
}

const etk::Vector<uint32_t>& ephysics::TriangleVertexArray::getIndices() const{
	return m_triangles;
}

ephysics::Triangle ephysics::TriangleVertexArray::getTriangle(uint32_t _id) const {
	ephysics::Triangle out;
	out[0] = m_vertices[m_triangles[_id*3]];
	out[1] = m_vertices[m_triangles[_id*3+1]];
	out[2] = m_vertices[m_triangles[_id*3+2]];
	return out;
}