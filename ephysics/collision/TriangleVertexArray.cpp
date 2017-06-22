/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/TriangleVertexArray.hpp>

using namespace ephysics;

// Constructor
/// Note that your data will not be copied int32_to the TriangleVertexArray and
/// therefore, you need to make sure that those data are always valid during
/// the lifetime of the TriangleVertexArray.
/**
 * @param nbVertices Number of vertices in the array
 * @param verticesStart Pointer to the first vertices of the array
 * @param verticesStride Number of bytes between the beginning of two consecutive vertices
 * @param nbTriangles Number of triangles in the array
 * @param indexesStart Pointer to the first triangle index
 * @param indexesStride Number of bytes between the beginning of two consecutive triangle indices
 * @param vertexDataType Type of data for the vertices (float, double)
 * @param indexDataType Type of data for the indices (short, int32_t)
 */
TriangleVertexArray::TriangleVertexArray(uint32_t nbVertices, void* verticesStart, int32_t verticesStride,
										 uint32_t nbTriangles, void* indexesStart, int32_t indexesStride,
										 VertexDataType vertexDataType, IndexDataType indexDataType) {
	m_numberVertices = nbVertices;
	m_verticesStart = reinterpret_cast<unsigned char*>(verticesStart);
	m_verticesStride = verticesStride;
	m_numberTriangles = nbTriangles;
	m_indicesStart = reinterpret_cast<unsigned char*>(indexesStart);
	m_indicesStride = indexesStride;
	m_vertexDataType = vertexDataType;
	m_indexDataType = indexDataType;
}

// Destructor
TriangleVertexArray::~TriangleVertexArray() {

}

// Return the vertex data type
TriangleVertexArray::VertexDataType TriangleVertexArray::getVertexDataType() const {
	return m_vertexDataType;
}

// Return the index data type
TriangleVertexArray::IndexDataType TriangleVertexArray::getIndexDataType() const {
   return m_indexDataType;
}

// Return the number of vertices
uint32_t TriangleVertexArray::getNbVertices() const {
	return m_numberVertices;
}

// Return the number of triangles
uint32_t TriangleVertexArray::getNbTriangles() const {
	return m_numberTriangles;
}

// Return the vertices stride (number of bytes)
int32_t TriangleVertexArray::getVerticesStride() const {
	return m_verticesStride;
}

// Return the indices stride (number of bytes)
int32_t TriangleVertexArray::getIndicesStride() const {
	return m_indicesStride;
}

// Return the pointer to the start of the vertices array
unsigned char* TriangleVertexArray::getVerticesStart() const {
	return m_verticesStart;
}

// Return the pointer to the start of the indices array
unsigned char* TriangleVertexArray::getIndicesStart() const {
	return m_indicesStart;
}