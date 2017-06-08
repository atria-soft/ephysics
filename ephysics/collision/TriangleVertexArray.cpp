/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/TriangleVertexArray.h>

using namespace reactphysics3d;

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
	mNbTriangles = nbTriangles;
	mIndicesStart = reinterpret_cast<unsigned char*>(indexesStart);
	mIndicesStride = indexesStride;
	mVertexDataType = vertexDataType;
	mIndexDataType = indexDataType;
}

// Destructor
TriangleVertexArray::~TriangleVertexArray() {

}
