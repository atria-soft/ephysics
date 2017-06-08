/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/configuration.h>

namespace reactphysics3d {

// Class TriangleVertexArray
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

	public:

		/// Data type for the vertices in the array
		enum VertexDataType {VERTEX_FLOAT_TYPE, VERTEX_DOUBLE_TYPE};

		/// Data type for the indices in the array
		enum IndexDataType {INDEX_INTEGER_TYPE, INDEX_SHORT_TYPE};

	protected:

		/// Number of vertices in the array
		uint32_t m_numberVertices;

		/// Pointer to the first vertex value in the array
		unsigned char* m_verticesStart;

		/// Stride (number of bytes) between the beginning of two vertices
		/// values in the array
		int32_t m_verticesStride;

		/// Number of triangles in the array
		uint32_t mNbTriangles;

		/// Pointer to the first vertex index of the array
		unsigned char* mIndicesStart;

		/// Stride (number of bytes) between the beginning of two indices in
		/// the array
		int32_t mIndicesStride;

		/// Data type of the vertices in the array
		VertexDataType mVertexDataType;

		/// Data type of the indices in the array
		IndexDataType mIndexDataType;

	public:

		/// Constructor
		TriangleVertexArray(uint32_t nbVertices, void* verticesStart, int32_t verticesStride,
							uint32_t nbTriangles, void* indexesStart, int32_t indexesStride,
							VertexDataType vertexDataType, IndexDataType indexDataType);

		/// Destructor
		virtual ~TriangleVertexArray();

		/// Return the vertex data type
		VertexDataType getVertexDataType() const;

		/// Return the index data type
		IndexDataType getIndexDataType() const;

		/// Return the number of vertices
		uint32_t getNbVertices() const;

		/// Return the number of triangles
		uint32_t getNbTriangles() const;

		/// Return the vertices stride (number of bytes)
		int32_t getVerticesStride() const;

		/// Return the indices stride (number of bytes)
		int32_t getIndicesStride() const;

		/// Return the pointer to the start of the vertices array
		unsigned char* getVerticesStart() const;

		/// Return the pointer to the start of the indices array
		unsigned char* getIndicesStart() const;
};

// Return the vertex data type
inline TriangleVertexArray::VertexDataType TriangleVertexArray::getVertexDataType() const {
	return mVertexDataType;
}

// Return the index data type
inline TriangleVertexArray::IndexDataType TriangleVertexArray::getIndexDataType() const {
   return mIndexDataType;
}

// Return the number of vertices
inline uint32_t TriangleVertexArray::getNbVertices() const {
	return m_numberVertices;
}

// Return the number of triangles
inline uint32_t TriangleVertexArray::getNbTriangles() const {
	return mNbTriangles;
}

// Return the vertices stride (number of bytes)
inline int32_t TriangleVertexArray::getVerticesStride() const {
	return m_verticesStride;
}

// Return the indices stride (number of bytes)
inline int32_t TriangleVertexArray::getIndicesStride() const {
	return mIndicesStride;
}

// Return the pointer to the start of the vertices array
inline unsigned char* TriangleVertexArray::getVerticesStart() const {
	return m_verticesStart;
}

// Return the pointer to the start of the indices array
inline unsigned char* TriangleVertexArray::getIndicesStart() const {
	return mIndicesStart;
}

}


