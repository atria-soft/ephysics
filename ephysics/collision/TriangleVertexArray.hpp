/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

#include <ephysics/configuration.hpp>

namespace ephysics {
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
			uint32_t m_numberVertices; //!< Number of vertices in the array
			unsigned char* m_verticesStart; //!< Pointer to the first vertex value in the array
			int32_t m_verticesStride; //!< Stride (number of bytes) between the beginning of two vertices values in the array
			uint32_t m_numberTriangles; //!< Number of triangles in the array
			unsigned char* m_indicesStart; //!< Pointer to the first vertex index of the array
			int32_t m_indicesStride; //!< Stride (number of bytes) between the beginning of two indices in the array
			VertexDataType m_vertexDataType; //!< Data type of the vertices in the array
			IndexDataType m_indexDataType; //!< Data type of the indices in the array
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


}


