/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <complex>
#include <ephysics/configuration.h>
#include <ephysics/collision/shapes/ConvexMeshShape.h>

using namespace reactphysics3d;

// Constructor to initialize with an array of 3D vertices.
/// This method creates an int32_ternal copy of the input vertices.
/**
 * @param arrayVertices Array with the vertices of the convex mesh
 * @param nbVertices Number of vertices in the convex mesh
 * @param stride Stride between the beginning of two elements in the vertices array
 * @param margin Collision margin (in meters) around the collision shape
 */
ConvexMeshShape::ConvexMeshShape(const float* arrayVertices, uint32_t nbVertices, int32_t stride, float margin)
				: ConvexShape(CONVEX_MESH, margin), m_numberVertices(nbVertices), m_minBounds(0, 0, 0),
				  m_maxBounds(0, 0, 0), m_isEdgesInformationUsed(false) {
	assert(nbVertices > 0);
	assert(stride > 0);

	const unsigned char* vertexPointer = (const unsigned char*) arrayVertices;

	// Copy all the vertices int32_to the int32_ternal array
	for (uint32_t i=0; i<m_numberVertices; i++) {
		const float* newPoint = (const float*) vertexPointer;
		m_vertices.push_back(Vector3(newPoint[0], newPoint[1], newPoint[2]));
		vertexPointer += stride;
	}

	// Recalculate the bounds of the mesh
	recalculateBounds();
}

// Constructor to initialize with a triangle mesh
/// This method creates an int32_ternal copy of the input vertices.
/**
 * @param triangleVertexArray Array with the vertices and indices of the vertices and triangles of the mesh
 * @param isEdgesInformationUsed True if you want to use edges information for collision detection (faster but requires more memory)
 * @param margin Collision margin (in meters) around the collision shape
 */
ConvexMeshShape::ConvexMeshShape(TriangleVertexArray* triangleVertexArray, bool isEdgesInformationUsed, float margin)
				: ConvexShape(CONVEX_MESH, margin), m_minBounds(0, 0, 0),
				  m_maxBounds(0, 0, 0), m_isEdgesInformationUsed(isEdgesInformationUsed) {

	TriangleVertexArray::VertexDataType vertexType = triangleVertexArray->getVertexDataType();
	TriangleVertexArray::IndexDataType indexType = triangleVertexArray->getIndexDataType();
	unsigned char* verticesStart = triangleVertexArray->getVerticesStart();
	unsigned char* indicesStart = triangleVertexArray->getIndicesStart();
	int32_t vertexStride = triangleVertexArray->getVerticesStride();
	int32_t indexStride = triangleVertexArray->getIndicesStride();

	// For each vertex of the mesh
	for (uint32_t v = 0; v < triangleVertexArray->getNbVertices(); v++) {

		// Get the vertices components of the triangle
		if (vertexType == TriangleVertexArray::VERTEX_FLOAT_TYPE) {
			const float* vertices = (float*)(verticesStart + v * vertexStride);

			Vector3 vertex(vertices[0], vertices[1], vertices[2] );
			vertex = vertex * mScaling;
			m_vertices.push_back(vertex);
		}
		else if (vertexType == TriangleVertexArray::VERTEX_DOUBLE_TYPE) {
			const double* vertices = (double*)(verticesStart + v * vertexStride);

			Vector3 vertex(vertices[0], vertices[1], vertices[2] );
			vertex = vertex * mScaling;
			m_vertices.push_back(vertex);
		}
	}

	// If we need to use the edges information of the mesh
	if (m_isEdgesInformationUsed) {

		// For each triangle of the mesh
		for (uint32_t triangleIndex=0; triangleIndex<triangleVertexArray->getNbTriangles(); triangleIndex++) {

			void* vertexIndexPointer = (indicesStart + triangleIndex * 3 * indexStride);

			uint32_t vertexIndex[3] = {0, 0, 0};

			// For each vertex of the triangle
			for (int32_t k=0; k < 3; k++) {

				// Get the index of the current vertex in the triangle
				if (indexType == TriangleVertexArray::INDEX_INTEGER_TYPE) {
					vertexIndex[k] = ((uint32_t*)vertexIndexPointer)[k];
				}
				else if (indexType == TriangleVertexArray::INDEX_SHORT_TYPE) {
					vertexIndex[k] = ((unsigned short*)vertexIndexPointer)[k];
				}
				else {
					assert(false);
				}
			}

			// Add information about the edges
			addEdge(vertexIndex[0], vertexIndex[1]);
			addEdge(vertexIndex[0], vertexIndex[2]);
			addEdge(vertexIndex[1], vertexIndex[2]);
		}
	}

	m_numberVertices = m_vertices.size();
	recalculateBounds();
}

// Constructor.
/// If you use this constructor, you will need to set the vertices manually one by one using
/// the addVertex() method.
ConvexMeshShape::ConvexMeshShape(float margin)
				: ConvexShape(CONVEX_MESH, margin), m_numberVertices(0), m_minBounds(0, 0, 0),
				  m_maxBounds(0, 0, 0), m_isEdgesInformationUsed(false) {

}

// Destructor
ConvexMeshShape::~ConvexMeshShape() {

}

// Return a local support point in a given direction without the object margin.
/// If the edges information is not used for collision detection, this method will go through
/// the whole vertices list and pick up the vertex with the largest dot product in the support
/// direction. This is an O(n) process with "n" being the number of vertices in the mesh.
/// However, if the edges information is used, we can cache the previous support vertex and use
/// it as a start in a hill-climbing (local search) process to find the new support vertex which
/// will be in most of the cases very close to the previous one. Using hill-climbing, this method
/// runs in almost constant time.
Vector3 ConvexMeshShape::getLocalSupportPointWithoutMargin(const Vector3& direction,
														   void** cachedCollisionData) const {

	assert(m_numberVertices == m_vertices.size());
	assert(cachedCollisionData != NULL);

	// Allocate memory for the cached collision data if not allocated yet
	if ((*cachedCollisionData) == NULL) {
		*cachedCollisionData = (int32_t*) malloc(sizeof(int32_t));
		*((int32_t*)(*cachedCollisionData)) = 0;
	}

	// If the edges information is used to speed up the collision detection
	if (m_isEdgesInformationUsed) {

		assert(m_edgesAdjacencyList.size() == m_numberVertices);

		uint32_t maxVertex = *((int32_t*)(*cachedCollisionData));
		float maxDotProduct = direction.dot(m_vertices[maxVertex]);
		bool isOptimal;

		// Perform hill-climbing (local search)
		do {
			isOptimal = true;

			assert(m_edgesAdjacencyList.at(maxVertex).size() > 0);

			// For all neighbors of the current vertex
			std::set<uint32_t>::const_iterator it;
			std::set<uint32_t>::const_iterator itBegin = m_edgesAdjacencyList.at(maxVertex).begin();
			std::set<uint32_t>::const_iterator itEnd = m_edgesAdjacencyList.at(maxVertex).end();
			for (it = itBegin; it != itEnd; ++it) {

				// Compute the dot product
				float dotProduct = direction.dot(m_vertices[*it]);

				// If the current vertex is a better vertex (larger dot product)
				if (dotProduct > maxDotProduct) {
					maxVertex = *it;
					maxDotProduct = dotProduct;
					isOptimal = false;
				}
			}

		} while(!isOptimal);

		// Cache the support vertex
		*((int32_t*)(*cachedCollisionData)) = maxVertex;

		// Return the support vertex
		return m_vertices[maxVertex] * mScaling;
	}
	else {  // If the edges information is not used

		double maxDotProduct = DECIMAL_SMALLEST;
		uint32_t indexMaxDotProduct = 0;

		// For each vertex of the mesh
		for (uint32_t i=0; i<m_numberVertices; i++) {

			// Compute the dot product of the current vertex
			double dotProduct = direction.dot(m_vertices[i]);

			// If the current dot product is larger than the maximum one
			if (dotProduct > maxDotProduct) {
				indexMaxDotProduct = i;
				maxDotProduct = dotProduct;
			}
		}

		assert(maxDotProduct >= float(0.0));

		// Return the vertex with the largest dot product in the support direction
		return m_vertices[indexMaxDotProduct] * mScaling;
	}
}

// Recompute the bounds of the mesh
void ConvexMeshShape::recalculateBounds() {

	// TODO : Only works if the local origin is inside the mesh
	//		=> Make it more robust (init with first vertex of mesh instead)

	m_minBounds.setToZero();
	m_maxBounds.setToZero();

	// For each vertex of the mesh
	for (uint32_t i=0; i<m_numberVertices; i++) {

		if (m_vertices[i].x > m_maxBounds.x) m_maxBounds.x = m_vertices[i].x;
		if (m_vertices[i].x < m_minBounds.x) m_minBounds.x = m_vertices[i].x;

		if (m_vertices[i].y > m_maxBounds.y) m_maxBounds.y = m_vertices[i].y;
		if (m_vertices[i].y < m_minBounds.y) m_minBounds.y = m_vertices[i].y;

		if (m_vertices[i].z > m_maxBounds.z) m_maxBounds.z = m_vertices[i].z;
		if (m_vertices[i].z < m_minBounds.z) m_minBounds.z = m_vertices[i].z;
	}

	// Apply the local scaling factor
	m_maxBounds = m_maxBounds * mScaling;
	m_minBounds = m_minBounds * mScaling;

	// Add the object margin to the bounds
	m_maxBounds += Vector3(mMargin, mMargin, mMargin);
	m_minBounds -= Vector3(mMargin, mMargin, mMargin);
}

// Raycast method with feedback information
bool ConvexMeshShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const {
	return proxyShape->mBody->mWorld.m_collisionDetection.mNarrowPhaseGJKAlgorithm.raycast(
									 ray, proxyShape, raycastInfo);
}
