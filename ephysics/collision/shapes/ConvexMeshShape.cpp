/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#include <ephysics/configuration.hpp>
#include <ephysics/collision/shapes/ConvexMeshShape.hpp>

using namespace ephysics;

ConvexMeshShape::ConvexMeshShape(const float* _arrayVertices,
                                 uint32_t _nbVertices,
                                 int32_t _stride,
                                 float _margin):
  ConvexShape(CONVEX_MESH, _margin),
  m_numberVertices(_nbVertices),
  m_minBounds(0, 0, 0),
  m_maxBounds(0, 0, 0),
  m_isEdgesInformationUsed(false) {
	assert(_nbVertices > 0);
	assert(_stride > 0);
	const unsigned char* vertexPointer = (const unsigned char*) _arrayVertices;
	// Copy all the vertices int32_to the int32_ternal array
	for (uint32_t iii=0; iii<m_numberVertices; iii++) {
		const float* newPoint = (const float*) vertexPointer;
		m_vertices.pushBack(vec3(newPoint[0], newPoint[1], newPoint[2]));
		vertexPointer += _stride;
	}
	// Recalculate the bounds of the mesh
	recalculateBounds();
}

ConvexMeshShape::ConvexMeshShape(TriangleVertexArray* _triangleVertexArray,
                                 bool _isEdgesInformationUsed,
                                 float _margin):
  ConvexShape(CONVEX_MESH, _margin),
  m_minBounds(0, 0, 0),
  m_maxBounds(0, 0, 0),
  m_isEdgesInformationUsed(_isEdgesInformationUsed) {
	// For each vertex of the mesh
	for (auto &it: _triangleVertexArray->getVertices()) {
		m_vertices.pushBack(it*m_scaling);
	}
	// If we need to use the edges information of the mesh
	if (m_isEdgesInformationUsed) {
		// For each triangle of the mesh
		for (size_t iii=0; iii<_triangleVertexArray->getNbTriangles(); iii++) {
			uint32_t vertexIndex[3] = {0, 0, 0};
			vertexIndex[0] = _triangleVertexArray->getIndices()[iii*3];
			vertexIndex[1] = _triangleVertexArray->getIndices()[iii*3+1];
			vertexIndex[2] = _triangleVertexArray->getIndices()[iii*3+2];
			// Add information about the edges
			addEdge(vertexIndex[0], vertexIndex[1]);
			addEdge(vertexIndex[0], vertexIndex[2]);
			addEdge(vertexIndex[1], vertexIndex[2]);
		}
	}
	m_numberVertices = m_vertices.size();
	recalculateBounds();
}

ConvexMeshShape::ConvexMeshShape(float _margin):
  ConvexShape(CONVEX_MESH, _margin),
  m_numberVertices(0),
  m_minBounds(0, 0, 0),
  m_maxBounds(0, 0, 0),
  m_isEdgesInformationUsed(false) {
	
}

vec3 ConvexMeshShape::getLocalSupportPointWithoutMargin(const vec3& _direction,
                                                        void** _cachedCollisionData) const {
	assert(m_numberVertices == m_vertices.size());
	assert(_cachedCollisionData != null);
	// Allocate memory for the cached collision data if not allocated yet
	if ((*_cachedCollisionData) == null) {
		*_cachedCollisionData = (int32_t*) malloc(sizeof(int32_t));
		*((int32_t*)(*_cachedCollisionData)) = 0;
	}
	// If the edges information is used to speed up the collision detection
	if (m_isEdgesInformationUsed) {
		assert(m_edgesAdjacencyList.size() == m_numberVertices);
		uint32_t maxVertex = *((int32_t*)(*_cachedCollisionData));
		float maxDotProduct = _direction.dot(m_vertices[maxVertex]);
		bool isOptimal;
		// Perform hill-climbing (local search)
		do {
			isOptimal = true;
			assert(m_edgesAdjacencyList[maxVertex].size() > 0);
			// For all neighbors of the current vertex
			etk::Set<uint32_t>::Iterator it;
			etk::Set<uint32_t>::Iterator itBegin = m_edgesAdjacencyList[maxVertex].begin();
			etk::Set<uint32_t>::Iterator itEnd = m_edgesAdjacencyList[maxVertex].end();
			for (it = itBegin; it != itEnd; ++it) {
				// Compute the dot product
				float dotProduct = _direction.dot(m_vertices[*it]);
				// If the current vertex is a better vertex (larger dot product)
				if (dotProduct > maxDotProduct) {
					maxVertex = *it;
					maxDotProduct = dotProduct;
					isOptimal = false;
				}
			}
		} while(!isOptimal);
		// Cache the support vertex
		*((int32_t*)(*_cachedCollisionData)) = maxVertex;
		// Return the support vertex
		return m_vertices[maxVertex] * m_scaling;
	} else {
		// If the edges information is not used
		double maxDotProduct = FLT_MIN;
		uint32_t indexMaxDotProduct = 0;
		// For each vertex of the mesh
		for (uint32_t i=0; i<m_numberVertices; i++) {
			// Compute the dot product of the current vertex
			double dotProduct = _direction.dot(m_vertices[i]);
			// If the current dot product is larger than the maximum one
			if (dotProduct > maxDotProduct) {
				indexMaxDotProduct = i;
				maxDotProduct = dotProduct;
			}
		}
		assert(maxDotProduct >= 0.0f);
		// Return the vertex with the largest dot product in the support direction
		return m_vertices[indexMaxDotProduct] * m_scaling;
	}
}

// Recompute the bounds of the mesh
void ConvexMeshShape::recalculateBounds() {
	// TODO : Only works if the local origin is inside the mesh
	//		=> Make it more robust (init with first vertex of mesh instead)
	m_minBounds.setZero();
	m_maxBounds.setZero();
	// For each vertex of the mesh
	for (uint32_t i=0; i<m_numberVertices; i++) {
		if (m_vertices[i].x() > m_maxBounds.x()) {
			m_maxBounds.setX(m_vertices[i].x());
		}
		if (m_vertices[i].x() < m_minBounds.x()) {
			m_minBounds.setX(m_vertices[i].x());
		}
		if (m_vertices[i].y() > m_maxBounds.y()) {
			m_maxBounds.setY(m_vertices[i].y());
		}
		if (m_vertices[i].y() < m_minBounds.y()) {
			m_minBounds.setY(m_vertices[i].y());
		}
		if (m_vertices[i].z() > m_maxBounds.z()) {
			m_maxBounds.setZ(m_vertices[i].z());
		}
		if (m_vertices[i].z() < m_minBounds.z()) {
			m_minBounds.setZ(m_vertices[i].z());
		}
	}
	// Apply the local scaling factor
	m_maxBounds = m_maxBounds * m_scaling;
	m_minBounds = m_minBounds * m_scaling;
	// Add the object margin to the bounds
	m_maxBounds += vec3(m_margin, m_margin, m_margin);
	m_minBounds -= vec3(m_margin, m_margin, m_margin);
}

bool ConvexMeshShape::raycast(const Ray& _ray, RaycastInfo& _raycastInfo, ProxyShape* _proxyShape) const {
	return _proxyShape->m_body->m_world.m_collisionDetection.m_narrowPhaseGJKAlgorithm.raycast(_ray, _proxyShape, _raycastInfo);
}

void ConvexMeshShape::setLocalScaling(const vec3& _scaling) {
	ConvexShape::setLocalScaling(_scaling);
	recalculateBounds();
}

size_t ConvexMeshShape::getSizeInBytes() const {
	return sizeof(ConvexMeshShape);
}

void ConvexMeshShape::getLocalBounds(vec3& _min, vec3& _max) const {
	_min = m_minBounds;
	_max = m_maxBounds;
}

void ConvexMeshShape::computeLocalInertiaTensor(etk::Matrix3x3& _tensor, float _mass) const {
	float factor = (1.0f / float(3.0)) * _mass;
	vec3 realExtent = 0.5f * (m_maxBounds - m_minBounds);
	assert(realExtent.x() > 0 && realExtent.y() > 0 && realExtent.z() > 0);
	float xSquare = realExtent.x() * realExtent.x();
	float ySquare = realExtent.y() * realExtent.y();
	float zSquare = realExtent.z() * realExtent.z();
	_tensor.setValue(factor * (ySquare + zSquare), 0.0, 0.0,
	                 0.0, factor * (xSquare + zSquare), 0.0,
	                 0.0, 0.0, factor * (xSquare + ySquare));
}

void ConvexMeshShape::addVertex(const vec3& _vertex) {
	// Add the vertex in to vertices array
	m_vertices.pushBack(_vertex);
	m_numberVertices++;
	// Update the bounds of the mesh
	if (_vertex.x() * m_scaling.x() > m_maxBounds.x()) {
		m_maxBounds.setX(_vertex.x() * m_scaling.x());
	}
	if (_vertex.x() * m_scaling.x() < m_minBounds.x()) {
		m_minBounds.setX(_vertex.x() * m_scaling.x());
	}
	if (_vertex.y() * m_scaling.y() > m_maxBounds.y()) {
		m_maxBounds.setY(_vertex.y() * m_scaling.y());
	}
	if (_vertex.y() * m_scaling.y() < m_minBounds.y()) {
		m_minBounds.setY(_vertex.y() * m_scaling.y());
	}
	if (_vertex.z() * m_scaling.z() > m_maxBounds.z()) {
		m_maxBounds.setZ(_vertex.z() * m_scaling.z());
	}
	if (_vertex.z() * m_scaling.z() < m_minBounds.z()) {
		m_minBounds.setZ(_vertex.z() * m_scaling.z());
	}
}

void ConvexMeshShape::addEdge(uint32_t _v1, uint32_t _v2) {
	// If the entry for vertex v1 does not exist in the adjacency list
	if (m_edgesAdjacencyList.count(_v1) == 0) {
		m_edgesAdjacencyList.add(_v1, etk::Set<uint32_t>());
	}
	// If the entry for vertex v2 does not exist in the adjacency list
	if (m_edgesAdjacencyList.count(_v2) == 0) {
		m_edgesAdjacencyList.add(_v2, etk::Set<uint32_t>());
	}
	// Add the edge in the adjacency list
	m_edgesAdjacencyList[_v1].add(_v2);
	m_edgesAdjacencyList[_v2].add(_v1);
}

bool ConvexMeshShape::isEdgesInformationUsed() const {
	return m_isEdgesInformationUsed;
}

void ConvexMeshShape::setIsEdgesInformationUsed(bool _isEdgesUsed) {
	m_isEdgesInformationUsed = _isEdgesUsed;
}

bool ConvexMeshShape::testPointInside(const vec3& _localPoint,
                                      ProxyShape* _proxyShape) const {
	// Use the GJK algorithm to test if the point is inside the convex mesh
	return _proxyShape->m_body->m_world.m_collisionDetection.m_narrowPhaseGJKAlgorithm.testPointInside(_localPoint, _proxyShape);
}