/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

#include <ephysics/collision/narrowphase/EPA/TriangleEPA.hpp>
#include <ephysics/collision/narrowphase/EPA/EdgeEPA.hpp>
#include <ephysics/collision/narrowphase/EPA/TrianglesStore.hpp>

using namespace ephysics;

TriangleEPA::TriangleEPA() {
	
}

TriangleEPA::TriangleEPA(uint32_t _indexVertex1, uint32_t _indexVertex2, uint32_t _indexVertex3):
  m_isObsolete(false) {
	m_indicesVertices[0] = _indexVertex1;
	m_indicesVertices[1] = _indexVertex2;
	m_indicesVertices[2] = _indexVertex3;
}

void TriangleEPA::set(uint32_t _indexVertex1, uint32_t _indexVertex2, uint32_t _indexVertex3) {
	m_isObsolete = false;
	m_indicesVertices[0] = _indexVertex1;
	m_indicesVertices[1] = _indexVertex2;
	m_indicesVertices[2] = _indexVertex3;
}

TriangleEPA::~TriangleEPA() {
	
}

bool TriangleEPA::computeClosestPoint(const vec3* _vertices) {
	const vec3& p0 = _vertices[m_indicesVertices[0]];
	vec3 v1 = _vertices[m_indicesVertices[1]] - p0;
	vec3 v2 = _vertices[m_indicesVertices[2]] - p0;
	float v1Dotv1 = v1.dot(v1);
	float v1Dotv2 = v1.dot(v2);
	float v2Dotv2 = v2.dot(v2);
	float p0Dotv1 = p0.dot(v1);
	float p0Dotv2 = p0.dot(v2);
	// Compute determinant
	m_determinant = v1Dotv1 * v2Dotv2 - v1Dotv2 * v1Dotv2;
	// Compute lambda values
	m_lambda1 = p0Dotv2 * v1Dotv2 - p0Dotv1 * v2Dotv2;
	m_lambda2 = p0Dotv1 * v1Dotv2 - p0Dotv2 * v1Dotv1;
	// If the determinant is positive
	if (m_determinant > 0.0) {
		// Compute the closest point v
		m_closestPoint = p0 + 1.0f / m_determinant * (m_lambda1 * v1 + m_lambda2 * v2);
		// Compute the square distance of closest point to the origin
		m_distSquare = m_closestPoint.dot(m_closestPoint);
		return true;
	}
	return false;
}

bool ephysics::link(const EdgeEPA& _edge0, const EdgeEPA& _edge1) {
	if (    _edge0.getSourceVertexIndex() == _edge1.getTargetVertexIndex()
	     && _edge0.getTargetVertexIndex() == _edge1.getSourceVertexIndex() ) {
		_edge0.getOwnerTriangle()->m_adjacentEdges[_edge0.getIndex()] = _edge1;
		_edge1.getOwnerTriangle()->m_adjacentEdges[_edge1.getIndex()] = _edge0;
		return true;
	}
	return false;
}

void ephysics::halfLink(const EdgeEPA& _edge0, const EdgeEPA& _edge1) {
	assert(    _edge0.getSourceVertexIndex() == _edge1.getTargetVertexIndex()
	        && _edge0.getTargetVertexIndex() == _edge1.getSourceVertexIndex());
	_edge0.getOwnerTriangle()->m_adjacentEdges[_edge0.getIndex()] = _edge1;
}


bool TriangleEPA::computeSilhouette(const vec3* _vertices, uint32_t _indexNewVertex,
									TrianglesStore& _triangleStore) {
	uint32_t first = _triangleStore.getNbTriangles();
	// Mark the current triangle as obsolete because it
	setIsObsolete(true);
	// Execute recursively the silhouette algorithm for the adjacent edges of neighboring
	// triangles of the current triangle
	bool result = m_adjacentEdges[0].computeSilhouette(_vertices, _indexNewVertex, _triangleStore) &&
				  m_adjacentEdges[1].computeSilhouette(_vertices, _indexNewVertex, _triangleStore) &&
				  m_adjacentEdges[2].computeSilhouette(_vertices, _indexNewVertex, _triangleStore);
	if (result) {
		int32_t i,j;
		// For each triangle face that contains the new vertex and an edge of the silhouette
		for (i=first, j=_triangleStore.getNbTriangles()-1;
			 i != _triangleStore.getNbTriangles(); j = i++) {
			TriangleEPA* triangle = &_triangleStore[i];
			halfLink(triangle->getAdjacentEdge(1), EdgeEPA(triangle, 1));
			if (!link(EdgeEPA(triangle, 0), EdgeEPA(&_triangleStore[j], 2))) {
				return false;
			}
		}
	}
	return result;
}
