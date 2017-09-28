/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#include <ephysics/collision/narrowphase/EPA/EdgeEPA.hpp>
#include <ephysics/collision/narrowphase/EPA/TriangleEPA.hpp>
#include <ephysics/collision/narrowphase/EPA/TrianglesStore.hpp>

using namespace ephysics;


EdgeEPA::EdgeEPA() {
	
}

EdgeEPA::EdgeEPA(TriangleEPA* ownerTriangle, int32_t index)
		: m_ownerTriangle(ownerTriangle), m_index(index) {
	assert(index >= 0 && index < 3);
}

EdgeEPA::EdgeEPA(const EdgeEPA& edge) {
	m_ownerTriangle = edge.m_ownerTriangle;
	m_index = edge.m_index;
}

EdgeEPA::~EdgeEPA() {
	
}

uint32_t EdgeEPA::getSourceVertexIndex() const {
	return (*m_ownerTriangle)[m_index];
}

uint32_t EdgeEPA::getTargetVertexIndex() const {
	return (*m_ownerTriangle)[indexOfNextCounterClockwiseEdge(m_index)];
}

bool EdgeEPA::computeSilhouette(const vec3* vertices, uint32_t indexNewVertex,
								TrianglesStore& triangleStore) {
	// If the edge has not already been visited
	if (!m_ownerTriangle->getIsObsolete()) {
		// If the triangle of this edge is not visible from the given point
		if (!m_ownerTriangle->isVisibleFromVertex(vertices, indexNewVertex)) {
			TriangleEPA* triangle = triangleStore.newTriangle(vertices, indexNewVertex,
															  getTargetVertexIndex(),
															  getSourceVertexIndex());
			// If the triangle has been created
			if (triangle != nullptr) {
				halfLink(EdgeEPA(triangle, 1), *this);
				return true;
			}
			return false;
		} else {
			// The current triangle is visible and therefore obsolete
			m_ownerTriangle->setIsObsolete(true);
			int32_t backup = triangleStore.getNbTriangles();
			if(!m_ownerTriangle->getAdjacentEdge(indexOfNextCounterClockwiseEdge(
												this->m_index)).computeSilhouette(vertices,
																				 indexNewVertex,
																				 triangleStore)) {
				m_ownerTriangle->setIsObsolete(false);
				TriangleEPA* triangle = triangleStore.newTriangle(vertices, indexNewVertex,
																  getTargetVertexIndex(),
																  getSourceVertexIndex());
				// If the triangle has been created
				if (triangle != nullptr) {
					halfLink(EdgeEPA(triangle, 1), *this);
					return true;
				}
				return false;
			} else if (!m_ownerTriangle->getAdjacentEdge(indexOfPreviousCounterClockwiseEdge(
												  this->m_index)).computeSilhouette(vertices,
																				   indexNewVertex,
																				   triangleStore)) {
				m_ownerTriangle->setIsObsolete(false);
				triangleStore.setNbTriangles(backup);
				TriangleEPA* triangle = triangleStore.newTriangle(vertices, indexNewVertex,
																  getTargetVertexIndex(),
																  getSourceVertexIndex());
				if (triangle != NULL) {
					halfLink(EdgeEPA(triangle, 1), *this);
					return true;
				}
				return false;
			}
		}
	}
	return true;
}

