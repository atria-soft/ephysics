/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#include <ephysics/collision/narrowphase/EPA/EdgeEPA.hpp>
#include <ephysics/collision/narrowphase/EPA/TriangleEPA.hpp>
#include <ephysics/collision/narrowphase/EPA/TrianglesStore.hpp>
#include <etk/types.hpp>

using namespace ephysics;


EdgeEPA::EdgeEPA() {
	
}

EdgeEPA::EdgeEPA(TriangleEPA* ownerTriangle, int32_t index):
  m_ownerTriangle(ownerTriangle),
  m_index(index) {
	assert(index >= 0 && index < 3);
}

EdgeEPA::EdgeEPA(const EdgeEPA& _obj):
  m_ownerTriangle(_obj.m_ownerTriangle),
  m_index(_obj.m_index) {
	
}

EdgeEPA::EdgeEPA(EdgeEPA&& _obj):
  m_ownerTriangle(null) {
	etk::swap(m_ownerTriangle, _obj.m_ownerTriangle);
	etk::swap(m_index, _obj.m_index);
}

uint32_t EdgeEPA::getSourceVertexIndex() const {
	return (*m_ownerTriangle)[m_index];
}

uint32_t EdgeEPA::getTargetVertexIndex() const {
	return (*m_ownerTriangle)[indexOfNextCounterClockwiseEdge(m_index)];
}

bool EdgeEPA::computeSilhouette(const vec3* _vertices, uint32_t _indexNewVertex,
								TrianglesStore& _triangleStore) {
	// If the edge has not already been visited
	if (!m_ownerTriangle->getIsObsolete()) {
		// If the triangle of this edge is not visible from the given point
		if (!m_ownerTriangle->isVisibleFromVertex(_vertices, _indexNewVertex)) {
			TriangleEPA* triangle = _triangleStore.newTriangle(_vertices, _indexNewVertex,
															  getTargetVertexIndex(),
															  getSourceVertexIndex());
			// If the triangle has been created
			if (triangle != null) {
				halfLink(EdgeEPA(triangle, 1), *this);
				return true;
			}
			return false;
		} else {
			// The current triangle is visible and therefore obsolete
			m_ownerTriangle->setIsObsolete(true);
			int32_t backup = _triangleStore.getNbTriangles();
			if(!m_ownerTriangle->getAdjacentEdge(indexOfNextCounterClockwiseEdge(this->m_index)).computeSilhouette(_vertices,
				                                                                                                   _indexNewVertex,
				                                                                                                   _triangleStore)) {
				m_ownerTriangle->setIsObsolete(false);
				TriangleEPA* triangle = _triangleStore.newTriangle(_vertices, _indexNewVertex,
																  getTargetVertexIndex(),
																  getSourceVertexIndex());
				// If the triangle has been created
				if (triangle != null) {
					halfLink(EdgeEPA(triangle, 1), *this);
					return true;
				}
				return false;
			} else if (!m_ownerTriangle->getAdjacentEdge(indexOfPreviousCounterClockwiseEdge(this->m_index)).computeSilhouette(_vertices,
				                                                                                                               _indexNewVertex,
				                                                                                                               _triangleStore)) {
				m_ownerTriangle->setIsObsolete(false);
				_triangleStore.resize(backup);
				TriangleEPA* triangle = _triangleStore.newTriangle(_vertices, _indexNewVertex,
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

