/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once
#include <ephysics/mathematics/mathematics.hpp>

namespace ephysics {
class TriangleEPA;
class TrianglesStore;
/** 
 * @brief Class EdgeEPA
 * This class represents an edge of the current polytope in the EPA algorithm.
 */
class EdgeEPA {
	private:
		/// Pointer to the triangle that contains this edge
		TriangleEPA* m_ownerTriangle;
		/// Index of the edge in the triangle (between 0 and 2).
		/// The edge with index i connect triangle vertices i and (i+1 % 3)
		int32_t m_index;
	public:
		/// Constructor
		EdgeEPA();
		/// Constructor
		EdgeEPA(TriangleEPA* ownerTriangle, int32_t index);
		/// Copy-constructor
		EdgeEPA(const EdgeEPA& _obj);
		/// Move-constructor
		EdgeEPA(EdgeEPA&& _obj);
		/// Return the pointer to the owner triangle
		TriangleEPA* getOwnerTriangle() const {
			return m_ownerTriangle;
		}
		/// Return the index of the edge in the triangle
		int32_t getIndex() const {
			return m_index;
		}
		/// Return index of the source vertex of the edge
		uint32_t getSourceVertexIndex() const;
		/// Return the index of the target vertex of the edge
		uint32_t getTargetVertexIndex() const;
		/// Execute the recursive silhouette algorithm from this edge
		bool computeSilhouette(const vec3* vertices, uint32_t index, TrianglesStore& triangleStore);
		/// Assignment operator
		EdgeEPA& operator=(const EdgeEPA& _obj) {
			m_ownerTriangle = _obj.m_ownerTriangle;
			m_index = _obj.m_index;
			return *this;
		}
		/// Move operator
		EdgeEPA& operator=(EdgeEPA&& _obj) {
			etk::swap(m_ownerTriangle, _obj.m_ownerTriangle);
			etk::swap(m_index, _obj.m_index);
			return *this;
		}
};

// Return the index of the next counter-clockwise edge of the ownver triangle
inline int32_t indexOfNextCounterClockwiseEdge(int32_t _iii) {
	return (_iii + 1) % 3;
}

// Return the index of the previous counter-clockwise edge of the ownver triangle
inline int32_t indexOfPreviousCounterClockwiseEdge(int32_t _iii) {
	return (_iii + 2) % 3;
}

}


