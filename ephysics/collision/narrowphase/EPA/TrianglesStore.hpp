/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once
#include <ephysics/collision/narrowphase/EPA/TriangleEPA.hpp>
namespace ephysics {
	const uint32_t MAX_TRIANGLES = 200;	 // Maximum number of triangles
	/**
	 * @brief This class stores several triangles of the polytope in the EPA algorithm.
	 */
	class TrianglesStore {
		private:
			TriangleEPA m_triangles[MAX_TRIANGLES]; //!< Triangles
			int32_t m_numberTriangles; //!< Number of triangles
			/// Private copy-constructor
			TrianglesStore(const TrianglesStore& triangleStore);
			/// Private assignment operator
			TrianglesStore& operator=(const TrianglesStore& triangleStore);
		public:
			/// Constructor
			TrianglesStore();
			/// Destructor
			~TrianglesStore();
			/// Clear all the storage
			void clear() {
				m_numberTriangles = 0;
			}
			/// Return the number of triangles
			int32_t getNbTriangles() const {
				return m_numberTriangles;
			}
			/// Set the number of triangles
			void setNbTriangles(int32_t _backup) {
				m_numberTriangles = _backup;
			}
			/// Return the last triangle
			TriangleEPA& last() {
				assert(m_numberTriangles > 0);
				return m_triangles[m_numberTriangles - 1];
			}
			/// Create a new triangle
			TriangleEPA* newTriangle(const vec3* _vertices, uint32_t _v0, uint32_t _v1, uint32_t _v2) {
				TriangleEPA* newTriangle = nullptr;
				// If we have not reached the maximum number of triangles
				if (m_numberTriangles != MAX_TRIANGLES) {
					newTriangle = &m_triangles[m_numberTriangles++];
					newTriangle->set(_v0, _v1, _v2);
					if (!newTriangle->computeClosestPoint(_vertices)) {
						m_numberTriangles--;
						newTriangle = nullptr;
					}
				}
				// Return the new triangle
				return newTriangle;
			}
			/// Access operator
			TriangleEPA& operator[](int32_t _id) {
				return m_triangles[_id];
			}
	};
}

