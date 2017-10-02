/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
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
			etk::Vector<TriangleEPA> m_triangles; //!< Triangles
			/// Private copy-constructor
			TrianglesStore(const TrianglesStore& triangleStore) = delete;
			/// Private assignment operator
			TrianglesStore& operator=(const TrianglesStore& triangleStore) = delete;
		public:
			/// Constructor
			TrianglesStore() = default;
			/// Clear all the storage
			void clear() {
				m_triangles.clear();
			}
			/// Return the number of triangles
			size_t getNbTriangles() const {
				return m_triangles.size();
			}
			/// Set the number of triangles
			void resize(int32_t _backup) {
				m_triangles.resize(_backup);
			}
			/// Return the last triangle
			TriangleEPA& last() {
				return m_triangles.back();
			}
			/// Create a new triangle
			TriangleEPA* newTriangle(const vec3* _vertices, uint32_t _v0, uint32_t _v1, uint32_t _v2) {
				// If we have not reached the maximum number of triangles
				if (m_triangles.size() < MAX_TRIANGLES) {
					TriangleEPA tmp(_v0, _v1, _v2);
					if (!tmp.computeClosestPoint(_vertices)) {
						return nullptr;
					}
					m_triangles.pushBack(etk::move(tmp));
					return &m_triangles.back();
				}
				// We are at the limit (internal)
				return nullptr;
			}
			/// Access operator
			TriangleEPA& operator[](int32_t _id) {
				return m_triangles[_id];
			}
	};
}

