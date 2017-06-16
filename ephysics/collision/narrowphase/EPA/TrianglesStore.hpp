/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

#include <ephysics/collision/narrowphase/EPA/TriangleEPA.hpp>


// Libraries
#include <cassert>

/// ReactPhysics3D namespace
namespace ephysics {

// Constants
const uint32_t MAX_TRIANGLES = 200;	 // Maximum number of triangles

// Class TriangleStore
/**
 * This class stores several triangles of the polytope in the EPA algorithm.
 */
class TrianglesStore {

	private:

		// -------------------- Attributes -------------------- //

		/// Triangles
		TriangleEPA mTriangles[MAX_TRIANGLES];

		/// Number of triangles
		int32_t m_numberTriangles;

		// -------------------- Methods -------------------- //

		/// Private copy-constructor
		TrianglesStore(const TrianglesStore& triangleStore);

		/// Private assignment operator
		TrianglesStore& operator=(const TrianglesStore& triangleStore);
		
	public:

		// -------------------- Methods -------------------- //

		/// Constructor
		TrianglesStore();

		/// Destructor
		~TrianglesStore();

		/// Clear all the storage
		void clear();

		/// Return the number of triangles
		int32_t getNbTriangles() const;

		/// Set the number of triangles
		void setNbTriangles(int32_t backup);

		/// Return the last triangle
		TriangleEPA& last();

		/// Create a new triangle
		TriangleEPA* newTriangle(const vec3* vertices, uint32_t v0, uint32_t v1, uint32_t v2);

		/// Access operator
		TriangleEPA& operator[](int32_t i);
};

// Clear all the storage
inline void TrianglesStore::clear() {
	m_numberTriangles = 0;
}

// Return the number of triangles
inline int32_t TrianglesStore::getNbTriangles() const {
	return m_numberTriangles;
}


inline void TrianglesStore::setNbTriangles(int32_t backup) {
	m_numberTriangles = backup;
}

// Return the last triangle
inline TriangleEPA& TrianglesStore::last() {
	assert(m_numberTriangles > 0);
	return mTriangles[m_numberTriangles - 1];
}

// Create a new triangle
inline TriangleEPA* TrianglesStore::newTriangle(const vec3* vertices,
												uint32_t v0,uint32_t v1, uint32_t v2) {
	TriangleEPA* newTriangle = NULL;

	// If we have not reached the maximum number of triangles
	if (m_numberTriangles != MAX_TRIANGLES) {
		newTriangle = &mTriangles[m_numberTriangles++];
		new (newTriangle) TriangleEPA(v0, v1, v2);
		if (!newTriangle->computeClosestPoint(vertices)) {
			m_numberTriangles--;
			newTriangle = NULL;
		}
	}

	// Return the new triangle
	return newTriangle;
}

// Access operator
inline TriangleEPA& TrianglesStore::operator[](int32_t i) {
	return mTriangles[i];
}

}

