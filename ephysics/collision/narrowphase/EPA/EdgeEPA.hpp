/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once


// Libraries
#include <ephysics/mathematics/mathematics.hpp>

/// ReactPhysics3D namespace
namespace ephysics {

// Class declarations
class TriangleEPA;
class TrianglesStore;

// Class EdgeEPA
/**
 * This class represents an edge of the current polytope in the EPA algorithm.
 */
class EdgeEPA {

	private:

		// -------------------- Attributes -------------------- //

		/// Pointer to the triangle that contains this edge
		TriangleEPA* mOwnerTriangle;

		/// Index of the edge in the triangle (between 0 and 2).
		/// The edge with index i connect triangle vertices i and (i+1 % 3)
		int32_t mIndex;

	public:

		// -------------------- Methods -------------------- //

		/// Constructor
		EdgeEPA();

		/// Constructor
		EdgeEPA(TriangleEPA* ownerTriangle, int32_t index);

		/// Copy-constructor
		EdgeEPA(const EdgeEPA& edge);

		/// Destructor
		~EdgeEPA();

		/// Return the pointer to the owner triangle
		TriangleEPA* getOwnerTriangle() const;

		/// Return the index of the edge in the triangle
		int32_t getIndex() const;

		/// Return index of the source vertex of the edge
		uint32_t getSourceVertexIndex() const;

		/// Return the index of the target vertex of the edge
		uint32_t getTargetVertexIndex() const;

		/// Execute the recursive silhouette algorithm from this edge
		bool computeSilhouette(const vec3* vertices, uint32_t index, TrianglesStore& triangleStore);

		/// Assignment operator
		EdgeEPA& operator=(const EdgeEPA& edge);
};

// Return the pointer to the owner triangle
inline TriangleEPA* EdgeEPA::getOwnerTriangle() const {
	return mOwnerTriangle;
}

// Return the edge index
inline int32_t EdgeEPA::getIndex() const {
	return mIndex;
}

// Assignment operator
inline EdgeEPA& EdgeEPA::operator=(const EdgeEPA& edge) {
	mOwnerTriangle = edge.mOwnerTriangle;
	mIndex = edge.mIndex;
	return *this;
}

// Return the index of the next counter-clockwise edge of the ownver triangle
inline int32_t indexOfNextCounterClockwiseEdge(int32_t i) {
	return (i + 1) % 3;
}

// Return the index of the previous counter-clockwise edge of the ownver triangle
inline int32_t indexOfPreviousCounterClockwiseEdge(int32_t i) {
	return (i + 2) % 3;
}

}


