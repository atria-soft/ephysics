/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/collision/narrowphase/GJK/Simplex.hpp>
#include <ephysics/collision/shapes/CollisionShape.hpp>
#include <ephysics/collision/CollisionShapeInfo.hpp>
#include <ephysics/constraint/ContactPoint.hpp>
#include <ephysics/collision/narrowphase/NarrowPhaseAlgorithm.hpp>
#include <ephysics/mathematics/mathematics.hpp>
#include <ephysics/collision/narrowphase/EPA/TriangleEPA.hpp>
#include <ephysics/memory/MemoryAllocator.hpp>
#include <algorithm>

/// ReactPhysics3D namespace
namespace ephysics {

// ---------- Constants ---------- //

/// Maximum number of support points of the polytope
const uint32_t MAX_SUPPORT_POINTS = 100;

/// Maximum number of facets of the polytope
const uint32_t MAX_FACETS = 200;


// Class TriangleComparison
/**
 * This class allows the comparison of two triangles in the heap
 * The comparison between two triangles is made using their square distance to the closest
 * point to the origin. The goal is that in the heap, the first triangle is the one with the
 * smallest square distance.
 */
class TriangleComparison {

	public:

		/// Comparison operator
		bool operator()(const TriangleEPA* face1, const TriangleEPA* face2) {
			return (face1->getDistSquare() > face2->getDistSquare());
		}
};


// Class EPAAlgorithm
/**
 * This class is the implementation of the Expanding Polytope Algorithm (EPA).
 * The EPA algorithm computes the penetration depth and contact points between
 * two enlarged objects (with margin) where the original objects (without margin)
 * int32_tersect. The penetration depth of a pair of int32_tersecting objects A and B is
 * the length of a point on the boundary of the Minkowski sum (A-B) closest to the
 * origin. The goal of the EPA algorithm is to start with an initial simplex polytope
 * that contains the origin and expend it in order to find the point on the boundary
 * of (A-B) that is closest to the origin. An initial simplex that contains origin
 * has been computed wit GJK algorithm. The EPA Algorithm will extend this simplex
 * polytope to find the correct penetration depth. The implementation of the EPA
 * algorithm is based on the book "Collision Detection in 3D Environments".
 */
class EPAAlgorithm {

	private:

		// -------------------- Attributes -------------------- //

		/// Reference to the memory allocator
		MemoryAllocator* m_memoryAllocator;

		/// Triangle comparison operator
		TriangleComparison mTriangleComparison;
		
		// -------------------- Methods -------------------- //

		/// Private copy-constructor
		EPAAlgorithm(const EPAAlgorithm& algorithm);

		/// Private assignment operator
		EPAAlgorithm& operator=(const EPAAlgorithm& algorithm);

		/// Add a triangle face in the candidate triangle heap
		void addFaceCandidate(TriangleEPA* triangle, TriangleEPA** heap, uint32_t& nbTriangles,
							  float upperBoundSquarePenDepth);

		/// Decide if the origin is in the tetrahedron.
		int32_t isOriginInTetrahedron(const vec3& p1, const vec3& p2,
								  const vec3& p3, const vec3& p4) const;

	public:

		// -------------------- Methods -------------------- //

		/// Constructor
		EPAAlgorithm();

		/// Destructor
		~EPAAlgorithm();

		/// Initalize the algorithm
		void init(MemoryAllocator* memoryAllocator);

		/// Compute the penetration depth with EPA algorithm.
		void computePenetrationDepthAndContactPoints(const Simplex& simplex,
													 CollisionShapeInfo shape1Info,
													 const etk::Transform3D& transform1,
													 CollisionShapeInfo shape2Info,
													 const etk::Transform3D& transform2,
													 vec3& v,
													NarrowPhaseCallback* narrowPhaseCallback);
};

// Add a triangle face in the candidate triangle heap in the EPA algorithm
inline void EPAAlgorithm::addFaceCandidate(TriangleEPA* triangle, TriangleEPA** heap,
										   uint32_t& nbTriangles, float upperBoundSquarePenDepth) {
	
	// If the closest point of the affine hull of triangle
	// points is int32_ternal to the triangle and if the distance
	// of the closest point from the origin is at most the
	// penetration depth upper bound
	if (triangle->isClosestPointInternalToTriangle() &&
		triangle->getDistSquare() <= upperBoundSquarePenDepth) {

		// Add the triangle face to the list of candidates
		heap[nbTriangles] = triangle;
		nbTriangles++;
		std::push_heap(&heap[0], &heap[nbTriangles], mTriangleComparison);
	}
}

// Initalize the algorithm
inline void EPAAlgorithm::init(MemoryAllocator* memoryAllocator) {
	m_memoryAllocator = memoryAllocator;
}

}

