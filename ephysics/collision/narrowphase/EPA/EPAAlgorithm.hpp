/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#pragma once
#include <ephysics/collision/narrowphase/GJK/Simplex.hpp>
#include <ephysics/collision/shapes/CollisionShape.hpp>
#include <ephysics/collision/CollisionShapeInfo.hpp>
#include <ephysics/constraint/ContactPoint.hpp>
#include <ephysics/collision/narrowphase/NarrowPhaseAlgorithm.hpp>
#include <ephysics/mathematics/mathematics.hpp>
#include <ephysics/collision/narrowphase/EPA/TriangleEPA.hpp>
#include <ephysics/debug.hpp>
#include <etk/Set.hpp>

namespace ephysics {
	/// Maximum number of support points of the polytope
	const uint32_t MAX_SUPPORT_POINTS = 100;
	/// Maximum number of facets of the polytope
	const uint32_t MAX_FACETS = 200;
	/**
	 * @brief Class EPAAlgorithm
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
			/// Private copy-constructor
			EPAAlgorithm(const EPAAlgorithm& _algorithm);
			/// Private assignment operator
			EPAAlgorithm& operator=(const EPAAlgorithm& _algorithm);
			/// Add a triangle face in the candidate triangle heap
			void addFaceCandidate(TriangleEPA* _triangle,
			                      etk::Set<TriangleEPA*>& _heap,
			                      float _upperBoundSquarePenDepth) {
				// If the closest point of the affine hull of triangle
				// points is int32_ternal to the triangle and if the distance
				// of the closest point from the origin is at most the
				// penetration depth upper bound
				if (    _triangle->isClosestPointInternalToTriangle()
				     && _triangle->getDistSquare() <= _upperBoundSquarePenDepth) {
					// Add the triangle face to the list of candidates
					_heap.add(_triangle);
					EPHY_INFO("add in heap:");
					for (size_t iii=0; iii<_heap.size(); ++iii) {
						EPHY_INFO("    [" << iii << "] " << _heap[iii]->getDistSquare());
					}
				}
			}
			// Decide if the origin is in the tetrahedron.
			/// Return 0 if the origin is in the tetrahedron and return the number (1,2,3 or 4) of
			/// the vertex that is wrong if the origin is not in the tetrahedron
			int32_t isOriginInTetrahedron(const vec3& _p1, const vec3& _p2, const vec3& _p3, const vec3& _p4) const;
		public:
			/// Constructor
			EPAAlgorithm();
			/// Destructor
			~EPAAlgorithm();
			/// Initalize the algorithm
			void init() {
				
			}
			// Compute the penetration depth with the EPA algorithm.
			/// This method computes the penetration depth and contact points between two
			/// enlarged objects (with margin) where the original objects (without margin)
			/// int32_tersect. An initial simplex that contains origin has been computed with
			/// GJK algorithm. The EPA Algorithm will extend this simplex polytope to find
			/// the correct penetration depth
			void computePenetrationDepthAndContactPoints(const Simplex& _simplex,
			                                             CollisionShapeInfo _shape1Info,
			                                             const etk::Transform3D& _transform1,
			                                             CollisionShapeInfo _shape2Info,
			                                             const etk::Transform3D& _transform2,
			                                             vec3& _v,
			                                             NarrowPhaseCallback* _narrowPhaseCallback);
	};
}

