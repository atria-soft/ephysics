/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

#include <ephysics/collision/narrowphase/NarrowPhaseAlgorithm.hpp>
#include <ephysics/constraint/ContactPoint.hpp>
#include <ephysics/collision/shapes/ConvexShape.hpp>
#include <ephysics/collision/narrowphase/EPA/EPAAlgorithm.hpp>

namespace ephysics {
	const float REL_ERROR = float(1.0e-3);
	const float REL_ERROR_SQUARE = REL_ERROR * REL_ERROR;
	const int32_t MAX_ITERATIONS_GJK_RAYCAST = 32;
	/**
	 * @brief This class implements a narrow-phase collision detection algorithm. This
	 * algorithm uses the ISA-GJK algorithm and the EPA algorithm. This
	 * implementation is based on the implementation discussed in the book
	 * "Collision Detection in Interactive 3D Environments" by Gino van den Bergen.
	 * This method implements the Hybrid Technique for calculating the
	 * penetration depth. The two objects are enlarged with a small margin. If
	 * the object int32_tersects in their margins, the penetration depth is quickly
	 * computed using the GJK algorithm on the original objects (without margin).
	 * If the original objects (without margin) int32_tersect, we run again the GJK
	 * algorithm on the enlarged objects (with margin) to compute simplex
	 * polytope that contains the origin and give it to the EPA (Expanding
	 * Polytope Algorithm) to compute the correct penetration depth between the
	 * enlarged objects.
	 */
	class GJKAlgorithm : public NarrowPhaseAlgorithm {
		private :
			EPAAlgorithm m_algoEPA; //!< EPA Algorithm
			/// Private copy-constructor
			GJKAlgorithm(const GJKAlgorithm& algorithm);
			/// Private assignment operator
			GJKAlgorithm& operator=(const GJKAlgorithm& algorithm);
			/// This method runs the GJK algorithm on the two enlarged objects (with margin)
			/// to compute a simplex polytope that contains the origin. The two objects are
			/// assumed to int32_tersect in the original objects (without margin). Therefore such
			/// a polytope must exist. Then, we give that polytope to the EPA algorithm to
			/// compute the correct penetration depth and contact points of the enlarged objects.
			void computePenetrationDepthForEnlargedObjects(const CollisionShapeInfo& shape1Info,
			                                               const etk::Transform3D& transform1,
			                                               const CollisionShapeInfo& shape2Info,
			                                               const etk::Transform3D& transform2,
			                                               NarrowPhaseCallback* narrowPhaseCallback,
			                                               vec3& v);
		public :
			/// Constructor
			GJKAlgorithm();
			/// Destructor
			~GJKAlgorithm();
			/// Initalize the algorithm
			virtual void init(CollisionDetection* _collisionDetection) {
				NarrowPhaseAlgorithm::init(_collisionDetection);
				m_algoEPA.init();
			};
			// Compute a contact info if the two collision shapes collide.
			/// This method implements the Hybrid Technique for computing the penetration depth by
			/// running the GJK algorithm on original objects (without margin). If the shapes int32_tersect
			/// only in the margins, the method compute the penetration depth and contact points
			/// (of enlarged objects). If the original objects (without margin) int32_tersect, we
			/// call the computePenetrationDepthForEnlargedObjects() method that run the GJK
			/// algorithm on the enlarged object to obtain a simplex polytope that contains the
			/// origin, they we give that simplex polytope to the EPA algorithm which will compute
			/// the correct penetration depth and contact points between the enlarged objects.
			virtual void testCollision(const CollisionShapeInfo& shape1Info,
			                           const CollisionShapeInfo& shape2Info,
			                           NarrowPhaseCallback* narrowPhaseCallback);
			/// Use the GJK Algorithm to find if a point is inside a convex collision shape
			bool testPointInside(const vec3& localPoint, ProxyShape* proxyShape);
			/// Ray casting algorithm agains a convex collision shape using the GJK Algorithm
			/// This method implements the GJK ray casting algorithm described by Gino Van Den Bergen in
			/// "Ray Casting against General Convex Objects with Application to Continuous Collision Detection".
			bool raycast(const Ray& ray, ProxyShape* proxyShape, RaycastInfo& raycastInfo);
	};
}

