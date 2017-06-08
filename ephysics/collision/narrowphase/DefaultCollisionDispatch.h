/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/collision/narrowphase/CollisionDispatch.h>
#include <ephysics/collision/narrowphase/ConcaveVsConvexAlgorithm.h>
#include <ephysics/collision/narrowphase/SphereVsSphereAlgorithm.h>
#include <ephysics/collision/narrowphase/GJK/GJKAlgorithm.h>

namespace reactphysics3d {

// Class DefaultCollisionDispatch
/**
 * This is the default collision dispatch configuration use in ReactPhysics3D.
 * Collision dispatching decides which collision
 * algorithm to use given two types of proxy collision shapes.
 */
class DefaultCollisionDispatch : public CollisionDispatch {

	protected:

		/// Sphere vs Sphere collision algorithm
		SphereVsSphereAlgorithm mSphereVsSphereAlgorithm;

		/// Concave vs Convex collision algorithm
		ConcaveVsConvexAlgorithm mConcaveVsConvexAlgorithm;

		/// GJK Algorithm
		GJKAlgorithm mGJKAlgorithm;

	public:

		/// Constructor
		DefaultCollisionDispatch();

		/// Destructor
		virtual ~DefaultCollisionDispatch();

		/// Initialize the collision dispatch configuration
		virtual void init(CollisionDetection* collisionDetection,
						  MemoryAllocator* memoryAllocator);

		/// Select and return the narrow-phase collision detection algorithm to
		/// use between two types of collision shapes.
		virtual NarrowPhaseAlgorithm* selectAlgorithm(int32_t type1, int32_t type2);
};

}


