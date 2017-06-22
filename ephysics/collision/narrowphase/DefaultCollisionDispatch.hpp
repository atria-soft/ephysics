/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

#include <ephysics/collision/narrowphase/CollisionDispatch.hpp>
#include <ephysics/collision/narrowphase/ConcaveVsConvexAlgorithm.hpp>
#include <ephysics/collision/narrowphase/SphereVsSphereAlgorithm.hpp>
#include <ephysics/collision/narrowphase/GJK/GJKAlgorithm.hpp>

namespace ephysics {
/**
 * @brief This is the default collision dispatch configuration use in ephysics.
 * Collision dispatching decides which collision
 * algorithm to use given two types of proxy collision shapes.
 */
class DefaultCollisionDispatch : public CollisionDispatch {
	protected:
		SphereVsSphereAlgorithm mSphereVsSphereAlgorithm; //!< Sphere vs Sphere collision algorithm
		ConcaveVsConvexAlgorithm mConcaveVsConvexAlgorithm; //!< Concave vs Convex collision algorithm
		GJKAlgorithm mGJKAlgorithm; //!< GJK Algorithm
	public:
		/// Constructor
		DefaultCollisionDispatch();
		/// Destructor
		virtual ~DefaultCollisionDispatch();
		/// Initialize the collision dispatch configuration
		virtual void init(CollisionDetection* _collisionDetection, MemoryAllocator* _memoryAllocator);
		/// Select and return the narrow-phase collision detection algorithm to
		/// use between two types of collision shapes.
		virtual NarrowPhaseAlgorithm* selectAlgorithm(int32_t _type1, int32_t _type2);
};

}


