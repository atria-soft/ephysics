/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/collision/narrowphase/NarrowPhaseAlgorithm.h>

namespace reactphysics3d {

// Class CollisionDispatch
/**
 * This is the abstract base class for dispatching the narrow-phase
 * collision detection algorithm. Collision dispatching decides which collision
 * algorithm to use given two types of proxy collision shapes.
 */
class CollisionDispatch {

	protected:

	public:

		/// Constructor
		CollisionDispatch() {}

		/// Destructor
		virtual ~CollisionDispatch() {}

		/// Initialize the collision dispatch configuration
		virtual void init(CollisionDetection* collisionDetection,
						  MemoryAllocator* memoryAllocator) {
		}

		/// Select and return the narrow-phase collision detection algorithm to
		/// use between two types of collision shapes.
		virtual NarrowPhaseAlgorithm* selectAlgorithm(int32_t shape1Type,
													  int32_t shape2Type)=0;
};

}


