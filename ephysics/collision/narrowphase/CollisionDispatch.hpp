/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

#include <ephysics/collision/narrowphase/NarrowPhaseAlgorithm.hpp>

namespace ephysics {
	/**
	 * @biref Abstract base class for dispatching the narrow-phase
	 * collision detection algorithm. Collision dispatching decides which collision
	 * algorithm to use given two types of proxy collision shapes.
	 */
	class CollisionDispatch {
		public:
			/// Constructor
			CollisionDispatch() {}
			/// Destructor
			virtual ~CollisionDispatch() = default;
			/// Initialize the collision dispatch configuration
			virtual void init(CollisionDetection* _collisionDetection,
			                  MemoryAllocator* _memoryAllocator) {
			}
			/// Select and return the narrow-phase collision detection algorithm to
			/// use between two types of collision shapes.
			virtual NarrowPhaseAlgorithm* selectAlgorithm(int32_t _shape1Type,
			                                              int32_t _shape2Type) = 0;
	};

}


