/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
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
			virtual void init(CollisionDetection* _collisionDetection) {
				// Nothing to do ...
			}
			/// Select and return the narrow-phase collision detection algorithm to
			/// use between two types of collision shapes.
			virtual NarrowPhaseAlgorithm* selectAlgorithm(int32_t _shape1Type,
			                                              int32_t _shape2Type) = 0;
	};

}


