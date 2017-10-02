/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#pragma once

#include <ephysics/body/Body.hpp>
#include <ephysics/constraint/ContactPoint.hpp>
#include <ephysics/collision/narrowphase/NarrowPhaseAlgorithm.hpp>

namespace ephysics {
	/**
	 * @brief It is used to compute the narrow-phase collision detection
	 * between two sphere collision shapes.
	 */
	class SphereVsSphereAlgorithm : public NarrowPhaseAlgorithm {
		protected :
			SphereVsSphereAlgorithm(const SphereVsSphereAlgorithm&) = delete;
			SphereVsSphereAlgorithm& operator=(const SphereVsSphereAlgorithm&) = delete;
		public :
			/**
			 * @brief Constructor
			 */
			SphereVsSphereAlgorithm();
			/**
			 * @brief Destructor
			 */
			virtual ~SphereVsSphereAlgorithm() = default;
			/**
			 * @brief Compute a contact info if the two bounding volume collide
			 */
			virtual void testCollision(const CollisionShapeInfo& _shape1Info,
			                           const CollisionShapeInfo& _shape2Info,
			                           NarrowPhaseCallback* _narrowPhaseCallback);
	};
}


