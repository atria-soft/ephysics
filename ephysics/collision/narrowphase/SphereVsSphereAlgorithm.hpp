/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
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


