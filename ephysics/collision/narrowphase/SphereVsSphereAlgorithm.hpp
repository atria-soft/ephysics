/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/body/Body.hpp>
#include <ephysics/constraint/ContactPoint.hpp>
#include <ephysics/collision/narrowphase/NarrowPhaseAlgorithm.hpp>


/// Namespace ReactPhysics3D
namespace ephysics {

// Class SphereVsSphereAlgorithm
/**
 * This class is used to compute the narrow-phase collision detection
 * between two sphere collision shapes.
 */
class SphereVsSphereAlgorithm : public NarrowPhaseAlgorithm {

	protected :

		// -------------------- Methods -------------------- //

		/// Private copy-constructor
		SphereVsSphereAlgorithm(const SphereVsSphereAlgorithm& algorithm);

		/// Private assignment operator
		SphereVsSphereAlgorithm& operator=(const SphereVsSphereAlgorithm& algorithm);
		
	public :

		// -------------------- Methods -------------------- //

		/// Constructor
		SphereVsSphereAlgorithm();

		/// Destructor
		virtual ~SphereVsSphereAlgorithm();

		/// Compute a contact info if the two bounding volume collide
		virtual void testCollision(const CollisionShapeInfo& shape1Info,
								   const CollisionShapeInfo& shape2Info,
								   NarrowPhaseCallback* narrowPhaseCallback);
};

}


