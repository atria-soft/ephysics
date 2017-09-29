/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

#include <ephysics/body/Body.hpp>
#include <ephysics/constraint/ContactPoint.hpp>
#include <ephysics/engine/OverlappingPair.hpp>
#include <ephysics/collision/CollisionShapeInfo.hpp>

namespace ephysics {

	class CollisionDetection;
	
	/**
	 * @brief It is the base class for a narrow-phase collision
	 * callback class.
	 */
	class NarrowPhaseCallback {
		public:
			virtual ~NarrowPhaseCallback() = default;
			/// Called by a narrow-phase collision algorithm when a new contact has been found
			virtual void notifyContact(OverlappingPair* _overlappingPair,
			                           const ContactPointInfo& _contactInfo) = 0;
	
	};
	
	/**
	 * @breif It is the base class for a  narrow-phase collision
	 * detection algorithm. The goal of the narrow phase algorithm is to
	 * compute information about the contact between two proxy shapes.
	 */
	class NarrowPhaseAlgorithm {
		protected :
			CollisionDetection* m_collisionDetection; //!< Pointer to the collision detection object
			OverlappingPair* m_currentOverlappingPair; //!< Overlapping pair of the bodies currently tested for collision
			/// Private copy-constructor
			NarrowPhaseAlgorithm(const NarrowPhaseAlgorithm& algorithm) = delete;
			/// Private assignment operator
			NarrowPhaseAlgorithm& operator=(const NarrowPhaseAlgorithm& algorithm) = delete;
		public :
			/// Constructor
			NarrowPhaseAlgorithm();
			/// Destructor
			virtual ~NarrowPhaseAlgorithm() = default;
			/// Initalize the algorithm
			virtual void init(CollisionDetection* _collisionDetection);
			/// Set the current overlapping pair of bodies
			void setCurrentOverlappingPair(OverlappingPair* _overlappingPair);
			/// Compute a contact info if the two bounding volume collide
			virtual void testCollision(const CollisionShapeInfo& _shape1Info,
			                           const CollisionShapeInfo& _shape2Info,
			                           NarrowPhaseCallback* _narrowPhaseCallback) = 0;
	};
}


