/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/body/Body.h>
#include <ephysics/constraint/ContactPoint.h>
#include <ephysics/memory/MemoryAllocator.h>
#include <ephysics/engine/OverlappingPair.h>
#include <ephysics/collision/CollisionShapeInfo.h>

/// Namespace ReactPhysics3D
namespace reactphysics3d {

class CollisionDetection;

// Class NarrowPhaseCallback
/**
 * This abstract class is the base class for a narrow-phase collision
 * callback class.
 */
class NarrowPhaseCallback {

	public:
		virtual ~NarrowPhaseCallback() = default;

		/// Called by a narrow-phase collision algorithm when a new contact has been found
		virtual void notifyContact(OverlappingPair* overlappingPair,
								   const ContactPointInfo& contactInfo)=0;

};

// Class NarrowPhaseAlgorithm
/**
 * This abstract class is the base class for a  narrow-phase collision
 * detection algorithm. The goal of the narrow phase algorithm is to
 * compute information about the contact between two proxy shapes.
 */
class NarrowPhaseAlgorithm {

	protected :

		// -------------------- Attributes -------------------- //

		/// Pointer to the collision detection object
		CollisionDetection* m_collisionDetection;

		/// Pointer to the memory allocator
		MemoryAllocator* mMemoryAllocator;

		/// Overlapping pair of the bodies currently tested for collision
		OverlappingPair* mCurrentOverlappingPair;
		
		// -------------------- Methods -------------------- //

		/// Private copy-constructor
		NarrowPhaseAlgorithm(const NarrowPhaseAlgorithm& algorithm);

		/// Private assignment operator
		NarrowPhaseAlgorithm& operator=(const NarrowPhaseAlgorithm& algorithm);

	public :

		// -------------------- Methods -------------------- //

		/// Constructor
		NarrowPhaseAlgorithm();

		/// Destructor
		virtual ~NarrowPhaseAlgorithm();

		/// Initalize the algorithm
		virtual void init(CollisionDetection* collisionDetection, MemoryAllocator* memoryAllocator);
		
		/// Set the current overlapping pair of bodies
		void setCurrentOverlappingPair(OverlappingPair* overlappingPair);

		/// Compute a contact info if the two bounding volume collide
		virtual void testCollision(const CollisionShapeInfo& shape1Info,
								   const CollisionShapeInfo& shape2Info,
								   NarrowPhaseCallback* narrowPhaseCallback)=0;
};

// Set the current overlapping pair of bodies
inline void NarrowPhaseAlgorithm::setCurrentOverlappingPair(OverlappingPair* overlappingPair) {
	mCurrentOverlappingPair = overlappingPair;
}

}


