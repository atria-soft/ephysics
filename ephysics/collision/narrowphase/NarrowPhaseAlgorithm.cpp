/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/narrowphase/NarrowPhaseAlgorithm.h>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
NarrowPhaseAlgorithm::NarrowPhaseAlgorithm()
					 : mMemoryAllocator(NULL), mCurrentOverlappingPair(NULL) {

}

// Destructor
NarrowPhaseAlgorithm::~NarrowPhaseAlgorithm() {

}

// Initalize the algorithm
void NarrowPhaseAlgorithm::init(CollisionDetection* collisionDetection, MemoryAllocator* memoryAllocator) {
	mCollisionDetection = collisionDetection;
	mMemoryAllocator = memoryAllocator;
}
