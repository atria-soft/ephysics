/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/narrowphase/NarrowPhaseAlgorithm.hpp>

// We want to use the ReactPhysics3D namespace
using namespace ephysics;

// Constructor
NarrowPhaseAlgorithm::NarrowPhaseAlgorithm()
					 : m_memoryAllocator(NULL), mCurrentOverlappingPair(NULL) {

}

// Destructor
NarrowPhaseAlgorithm::~NarrowPhaseAlgorithm() {

}

// Initalize the algorithm
void NarrowPhaseAlgorithm::init(CollisionDetection* collisionDetection, MemoryAllocator* memoryAllocator) {
	m_collisionDetection = collisionDetection;
	m_memoryAllocator = memoryAllocator;
}
