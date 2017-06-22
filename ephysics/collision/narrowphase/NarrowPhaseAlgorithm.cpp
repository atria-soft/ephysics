/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

#include <ephysics/collision/narrowphase/NarrowPhaseAlgorithm.hpp>

using namespace ephysics;

NarrowPhaseAlgorithm::NarrowPhaseAlgorithm()
					 : m_memoryAllocator(NULL), m_currentOverlappingPair(NULL) {

}

NarrowPhaseAlgorithm::~NarrowPhaseAlgorithm() {

}

void NarrowPhaseAlgorithm::init(CollisionDetection* collisionDetection, MemoryAllocator* memoryAllocator) {
	m_collisionDetection = collisionDetection;
	m_memoryAllocator = memoryAllocator;
}

void NarrowPhaseAlgorithm::setCurrentOverlappingPair(OverlappingPair* _overlappingPair) {
	m_currentOverlappingPair = _overlappingPair;
}
