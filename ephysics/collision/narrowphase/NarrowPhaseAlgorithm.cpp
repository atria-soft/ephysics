/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

#include <ephysics/collision/narrowphase/NarrowPhaseAlgorithm.hpp>

using namespace ephysics;

NarrowPhaseAlgorithm::NarrowPhaseAlgorithm():
  m_currentOverlappingPair(nullptr) {
	
}

void NarrowPhaseAlgorithm::init(CollisionDetection* _collisionDetection) {
	m_collisionDetection = _collisionDetection;
}

void NarrowPhaseAlgorithm::setCurrentOverlappingPair(OverlappingPair* _overlappingPair) {
	m_currentOverlappingPair = _overlappingPair;
}
