/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
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
