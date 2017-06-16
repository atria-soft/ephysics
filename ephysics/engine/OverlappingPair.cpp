/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/engine/OverlappingPair.hpp>

using namespace ephysics;


// Constructor
OverlappingPair::OverlappingPair(ProxyShape* shape1, ProxyShape* shape2, int32_t nbMaxContactManifolds, MemoryAllocator& memoryAllocator):
  m_contactManifoldSet(shape1, shape2, memoryAllocator, nbMaxContactManifolds),
  m_cachedSeparatingAxis(1.0, 1.0, 1.0) {
	
}

// Destructor
OverlappingPair::~OverlappingPair() {
	
}
