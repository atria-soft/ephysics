/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/engine/Island.hpp>

using namespace ephysics;

// Constructor
Island::Island(uint32_t nbMaxBodies, uint32_t nbMaxContactManifolds, uint32_t nbMaxJoints,
			   MemoryAllocator& memoryAllocator)
	   : m_bodies(NULL), m_contactManifolds(NULL), m_joints(NULL), m_numberBodies(0),
		 m_numberContactManifolds(0), m_numberJoints(0), m_memoryAllocator(memoryAllocator) {

	// Allocate memory for the arrays
	m_numberAllocatedBytesBodies = sizeof(RigidBody*) * nbMaxBodies;
	m_bodies = (RigidBody**) m_memoryAllocator.allocate(m_numberAllocatedBytesBodies);
	m_numberAllocatedBytesContactManifolds = sizeof(ContactManifold*) * nbMaxContactManifolds;
	m_contactManifolds = (ContactManifold**) m_memoryAllocator.allocate(
																m_numberAllocatedBytesContactManifolds);
	m_numberAllocatedBytesJoints = sizeof(Joint*) * nbMaxJoints;
	m_joints = (Joint**) m_memoryAllocator.allocate(m_numberAllocatedBytesJoints);
}

// Destructor
Island::~Island() {

	// Release the memory of the arrays
	m_memoryAllocator.release(m_bodies, m_numberAllocatedBytesBodies);
	m_memoryAllocator.release(m_contactManifolds, m_numberAllocatedBytesContactManifolds);
	m_memoryAllocator.release(m_joints, m_numberAllocatedBytesJoints);
}
