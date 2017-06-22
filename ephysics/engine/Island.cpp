/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

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

// Add a body int32_to the island
void Island::addBody(RigidBody* body) {
	assert(!body->isSleeping());
	m_bodies[m_numberBodies] = body;
	m_numberBodies++;
}

// Add a contact manifold int32_to the island
void Island::addContactManifold(ContactManifold* contactManifold) {
	m_contactManifolds[m_numberContactManifolds] = contactManifold;
	m_numberContactManifolds++;
}

// Add a joint int32_to the island
void Island::addJoint(Joint* joint) {
	m_joints[m_numberJoints] = joint;
	m_numberJoints++;
}

// Return the number of bodies in the island
uint32_t Island::getNbBodies() const {
	return m_numberBodies;
}

// Return the number of contact manifolds in the island
uint32_t Island::getNbContactManifolds() const {
	return m_numberContactManifolds;
}

// Return the number of joints in the island
uint32_t Island::getNbJoints() const {
	return m_numberJoints;
}

// Return a pointer to the array of bodies
RigidBody** Island::getBodies() {
	return m_bodies;
}

// Return a pointer to the array of contact manifolds
ContactManifold** Island::getContactManifold() {
	return m_contactManifolds;
}

// Return a pointer to the array of joints
Joint** Island::getJoints() {
	return m_joints;
}
