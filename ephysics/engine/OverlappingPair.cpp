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

// Return the pointer to first body
ProxyShape* OverlappingPair::getShape1() const {
	return m_contactManifoldSet.getShape1();
}

// Return the pointer to second body
ProxyShape* OverlappingPair::getShape2() const {
	return m_contactManifoldSet.getShape2();
}

// Add a contact to the contact manifold
void OverlappingPair::addContact(ContactPoint* contact) {
	m_contactManifoldSet.addContactPoint(contact);
}

// Update the contact manifold
void OverlappingPair::update() {
	m_contactManifoldSet.update();
}

// Return the cached separating axis
vec3 OverlappingPair::getCachedSeparatingAxis() const {
	return m_cachedSeparatingAxis;
}

// Set the cached separating axis
void OverlappingPair::setCachedSeparatingAxis(const vec3& _axis) {
	m_cachedSeparatingAxis = _axis;
}

// Return the number of contact points in the contact manifold
uint32_t OverlappingPair::getNbContactPoints() const {
	return m_contactManifoldSet.getTotalNbContactPoints();
}

// Return the contact manifold
const ContactManifoldSet& OverlappingPair::getContactManifoldSet() {
	return m_contactManifoldSet;
}

// Return the pair of bodies index
overlappingpairid OverlappingPair::computeID(ProxyShape* _shape1, ProxyShape* _shape2) {
	assert(    _shape1->m_broadPhaseID >= 0
	        && _shape2->m_broadPhaseID >= 0);

	// Construct the pair of body index
	overlappingpairid pairID = _shape1->m_broadPhaseID < _shape2->m_broadPhaseID ?
							 std::make_pair(_shape1->m_broadPhaseID, _shape2->m_broadPhaseID) :
							 std::make_pair(_shape2->m_broadPhaseID, _shape1->m_broadPhaseID);
	assert(pairID.first != pairID.second);
	return pairID;
}

// Return the pair of bodies index
bodyindexpair OverlappingPair::computeBodiesIndexPair(CollisionBody* _body1,
															 CollisionBody* _body2) {

	// Construct the pair of body index
	bodyindexpair indexPair = _body1->getID() < _body2->getID() ?
								 std::make_pair(_body1->getID(), _body2->getID()) :
								 std::make_pair(_body2->getID(), _body1->getID());
	assert(indexPair.first != indexPair.second);
	return indexPair;
}

// Clear the contact points of the contact manifold
void OverlappingPair::clearContactPoints() {
	m_contactManifoldSet.clear();
}
