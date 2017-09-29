/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/engine/OverlappingPair.hpp>

using namespace ephysics;

OverlappingPair::OverlappingPair(ProxyShape* _shape1, ProxyShape* _shape2, int32_t _nbMaxContactManifolds):
  m_contactManifoldSet(_shape1, _shape2, _nbMaxContactManifolds),
  m_cachedSeparatingAxis(1.0, 1.0, 1.0) {
	
}

ProxyShape* OverlappingPair::getShape1() const {
	return m_contactManifoldSet.getShape1();
}

ProxyShape* OverlappingPair::getShape2() const {
	return m_contactManifoldSet.getShape2();
}

void OverlappingPair::addContact(ContactPoint* _contact) {
	m_contactManifoldSet.addContactPoint(_contact);
}

void OverlappingPair::update() {
	m_contactManifoldSet.update();
}

vec3 OverlappingPair::getCachedSeparatingAxis() const {
	return m_cachedSeparatingAxis;
}

void OverlappingPair::setCachedSeparatingAxis(const vec3& _axis) {
	m_cachedSeparatingAxis = _axis;
}

uint32_t OverlappingPair::getNbContactPoints() const {
	return m_contactManifoldSet.getTotalNbContactPoints();
}

const ContactManifoldSet& OverlappingPair::getContactManifoldSet() {
	return m_contactManifoldSet;
}

overlappingpairid OverlappingPair::computeID(ProxyShape* _shape1, ProxyShape* _shape2) {
	assert(    _shape1->m_broadPhaseID >= 0
	        && _shape2->m_broadPhaseID >= 0);
	// Construct the pair of body index
	overlappingpairid pairID = _shape1->m_broadPhaseID < _shape2->m_broadPhaseID ?
							 etk::makePair(_shape1->m_broadPhaseID, _shape2->m_broadPhaseID) :
							 etk::makePair(_shape2->m_broadPhaseID, _shape1->m_broadPhaseID);
	assert(pairID.first != pairID.second);
	return pairID;
}

bodyindexpair OverlappingPair::computeBodiesIndexPair(CollisionBody* _body1,
															 CollisionBody* _body2) {
	// Construct the pair of body index
	bodyindexpair indexPair = _body1->getID() < _body2->getID() ?
								 etk::makePair(_body1->getID(), _body2->getID()) :
								 etk::makePair(_body2->getID(), _body1->getID());
	assert(indexPair.first != indexPair.second);
	return indexPair;
}

void OverlappingPair::clearContactPoints() {
	m_contactManifoldSet.clear();
}

