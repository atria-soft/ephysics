/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#include <ephysics/engine/Island.hpp>
#include <ephysics/debug.hpp>

using namespace ephysics;

ephysics::Island::Island(size_t _nbMaxBodies,
                         size_t _nbMaxContactManifolds,
                         size_t _nbMaxJoints) {
	// Allocate memory for the arrays
	m_bodies.reserve(_nbMaxBodies);
	m_contactManifolds.reserve(_nbMaxContactManifolds);
	m_joints.reserve(_nbMaxJoints);
}


void ephysics::Island::addBody(ephysics::RigidBody* _body) {
	if (_body->isSleeping() == true) {
		EPHY_ERROR("Try to add a body that is sleeping ...");
		return;
	}
	m_bodies.pushBack(_body);
}

void ephysics::Island::addContactManifold(ephysics::ContactManifold* _contactManifold) {
	m_contactManifolds.pushBack(_contactManifold);
}

void ephysics::Island::addJoint(ephysics::Joint* _joint) {
	m_joints.pushBack(_joint);
}

size_t ephysics::Island::getNbBodies() const {
	return m_bodies.size();
}

size_t ephysics::Island::getNbContactManifolds() const {
	return m_contactManifolds.size();
}

size_t ephysics::Island::getNbJoints() const {
	return m_joints.size();
}

ephysics::RigidBody** ephysics::Island::getBodies() {
	return &m_bodies[0];
}

ephysics::ContactManifold** ephysics::Island::getContactManifold() {
	return &m_contactManifolds[0];
}

ephysics::Joint** ephysics::Island::getJoints() {
	return &m_joints[0];
}

void ephysics::Island::resetStaticBobyNotInIsland() {
	for (auto &it: m_bodies) {
		if (it->getType() == STATIC) {
			it->m_isAlreadyInIsland = false;
		}
	}
}
