/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#include <ephysics/constraint/Joint.hpp>

using namespace ephysics;

Joint::Joint(const JointInfo& jointInfo)
		   :m_body1(jointInfo.body1), m_body2(jointInfo.body2), m_type(jointInfo.type),
			m_positionCorrectionTechnique(jointInfo.positionCorrectionTechnique),
			m_isCollisionEnabled(jointInfo.isCollisionEnabled), m_isAlreadyInIsland(false) {

	assert(m_body1 != null);
	assert(m_body2 != null);
}

Joint::~Joint() {

}

// Return the reference to the body 1
/**
 * @return The first body involved in the joint
 */
RigidBody* Joint::getBody1() const {
	return m_body1;
}

// Return the reference to the body 2
/**
 * @return The second body involved in the joint
 */
RigidBody* Joint::getBody2() const {
	return m_body2;
}

// Return true if the joint is active
/**
 * @return True if the joint is active
 */
bool Joint::isActive() const {
	return (m_body1->isActive() && m_body2->isActive());
}

// Return the type of the joint
/**
 * @return The type of the joint
 */
JointType Joint::getType() const {
	return m_type;
}

// Return true if the collision between the two bodies of the joint is enabled
/**
 * @return True if the collision is enabled between the two bodies of the joint
 *			  is enabled and false otherwise
 */
bool Joint::isCollisionEnabled() const {
	return m_isCollisionEnabled;
}

// Return true if the joint has already been added int32_to an island
bool Joint::isAlreadyInIsland() const {
	return m_isAlreadyInIsland;
}

