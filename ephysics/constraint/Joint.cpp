/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/constraint/Joint.hpp>

using namespace ephysics;

// Constructor
Joint::Joint(const JointInfo& jointInfo)
		   :m_body1(jointInfo.body1), m_body2(jointInfo.body2), m_type(jointInfo.type),
			m_positionCorrectionTechnique(jointInfo.positionCorrectionTechnique),
			m_isCollisionEnabled(jointInfo.isCollisionEnabled), m_isAlreadyInIsland(false) {

	assert(m_body1 != NULL);
	assert(m_body2 != NULL);
}

// Destructor
Joint::~Joint() {

}
