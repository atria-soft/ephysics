/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/constraint/Joint.h>

using namespace reactphysics3d;

// Constructor
Joint::Joint(const JointInfo& jointInfo)
		   :mBody1(jointInfo.body1), mBody2(jointInfo.body2), mType(jointInfo.type),
			mPositionCorrectionTechnique(jointInfo.positionCorrectionTechnique),
			mIsCollisionEnabled(jointInfo.isCollisionEnabled), mIsAlreadyInIsland(false) {

	assert(mBody1 != NULL);
	assert(mBody2 != NULL);
}

// Destructor
Joint::~Joint() {

}
