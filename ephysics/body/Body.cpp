/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/body/Body.h>
#include <ephysics/collision/shapes/CollisionShape.h>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
/**
 * @param id ID of the new body
 */
Body::Body(bodyindex id)
	 : mID(id), mIsAlreadyInIsland(false), mIsAllowedToSleep(true), mIsActive(true),
	   mIsSleeping(false), mSleepTime(0), mUserData(NULL) {

}

// Destructor
Body::~Body() {

}
