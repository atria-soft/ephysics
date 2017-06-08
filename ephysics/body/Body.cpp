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
	 : m_id(id), m_isAlreadyInIsland(false), m_isAllowedToSleep(true), m_isActive(true),
	   m_isSleeping(false), m_sleepTime(0), m_userData(NULL) {

}

// Destructor
Body::~Body() {

}
