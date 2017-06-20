/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

#include <ephysics/body/Body.hpp>
#include <ephysics/collision/shapes/CollisionShape.hpp>


ephysics::Body::Body(bodyindex _id):
  m_id(_id),
  m_isAlreadyInIsland(false),
  m_isAllowedToSleep(true),
  m_isActive(true),
  m_isSleeping(false),
  m_sleepTime(0),
  m_userData(nullptr) {
	
}