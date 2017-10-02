/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
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