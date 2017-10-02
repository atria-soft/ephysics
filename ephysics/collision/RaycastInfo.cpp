/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#include <ephysics/collision/RaycastInfo.hpp>
#include <ephysics/collision/ProxyShape.hpp>

using namespace ephysics;

// Ray cast test against a proxy shape
float RaycastTest::raycastAgainstShape(ProxyShape* shape, const Ray& ray) {

	// Ray casting test against the collision shape
	RaycastInfo raycastInfo;
	bool isHit = shape->raycast(ray, raycastInfo);

	// If the ray hit the collision shape
	if (isHit) {

		// Report the hit to the user and return the
		// user hit fraction value
		return userCallback->notifyRaycastHit(raycastInfo);
	}

	return ray.maxFraction;
}
