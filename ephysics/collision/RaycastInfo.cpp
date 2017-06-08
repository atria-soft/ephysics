/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries

#include <ephysics/collision/RaycastInfo.h>
#include <ephysics/collision/ProxyShape.h>

using namespace reactphysics3d;

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
