/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/ProxyShape.h>

using namespace reactphysics3d;

// Constructor
/**
 * @param body Pointer to the parent body
 * @param shape Pointer to the collision shape
 * @param transform Transformation from collision shape local-space to body local-space
 * @param mass Mass of the collision shape (in kilograms)
 */
ProxyShape::ProxyShape(CollisionBody* body, CollisionShape* shape, const Transform& transform, float mass)
		   :m_body(body), m_collisionShape(shape), m_localToBodyTransform(transform), m_mass(mass),
			m_next(NULL), m_broadPhaseID(-1), m_cachedCollisionData(NULL), m_userData(NULL),
			m_collisionCategoryBits(0x0001), m_collideWithMaskBits(0xFFFF) {

}

// Destructor
ProxyShape::~ProxyShape() {

	// Release the cached collision data memory
	if (m_cachedCollisionData != NULL) {
		free(m_cachedCollisionData);
	}
}

// Return true if a point is inside the collision shape
/**
 * @param worldPoint Point to test in world-space coordinates
 * @return True if the point is inside the collision shape
 */
bool ProxyShape::testPointInside(const Vector3& worldPoint) {
	const Transform localToWorld = m_body->getTransform() * m_localToBodyTransform;
	const Vector3 localPoint = localToWorld.getInverse() * worldPoint;
	return m_collisionShape->testPointInside(localPoint, this);
}

// Raycast method with feedback information
/**
 * @param ray Ray to use for the raycasting
 * @param[out] raycastInfo Result of the raycasting that is valid only if the
 *			 methods returned true
 * @return True if the ray hit the collision shape
 */
bool ProxyShape::raycast(const Ray& ray, RaycastInfo& raycastInfo) {

	// If the corresponding body is not active, it cannot be hit by rays
	if (!m_body->isActive()) return false;

	// Convert the ray int32_to the local-space of the collision shape
	const Transform localToWorldTransform = getLocalToWorldTransform();
	const Transform worldToLocalTransform = localToWorldTransform.getInverse();
	Ray rayLocal(worldToLocalTransform * ray.point1,
				 worldToLocalTransform * ray.point2,
				 ray.maxFraction);

	bool isHit = m_collisionShape->raycast(rayLocal, raycastInfo, this);

	// Convert the raycast info int32_to world-space
	raycastInfo.worldPoint = localToWorldTransform * raycastInfo.worldPoint;
	raycastInfo.worldNormal = localToWorldTransform.getOrientation() * raycastInfo.worldNormal;
	raycastInfo.worldNormal.normalize();

	return isHit;
}
