/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/ProxyShape.hpp>

using namespace ephysics;

// Constructor
/**
 * @param body Pointer to the parent body
 * @param shape Pointer to the collision shape
 * @param transform etk::Transform3Dation from collision shape local-space to body local-space
 * @param mass Mass of the collision shape (in kilograms)
 */
ProxyShape::ProxyShape(CollisionBody* body, CollisionShape* shape, const etk::Transform3D& transform, float mass)
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
bool ProxyShape::testPointInside(const vec3& worldPoint) {
	const etk::Transform3D localToWorld = m_body->getTransform() * m_localToBodyTransform;
	const vec3 localPoint = localToWorld.getInverse() * worldPoint;
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
	const etk::Transform3D localToWorldTransform = getLocalToWorldTransform();
	const etk::Transform3D worldToLocalTransform = localToWorldTransform.getInverse();
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

// Return the pointer to the cached collision data
void** ProxyShape::getCachedCollisionData()  {
	return &m_cachedCollisionData;
}

// Return the collision shape
/**
 * @return Pointer to the int32_ternal collision shape
 */
const CollisionShape* ProxyShape::getCollisionShape() const {
	return m_collisionShape;
}

// Return the parent body
/**
 * @return Pointer to the parent body
 */
CollisionBody* ProxyShape::getBody() const {
	return m_body;
}

// Return the mass of the collision shape
/**
 * @return Mass of the collision shape (in kilograms)
 */
float ProxyShape::getMass() const {
	return m_mass;
}

// Return a pointer to the user data attached to this body
/**
 * @return A pointer to the user data stored int32_to the proxy shape
 */
void* ProxyShape::getUserData() const {
	return m_userData;
}

// Attach user data to this body
/**
 * @param userData Pointer to the user data you want to store within the proxy shape
 */
void ProxyShape::setUserData(void* userData) {
	m_userData = userData;
}

// Return the local to parent body transform
/**
 * @return The transformation that transforms the local-space of the collision shape
 *		 to the local-space of the parent body
 */
const etk::Transform3D& ProxyShape::getLocalToBodyTransform() const {
	return m_localToBodyTransform;
}

// Set the local to parent body transform
void ProxyShape::setLocalToBodyTransform(const etk::Transform3D& transform) {

	m_localToBodyTransform = transform;

	m_body->setIsSleeping(false);

	// Notify the body that the proxy shape has to be updated in the broad-phase
	m_body->updateProxyShapeInBroadPhase(this, true);
}

// Return the local to world transform
/**
 * @return The transformation that transforms the local-space of the collision
 *		 shape to the world-space
 */
const etk::Transform3D ProxyShape::getLocalToWorldTransform() const {
	return m_body->m_transform * m_localToBodyTransform;
}

// Return the next proxy shape in the linked list of proxy shapes
/**
 * @return Pointer to the next proxy shape in the linked list of proxy shapes
 */
ProxyShape* ProxyShape::getNext() {
	return m_next;
}

// Return the next proxy shape in the linked list of proxy shapes
/**
 * @return Pointer to the next proxy shape in the linked list of proxy shapes
 */
const ProxyShape* ProxyShape::getNext() const {
	return m_next;
}

// Return the collision category bits
/**
 * @return The collision category bits mask of the proxy shape
 */
unsigned short ProxyShape::getCollisionCategoryBits() const {
	return m_collisionCategoryBits;
}

// Set the collision category bits
/**
 * @param collisionCategoryBits The collision category bits mask of the proxy shape
 */
void ProxyShape::setCollisionCategoryBits(unsigned short collisionCategoryBits) {
	m_collisionCategoryBits = collisionCategoryBits;
}

// Return the collision bits mask
/**
 * @return The bits mask that specifies with which collision category this shape will collide
 */
unsigned short ProxyShape::getCollideWithMaskBits() const {
	return m_collideWithMaskBits;
}

// Set the collision bits mask
/**
 * @param collideWithMaskBits The bits mask that specifies with which collision category this shape will collide
 */
void ProxyShape::setCollideWithMaskBits(unsigned short collideWithMaskBits) {
	m_collideWithMaskBits = collideWithMaskBits;
}

// Return the local scaling vector of the collision shape
/**
 * @return The local scaling vector
 */
vec3 ProxyShape::getLocalScaling() const {
	return m_collisionShape->getScaling();
}

// Set the local scaling vector of the collision shape
/**
 * @param scaling The new local scaling vector
 */
void ProxyShape::setLocalScaling(const vec3& scaling) {

	// Set the local scaling of the collision shape
	m_collisionShape->setLocalScaling(scaling);

	m_body->setIsSleeping(false);

	// Notify the body that the proxy shape has to be updated in the broad-phase
	m_body->updateProxyShapeInBroadPhase(this, true);
}
