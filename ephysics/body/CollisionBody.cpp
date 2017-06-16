/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

 // Libraries
#include <ephysics/body/CollisionBody.hpp>
#include <ephysics/engine/CollisionWorld.hpp>
#include <ephysics/collision/ContactManifold.hpp>

// We want to use the ReactPhysics3D namespace
using namespace ephysics;

// Constructor
/**
 * @param transform The transform of the body
 * @param world The physics world where the body is created
 * @param id ID of the body
 */
CollisionBody::CollisionBody(const etk::Transform3D& transform, CollisionWorld& world, bodyindex id)
			  : Body(id), m_type(DYNAMIC), m_transform(transform), m_proxyCollisionShapes(NULL),
				m_numberCollisionShapes(0), m_contactManifoldsList(NULL), m_world(world) {

}

// Destructor
CollisionBody::~CollisionBody() {
	assert(m_contactManifoldsList == NULL);

	// Remove all the proxy collision shapes of the body
	removeAllCollisionShapes();
}

inline void CollisionBody::setType(BodyType _type) {
	m_type = _type;
	if (m_type == STATIC) {
		// Update the broad-phase state of the body
		updateBroadPhaseState();
	}
}


// Add a collision shape to the body. Note that you can share a collision
// shape between several bodies using the same collision shape instance to
// when you add the shape to the different bodies. Do not forget to delete
// the collision shape you have created at the end of your program.
/// This method will return a pointer to a new proxy shape. A proxy shape is
/// an object that links a collision shape and a given body. You can use the
/// returned proxy shape to get and set information about the corresponding
/// collision shape for that body.
/**
 * @param collisionShape A pointer to the collision shape you want to add to the body
 * @param transform The transformation of the collision shape that transforms the
 *		local-space of the collision shape int32_to the local-space of the body
 * @return A pointer to the proxy shape that has been created to link the body to
 *		 the new collision shape you have added.
 */
ProxyShape* CollisionBody::addCollisionShape(CollisionShape* collisionShape,
											 const etk::Transform3D& transform) {

	// Create a new proxy collision shape to attach the collision shape to the body
	ProxyShape* proxyShape = new (m_world.m_memoryAllocator.allocate(
									  sizeof(ProxyShape))) ProxyShape(this, collisionShape,
																	  transform, float(1));

	// Add it to the list of proxy collision shapes of the body
	if (m_proxyCollisionShapes == NULL) {
		m_proxyCollisionShapes = proxyShape;
	}
	else {
		proxyShape->m_next = m_proxyCollisionShapes;
		m_proxyCollisionShapes = proxyShape;
	}

	// Compute the world-space AABB of the new collision shape
	AABB aabb;
	collisionShape->computeAABB(aabb, m_transform * transform);

	// Notify the collision detection about this new collision shape
	m_world.m_collisionDetection.addProxyCollisionShape(proxyShape, aabb);

	m_numberCollisionShapes++;

	// Return a pointer to the collision shape
	return proxyShape;
}

// Remove a collision shape from the body
/// To remove a collision shape, you need to specify the pointer to the proxy
/// shape that has been returned when you have added the collision shape to the
/// body
/**
 * @param proxyShape The pointer of the proxy shape you want to remove
 */
void CollisionBody::removeCollisionShape(const ProxyShape* proxyShape) {

	ProxyShape* current = m_proxyCollisionShapes;

	// If the the first proxy shape is the one to remove
	if (current == proxyShape) {
		m_proxyCollisionShapes = current->m_next;

		if (m_isActive) {
			m_world.m_collisionDetection.removeProxyCollisionShape(current);
		}

		current->~ProxyShape();
		m_world.m_memoryAllocator.release(current, sizeof(ProxyShape));
		m_numberCollisionShapes--;
		return;
	}

	// Look for the proxy shape that contains the collision shape in parameter
	while(current->m_next != NULL) {

		// If we have found the collision shape to remove
		if (current->m_next == proxyShape) {

			// Remove the proxy collision shape
			ProxyShape* elementToRemove = current->m_next;
			current->m_next = elementToRemove->m_next;

			if (m_isActive) {
				m_world.m_collisionDetection.removeProxyCollisionShape(elementToRemove);
			}

			elementToRemove->~ProxyShape();
			m_world.m_memoryAllocator.release(elementToRemove, sizeof(ProxyShape));
			m_numberCollisionShapes--;
			return;
		}

		// Get the next element in the list
		current = current->m_next;
	}
}

// Remove all the collision shapes
void CollisionBody::removeAllCollisionShapes() {

	ProxyShape* current = m_proxyCollisionShapes;

	// Look for the proxy shape that contains the collision shape in parameter
	while(current != NULL) {

		// Remove the proxy collision shape
		ProxyShape* nextElement = current->m_next;

		if (m_isActive) {
			m_world.m_collisionDetection.removeProxyCollisionShape(current);
		}

		current->~ProxyShape();
		m_world.m_memoryAllocator.release(current, sizeof(ProxyShape));

		// Get the next element in the list
		current = nextElement;
	}

	m_proxyCollisionShapes = NULL;
}

// Reset the contact manifold lists
void CollisionBody::resetContactManifoldsList() {

	// Delete the linked list of contact manifolds of that body
	ContactManifoldListElement* currentElement = m_contactManifoldsList;
	while (currentElement != NULL) {
		ContactManifoldListElement* nextElement = currentElement->next;

		// Delete the current element
		currentElement->~ContactManifoldListElement();
		m_world.m_memoryAllocator.release(currentElement, sizeof(ContactManifoldListElement));

		currentElement = nextElement;
	}
	m_contactManifoldsList = NULL;
}

// Update the broad-phase state for this body (because it has moved for instance)
void CollisionBody::updateBroadPhaseState() const {

	// For all the proxy collision shapes of the body
	for (ProxyShape* shape = m_proxyCollisionShapes; shape != NULL; shape = shape->m_next) {

		// Update the proxy
		updateProxyShapeInBroadPhase(shape);
	}
}

// Update the broad-phase state of a proxy collision shape of the body
void CollisionBody::updateProxyShapeInBroadPhase(ProxyShape* proxyShape, bool forceReinsert) const {

	// Recompute the world-space AABB of the collision shape
	AABB aabb;
	proxyShape->getCollisionShape()->computeAABB(aabb, m_transform * proxyShape->getLocalToBodyTransform());

	// Update the broad-phase state for the proxy collision shape
	m_world.m_collisionDetection.updateProxyCollisionShape(proxyShape, aabb, vec3(0, 0, 0), forceReinsert);
}

// Set whether or not the body is active
/**
 * @param isActive True if you want to activate the body
 */
void CollisionBody::setIsActive(bool isActive) {

	// If the state does not change
	if (m_isActive == isActive) return;

	Body::setIsActive(isActive);

	// If we have to activate the body
	if (isActive) {

		// For each proxy shape of the body
		for (ProxyShape* shape = m_proxyCollisionShapes; shape != NULL; shape = shape->m_next) {

			// Compute the world-space AABB of the new collision shape
			AABB aabb;
			shape->getCollisionShape()->computeAABB(aabb, m_transform * shape->m_localToBodyTransform);

			// Add the proxy shape to the collision detection
			m_world.m_collisionDetection.addProxyCollisionShape(shape, aabb);
		}
	}
	else {  // If we have to deactivate the body

		// For each proxy shape of the body
		for (ProxyShape* shape = m_proxyCollisionShapes; shape != NULL; shape = shape->m_next) {

			// Remove the proxy shape from the collision detection
			m_world.m_collisionDetection.removeProxyCollisionShape(shape);
		}

		// Reset the contact manifold list of the body
		resetContactManifoldsList();
	}
}

// Ask the broad-phase to test again the collision shapes of the body for collision
// (as if the body has moved).
void CollisionBody::askForBroadPhaseCollisionCheck() const {

	// For all the proxy collision shapes of the body
	for (ProxyShape* shape = m_proxyCollisionShapes; shape != NULL; shape = shape->m_next) {

		m_world.m_collisionDetection.askForBroadPhaseCollisionCheck(shape);  
	}
}

// Reset the m_isAlreadyInIsland variable of the body and contact manifolds.
/// This method also returns the number of contact manifolds of the body.
int32_t CollisionBody::resetIsAlreadyInIslandAndCountManifolds() {

	m_isAlreadyInIsland = false;

	int32_t nbManifolds = 0;

	// Reset the m_isAlreadyInIsland variable of the contact manifolds for
	// this body
	ContactManifoldListElement* currentElement = m_contactManifoldsList;
	while (currentElement != NULL) {
		currentElement->contactManifold->m_isAlreadyInIsland = false;
		currentElement = currentElement->next;
		nbManifolds++;
	}

	return nbManifolds;
}

// Return true if a point is inside the collision body
/// This method returns true if a point is inside any collision shape of the body
/**
 * @param worldPoint The point to test (in world-space coordinates)
 * @return True if the point is inside the body
 */
bool CollisionBody::testPointInside(const vec3& worldPoint) const {

	// For each collision shape of the body
	for (ProxyShape* shape = m_proxyCollisionShapes; shape != NULL; shape = shape->m_next) {

		// Test if the point is inside the collision shape
		if (shape->testPointInside(worldPoint)) return true;
	}

	return false;
}

// Raycast method with feedback information
/// The method returns the closest hit among all the collision shapes of the body
/**
* @param ray The ray used to raycast agains the body
* @param[out] raycastInfo Structure that contains the result of the raycasting
*						 (valid only if the method returned true)
* @return True if the ray hit the body and false otherwise
*/
bool CollisionBody::raycast(const Ray& ray, RaycastInfo& raycastInfo) {

	// If the body is not active, it cannot be hit by rays
	if (!m_isActive) return false;

	bool isHit = false;
	Ray rayTemp(ray);

	// For each collision shape of the body
	for (ProxyShape* shape = m_proxyCollisionShapes; shape != NULL; shape = shape->m_next) {

		// Test if the ray hits the collision shape
		if (shape->raycast(rayTemp, raycastInfo)) {
			rayTemp.maxFraction = raycastInfo.hitFraction;
			isHit = true;
		}
	}

	return isHit;
}

// Compute and return the AABB of the body by merging all proxy shapes AABBs
/**
* @return The axis-aligned bounding box (AABB) of the body in world-space coordinates
*/
AABB CollisionBody::getAABB() const {

	AABB bodyAABB;

	if (m_proxyCollisionShapes == NULL) return bodyAABB;

	m_proxyCollisionShapes->getCollisionShape()->computeAABB(bodyAABB, m_transform * m_proxyCollisionShapes->getLocalToBodyTransform());

	// For each proxy shape of the body
	for (ProxyShape* shape = m_proxyCollisionShapes->m_next; shape != NULL; shape = shape->m_next) {

		// Compute the world-space AABB of the collision shape
		AABB aabb;
		shape->getCollisionShape()->computeAABB(aabb, m_transform * shape->getLocalToBodyTransform());

		// Merge the proxy shape AABB with the current body AABB
		bodyAABB.mergeWithAABB(aabb);
	}

	return bodyAABB;
}
