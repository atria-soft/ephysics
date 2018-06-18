/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

#include <ephysics/engine/CollisionWorld.hpp>
#include <ephysics/debug.hpp>

using namespace ephysics;
using namespace std;

CollisionWorld::CollisionWorld() :
  m_collisionDetection(this),
  m_currentBodyID(0),
  m_eventListener(nullptr) {
	
}

CollisionWorld::~CollisionWorld() {
	// Destroy all the collision bodies that have not been removed
	etk::Set<CollisionBody*>::Iterator itBodies;
	for (itBodies = m_bodies.begin(); itBodies != m_bodies.end(); ) {
		 etk::Set<CollisionBody*>::Iterator itToRemove = itBodies;
		 ++itBodies;
		destroyCollisionBody(*itToRemove);
	}
	assert(m_bodies.empty());
}

/**
 * @brief Create a collision body and add it to the world
 * @param transform etk::Transform3Dation mapping the local-space of the body to world-space
 * @return A pointer to the body that has been created in the world
 */
CollisionBody* CollisionWorld::createCollisionBody(const etk::Transform3D& transform) {
	// Get the next available body ID
	bodyindex bodyID = computeNextAvailableBodyID();
	// Largest index cannot be used (it is used for invalid index)
	EPHY_ASSERT(bodyID < UINT64_MAX, "index too big");
	// Create the collision body
	CollisionBody* collisionBody = new CollisionBody(transform, *this, bodyID);
	EPHY_ASSERT(collisionBody != nullptr, "empty Body collision");
	// Add the collision body to the world
	m_bodies.add(collisionBody);
	// Return the pointer to the rigid body
	return collisionBody;
}

/**
 * @brief Destroy a collision body
 * @param collisionBody Pointer to the body to destroy
 */
void CollisionWorld::destroyCollisionBody(CollisionBody* collisionBody) {

	// Remove all the collision shapes of the body
	collisionBody->removeAllCollisionShapes();

	// Add the body ID to the list of free IDs
	m_freeBodiesIDs.pushBack(collisionBody->getID());

	// Remove the collision body from the list of bodies
	m_bodies.erase(m_bodies.find(collisionBody));

	delete collisionBody;
	collisionBody = nullptr;
}

// Return the next available body ID
bodyindex CollisionWorld::computeNextAvailableBodyID() {

	// Compute the body ID
	bodyindex bodyID;
	if (!m_freeBodiesIDs.empty()) {
		bodyID = m_freeBodiesIDs.back();
		m_freeBodiesIDs.popBack();
	}
	else {
		bodyID = m_currentBodyID;
		m_currentBodyID++;
	}

	return bodyID;
}

// Reset all the contact manifolds linked list of each body
void CollisionWorld::resetContactManifoldListsOfBodies() {

	// For each rigid body of the world
	for (etk::Set<CollisionBody*>::Iterator it = m_bodies.begin(); it != m_bodies.end(); ++it) {

		// Reset the contact manifold list of the body
		(*it)->resetContactManifoldsList();
	}
}

// Test if the AABBs of two bodies overlap
/**
 * @param body1 Pointer to the first body to test
 * @param body2 Pointer to the second body to test
 * @return True if the AABBs of the two bodies overlap and false otherwise
 */
bool CollisionWorld::testAABBOverlap(const CollisionBody* body1,
									 const CollisionBody* body2) const {

	// If one of the body is not active, we return no overlap
	if (!body1->isActive() || !body2->isActive()) return false;

	// Compute the AABBs of both bodies
	AABB body1AABB = body1->getAABB();
	AABB body2AABB = body2->getAABB();

	// Return true if the two AABBs overlap
	return body1AABB.testCollision(body2AABB);
}

// Test and report collisions between a given shape and all the others
// shapes of the world.
/**
 * @param shape Pointer to the proxy shape to test
 * @param callback Pointer to the object with the callback method
 */
void CollisionWorld::testCollision(const ProxyShape* shape,
								   CollisionCallback* callback) {

	// Reset all the contact manifolds lists of each body
	resetContactManifoldListsOfBodies();

	// Create the sets of shapes
	etk::Set<uint32_t> shapes;
	shapes.add(shape->m_broadPhaseID);
	etk::Set<uint32_t> emptySet;

	// Perform the collision detection and report contacts
	m_collisionDetection.testCollisionBetweenShapes(callback, shapes, emptySet);
}

// Test and report collisions between two given shapes
/**
 * @param shape1 Pointer to the first proxy shape to test
 * @param shape2 Pointer to the second proxy shape to test
 * @param callback Pointer to the object with the callback method
 */
void CollisionWorld::testCollision(const ProxyShape* shape1,
								   const ProxyShape* shape2,
								   CollisionCallback* callback) {

	// Reset all the contact manifolds lists of each body
	resetContactManifoldListsOfBodies();

	// Create the sets of shapes
	etk::Set<uint32_t> shapes1;
	shapes1.add(shape1->m_broadPhaseID);
	etk::Set<uint32_t> shapes2;
	shapes2.add(shape2->m_broadPhaseID);

	// Perform the collision detection and report contacts
	m_collisionDetection.testCollisionBetweenShapes(callback, shapes1, shapes2);
}

// Test and report collisions between a body and all the others bodies of the
// world
/**
 * @param body Pointer to the first body to test
 * @param callback Pointer to the object with the callback method
 */
void CollisionWorld::testCollision(const CollisionBody* body,
								   CollisionCallback* callback) {

	// Reset all the contact manifolds lists of each body
	resetContactManifoldListsOfBodies();

	// Create the sets of shapes
	etk::Set<uint32_t> shapes1;

	// For each shape of the body
	for (const ProxyShape* shape=body->getProxyShapesList(); shape != nullptr;
		 shape = shape->getNext()) {
		shapes1.add(shape->m_broadPhaseID);
	}

	etk::Set<uint32_t> emptySet;

	// Perform the collision detection and report contacts
	m_collisionDetection.testCollisionBetweenShapes(callback, shapes1, emptySet);
}

// Test and report collisions between two bodies
/**
 * @param body1 Pointer to the first body to test
 * @param body2 Pointer to the second body to test
 * @param callback Pointer to the object with the callback method
 */
void CollisionWorld::testCollision(const CollisionBody* body1,
								   const CollisionBody* body2,
								   CollisionCallback* callback) {

	// Reset all the contact manifolds lists of each body
	resetContactManifoldListsOfBodies();

	// Create the sets of shapes
	etk::Set<uint32_t> shapes1;
	for (const ProxyShape* shape=body1->getProxyShapesList(); shape != nullptr;
		 shape = shape->getNext()) {
		shapes1.add(shape->m_broadPhaseID);
	}

	etk::Set<uint32_t> shapes2;
	for (const ProxyShape* shape=body2->getProxyShapesList(); shape != nullptr;
		 shape = shape->getNext()) {
		shapes2.add(shape->m_broadPhaseID);
	}

	// Perform the collision detection and report contacts
	m_collisionDetection.testCollisionBetweenShapes(callback, shapes1, shapes2);
}

// Test and report collisions between all shapes of the world
/**
 * @param callback Pointer to the object with the callback method
 */
void CollisionWorld::testCollision(CollisionCallback* callback) {

	// Reset all the contact manifolds lists of each body
	resetContactManifoldListsOfBodies();

	etk::Set<uint32_t> emptySet;

	// Perform the collision detection and report contacts
	m_collisionDetection.testCollisionBetweenShapes(callback, emptySet, emptySet);
}

