/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
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
	while(m_bodies.size() != 0) {
		destroyCollisionBody(m_bodies[0]);
	}
}


CollisionBody* CollisionWorld::createCollisionBody(const etk::Transform3D& _transform) {
	// Get the next available body ID
	bodyindex bodyID = computeNextAvailableBodyID();
	// Largest index cannot be used (it is used for invalid index)
	EPHY_ASSERT(bodyID < UINT64_MAX, "index too big");
	// Create the collision body
	CollisionBody* collisionBody = ETK_NEW(CollisionBody, _transform, *this, bodyID);
	EPHY_ASSERT(collisionBody != nullptr, "empty Body collision");
	// Add the collision body to the world
	m_bodies.add(collisionBody);
	// Return the pointer to the rigid body
	return collisionBody;
}

void CollisionWorld::destroyCollisionBody(CollisionBody* _collisionBody) {
	// Remove all the collision shapes of the body
	_collisionBody->removeAllCollisionShapes();
	// Add the body ID to the list of free IDs
	m_freeBodiesIDs.pushBack(_collisionBody->getID());
	// Remove the collision body from the list of bodies
	m_bodies.erase(m_bodies.find(_collisionBody));
	ETK_DELETE(CollisionBody, _collisionBody);
	_collisionBody = nullptr;
}

bodyindex CollisionWorld::computeNextAvailableBodyID() {
	// Compute the body ID
	bodyindex bodyID;
	if (!m_freeBodiesIDs.empty()) {
		bodyID = m_freeBodiesIDs.back();
		m_freeBodiesIDs.popBack();
	} else {
		bodyID = m_currentBodyID;
		m_currentBodyID++;
	}
	return bodyID;
}

void CollisionWorld::resetContactManifoldListsOfBodies() {
	// For each rigid body of the world
	for (etk::Set<CollisionBody*>::Iterator it = m_bodies.begin(); it != m_bodies.end(); ++it) {
		// Reset the contact manifold list of the body
		(*it)->resetContactManifoldsList();
	}
}

bool CollisionWorld::testAABBOverlap(const CollisionBody* _body1, const CollisionBody* _body2) const {
	// If one of the body is not active, we return no overlap
	if (    !_body1->isActive()
	     || !_body2->isActive()) {
		return false;
	}
	// Compute the AABBs of both bodies
	AABB body1AABB = _body1->getAABB();
	AABB body2AABB = _body2->getAABB();
	// Return true if the two AABBs overlap
	return body1AABB.testCollision(body2AABB);
}

void CollisionWorld::testCollision(const ProxyShape* _shape, CollisionCallback* _callback) {
	// Reset all the contact manifolds lists of each body
	resetContactManifoldListsOfBodies();
	// Create the sets of shapes
	etk::Set<uint32_t> shapes;
	shapes.add(_shape->m_broadPhaseID);
	etk::Set<uint32_t> emptySet;
	// Perform the collision detection and report contacts
	m_collisionDetection.testCollisionBetweenShapes(_callback, shapes, emptySet);
}

void CollisionWorld::testCollision(const ProxyShape* _shape1, const ProxyShape* _shape2, CollisionCallback* _callback) {
	// Reset all the contact manifolds lists of each body
	resetContactManifoldListsOfBodies();
	// Create the sets of shapes
	etk::Set<uint32_t> shapes1;
	shapes1.add(_shape1->m_broadPhaseID);
	etk::Set<uint32_t> shapes2;
	shapes2.add(_shape2->m_broadPhaseID);
	// Perform the collision detection and report contacts
	m_collisionDetection.testCollisionBetweenShapes(_callback, shapes1, shapes2);
}

void CollisionWorld::testCollision(const CollisionBody* _body, CollisionCallback* _callback) {
	// Reset all the contact manifolds lists of each body
	resetContactManifoldListsOfBodies();
	// Create the sets of shapes
	etk::Set<uint32_t> shapes1;
	// For each shape of the body
	for (const ProxyShape* shape = _body->getProxyShapesList();
	     shape != nullptr;
	     shape = shape->getNext()) {
		shapes1.add(shape->m_broadPhaseID);
	}
	etk::Set<uint32_t> emptySet;
	// Perform the collision detection and report contacts
	m_collisionDetection.testCollisionBetweenShapes(_callback, shapes1, emptySet);
}

void CollisionWorld::testCollision(const CollisionBody* _body1, const CollisionBody* _body2, CollisionCallback* _callback) {
	// Reset all the contact manifolds lists of each body
	resetContactManifoldListsOfBodies();
	// Create the sets of shapes
	etk::Set<uint32_t> shapes1;
	for (const ProxyShape* shape = _body1->getProxyShapesList();
	     shape != nullptr;
	     shape = shape->getNext()) {
		shapes1.add(shape->m_broadPhaseID);
	}
	etk::Set<uint32_t> shapes2;
	for (const ProxyShape* shape = _body2->getProxyShapesList();
	     shape != nullptr;
	     shape = shape->getNext()) {
		shapes2.add(shape->m_broadPhaseID);
	}
	// Perform the collision detection and report contacts
	m_collisionDetection.testCollisionBetweenShapes(_callback, shapes1, shapes2);
}

void CollisionWorld::testCollision(CollisionCallback* _callback) {
	// Reset all the contact manifolds lists of each body
	resetContactManifoldListsOfBodies();
	etk::Set<uint32_t> emptySet;
	// Perform the collision detection and report contacts
	m_collisionDetection.testCollisionBetweenShapes(_callback, emptySet, emptySet);
}

