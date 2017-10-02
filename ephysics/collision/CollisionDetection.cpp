/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#include <ephysics/collision/CollisionDetection.hpp>
#include <ephysics/engine/CollisionWorld.hpp>
#include <ephysics/body/Body.hpp>
#include <ephysics/collision/shapes/BoxShape.hpp>
#include <ephysics/body/RigidBody.hpp>
#include <ephysics/configuration.hpp>

// We want to use the ReactPhysics3D namespace
using namespace ephysics;
using namespace std;

// Constructor
CollisionDetection::CollisionDetection(CollisionWorld* _world):
  m_world(_world),
  m_broadPhaseAlgorithm(*this),
  m_isCollisionShapesAdded(false) {
	// Set the default collision dispatch configuration
	setCollisionDispatch(&m_defaultCollisionDispatch);
	// Fill-in the collision detection matrix with algorithms
	fillInCollisionMatrix();
}

CollisionDetection::~CollisionDetection() {
	
}

void CollisionDetection::computeCollisionDetection() {
	PROFILE("CollisionDetection::computeCollisionDetection()");
	// Compute the broad-phase collision detection
	computeBroadPhase();
	// Compute the narrow-phase collision detection
	computeNarrowPhase();
}

void CollisionDetection::testCollisionBetweenShapes(CollisionCallback* callback,
													const etk::Set<uint32_t>& shapes1,
													const etk::Set<uint32_t>& shapes2) {
	// Compute the broad-phase collision detection
	computeBroadPhase();
	// Delete all the contact points in the currently overlapping pairs
	clearContactPoints();
	// Compute the narrow-phase collision detection among given sets of shapes
	computeNarrowPhaseBetweenShapes(callback, shapes1, shapes2);
}

void CollisionDetection::reportCollisionBetweenShapes(CollisionCallback* callback,
													  const etk::Set<uint32_t>& shapes1,
													  const etk::Set<uint32_t>& shapes2) {
	// For each possible collision pair of bodies
	etk::Map<overlappingpairid, OverlappingPair*>::Iterator it;
	for (it = m_overlappingPairs.begin(); it != m_overlappingPairs.end(); ++it) {
		OverlappingPair* pair = it->second;
		const ProxyShape* shape1 = pair->getShape1();
		const ProxyShape* shape2 = pair->getShape2();
		assert(shape1->m_broadPhaseID != shape2->m_broadPhaseID);
		// If both shapes1 and shapes2 sets are non-empty, we check that
		// shape1 is among on set and shape2 is among the other one
		if (    !shapes1.empty()
		     && !shapes2.empty()
		     && (    shapes1.count(shape1->m_broadPhaseID) == 0
		          || shapes2.count(shape2->m_broadPhaseID) == 0 )
		     && (    shapes1.count(shape2->m_broadPhaseID) == 0
		          || shapes2.count(shape1->m_broadPhaseID) == 0 ) ) {
			continue;
		}
		if (    !shapes1.empty()
		     && shapes2.empty()
		     && shapes1.count(shape1->m_broadPhaseID) == 0
		     && shapes1.count(shape2->m_broadPhaseID) == 0) {
			continue;
		}
		if (    !shapes2.empty()
		     && shapes1.empty()
		     && shapes2.count(shape1->m_broadPhaseID) == 0
		     && shapes2.count(shape2->m_broadPhaseID) == 0) {
			continue;
		}
		// For each contact manifold set of the overlapping pair
		const ContactManifoldSet& manifoldSet = pair->getContactManifoldSet();
		for (int32_t j=0; j<manifoldSet.getNbContactManifolds(); j++) {
			const ContactManifold* manifold = manifoldSet.getContactManifold(j);
			// For each contact manifold of the manifold set
			for (uint32_t i=0; i<manifold->getNbContactPoints(); i++) {
				ContactPoint* contactPoint = manifold->getContactPoint(i);
				// Create the contact info object for the contact
				ContactPointInfo contactInfo(manifold->getShape1(), manifold->getShape2(),
											 manifold->getShape1()->getCollisionShape(),
											 manifold->getShape2()->getCollisionShape(),
											 contactPoint->getNormal(),
											 contactPoint->getPenetrationDepth(),
											 contactPoint->getLocalPointOnBody1(),
											 contactPoint->getLocalPointOnBody2());
				// Notify the collision callback about this new contact
				if (callback != nullptr) {
					callback->notifyContact(contactInfo);
				}
			}
		}
	}
}

void CollisionDetection::computeBroadPhase() {
	PROFILE("CollisionDetection::computeBroadPhase()");
	// If new collision shapes have been added to bodies
	if (m_isCollisionShapesAdded) {
		// Ask the broad-phase to recompute the overlapping pairs of collision
		// shapes. This call can only add new overlapping pairs in the collision
		// detection.
		m_broadPhaseAlgorithm.computeOverlappingPairs();
	}
}

void CollisionDetection::computeNarrowPhase() {
	PROFILE("CollisionDetection::computeNarrowPhase()");
	// Clear the set of overlapping pairs in narrow-phase contact
	m_contactOverlappingPairs.clear();
	// For each possible collision pair of bodies
	etk::Map<overlappingpairid, OverlappingPair*>::Iterator it;
	for (it = m_overlappingPairs.begin(); it != m_overlappingPairs.end(); ) {
		OverlappingPair* pair = it->second;
		ProxyShape* shape1 = pair->getShape1();
		ProxyShape* shape2 = pair->getShape2();
		assert(shape1->m_broadPhaseID != shape2->m_broadPhaseID);
		// Check if the collision filtering allows collision between the two shapes and
		// that the two shapes are still overlapping. Otherwise, we destroy the
		// overlapping pair
		if (((shape1->getCollideWithMaskBits() & shape2->getCollisionCategoryBits()) == 0 ||
			 (shape1->getCollisionCategoryBits() & shape2->getCollideWithMaskBits()) == 0) ||
			 !m_broadPhaseAlgorithm.testOverlappingShapes(shape1, shape2)) {
			// TODO : Remove all the contact manifold of the overlapping pair from the contact manifolds list of the two bodies involved
			// Destroy the overlapping pair
			delete it->second;
			it->second = nullptr;
			it = m_overlappingPairs.erase(it);
			continue;
		} else {
			++it;
		}
		CollisionBody* const body1 = shape1->getBody();
		CollisionBody* const body2 = shape2->getBody();
		// Update the contact cache of the overlapping pair
		pair->update();
		// Check that at least one body is awake and not static
		bool isBody1Active = !body1->isSleeping() && body1->getType() != STATIC;
		bool isBody2Active = !body2->isSleeping() && body2->getType() != STATIC;
		if (!isBody1Active && !isBody2Active) {
			continue;
		}
		// Check if the bodies are in the set of bodies that cannot collide between each other
		bodyindexpair bodiesIndex = OverlappingPair::computeBodiesIndexPair(body1, body2);
		if (m_noCollisionPairs.count(bodiesIndex) > 0) {
			continue;
		}
		// Select the narrow phase algorithm to use according to the two collision shapes
		const CollisionShapeType shape1Type = shape1->getCollisionShape()->getType();
		const CollisionShapeType shape2Type = shape2->getCollisionShape()->getType();
		NarrowPhaseAlgorithm* narrowPhaseAlgorithm = m_collisionMatrix[shape1Type][shape2Type];
		// If there is no collision algorithm between those two kinds of shapes
		if (narrowPhaseAlgorithm == nullptr) {
			continue;
		}
		// Notify the narrow-phase algorithm about the overlapping pair we are going to test
		narrowPhaseAlgorithm->setCurrentOverlappingPair(pair);
		// Create the CollisionShapeInfo objects
		CollisionShapeInfo shape1Info(shape1, shape1->getCollisionShape(), shape1->getLocalToWorldTransform(),
									  pair, shape1->getCachedCollisionData());
		CollisionShapeInfo shape2Info(shape2, shape2->getCollisionShape(), shape2->getLocalToWorldTransform(),
									  pair, shape2->getCachedCollisionData());
		
		// Use the narrow-phase collision detection algorithm to check
		// if there really is a collision. If a collision occurs, the
		// notifyContact() callback method will be called.
		narrowPhaseAlgorithm->testCollision(shape1Info, shape2Info, this);
	}
	// Add all the contact manifolds (between colliding bodies) to the bodies
	addAllContactManifoldsToBodies();
}

void CollisionDetection::computeNarrowPhaseBetweenShapes(CollisionCallback* callback,
														 const etk::Set<uint32_t>& shapes1,
														 const etk::Set<uint32_t>& shapes2) {
	m_contactOverlappingPairs.clear();
	// For each possible collision pair of bodies
	etk::Map<overlappingpairid, OverlappingPair*>::Iterator it;
	for (it = m_overlappingPairs.begin(); it != m_overlappingPairs.end(); ) {
		OverlappingPair* pair = it->second;
		ProxyShape* shape1 = pair->getShape1();
		ProxyShape* shape2 = pair->getShape2();
		assert(shape1->m_broadPhaseID != shape2->m_broadPhaseID);
		// If both shapes1 and shapes2 sets are non-empty, we check that
		// shape1 is among on set and shape2 is among the other one
		if (    !shapes1.empty()
		     && !shapes2.empty()
		     && (    shapes1.count(shape1->m_broadPhaseID) == 0
		          || shapes2.count(shape2->m_broadPhaseID) == 0 )
		     && (    shapes1.count(shape2->m_broadPhaseID) == 0
		          || shapes2.count(shape1->m_broadPhaseID) == 0 ) ) {
			++it;
			continue;
		}
		if (    !shapes1.empty()
		     && shapes2.empty()
		     && shapes1.count(shape1->m_broadPhaseID) == 0
		     && shapes1.count(shape2->m_broadPhaseID) == 0) {
			++it;
			continue;
		}
		if (    !shapes2.empty()
		     && shapes1.empty()
		     && shapes2.count(shape1->m_broadPhaseID) == 0
		     && shapes2.count(shape2->m_broadPhaseID) == 0) {
			++it;
			continue;
		}
		// Check if the collision filtering allows collision between the two shapes and
		// that the two shapes are still overlapping. Otherwise, we destroy the
		// overlapping pair
		if (((shape1->getCollideWithMaskBits() & shape2->getCollisionCategoryBits()) == 0 ||
			 (shape1->getCollisionCategoryBits() & shape2->getCollideWithMaskBits()) == 0) ||
			 !m_broadPhaseAlgorithm.testOverlappingShapes(shape1, shape2)) {
			// TODO : Remove all the contact manifold of the overlapping pair from the contact manifolds list of the two bodies involved
			// Destroy the overlapping pair
			delete it->second;
			it->second = nullptr;
			it = m_overlappingPairs.erase(it);
			continue;
		} else {
			++it;
		}
		CollisionBody* const body1 = shape1->getBody();
		CollisionBody* const body2 = shape2->getBody();
		// Update the contact cache of the overlapping pair
		pair->update();
		// Check if the two bodies are allowed to collide, otherwise, we do not test for collision
		if (body1->getType() != DYNAMIC && body2->getType() != DYNAMIC) {
			continue;
		}
		bodyindexpair bodiesIndex = OverlappingPair::computeBodiesIndexPair(body1, body2);
		if (m_noCollisionPairs.count(bodiesIndex) > 0) {
			continue;
		}
		// Check if the two bodies are sleeping, if so, we do no test collision between them
		if (body1->isSleeping() && body2->isSleeping()) {
			continue;
		}
		// Select the narrow phase algorithm to use according to the two collision shapes
		const CollisionShapeType shape1Type = shape1->getCollisionShape()->getType();
		const CollisionShapeType shape2Type = shape2->getCollisionShape()->getType();
		NarrowPhaseAlgorithm* narrowPhaseAlgorithm = m_collisionMatrix[shape1Type][shape2Type];
		// If there is no collision algorithm between those two kinds of shapes
		if (narrowPhaseAlgorithm == nullptr) {
			continue;
		}
		// Notify the narrow-phase algorithm about the overlapping pair we are going to test
		narrowPhaseAlgorithm->setCurrentOverlappingPair(pair);
		// Create the CollisionShapeInfo objects
		CollisionShapeInfo shape1Info(shape1, shape1->getCollisionShape(), shape1->getLocalToWorldTransform(),
									  pair, shape1->getCachedCollisionData());
		CollisionShapeInfo shape2Info(shape2, shape2->getCollisionShape(), shape2->getLocalToWorldTransform(),
									  pair, shape2->getCachedCollisionData());
		TestCollisionBetweenShapesCallback narrowPhaseCallback(callback);
		// Use the narrow-phase collision detection algorithm to check
		// if there really is a collision
		narrowPhaseAlgorithm->testCollision(shape1Info, shape2Info, &narrowPhaseCallback);
	}
	// Add all the contact manifolds (between colliding bodies) to the bodies
	addAllContactManifoldsToBodies();
}

void CollisionDetection::broadPhaseNotifyOverlappingPair(ProxyShape* shape1, ProxyShape* shape2) {
	assert(shape1->m_broadPhaseID != shape2->m_broadPhaseID);
	// If the two proxy collision shapes are from the same body, skip it
	if (shape1->getBody()->getID() == shape2->getBody()->getID()) {
		return;
	}
	// Check if the collision filtering allows collision between the two shapes
	if (    (shape1->getCollideWithMaskBits() & shape2->getCollisionCategoryBits()) == 0
	     || (shape1->getCollisionCategoryBits() & shape2->getCollideWithMaskBits()) == 0) {
		return;
	}
	// Compute the overlapping pair ID
	overlappingpairid pairID = OverlappingPair::computeID(shape1, shape2);
	// Check if the overlapping pair already exists
	if (m_overlappingPairs.find(pairID) != m_overlappingPairs.end()) return;
	// Compute the maximum number of contact manifolds for this pair
	int32_t nbMaxManifolds = CollisionShape::computeNbMaxContactManifolds(shape1->getCollisionShape()->getType(),
																	  shape2->getCollisionShape()->getType());
	// Create the overlapping pair and add it int32_to the set of overlapping pairs
	OverlappingPair* newPair = new OverlappingPair(shape1, shape2, nbMaxManifolds);
	assert(newPair != nullptr);
	m_overlappingPairs.set(pairID, newPair);
	// Wake up the two bodies
	shape1->getBody()->setIsSleeping(false);
	shape2->getBody()->setIsSleeping(false);
}

void CollisionDetection::removeProxyCollisionShape(ProxyShape* proxyShape) {
	// Remove all the overlapping pairs involving this proxy shape
	etk::Map<overlappingpairid, OverlappingPair*>::Iterator it;
	for (it = m_overlappingPairs.begin(); it != m_overlappingPairs.end(); ) {
		if (it->second->getShape1()->m_broadPhaseID == proxyShape->m_broadPhaseID||
			it->second->getShape2()->m_broadPhaseID == proxyShape->m_broadPhaseID) {
			// TODO : Remove all the contact manifold of the overlapping pair from the contact manifolds list of the two bodies involved
			// Destroy the overlapping pair
			delete it->second;
			it->second = nullptr;
			it = m_overlappingPairs.erase(it);
		} else {
			++it;
		}
	}
	// Remove the body from the broad-phase
	m_broadPhaseAlgorithm.removeProxyCollisionShape(proxyShape);
}

void CollisionDetection::notifyContact(OverlappingPair* overlappingPair, const ContactPointInfo& contactInfo) {
	// If it is the first contact since the pairs are overlapping
	if (overlappingPair->getNbContactPoints() == 0) {
		// Trigger a callback event
		if (m_world->m_eventListener != NULL) m_world->m_eventListener->beginContact(contactInfo);
	}
	// Create a new contact
	createContact(overlappingPair, contactInfo);
	// Trigger a callback event for the new contact
	if (m_world->m_eventListener != NULL) m_world->m_eventListener->newContact(contactInfo);
}

void CollisionDetection::createContact(OverlappingPair* overlappingPair, const ContactPointInfo& contactInfo) {
	// Create a new contact
	ContactPoint* contact = new ContactPoint(contactInfo);
	// Add the contact to the contact manifold set of the corresponding overlapping pair
	overlappingPair->addContact(contact);
	// Add the overlapping pair int32_to the set of pairs in contact during narrow-phase
	overlappingpairid pairId = OverlappingPair::computeID(overlappingPair->getShape1(),
														  overlappingPair->getShape2());
	m_contactOverlappingPairs.set(pairId, overlappingPair);
}

void CollisionDetection::addAllContactManifoldsToBodies() {
	// For each overlapping pairs in contact during the narrow-phase
	etk::Map<overlappingpairid, OverlappingPair*>::Iterator it;
	for (it = m_contactOverlappingPairs.begin(); it != m_contactOverlappingPairs.end(); ++it) {
		// Add all the contact manifolds of the pair int32_to the list of contact manifolds
		// of the two bodies involved in the contact
		addContactManifoldToBody(it->second);
	}
}

void CollisionDetection::addContactManifoldToBody(OverlappingPair* pair) {
	assert(pair != nullptr);
	CollisionBody* body1 = pair->getShape1()->getBody();
	CollisionBody* body2 = pair->getShape2()->getBody();
	const ContactManifoldSet& manifoldSet = pair->getContactManifoldSet();
	// For each contact manifold in the set of manifolds in the pair
	for (int32_t i=0; i<manifoldSet.getNbContactManifolds(); i++) {
		ContactManifold* contactManifold = manifoldSet.getContactManifold(i);
		assert(contactManifold->getNbContactPoints() > 0);
		// Add the contact manifold at the beginning of the linked
		// list of contact manifolds of the first body
		body1->m_contactManifoldsList = new ContactManifoldListElement(contactManifold, body1->m_contactManifoldsList);;
		// Add the contact manifold at the beginning of the linked
		// list of the contact manifolds of the second body
		body2->m_contactManifoldsList = new ContactManifoldListElement(contactManifold, body2->m_contactManifoldsList);;
	}
}

void CollisionDetection::clearContactPoints() {

	// For each overlapping pair
	etk::Map<overlappingpairid, OverlappingPair*>::Iterator it;
	for (it = m_overlappingPairs.begin(); it != m_overlappingPairs.end(); ++it) {
		it->second->clearContactPoints();
	}
}

void CollisionDetection::fillInCollisionMatrix() {
	// For each possible type of collision shape
	for (int32_t i=0; i<NB_COLLISION_SHAPE_TYPES; i++) {
		for (int32_t j=0; j<NB_COLLISION_SHAPE_TYPES; j++) {
			m_collisionMatrix[i][j] = m_collisionDispatch->selectAlgorithm(i, j);
		}
	}
}

EventListener* CollisionDetection::getWorldEventListener() {
   return m_world->m_eventListener;
}

void TestCollisionBetweenShapesCallback::notifyContact(OverlappingPair* _overlappingPair,
                                                       const ContactPointInfo& _contactInfo) {
	m_collisionCallback->notifyContact(_contactInfo);
}

NarrowPhaseAlgorithm* CollisionDetection::getCollisionAlgorithm(CollisionShapeType _shape1Type, CollisionShapeType _shape2Type) const {
	return m_collisionMatrix[_shape1Type][_shape2Type];
}

void CollisionDetection::setCollisionDispatch(CollisionDispatch* _collisionDispatch) {
	m_collisionDispatch = _collisionDispatch;
	m_collisionDispatch->init(this);
	// Fill-in the collision matrix with the new algorithms to use
	fillInCollisionMatrix();
}

void CollisionDetection::addProxyCollisionShape(ProxyShape* _proxyShape, const AABB& _aabb) {
	// Add the body to the broad-phase
	m_broadPhaseAlgorithm.addProxyCollisionShape(_proxyShape, _aabb);
	m_isCollisionShapesAdded = true;
}

void CollisionDetection::addNoCollisionPair(CollisionBody* body1, CollisionBody* body2) {
	m_noCollisionPairs.set(OverlappingPair::computeBodiesIndexPair(body1, body2));
}

void CollisionDetection::removeNoCollisionPair(CollisionBody* body1,
													  CollisionBody* body2) {
	m_noCollisionPairs.erase(m_noCollisionPairs.find(OverlappingPair::computeBodiesIndexPair(body1, body2)));
}

void CollisionDetection::askForBroadPhaseCollisionCheck(ProxyShape* shape) {
	m_broadPhaseAlgorithm.addMovedCollisionShape(shape->m_broadPhaseID);
}

void CollisionDetection::updateProxyCollisionShape(ProxyShape* shape, const AABB& aabb,
														  const vec3& displacement, bool forceReinsert) {
	m_broadPhaseAlgorithm.updateProxyCollisionShape(shape, aabb, displacement);
}

void CollisionDetection::raycast(RaycastCallback* raycastCallback,
										const Ray& ray,
										unsigned short raycastWithCategoryMaskBits) const {
	PROFILE("CollisionDetection::raycast()");
	RaycastTest rayCastTest(raycastCallback);
	// Ask the broad-phase algorithm to call the testRaycastAgainstShape()
	// callback method for each proxy shape hit by the ray in the broad-phase
	m_broadPhaseAlgorithm.raycast(ray, rayCastTest, raycastWithCategoryMaskBits);
}

bool CollisionDetection::testAABBOverlap(const ProxyShape* shape1,
												const ProxyShape* shape2) const {
	// If one of the shape's body is not active, we return no overlap
	if (!shape1->getBody()->isActive() || !shape2->getBody()->isActive()) {
		return false;
	}
	return m_broadPhaseAlgorithm.testOverlappingShapes(shape1, shape2);
}

CollisionWorld* CollisionDetection::getWorld() {
	return m_world;
}


