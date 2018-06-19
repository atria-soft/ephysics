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

void CollisionDetection::testCollisionBetweenShapes(CollisionCallback* _callback, const etk::Set<uint32_t>& _shapes1, const etk::Set<uint32_t>& _shapes2) {
	// Compute the broad-phase collision detection
	computeBroadPhase();
	// Delete all the contact points in the currently overlapping pairs
	clearContactPoints();
	// Compute the narrow-phase collision detection among given sets of shapes
	computeNarrowPhaseBetweenShapes(_callback, _shapes1, _shapes2);
}

void CollisionDetection::reportCollisionBetweenShapes(CollisionCallback* _callback, const etk::Set<uint32_t>& _shapes1, const etk::Set<uint32_t>& _shapes2) {
	// For each possible collision pair of bodies
	etk::Map<overlappingpairid, OverlappingPair*>::Iterator it;
	for (it = m_overlappingPairs.begin(); it != m_overlappingPairs.end(); ++it) {
		OverlappingPair* pair = it->second;
		const ProxyShape* shape1 = pair->getShape1();
		const ProxyShape* shape2 = pair->getShape2();
		assert(shape1->m_broadPhaseID != shape2->m_broadPhaseID);
		// If both shapes1 and shapes2 sets are non-empty, we check that
		// shape1 is among on set and shape2 is among the other one
		if (    !_shapes1.empty()
		     && !_shapes2.empty()
		     && (    _shapes1.count(shape1->m_broadPhaseID) == 0
		          || _shapes2.count(shape2->m_broadPhaseID) == 0 )
		     && (    _shapes1.count(shape2->m_broadPhaseID) == 0
		          || _shapes2.count(shape1->m_broadPhaseID) == 0 ) ) {
			continue;
		}
		if (    !_shapes1.empty()
		     && _shapes2.empty()
		     && _shapes1.count(shape1->m_broadPhaseID) == 0
		     && _shapes1.count(shape2->m_broadPhaseID) == 0) {
			continue;
		}
		if (    !_shapes2.empty()
		     && _shapes1.empty()
		     && _shapes2.count(shape1->m_broadPhaseID) == 0
		     && _shapes2.count(shape2->m_broadPhaseID) == 0) {
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
				if (_callback != null) {
					_callback->notifyContact(contactInfo);
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
			ETK_DELETE(OverlappingPair, it->second);
			it->second = null;
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
		if (narrowPhaseAlgorithm == null) {
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

void CollisionDetection::computeNarrowPhaseBetweenShapes(CollisionCallback* _callback, const etk::Set<uint32_t>& _shapes1, const etk::Set<uint32_t>& _shapes2) {
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
		if (    !_shapes1.empty()
		     && !_shapes2.empty()
		     && (    _shapes1.count(shape1->m_broadPhaseID) == 0
		          || _shapes2.count(shape2->m_broadPhaseID) == 0 )
		     && (    _shapes1.count(shape2->m_broadPhaseID) == 0
		          || _shapes2.count(shape1->m_broadPhaseID) == 0 ) ) {
			++it;
			continue;
		}
		if (    !_shapes1.empty()
		     && _shapes2.empty()
		     && _shapes1.count(shape1->m_broadPhaseID) == 0
		     && _shapes1.count(shape2->m_broadPhaseID) == 0) {
			++it;
			continue;
		}
		if (    !_shapes2.empty()
		     && _shapes1.empty()
		     && _shapes2.count(shape1->m_broadPhaseID) == 0
		     && _shapes2.count(shape2->m_broadPhaseID) == 0) {
			++it;
			continue;
		}
		// Check if the collision filtering allows collision between the two shapes and
		// that the two shapes are still overlapping. Otherwise, we destroy the
		// overlapping pair
		if (    (    (shape1->getCollideWithMaskBits() & shape2->getCollisionCategoryBits()) == 0
		          || (shape1->getCollisionCategoryBits() & shape2->getCollideWithMaskBits()) == 0 )
		     || !m_broadPhaseAlgorithm.testOverlappingShapes(shape1, shape2) ) {
			// TODO : Remove all the contact manifold of the overlapping pair from the contact manifolds list of the two bodies involved
			// Destroy the overlapping pair
			ETK_DELETE(OverlappingPair, it->second);
			it->second = null;
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
		if (narrowPhaseAlgorithm == null) {
			continue;
		}
		// Notify the narrow-phase algorithm about the overlapping pair we are going to test
		narrowPhaseAlgorithm->setCurrentOverlappingPair(pair);
		// Create the CollisionShapeInfo objects
		CollisionShapeInfo shape1Info(shape1,
		                              shape1->getCollisionShape(),
		                              shape1->getLocalToWorldTransform(),
		                              pair,
		                              shape1->getCachedCollisionData());
		CollisionShapeInfo shape2Info(shape2,
		                              shape2->getCollisionShape(),
		                              shape2->getLocalToWorldTransform(),
		                              pair,
		                              shape2->getCachedCollisionData());
		TestCollisionBetweenShapesCallback narrowPhaseCallback(_callback);
		// Use the narrow-phase collision detection algorithm to check
		// if there really is a collision
		narrowPhaseAlgorithm->testCollision(shape1Info, shape2Info, &narrowPhaseCallback);
	}
	// Add all the contact manifolds (between colliding bodies) to the bodies
	addAllContactManifoldsToBodies();
}

void CollisionDetection::broadPhaseNotifyOverlappingPair(ProxyShape* _shape1, ProxyShape* _shape2) {
	assert(_shape1->m_broadPhaseID != _shape2->m_broadPhaseID);
	// If the two proxy collision shapes are from the same body, skip it
	if (_shape1->getBody()->getID() == _shape2->getBody()->getID()) {
		return;
	}
	// Check if the collision filtering allows collision between the two shapes
	if (    (_shape1->getCollideWithMaskBits() & _shape2->getCollisionCategoryBits()) == 0
	     || (_shape1->getCollisionCategoryBits() & _shape2->getCollideWithMaskBits()) == 0) {
		return;
	}
	// Compute the overlapping pair ID
	overlappingpairid pairID = OverlappingPair::computeID(_shape1, _shape2);
	// Check if the overlapping pair already exists
	if (m_overlappingPairs.find(pairID) != m_overlappingPairs.end()) return;
	// Compute the maximum number of contact manifolds for this pair
	int32_t nbMaxManifolds = CollisionShape::computeNbMaxContactManifolds(_shape1->getCollisionShape()->getType(),
	                                                                      _shape2->getCollisionShape()->getType());
	// Create the overlapping pair and add it int32_to the set of overlapping pairs
	OverlappingPair* newPair = ETK_NEW(OverlappingPair, _shape1, _shape2, nbMaxManifolds);
	assert(newPair != null);
	m_overlappingPairs.set(pairID, newPair);
	// Wake up the two bodies
	_shape1->getBody()->setIsSleeping(false);
	_shape2->getBody()->setIsSleeping(false);
}

void CollisionDetection::removeProxyCollisionShape(ProxyShape* _proxyShape) {
	// Remove all the overlapping pairs involving this proxy shape
	etk::Map<overlappingpairid, OverlappingPair*>::Iterator it;
	for (it = m_overlappingPairs.begin(); it != m_overlappingPairs.end(); ) {
		if (it->second->getShape1()->m_broadPhaseID == _proxyShape->m_broadPhaseID||
			it->second->getShape2()->m_broadPhaseID == _proxyShape->m_broadPhaseID) {
			// TODO : Remove all the contact manifold of the overlapping pair from the contact manifolds list of the two bodies involved
			// Destroy the overlapping pair
			ETK_DELETE(OverlappingPair, it->second);
			it->second = null;
			it = m_overlappingPairs.erase(it);
		} else {
			++it;
		}
	}
	// Remove the body from the broad-phase
	m_broadPhaseAlgorithm.removeProxyCollisionShape(_proxyShape);
}

void CollisionDetection::notifyContact(OverlappingPair* _overlappingPair, const ContactPointInfo& _contactInfo) {
	// If it is the first contact since the pairs are overlapping
	if (_overlappingPair->getNbContactPoints() == 0) {
		// Trigger a callback event
		if (m_world->m_eventListener != NULL) {
			m_world->m_eventListener->beginContact(_contactInfo);
		}
	}
	// Create a new contact
	createContact(_overlappingPair, _contactInfo);
	// Trigger a callback event for the new contact
	if (m_world->m_eventListener != NULL) {
		m_world->m_eventListener->newContact(_contactInfo);
	}
}

void CollisionDetection::createContact(OverlappingPair* _overlappingPair, const ContactPointInfo& _contactInfo) {
	// Create a new contact
	ContactPoint* contact = ETK_NEW(ContactPoint, _contactInfo);
	// Add the contact to the contact manifold set of the corresponding overlapping pair
	_overlappingPair->addContact(contact);
	// Add the overlapping pair int32_to the set of pairs in contact during narrow-phase
	overlappingpairid pairId = OverlappingPair::computeID(_overlappingPair->getShape1(),
	                                                      _overlappingPair->getShape2());
	m_contactOverlappingPairs.set(pairId, _overlappingPair);
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

void CollisionDetection::addContactManifoldToBody(OverlappingPair* _pair) {
	assert(_pair != null);
	CollisionBody* body1 = _pair->getShape1()->getBody();
	CollisionBody* body2 = _pair->getShape2()->getBody();
	const ContactManifoldSet& manifoldSet = _pair->getContactManifoldSet();
	// For each contact manifold in the set of manifolds in the pair
	for (int32_t i=0; i<manifoldSet.getNbContactManifolds(); i++) {
		ContactManifold* contactManifold = manifoldSet.getContactManifold(i);
		assert(contactManifold->getNbContactPoints() > 0);
		// Add the contact manifold at the beginning of the linked
		// list of contact manifolds of the first body
		body1->m_contactManifoldsList = ETK_NEW(ContactManifoldListElement, contactManifold, body1->m_contactManifoldsList);;
		// Add the contact manifold at the beginning of the linked
		// list of the contact manifolds of the second body
		body2->m_contactManifoldsList = ETK_NEW(ContactManifoldListElement, contactManifold, body2->m_contactManifoldsList);;
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

void TestCollisionBetweenShapesCallback::notifyContact(OverlappingPair* _overlappingPair, const ContactPointInfo& _contactInfo) {
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

void CollisionDetection::addNoCollisionPair(CollisionBody* _body1, CollisionBody* _body2) {
	m_noCollisionPairs.set(OverlappingPair::computeBodiesIndexPair(_body1, _body2));
}

void CollisionDetection::removeNoCollisionPair(CollisionBody* _body1, CollisionBody* _body2) {
	m_noCollisionPairs.erase(m_noCollisionPairs.find(OverlappingPair::computeBodiesIndexPair(_body1, _body2)));
}

void CollisionDetection::askForBroadPhaseCollisionCheck(ProxyShape* _shape) {
	m_broadPhaseAlgorithm.addMovedCollisionShape(_shape->m_broadPhaseID);
}

void CollisionDetection::updateProxyCollisionShape(ProxyShape* _shape, const AABB& _aabb, const vec3& _displacement, bool _forceReinsert) {
	m_broadPhaseAlgorithm.updateProxyCollisionShape(_shape, _aabb, _displacement);
}

void CollisionDetection::raycast(RaycastCallback* _raycastCallback, const Ray& _ray, unsigned short _raycastWithCategoryMaskBits) const {
	PROFILE("CollisionDetection::raycast()");
	RaycastTest rayCastTest(_raycastCallback);
	// Ask the broad-phase algorithm to call the testRaycastAgainstShape()
	// callback method for each proxy shape hit by the ray in the broad-phase
	m_broadPhaseAlgorithm.raycast(_ray, rayCastTest, _raycastWithCategoryMaskBits);
}

bool CollisionDetection::testAABBOverlap(const ProxyShape* _shape1, const ProxyShape* _shape2) const {
	// If one of the shape's body is not active, we return no overlap
	if (    !_shape1->getBody()->isActive()
	     || !_shape2->getBody()->isActive()) {
		return false;
	}
	return m_broadPhaseAlgorithm.testOverlappingShapes(_shape1, _shape2);
}

CollisionWorld* CollisionDetection::getWorld() {
	return m_world;
}


