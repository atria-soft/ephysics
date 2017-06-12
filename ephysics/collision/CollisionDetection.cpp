/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/CollisionDetection.h>
#include <ephysics/engine/CollisionWorld.h>
#include <ephysics/body/Body.h>
#include <ephysics/collision/shapes/BoxShape.h>
#include <ephysics/body/RigidBody.h>
#include <ephysics/configuration.h>
#include <cassert>
#include <complex>
#include <set>
#include <utility>
#include <utility>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;
using namespace std;

// Constructor
CollisionDetection::CollisionDetection(CollisionWorld* world, MemoryAllocator& memoryAllocator)
				   : m_memoryAllocator(memoryAllocator),
					 m_world(world), m_broadPhaseAlgorithm(*this),
					 m_isCollisionShapesAdded(false) {

	// Set the default collision dispatch configuration
	setCollisionDispatch(&m_defaultCollisionDispatch);

	// Fill-in the collision detection matrix with algorithms
	fillInCollisionMatrix();
}

// Destructor
CollisionDetection::~CollisionDetection() {

}

// Compute the collision detection
void CollisionDetection::computeCollisionDetection() {

	PROFILE("CollisionDetection::computeCollisionDetection()");
		
	// Compute the broad-phase collision detection
	computeBroadPhase();
	
	// Compute the narrow-phase collision detection
	computeNarrowPhase();
}

// Compute the collision detection
void CollisionDetection::testCollisionBetweenShapes(CollisionCallback* callback,
													const std::set<uint32_t>& shapes1,
													const std::set<uint32_t>& shapes2) {

	// Compute the broad-phase collision detection
	computeBroadPhase();

	// Delete all the contact points in the currently overlapping pairs
	clearContactPoints();

	// Compute the narrow-phase collision detection among given sets of shapes
	computeNarrowPhaseBetweenShapes(callback, shapes1, shapes2);
}

// Report collision between two sets of shapes
void CollisionDetection::reportCollisionBetweenShapes(CollisionCallback* callback,
													  const std::set<uint32_t>& shapes1,
													  const std::set<uint32_t>& shapes2) {

	// For each possible collision pair of bodies
	map<overlappingpairid, OverlappingPair*>::iterator it;
	for (it = m_overlappingPairs.begin(); it != m_overlappingPairs.end(); ++it) {

		OverlappingPair* pair = it->second;

		const ProxyShape* shape1 = pair->getShape1();
		const ProxyShape* shape2 = pair->getShape2();

		assert(shape1->m_broadPhaseID != shape2->m_broadPhaseID);

		// If both shapes1 and shapes2 sets are non-empty, we check that
		// shape1 is among on set and shape2 is among the other one
		if (!shapes1.empty() && !shapes2.empty() &&
			(shapes1.count(shape1->m_broadPhaseID) == 0 || shapes2.count(shape2->m_broadPhaseID) == 0) &&
			(shapes1.count(shape2->m_broadPhaseID) == 0 || shapes2.count(shape1->m_broadPhaseID) == 0)) {
			continue;
		}
		if (!shapes1.empty() && shapes2.empty() &&
			shapes1.count(shape1->m_broadPhaseID) == 0 && shapes1.count(shape2->m_broadPhaseID) == 0)
		{
			continue;
		}
		if (!shapes2.empty() && shapes1.empty() &&
			shapes2.count(shape1->m_broadPhaseID) == 0 && shapes2.count(shape2->m_broadPhaseID) == 0)
		{
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
				if (callback != NULL) callback->notifyContact(contactInfo);
			}
		}
	}
}

// Compute the broad-phase collision detection
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

// Compute the narrow-phase collision detection
void CollisionDetection::computeNarrowPhase() {

	PROFILE("CollisionDetection::computeNarrowPhase()");

	// Clear the set of overlapping pairs in narrow-phase contact
	m_contactOverlappingPairs.clear();
	
	// For each possible collision pair of bodies
	map<overlappingpairid, OverlappingPair*>::iterator it;
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

			std::map<overlappingpairid, OverlappingPair*>::iterator itToRemove = it;
			++it;

			// TODO : Remove all the contact manifold of the overlapping pair from the contact manifolds list of the two bodies involved

			// Destroy the overlapping pair
			itToRemove->second->~OverlappingPair();
			m_world->m_memoryAllocator.release(itToRemove->second, sizeof(OverlappingPair));
			m_overlappingPairs.erase(itToRemove);
			continue;
		}
		else {
			++it;
		}

		CollisionBody* const body1 = shape1->getBody();
		CollisionBody* const body2 = shape2->getBody();
		
		// Update the contact cache of the overlapping pair
		pair->update();

		// Check that at least one body is awake and not static
		bool isBody1Active = !body1->isSleeping() && body1->getType() != STATIC;
		bool isBody2Active = !body2->isSleeping() && body2->getType() != STATIC;
		if (!isBody1Active && !isBody2Active) continue;

		// Check if the bodies are in the set of bodies that cannot collide between each other
		bodyindexpair bodiesIndex = OverlappingPair::computeBodiesIndexPair(body1, body2);
		if (m_noCollisionPairs.count(bodiesIndex) > 0) continue;
		
		// Select the narrow phase algorithm to use according to the two collision shapes
		const CollisionShapeType shape1Type = shape1->getCollisionShape()->getType();
		const CollisionShapeType shape2Type = shape2->getCollisionShape()->getType();
		NarrowPhaseAlgorithm* narrowPhaseAlgorithm = m_collisionMatrix[shape1Type][shape2Type];

		// If there is no collision algorithm between those two kinds of shapes
		if (narrowPhaseAlgorithm == NULL) continue;
		
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

// Compute the narrow-phase collision detection
void CollisionDetection::computeNarrowPhaseBetweenShapes(CollisionCallback* callback,
														 const std::set<uint32_t>& shapes1,
														 const std::set<uint32_t>& shapes2) {

	m_contactOverlappingPairs.clear();

	// For each possible collision pair of bodies
	map<overlappingpairid, OverlappingPair*>::iterator it;
	for (it = m_overlappingPairs.begin(); it != m_overlappingPairs.end(); ) {

		OverlappingPair* pair = it->second;

		ProxyShape* shape1 = pair->getShape1();
		ProxyShape* shape2 = pair->getShape2();

		assert(shape1->m_broadPhaseID != shape2->m_broadPhaseID);

		// If both shapes1 and shapes2 sets are non-empty, we check that
		// shape1 is among on set and shape2 is among the other one
		if (!shapes1.empty() && !shapes2.empty() &&
			(shapes1.count(shape1->m_broadPhaseID) == 0 || shapes2.count(shape2->m_broadPhaseID) == 0) &&
			(shapes1.count(shape2->m_broadPhaseID) == 0 || shapes2.count(shape1->m_broadPhaseID) == 0)) {
			++it;
			continue;
		}
		if (!shapes1.empty() && shapes2.empty() &&
			shapes1.count(shape1->m_broadPhaseID) == 0 && shapes1.count(shape2->m_broadPhaseID) == 0)
		{
			++it;
			continue;
		}
		if (!shapes2.empty() && shapes1.empty() &&
			shapes2.count(shape1->m_broadPhaseID) == 0 && shapes2.count(shape2->m_broadPhaseID) == 0)
		{
			++it;
			continue;
		}

		// Check if the collision filtering allows collision between the two shapes and
		// that the two shapes are still overlapping. Otherwise, we destroy the
		// overlapping pair
		if (((shape1->getCollideWithMaskBits() & shape2->getCollisionCategoryBits()) == 0 ||
			 (shape1->getCollisionCategoryBits() & shape2->getCollideWithMaskBits()) == 0) ||
			 !m_broadPhaseAlgorithm.testOverlappingShapes(shape1, shape2)) {

			std::map<overlappingpairid, OverlappingPair*>::iterator itToRemove = it;
			++it;

			// TODO : Remove all the contact manifold of the overlapping pair from the contact manifolds list of the two bodies involved

			// Destroy the overlapping pair
			itToRemove->second->~OverlappingPair();
			m_world->m_memoryAllocator.release(itToRemove->second, sizeof(OverlappingPair));
			m_overlappingPairs.erase(itToRemove);
			continue;
		}
		else {
			++it;
		}

		CollisionBody* const body1 = shape1->getBody();
		CollisionBody* const body2 = shape2->getBody();

		// Update the contact cache of the overlapping pair
		pair->update();

		// Check if the two bodies are allowed to collide, otherwise, we do not test for collision
		if (body1->getType() != DYNAMIC && body2->getType() != DYNAMIC) continue;
		bodyindexpair bodiesIndex = OverlappingPair::computeBodiesIndexPair(body1, body2);
		if (m_noCollisionPairs.count(bodiesIndex) > 0) continue;

		// Check if the two bodies are sleeping, if so, we do no test collision between them
		if (body1->isSleeping() && body2->isSleeping()) continue;

		// Select the narrow phase algorithm to use according to the two collision shapes
		const CollisionShapeType shape1Type = shape1->getCollisionShape()->getType();
		const CollisionShapeType shape2Type = shape2->getCollisionShape()->getType();
		NarrowPhaseAlgorithm* narrowPhaseAlgorithm = m_collisionMatrix[shape1Type][shape2Type];

		// If there is no collision algorithm between those two kinds of shapes
		if (narrowPhaseAlgorithm == NULL) continue;

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

// Allow the broadphase to notify the collision detection about an overlapping pair.
/// This method is called by the broad-phase collision detection algorithm
void CollisionDetection::broadPhaseNotifyOverlappingPair(ProxyShape* shape1, ProxyShape* shape2) {

	assert(shape1->m_broadPhaseID != shape2->m_broadPhaseID);

	// If the two proxy collision shapes are from the same body, skip it
	if (shape1->getBody()->getID() == shape2->getBody()->getID()) return;

	// Check if the collision filtering allows collision between the two shapes
	if ((shape1->getCollideWithMaskBits() & shape2->getCollisionCategoryBits()) == 0 ||
		(shape1->getCollisionCategoryBits() & shape2->getCollideWithMaskBits()) == 0) return;

	// Compute the overlapping pair ID
	overlappingpairid pairID = OverlappingPair::computeID(shape1, shape2);

	// Check if the overlapping pair already exists
	if (m_overlappingPairs.find(pairID) != m_overlappingPairs.end()) return;

	// Compute the maximum number of contact manifolds for this pair
	int32_t nbMaxManifolds = CollisionShape::computeNbMaxContactManifolds(shape1->getCollisionShape()->getType(),
																	  shape2->getCollisionShape()->getType());

	// Create the overlapping pair and add it int32_to the set of overlapping pairs
	OverlappingPair* newPair = new (m_world->m_memoryAllocator.allocate(sizeof(OverlappingPair)))
							  OverlappingPair(shape1, shape2, nbMaxManifolds, m_world->m_memoryAllocator);
	assert(newPair != NULL);

#ifndef NDEBUG
	std::pair<map<overlappingpairid, OverlappingPair*>::iterator, bool> check =
#endif
	m_overlappingPairs.insert(make_pair(pairID, newPair));
	assert(check.second);

	// Wake up the two bodies
	shape1->getBody()->setIsSleeping(false);
	shape2->getBody()->setIsSleeping(false);
}

// Remove a body from the collision detection
void CollisionDetection::removeProxyCollisionShape(ProxyShape* proxyShape) {

	// Remove all the overlapping pairs involving this proxy shape
	std::map<overlappingpairid, OverlappingPair*>::iterator it;
	for (it = m_overlappingPairs.begin(); it != m_overlappingPairs.end(); ) {
		if (it->second->getShape1()->m_broadPhaseID == proxyShape->m_broadPhaseID||
			it->second->getShape2()->m_broadPhaseID == proxyShape->m_broadPhaseID) {
			std::map<overlappingpairid, OverlappingPair*>::iterator itToRemove = it;
			++it;

			// TODO : Remove all the contact manifold of the overlapping pair from the contact manifolds list of the two bodies involved

			// Destroy the overlapping pair
			itToRemove->second->~OverlappingPair();
			m_world->m_memoryAllocator.release(itToRemove->second, sizeof(OverlappingPair));
			m_overlappingPairs.erase(itToRemove);
		}
		else {
			++it;
		}
	}

	// Remove the body from the broad-phase
	m_broadPhaseAlgorithm.removeProxyCollisionShape(proxyShape);
}

// Called by a narrow-phase collision algorithm when a new contact has been found
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

// Create a new contact
void CollisionDetection::createContact(OverlappingPair* overlappingPair,
									   const ContactPointInfo& contactInfo) {

	// Create a new contact
	ContactPoint* contact = new (m_world->m_memoryAllocator.allocate(sizeof(ContactPoint)))
								 ContactPoint(contactInfo);

	// Add the contact to the contact manifold set of the corresponding overlapping pair
	overlappingPair->addContact(contact);

	// Add the overlapping pair int32_to the set of pairs in contact during narrow-phase
	overlappingpairid pairId = OverlappingPair::computeID(overlappingPair->getShape1(),
														  overlappingPair->getShape2());
	m_contactOverlappingPairs[pairId] = overlappingPair;
}

void CollisionDetection::addAllContactManifoldsToBodies() {

	// For each overlapping pairs in contact during the narrow-phase
	std::map<overlappingpairid, OverlappingPair*>::iterator it;
	for (it = m_contactOverlappingPairs.begin(); it != m_contactOverlappingPairs.end(); ++it) {

		// Add all the contact manifolds of the pair int32_to the list of contact manifolds
		// of the two bodies involved in the contact
		addContactManifoldToBody(it->second);
	}
}

// Add a contact manifold to the linked list of contact manifolds of the two bodies involved
// in the corresponding contact
void CollisionDetection::addContactManifoldToBody(OverlappingPair* pair) {

	assert(pair != NULL);

	CollisionBody* body1 = pair->getShape1()->getBody();
	CollisionBody* body2 = pair->getShape2()->getBody();
	const ContactManifoldSet& manifoldSet = pair->getContactManifoldSet();

	// For each contact manifold in the set of manifolds in the pair
	for (int32_t i=0; i<manifoldSet.getNbContactManifolds(); i++) {

		ContactManifold* contactManifold = manifoldSet.getContactManifold(i);

		assert(contactManifold->getNbContactPoints() > 0);

		// Add the contact manifold at the beginning of the linked
		// list of contact manifolds of the first body
		void* allocatedMemory1 = m_world->m_memoryAllocator.allocate(sizeof(ContactManifoldListElement));
		ContactManifoldListElement* listElement1 = new (allocatedMemory1)
													  ContactManifoldListElement(contactManifold,
																		 body1->m_contactManifoldsList);
		body1->m_contactManifoldsList = listElement1;

		// Add the contact manifold at the beginning of the linked
		// list of the contact manifolds of the second body
		void* allocatedMemory2 = m_world->m_memoryAllocator.allocate(sizeof(ContactManifoldListElement));
		ContactManifoldListElement* listElement2 = new (allocatedMemory2)
													  ContactManifoldListElement(contactManifold,
																		 body2->m_contactManifoldsList);
		body2->m_contactManifoldsList = listElement2;
	}
}

// Delete all the contact points in the currently overlapping pairs
void CollisionDetection::clearContactPoints() {

	// For each overlapping pair
	std::map<overlappingpairid, OverlappingPair*>::iterator it;
	for (it = m_overlappingPairs.begin(); it != m_overlappingPairs.end(); ++it) {
		it->second->clearContactPoints();
	}
}

// Fill-in the collision detection matrix
void CollisionDetection::fillInCollisionMatrix() {

	// For each possible type of collision shape
	for (int32_t i=0; i<NB_COLLISION_SHAPE_TYPES; i++) {
		for (int32_t j=0; j<NB_COLLISION_SHAPE_TYPES; j++) {
			m_collisionMatrix[i][j] = m_collisionDispatch->selectAlgorithm(i, j);
		}
	}
}

// Return the world event listener
EventListener* CollisionDetection::getWorldEventListener() {
   return m_world->m_eventListener;
}

/// Return a reference to the world memory allocator
MemoryAllocator& CollisionDetection::getWorldMemoryAllocator() {
  return m_world->m_memoryAllocator;
}

// Called by a narrow-phase collision algorithm when a new contact has been found
void TestCollisionBetweenShapesCallback::notifyContact(OverlappingPair* overlappingPair,
						   const ContactPointInfo& contactInfo) {
	m_collisionCallback->notifyContact(contactInfo);
}
