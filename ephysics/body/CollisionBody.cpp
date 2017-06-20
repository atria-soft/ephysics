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


CollisionBody::CollisionBody(const etk::Transform3D& _transform, CollisionWorld& _world, bodyindex _id):
  Body(_id),
  m_type(DYNAMIC),
  m_transform(_transform),
  m_proxyCollisionShapes(nullptr),
  m_numberCollisionShapes(0),
  m_contactManifoldsList(nullptr),
  m_world(_world) {
	
}

CollisionBody::~CollisionBody() {
	assert(m_contactManifoldsList == nullptr);

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


ProxyShape* CollisionBody::addCollisionShape(CollisionShape* _collisionShape,
                                             const etk::Transform3D& _transform) {
	// Create a new proxy collision shape to attach the collision shape to the body
	ProxyShape* proxyShape = new (m_world.m_memoryAllocator.allocate(sizeof(ProxyShape))) ProxyShape(this, _collisionShape,_transform, float(1));
	// Add it to the list of proxy collision shapes of the body
	if (m_proxyCollisionShapes == nullptr) {
		m_proxyCollisionShapes = proxyShape;
	} else {
		proxyShape->m_next = m_proxyCollisionShapes;
		m_proxyCollisionShapes = proxyShape;
	}
	AABB aabb;
	_collisionShape->computeAABB(aabb, m_transform * _transform);
	m_world.m_collisionDetection.addProxyCollisionShape(proxyShape, aabb);
	m_numberCollisionShapes++;
	return proxyShape;
}

void CollisionBody::removeCollisionShape(const ProxyShape* _proxyShape) {
	ProxyShape* current = m_proxyCollisionShapes;
	// If the the first proxy shape is the one to remove
	if (current == _proxyShape) {
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
	while(current->m_next != nullptr) {
		// If we have found the collision shape to remove
		if (current->m_next == _proxyShape) {
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


void CollisionBody::removeAllCollisionShapes() {
	ProxyShape* current = m_proxyCollisionShapes;
	// Look for the proxy shape that contains the collision shape in parameter
	while(current != nullptr) {
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
	m_proxyCollisionShapes = nullptr;
}


void CollisionBody::resetContactManifoldsList() {
	// Delete the linked list of contact manifolds of that body
	ContactManifoldListElement* currentElement = m_contactManifoldsList;
	while (currentElement != nullptr) {
		ContactManifoldListElement* nextElement = currentElement->next;
		// Delete the current element
		currentElement->~ContactManifoldListElement();
		m_world.m_memoryAllocator.release(currentElement, sizeof(ContactManifoldListElement));
		currentElement = nextElement;
	}
	m_contactManifoldsList = nullptr;
}


void CollisionBody::updateBroadPhaseState() const {
	// For all the proxy collision shapes of the body
	for (ProxyShape* shape = m_proxyCollisionShapes; shape != nullptr; shape = shape->m_next) {
		// Update the proxy
		updateProxyShapeInBroadPhase(shape);
	}
}


void CollisionBody::updateProxyShapeInBroadPhase(ProxyShape* _proxyShape, bool _forceReinsert) const {
	AABB aabb;
	_proxyShape->getCollisionShape()->computeAABB(aabb, m_transform * _proxyShape->getLocalToBodyTransform());
	m_world.m_collisionDetection.updateProxyCollisionShape(_proxyShape, aabb, vec3(0, 0, 0), _forceReinsert);
}


void CollisionBody::setIsActive(bool _isActive) {
	// If the state does not change
	if (m_isActive == _isActive) {
		return;
	}
	Body::setIsActive(_isActive);
	// If we have to activate the body
	if (_isActive == true) {
		for (ProxyShape* shape = m_proxyCollisionShapes; shape != nullptr; shape = shape->m_next) {
			AABB aabb;
			shape->getCollisionShape()->computeAABB(aabb, m_transform * shape->m_localToBodyTransform);
			m_world.m_collisionDetection.addProxyCollisionShape(shape, aabb);
		}
	} else {
		for (ProxyShape* shape = m_proxyCollisionShapes; shape != nullptr; shape = shape->m_next) {
			m_world.m_collisionDetection.removeProxyCollisionShape(shape);
		}
		resetContactManifoldsList();
	}
}


void CollisionBody::askForBroadPhaseCollisionCheck() const {
	for (ProxyShape* shape = m_proxyCollisionShapes; shape != nullptr; shape = shape->m_next) {
		m_world.m_collisionDetection.askForBroadPhaseCollisionCheck(shape);  
	}
}


int32_t CollisionBody::resetIsAlreadyInIslandAndCountManifolds() {
	m_isAlreadyInIsland = false;
	int32_t nbManifolds = 0;
	// Reset the m_isAlreadyInIsland variable of the contact manifolds for this body
	ContactManifoldListElement* currentElement = m_contactManifoldsList;
	while (currentElement != nullptr) {
		currentElement->contactManifold->m_isAlreadyInIsland = false;
		currentElement = currentElement->next;
		nbManifolds++;
	}
	return nbManifolds;
}

bool CollisionBody::testPointInside(const vec3& _worldPoint) const {
	for (ProxyShape* shape = m_proxyCollisionShapes; shape != nullptr; shape = shape->m_next) {
		if (shape->testPointInside(_worldPoint)) return true;
	}
	return false;
}

bool CollisionBody::raycast(const Ray& _ray, RaycastInfo& _raycastInfo) {
	if (m_isActive == false) {
		return false;
	}
	bool isHit = false;
	Ray rayTemp(_ray);
	for (ProxyShape* shape = m_proxyCollisionShapes; shape != nullptr; shape = shape->m_next) {
		// Test if the ray hits the collision shape
		if (shape->raycast(rayTemp, _raycastInfo)) {
			rayTemp.maxFraction = _raycastInfo.hitFraction;
			isHit = true;
		}
	}
	return isHit;
}

AABB CollisionBody::getAABB() const {
	AABB bodyAABB;
	if (m_proxyCollisionShapes == nullptr) {
		return bodyAABB;
	}
	m_proxyCollisionShapes->getCollisionShape()->computeAABB(bodyAABB, m_transform * m_proxyCollisionShapes->getLocalToBodyTransform());
	for (ProxyShape* shape = m_proxyCollisionShapes->m_next; shape != nullptr; shape = shape->m_next) {
		AABB aabb;
		shape->getCollisionShape()->computeAABB(aabb, m_transform * shape->getLocalToBodyTransform());
		bodyAABB.mergeWithAABB(aabb);
	}
	return bodyAABB;
}

