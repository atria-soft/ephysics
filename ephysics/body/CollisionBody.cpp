/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#include <ephysics/body/CollisionBody.hpp>
#include <ephysics/engine/CollisionWorld.hpp>
#include <ephysics/collision/ContactManifold.hpp>

// We want to use the ReactPhysics3D namespace
using namespace ephysics;


CollisionBody::CollisionBody(const etk::Transform3D& _transform, CollisionWorld& _world, bodyindex _id):
  Body(_id),
  m_type(DYNAMIC),
  m_transform(_transform),
  m_proxyCollisionShapes(null),
  m_numberCollisionShapes(0),
  m_contactManifoldsList(null),
  m_world(_world) {
	
}

CollisionBody::~CollisionBody() {
	assert(m_contactManifoldsList == null);

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
	// Create a proxy collision shape to attach the collision shape to the body
	ProxyShape* proxyShape = ETK_NEW(ProxyShape, this, _collisionShape,_transform, float(1));
	// Add it to the list of proxy collision shapes of the body
	if (m_proxyCollisionShapes == null) {
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
		ETK_DELETE(ProxyShape, current);
		current = null;
		m_numberCollisionShapes--;
		return;
	}
	// Look for the proxy shape that contains the collision shape in parameter
	while(current->m_next != null) {
		// If we have found the collision shape to remove
		if (current->m_next == _proxyShape) {
			// Remove the proxy collision shape
			ProxyShape* elementToRemove = current->m_next;
			current->m_next = elementToRemove->m_next;
			if (m_isActive) {
				m_world.m_collisionDetection.removeProxyCollisionShape(elementToRemove);
			}
			ETK_DELETE(ProxyShape, elementToRemove);
			elementToRemove = null;
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
	while(current != null) {
		// Remove the proxy collision shape
		ProxyShape* nextElement = current->m_next;
		if (m_isActive) {
			m_world.m_collisionDetection.removeProxyCollisionShape(current);
		}
		ETK_DELETE(ProxyShape, current);
		// Get the next element in the list
		current = nextElement;
	}
	m_proxyCollisionShapes = null;
}


void CollisionBody::resetContactManifoldsList() {
	// Delete the linked list of contact manifolds of that body
	ContactManifoldListElement* currentElement = m_contactManifoldsList;
	while (currentElement != null) {
		ContactManifoldListElement* nextElement = currentElement->next;
		// Delete the current element
		ETK_DELETE(ContactManifoldListElement, currentElement);
		currentElement = nextElement;
	}
	m_contactManifoldsList = null;
}


void CollisionBody::updateBroadPhaseState() const {
	// For all the proxy collision shapes of the body
	for (ProxyShape* shape = m_proxyCollisionShapes; shape != null; shape = shape->m_next) {
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
		for (ProxyShape* shape = m_proxyCollisionShapes; shape != null; shape = shape->m_next) {
			AABB aabb;
			shape->getCollisionShape()->computeAABB(aabb, m_transform * shape->m_localToBodyTransform);
			m_world.m_collisionDetection.addProxyCollisionShape(shape, aabb);
		}
	} else {
		for (ProxyShape* shape = m_proxyCollisionShapes; shape != null; shape = shape->m_next) {
			m_world.m_collisionDetection.removeProxyCollisionShape(shape);
		}
		resetContactManifoldsList();
	}
}


void CollisionBody::askForBroadPhaseCollisionCheck() const {
	for (ProxyShape* shape = m_proxyCollisionShapes; shape != null; shape = shape->m_next) {
		m_world.m_collisionDetection.askForBroadPhaseCollisionCheck(shape);  
	}
}


int32_t CollisionBody::resetIsAlreadyInIslandAndCountManifolds() {
	m_isAlreadyInIsland = false;
	int32_t nbManifolds = 0;
	// Reset the m_isAlreadyInIsland variable of the contact manifolds for this body
	ContactManifoldListElement* currentElement = m_contactManifoldsList;
	while (currentElement != null) {
		currentElement->contactManifold->m_isAlreadyInIsland = false;
		currentElement = currentElement->next;
		nbManifolds++;
	}
	return nbManifolds;
}

bool CollisionBody::testPointInside(const vec3& _worldPoint) const {
	for (ProxyShape* shape = m_proxyCollisionShapes; shape != null; shape = shape->m_next) {
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
	for (ProxyShape* shape = m_proxyCollisionShapes; shape != null; shape = shape->m_next) {
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
	if (m_proxyCollisionShapes == null) {
		return bodyAABB;
	}
	m_proxyCollisionShapes->getCollisionShape()->computeAABB(bodyAABB, m_transform * m_proxyCollisionShapes->getLocalToBodyTransform());
	for (ProxyShape* shape = m_proxyCollisionShapes->m_next; shape != null; shape = shape->m_next) {
		AABB aabb;
		shape->getCollisionShape()->computeAABB(aabb, m_transform * shape->getLocalToBodyTransform());
		bodyAABB.mergeWithAABB(aabb);
	}
	return bodyAABB;
}

