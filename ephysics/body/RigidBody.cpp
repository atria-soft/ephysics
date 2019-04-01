/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#include <ephysics/body/RigidBody.hpp>
#include <ephysics/constraint/Joint.hpp>
#include <ephysics/collision/shapes/CollisionShape.hpp>
#include <ephysics/engine/DynamicsWorld.hpp>
#include <ephysics/debug.hpp>

using namespace ephysics;


RigidBody::RigidBody(const etk::Transform3D& _transform, CollisionWorld& _world, bodyindex _id):
  CollisionBody(_transform, _world, _id),
  m_initMass(1.0f),
  m_centerOfMassLocal(0, 0, 0),
  m_centerOfMassWorld(_transform.getPosition()),
  m_isGravityEnabled(true),
  m_linearDamping(0.0f),
  m_angularDamping(float(0.0)),
  m_jointsList(null) {
	// Compute the inverse mass
	m_massInverse = 1.0f / m_initMass;
}

RigidBody::~RigidBody() {
	assert(m_jointsList == null);
}


void RigidBody::setType(BodyType _type) {
	if (m_type == _type) {
		return;
	}
	CollisionBody::setType(_type);
	recomputeMassInformation();
	if (m_type == STATIC) {
		// Reset the velocity to zero
		m_linearVelocity.setZero();
		m_angularVelocity.setZero();
	}
	if (    m_type == STATIC
	     || m_type == KINEMATIC) {
		// Reset the inverse mass and inverse inertia tensor to zero
		m_massInverse = 0.0f;
		m_inertiaTensorLocal.setZero();
		m_inertiaTensorLocalInverse.setZero();
	} else {
		m_massInverse = 1.0f / m_initMass;
		m_inertiaTensorLocalInverse = m_inertiaTensorLocal.getInverse();
	}
	setIsSleeping(false);
	resetContactManifoldsList();
	// Ask the broad-phase to test again the collision shapes of the body for collision detection (as if the body has moved)
	askForBroadPhaseCollisionCheck();
	m_externalForce.setZero();
	m_externalTorque.setZero();
}


void RigidBody::setInertiaTensorLocal(const etk::Matrix3x3& _inertiaTensorLocal) {
	if (m_type != DYNAMIC) {
		return;
	}
	m_inertiaTensorLocal = _inertiaTensorLocal;
	m_inertiaTensorLocalInverse = m_inertiaTensorLocal.getInverse();
}


void RigidBody::setCenterOfMassLocal(const vec3& _centerOfMassLocal) {
	if (m_type != DYNAMIC) {
		return;
	}
	const vec3 oldCenterOfMass = m_centerOfMassWorld;
	m_centerOfMassLocal = _centerOfMassLocal;
	m_centerOfMassWorld = m_transform * m_centerOfMassLocal;
	m_linearVelocity += m_angularVelocity.cross(m_centerOfMassWorld - oldCenterOfMass);
}


void RigidBody::setMass(float _mass) {
	if (m_type != DYNAMIC) {
		return;
	}
	m_initMass = _mass;
	if (m_initMass > 0.0f) {
		m_massInverse = 1.0f / m_initMass;
	} else {
		m_initMass = 1.0f;
		m_massInverse = 1.0f;
	}
}

void RigidBody::removeJointFrom_jointsList(const Joint* _joint) {
	assert(_joint != null);
	assert(m_jointsList != null);
	// Remove the joint from the linked list of the joints of the first body
	if (m_jointsList->joint == _joint) {   // If the first element is the one to remove
		JointListElement* elementToRemove = m_jointsList;
		m_jointsList = elementToRemove->next;
		ETK_DELETE(JointListElement, elementToRemove);
		elementToRemove = null;
	}
	else {  // If the element to remove is not the first one in the list
		JointListElement* currentElement = m_jointsList;
		while (currentElement->next != null) {
			if (currentElement->next->joint == _joint) {
				JointListElement* elementToRemove = currentElement->next;
				currentElement->next = elementToRemove->next;
				ETK_DELETE(JointListElement, elementToRemove);
				elementToRemove = null;
				break;
			}
			currentElement = currentElement->next;
		}
	}
}


ProxyShape* RigidBody::addCollisionShape(CollisionShape* _collisionShape,
                                         const etk::Transform3D& _transform,
                                         float _mass) {
	assert(_mass > 0.0f);
	// Create a new proxy collision shape to attach the collision shape to the body
	ProxyShape* proxyShape = ETK_NEW(ProxyShape, this, _collisionShape, _transform, _mass);
	// Add it to the list of proxy collision shapes of the body
	if (m_proxyCollisionShapes == null) {
		m_proxyCollisionShapes = proxyShape;
	} else {
		proxyShape->m_next = m_proxyCollisionShapes;
		m_proxyCollisionShapes = proxyShape;
	}
	// Compute the world-space AABB of the new collision shape
	AABB aabb;
	_collisionShape->computeAABB(aabb, m_transform * _transform);
	// Notify the collision detection about this new collision shape
	m_world.m_collisionDetection.addProxyCollisionShape(proxyShape, aabb);
	m_numberCollisionShapes++;
	recomputeMassInformation();
	return proxyShape;
}

void RigidBody::removeCollisionShape(const ProxyShape* _proxyShape) {
	CollisionBody::removeCollisionShape(_proxyShape);
	recomputeMassInformation();
}


void RigidBody::setLinearVelocity(const vec3& _linearVelocity) {
	if (m_type == STATIC) {
		return;
	}
	m_linearVelocity = _linearVelocity;
	if (m_linearVelocity.length2() > 0.0f) {
		setIsSleeping(false);
	}
}


void RigidBody::setAngularVelocity(const vec3& _angularVelocity) {
	if (m_type == STATIC) {
		return;
	}
	m_angularVelocity = _angularVelocity;
	if (m_angularVelocity.length2() > 0.0f) {
		setIsSleeping(false);
	}
}

void RigidBody::setIsSleeping(bool _isSleeping) {
	if (_isSleeping) {
		m_linearVelocity.setZero();
		m_angularVelocity.setZero();
		m_externalForce.setZero();
		m_externalTorque.setZero();
	}
	Body::setIsSleeping(_isSleeping);
}

void RigidBody::updateTransformWithCenterOfMass() {
	// Translate the body according to the translation of the center of mass position
	m_transform.setPosition(m_centerOfMassWorld - m_transform.getOrientation() * m_centerOfMassLocal);
	if (isnan(m_transform.getPosition().x()) == true) {
		EPHY_CRITICAL("updateTransformWithCenterOfMass: " << m_transform);
	}
	if (isinf(m_transform.getOrientation().z()) == true) {
		EPHY_CRITICAL("         set transform: " << m_transform);
	}
}

void RigidBody::setTransform(const etk::Transform3D& _transform) {
	EPHY_DEBUG("         set transform: " << m_transform << " ==> " << _transform);
	if (isnan(_transform.getPosition().x()) == true) {
		EPHY_CRITICAL("         set transform: " << m_transform << " ==> " << _transform);
	}
	if (isinf(_transform.getOrientation().z()) == true) {
		EPHY_CRITICAL("         set transform: " << m_transform << " ==> " << _transform);
	}
	m_transform = _transform;
	const vec3 oldCenterOfMass = m_centerOfMassWorld;
	// Compute the new center of mass in world-space coordinates
	m_centerOfMassWorld = m_transform * m_centerOfMassLocal;
	// Update the linear velocity of the center of mass
	m_linearVelocity += m_angularVelocity.cross(m_centerOfMassWorld - oldCenterOfMass);
	updateBroadPhaseState();
}

void RigidBody::recomputeMassInformation() {
	m_initMass = 0.0f;
	m_massInverse = 0.0f;
	m_inertiaTensorLocal.setZero();
	m_inertiaTensorLocalInverse.setZero();
	m_centerOfMassLocal.setZero();
	// If it is STATIC or KINEMATIC body
	if (m_type == STATIC || m_type == KINEMATIC) {
		m_centerOfMassWorld = m_transform.getPosition();
		return;
	}
	assert(m_type == DYNAMIC);
	// Compute the total mass of the body
	for (ProxyShape* shape = m_proxyCollisionShapes; shape != NULL; shape = shape->m_next) {
		m_initMass += shape->getMass();
		m_centerOfMassLocal += shape->getLocalToBodyTransform().getPosition() * shape->getMass();
	}
	if (m_initMass > 0.0f) {
		m_massInverse = 1.0f / m_initMass;
	} else {
		m_initMass = 1.0f;
		m_massInverse = 1.0f;
	}
	// Compute the center of mass
	const vec3 oldCenterOfMass = m_centerOfMassWorld;
	m_centerOfMassLocal *= m_massInverse;
	m_centerOfMassWorld = m_transform * m_centerOfMassLocal;
	// Compute the total mass and inertia tensor using all the collision shapes
	for (ProxyShape* shape = m_proxyCollisionShapes; shape != null; shape = shape->m_next) {
		// Get the inertia tensor of the collision shape in its local-space
		etk::Matrix3x3 inertiaTensor;
		shape->getCollisionShape()->computeLocalInertiaTensor(inertiaTensor, shape->getMass());
		// Convert the collision shape inertia tensor int32_to the local-space of the body
		const etk::Transform3D& shapeTransform = shape->getLocalToBodyTransform();
		etk::Matrix3x3 rotationMatrix = shapeTransform.getOrientation().getMatrix();
		inertiaTensor = rotationMatrix * inertiaTensor * rotationMatrix.getTranspose();
		// Use the parallel axis theorem to convert the inertia tensor w.r.t the collision shape
		// center int32_to a inertia tensor w.r.t to the body origin.
		vec3 offset = shapeTransform.getPosition() - m_centerOfMassLocal;
		float offsetSquare = offset.length2();
		vec3 off1 = offset * (-offset.x());
		vec3 off2 = offset * (-offset.y());
		vec3 off3 = offset * (-offset.z());
		etk::Matrix3x3 offsetMatrix(off1.x()+offsetSquare, off1.y(),              off1.z(),
		                            off2.x(),              off2.y()+offsetSquare, off2.z(),
		                            off3.x(),              off3.y(),              off3.z()+offsetSquare);
		offsetMatrix *= shape->getMass();
		m_inertiaTensorLocal += inertiaTensor + offsetMatrix;
	}
	// Compute the local inverse inertia tensor
	m_inertiaTensorLocalInverse = m_inertiaTensorLocal.getInverse();
	// Update the linear velocity of the center of mass
	m_linearVelocity += m_angularVelocity.cross(m_centerOfMassWorld - oldCenterOfMass);
}


void RigidBody::updateBroadPhaseState() const {
	PROFILE("RigidBody::updateBroadPhaseState()");
	DynamicsWorld& world = static_cast<DynamicsWorld&>(m_world);
	const vec3 displacement = world.m_timeStep * m_linearVelocity;
	// For all the proxy collision shapes of the body
	for (ProxyShape* shape = m_proxyCollisionShapes; shape != null; shape = shape->m_next) {
		// Recompute the world-space AABB of the collision shape
		AABB aabb;
		EPHY_VERBOSE("         : " << aabb.getMin() << " " << aabb.getMax());
		EPHY_VERBOSE("         m_transform: " << m_transform);
		shape->getCollisionShape()->computeAABB(aabb, m_transform *shape->getLocalToBodyTransform());
		EPHY_VERBOSE("         : " << aabb.getMin() << " " << aabb.getMax());
		// Update the broad-phase state for the proxy collision shape
		m_world.m_collisionDetection.updateProxyCollisionShape(shape, aabb, displacement);
	}
}


void RigidBody::applyForceToCenterOfMass(const vec3& _force) {
	if (m_type != DYNAMIC) {
		return;
	}
	if (m_isSleeping) {
		setIsSleeping(false);
	}
	m_externalForce += _force;
}

void RigidBody::applyForce(const vec3& _force, const vec3& _point) {
	if (m_type != DYNAMIC) {
		return;
	}
	if (m_isSleeping) {
		setIsSleeping(false);
	}
	m_externalForce += _force;
	m_externalTorque += (_point - m_centerOfMassWorld).cross(_force);
}

void RigidBody::applyTorque(const vec3& _torque) {
	if (m_type != DYNAMIC) {
		return;
	}
	if (m_isSleeping) {
		setIsSleeping(false);
	}
	m_externalTorque += _torque;
}