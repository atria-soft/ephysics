/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/body/RigidBody.h>
#include <ephysics/constraint/Joint.h>
#include <ephysics/collision/shapes/CollisionShape.h>
#include <ephysics/engine/DynamicsWorld.h>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;


RigidBody::RigidBody(const etk::Transform3D& transform, CollisionWorld& world, bodyindex id)
		  : CollisionBody(transform, world, id), m_initMass(1.0f),
			m_centerOfMassLocal(0, 0, 0), m_centerOfMassWorld(transform.getPosition()),
			m_isGravityEnabled(true), m_linearDamping(0.0f), m_angularDamping(float(0.0)),
			m_jointsList(NULL) {

	// Compute the inverse mass
	m_massInverse = 1.0f / m_initMass;
}

// Destructor
RigidBody::~RigidBody() {
	assert(m_jointsList == NULL);
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

// Set the local inertia tensor of the body (in local-space coordinates)
/**
 * @param inertiaTensorLocal The 3x3 inertia tensor matrix of the body in local-space
 *						   coordinates
 */
void RigidBody::setInertiaTensorLocal(const etk::Matrix3x3& inertiaTensorLocal) {

	if (m_type != DYNAMIC) return;

	m_inertiaTensorLocal = inertiaTensorLocal;

	// Compute the inverse local inertia tensor
	m_inertiaTensorLocalInverse = m_inertiaTensorLocal.getInverse();
}

// Set the local center of mass of the body (in local-space coordinates)
/**
 * @param centerOfMassLocal The center of mass of the body in local-space
 *						  coordinates
 */
void RigidBody::setCenterOfMassLocal(const vec3& centerOfMassLocal) {

	if (m_type != DYNAMIC) return;

	const vec3 oldCenterOfMass = m_centerOfMassWorld;
	m_centerOfMassLocal = centerOfMassLocal;

	// Compute the center of mass in world-space coordinates
	m_centerOfMassWorld = m_transform * m_centerOfMassLocal;

	// Update the linear velocity of the center of mass
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

void RigidBody::removeJointFrom_jointsList(MemoryAllocator& memoryAllocator, const Joint* joint) {
	assert(joint != nullptr);
	assert(m_jointsList != nullptr);
	// Remove the joint from the linked list of the joints of the first body
	if (m_jointsList->joint == joint) {   // If the first element is the one to remove
		JointListElement* elementToRemove = m_jointsList;
		m_jointsList = elementToRemove->next;
		elementToRemove->~JointListElement();
		memoryAllocator.release(elementToRemove, sizeof(JointListElement));
	}
	else {  // If the element to remove is not the first one in the list
		JointListElement* currentElement = m_jointsList;
		while (currentElement->next != nullptr) {
			if (currentElement->next->joint == joint) {
				JointListElement* elementToRemove = currentElement->next;
				currentElement->next = elementToRemove->next;
				elementToRemove->~JointListElement();
				memoryAllocator.release(elementToRemove, sizeof(JointListElement));
				break;
			}
			currentElement = currentElement->next;
		}
	}
}

// Add a collision shape to the body.
/// When you add a collision shape to the body, an int32_ternal copy of this
/// collision shape will be created int32_ternally. Therefore, you can delete it
/// right after calling this method or use it later to add it to another body.
/// This method will return a pointer to a new proxy shape. A proxy shape is
/// an object that links a collision shape and a given body. You can use the
/// returned proxy shape to get and set information about the corresponding
/// collision shape for that body.
/**
 * @param collisionShape The collision shape you want to add to the body
 * @param transform The transformation of the collision shape that transforms the
 *		local-space of the collision shape int32_to the local-space of the body
 * @param mass Mass (in kilograms) of the collision shape you want to add
 * @return A pointer to the proxy shape that has been created to link the body to
 *		 the new collision shape you have added.
 */
ProxyShape* RigidBody::addCollisionShape(CollisionShape* collisionShape,
										 const etk::Transform3D& transform,
										 float mass) {

	assert(mass > 0.0f);

	// Create a new proxy collision shape to attach the collision shape to the body
	ProxyShape* proxyShape = new (m_world.m_memoryAllocator.allocate(
									  sizeof(ProxyShape))) ProxyShape(this, collisionShape,
																	  transform, mass);

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

	// Recompute the center of mass, total mass and inertia tensor of the body with the new
	// collision shape
	recomputeMassInformation();

	// Return a pointer to the proxy collision shape
	return proxyShape;
}

// Remove a collision shape from the body
/// To remove a collision shape, you need to specify the pointer to the proxy
/// shape that has been returned when you have added the collision shape to the
/// body
/**
 * @param proxyShape The pointer of the proxy shape you want to remove
 */
void RigidBody::removeCollisionShape(const ProxyShape* proxyShape) {
	CollisionBody::removeCollisionShape(proxyShape);
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


void RigidBody::setTransform(const etk::Transform3D& _transform) {
	m_transform = _transform;
	const vec3 oldCenterOfMass = m_centerOfMassWorld;
	// Compute the new center of mass in world-space coordinates
	m_centerOfMassWorld = m_transform * m_centerOfMassLocal;
	// Update the linear velocity of the center of mass
	m_linearVelocity += m_angularVelocity.cross(m_centerOfMassWorld - oldCenterOfMass);
	updateBroadPhaseState();
}

// Recompute the center of mass, total mass and inertia tensor of the body using all
// the collision shapes attached to the body.
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
	for (ProxyShape* shape = m_proxyCollisionShapes; shape != nullptr; shape = shape->m_next) {
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
	for (ProxyShape* shape = m_proxyCollisionShapes; shape != nullptr; shape = shape->m_next) {

		// Recompute the world-space AABB of the collision shape
		AABB aabb;
		shape->getCollisionShape()->computeAABB(aabb, m_transform *shape->getLocalToBodyTransform());

		// Update the broad-phase state for the proxy collision shape
		m_world.m_collisionDetection.updateProxyCollisionShape(shape, aabb, displacement);
	}
}


void RigidBody::applyForceToCenterOfMass(const vec3& _force) {
	// If it is not a dynamic body, we do nothing
	if (m_type != DYNAMIC) {
		return;
	}
	// Awake the body if it was sleeping
	if (m_isSleeping) {
		setIsSleeping(false);
	}
	// Add the force
	m_externalForce += _force;
}

void RigidBody::applyForce(const vec3& _force, const vec3& _point) {
	// If it is not a dynamic body, we do nothing
	if (m_type != DYNAMIC) {
		return;
	}
	// Awake the body if it was sleeping
	if (m_isSleeping) {
		setIsSleeping(false);
	}
	// Add the force and torque
	m_externalForce += _force;
	m_externalTorque += (_point - m_centerOfMassWorld).cross(_force);
}

void RigidBody::applyTorque(const vec3& _torque) {
	// If it is not a dynamic body, we do nothing
	if (m_type != DYNAMIC) {
		return;
	}
	// Awake the body if it was sleeping
	if (m_isSleeping) {
		setIsSleeping(false);
	}
	// Add the torque
	m_externalTorque += _torque;
}