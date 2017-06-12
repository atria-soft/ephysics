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

// Constructor
/**
* @param transform The transformation of the body
* @param world The world where the body has been added
* @param id The ID of the body
*/
RigidBody::RigidBody(const Transform& transform, CollisionWorld& world, bodyindex id)
		  : CollisionBody(transform, world, id), mInitMass(float(1.0)),
			mCenterOfMassLocal(0, 0, 0), mCenterOfMassWorld(transform.getPosition()),
			m_isGravityEnabled(true), mLinearDamping(float(0.0)), mAngularDamping(float(0.0)),
			m_jointsList(NULL) {

	// Compute the inverse mass
	m_massInverse = float(1.0) / mInitMass;
}

// Destructor
RigidBody::~RigidBody() {
	assert(m_jointsList == NULL);
}

// Set the type of the body
/// The type of the body can either STATIC, KINEMATIC or DYNAMIC as described bellow:
/// STATIC : A static body has infinite mass, zero velocity but the position can be
///		  changed manually. A static body does not collide with other static or kinematic bodies.
/// KINEMATIC : A kinematic body has infinite mass, the velocity can be changed manually and its
///			 position is computed by the physics engine. A kinematic body does not collide with
///			 other static or kinematic bodies.
/// DYNAMIC : A dynamic body has non-zero mass, non-zero velocity determined by forces and its
///		   position is determined by the physics engine. A dynamic body can collide with other
///		   dynamic, static or kinematic bodies.
/**
 * @param type The type of the body (STATIC, KINEMATIC, DYNAMIC)
 */
void RigidBody::setType(BodyType type) {

	if (m_type == type) return;

	CollisionBody::setType(type);

	// Recompute the total mass, center of mass and inertia tensor
	recomputeMassInformation();

	// If it is a static body
	if (m_type == STATIC) {

		// Reset the velocity to zero
		mLinearVelocity.setToZero();
		mAngularVelocity.setToZero();
	}

	// If it is a static or a kinematic body
	if (m_type == STATIC || m_type == KINEMATIC) {

		// Reset the inverse mass and inverse inertia tensor to zero
		m_massInverse = float(0.0);
		mInertiaTensorLocal.setToZero();
		mInertiaTensorLocalInverse.setToZero();

	}
	else {  // If it is a dynamic body
		m_massInverse = float(1.0) / mInitMass;
		mInertiaTensorLocalInverse = mInertiaTensorLocal.getInverse();
	}

	// Awake the body
	setIsSleeping(false);

	// Remove all the contacts with this body
	resetContactManifoldsList();

	// Ask the broad-phase to test again the collision shapes of the body for collision
	// detection (as if the body has moved)
	askForBroadPhaseCollisionCheck();

	// Reset the force and torque on the body
	mExternalForce.setToZero();
	mExternalTorque.setToZero();
}

// Set the local inertia tensor of the body (in local-space coordinates)
/**
 * @param inertiaTensorLocal The 3x3 inertia tensor matrix of the body in local-space
 *						   coordinates
 */
void RigidBody::setInertiaTensorLocal(const Matrix3x3& inertiaTensorLocal) {

	if (m_type != DYNAMIC) return;

	mInertiaTensorLocal = inertiaTensorLocal;

	// Compute the inverse local inertia tensor
	mInertiaTensorLocalInverse = mInertiaTensorLocal.getInverse();
}

// Set the local center of mass of the body (in local-space coordinates)
/**
 * @param centerOfMassLocal The center of mass of the body in local-space
 *						  coordinates
 */
void RigidBody::setCenterOfMassLocal(const Vector3& centerOfMassLocal) {

	if (m_type != DYNAMIC) return;

	const Vector3 oldCenterOfMass = mCenterOfMassWorld;
	mCenterOfMassLocal = centerOfMassLocal;

	// Compute the center of mass in world-space coordinates
	mCenterOfMassWorld = m_transform * mCenterOfMassLocal;

	// Update the linear velocity of the center of mass
	mLinearVelocity += mAngularVelocity.cross(mCenterOfMassWorld - oldCenterOfMass);
}

// Set the mass of the rigid body
/**
 * @param mass The mass (in kilograms) of the body
 */
void RigidBody::setMass(float mass) {

	if (m_type != DYNAMIC) return;

	mInitMass = mass;

	if (mInitMass > float(0.0)) {
		m_massInverse = float(1.0) / mInitMass;
	}
	else {
		mInitMass = float(1.0);
		m_massInverse = float(1.0);
	}
}

// Remove a joint from the joints list
void RigidBody::removeJointFrom_jointsList(MemoryAllocator& memoryAllocator, const Joint* joint) {

	assert(joint != NULL);
	assert(m_jointsList != NULL);

	// Remove the joint from the linked list of the joints of the first body
	if (m_jointsList->joint == joint) {   // If the first element is the one to remove
		JointListElement* elementToRemove = m_jointsList;
		m_jointsList = elementToRemove->next;
		elementToRemove->~JointListElement();
		memoryAllocator.release(elementToRemove, sizeof(JointListElement));
	}
	else {  // If the element to remove is not the first one in the list
		JointListElement* currentElement = m_jointsList;
		while (currentElement->next != NULL) {
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
										 const Transform& transform,
										 float mass) {

	assert(mass > float(0.0));

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

	// Remove the collision shape
	CollisionBody::removeCollisionShape(proxyShape);

	// Recompute the total mass, center of mass and inertia tensor
	recomputeMassInformation();
}

// Set the linear velocity of the rigid body.
/**
 * @param linearVelocity Linear velocity vector of the body
 */
void RigidBody::setLinearVelocity(const Vector3& linearVelocity) {

	// If it is a static body, we do nothing
	if (m_type == STATIC) return;

	// Update the linear velocity of the current body state
	mLinearVelocity = linearVelocity;

	// If the linear velocity is not zero, awake the body
	if (mLinearVelocity.lengthSquare() > float(0.0)) {
		setIsSleeping(false);
	}
}

// Set the angular velocity.
/**
* @param angularVelocity The angular velocity vector of the body
*/
void RigidBody::setAngularVelocity(const Vector3& angularVelocity) {

	// If it is a static body, we do nothing
	if (m_type == STATIC) return;

	// Set the angular velocity
	mAngularVelocity = angularVelocity;

	// If the velocity is not zero, awake the body
	if (mAngularVelocity.lengthSquare() > float(0.0)) {
		setIsSleeping(false);
	}
}

// Set the current position and orientation
/**
 * @param transform The transformation of the body that transforms the local-space
 *				  of the body int32_to world-space
 */
void RigidBody::setTransform(const Transform& transform) {

	// Update the transform of the body
	m_transform = transform;

	const Vector3 oldCenterOfMass = mCenterOfMassWorld;

	// Compute the new center of mass in world-space coordinates
	mCenterOfMassWorld = m_transform * mCenterOfMassLocal;

	// Update the linear velocity of the center of mass
	mLinearVelocity += mAngularVelocity.cross(mCenterOfMassWorld - oldCenterOfMass);

	// Update the broad-phase state of the body
	updateBroadPhaseState();
}

// Recompute the center of mass, total mass and inertia tensor of the body using all
// the collision shapes attached to the body.
void RigidBody::recomputeMassInformation() {

	mInitMass = float(0.0);
	m_massInverse = float(0.0);
	mInertiaTensorLocal.setToZero();
	mInertiaTensorLocalInverse.setToZero();
	mCenterOfMassLocal.setToZero();

	// If it is STATIC or KINEMATIC body
	if (m_type == STATIC || m_type == KINEMATIC) {
		mCenterOfMassWorld = m_transform.getPosition();
		return;
	}

	assert(m_type == DYNAMIC);

	// Compute the total mass of the body
	for (ProxyShape* shape = m_proxyCollisionShapes; shape != NULL; shape = shape->m_next) {
		mInitMass += shape->getMass();
		mCenterOfMassLocal += shape->getLocalToBodyTransform().getPosition() * shape->getMass();
	}

	if (mInitMass > float(0.0)) {
		m_massInverse = float(1.0) / mInitMass;
	}
	else {
		mInitMass = float(1.0);
		m_massInverse = float(1.0);
	}

	// Compute the center of mass
	const Vector3 oldCenterOfMass = mCenterOfMassWorld;
	mCenterOfMassLocal *= m_massInverse;
	mCenterOfMassWorld = m_transform * mCenterOfMassLocal;

	// Compute the total mass and inertia tensor using all the collision shapes
	for (ProxyShape* shape = m_proxyCollisionShapes; shape != NULL; shape = shape->m_next) {

		// Get the inertia tensor of the collision shape in its local-space
		Matrix3x3 inertiaTensor;
		shape->getCollisionShape()->computeLocalInertiaTensor(inertiaTensor, shape->getMass());

		// Convert the collision shape inertia tensor int32_to the local-space of the body
		const Transform& shapeTransform = shape->getLocalToBodyTransform();
		Matrix3x3 rotationMatrix = shapeTransform.getOrientation().getMatrix();
		inertiaTensor = rotationMatrix * inertiaTensor * rotationMatrix.getTranspose();

		// Use the parallel axis theorem to convert the inertia tensor w.r.t the collision shape
		// center int32_to a inertia tensor w.r.t to the body origin.
		Vector3 offset = shapeTransform.getPosition() - mCenterOfMassLocal;
		float offsetSquare = offset.lengthSquare();
		Matrix3x3 offsetMatrix;
		offsetMatrix[0].setAllValues(offsetSquare, float(0.0), float(0.0));
		offsetMatrix[1].setAllValues(float(0.0), offsetSquare, float(0.0));
		offsetMatrix[2].setAllValues(float(0.0), float(0.0), offsetSquare);
		offsetMatrix[0] += offset * (-offset.x);
		offsetMatrix[1] += offset * (-offset.y);
		offsetMatrix[2] += offset * (-offset.z);
		offsetMatrix *= shape->getMass();

		mInertiaTensorLocal += inertiaTensor + offsetMatrix;
	}

	// Compute the local inverse inertia tensor
	mInertiaTensorLocalInverse = mInertiaTensorLocal.getInverse();

	// Update the linear velocity of the center of mass
	mLinearVelocity += mAngularVelocity.cross(mCenterOfMassWorld - oldCenterOfMass);
}

// Update the broad-phase state for this body (because it has moved for instance)
void RigidBody::updateBroadPhaseState() const {

	PROFILE("RigidBody::updateBroadPhaseState()");

	DynamicsWorld& world = static_cast<DynamicsWorld&>(m_world);
 	 const Vector3 displacement = world.m_timeStep * mLinearVelocity;

	// For all the proxy collision shapes of the body
	for (ProxyShape* shape = m_proxyCollisionShapes; shape != NULL; shape = shape->m_next) {

		// Recompute the world-space AABB of the collision shape
		AABB aabb;
		shape->getCollisionShape()->computeAABB(aabb, m_transform *shape->getLocalToBodyTransform());

		// Update the broad-phase state for the proxy collision shape
		m_world.m_collisionDetection.updateProxyCollisionShape(shape, aabb, displacement);
	}
}

