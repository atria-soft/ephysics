/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#include <ephysics/engine/DynamicsWorld.hpp>
#include <ephysics/constraint/BallAndSocketJoint.hpp>
#include <ephysics/constraint/SliderJoint.hpp>
#include <ephysics/constraint/HingeJoint.hpp>
#include <ephysics/constraint/FixedJoint.hpp>
#include <ephysics/debug.hpp>

ephysics::DynamicsWorld::DynamicsWorld(const vec3& _gravity):
  CollisionWorld(),
  m_contactSolver(m_mapBodyToConstrainedVelocityIndex),
  m_constraintSolver(m_mapBodyToConstrainedVelocityIndex),
  m_nbVelocitySolverIterations(DEFAULT_VELOCITY_SOLVER_NB_ITERATIONS),
  m_nbPositionSolverIterations(DEFAULT_POSITION_SOLVER_NB_ITERATIONS),
  m_isSleepingEnabled(SPLEEPING_ENABLED),
  m_gravity(_gravity),
  m_isGravityEnabled(true),
  m_numberBodiesCapacity(0),
  m_sleepLinearVelocity(DEFAULT_SLEEP_LINEAR_VELOCITY),
  m_sleepAngularVelocity(DEFAULT_SLEEP_ANGULAR_VELOCITY),
  m_timeBeforeSleep(DEFAULT_TIME_BEFORE_SLEEP) {
	
}

ephysics::DynamicsWorld::~DynamicsWorld() {
	// Destroy all the joints that have not been removed
	etk::Set<ephysics::Joint*>::Iterator itJoints;
	for (itJoints = m_joints.begin(); itJoints != m_joints.end();) {
		etk::Set<ephysics::Joint*>::Iterator itToRemove = itJoints;
		++itJoints;
		destroyJoint(*itToRemove);
	}
	// Destroy all the rigid bodies that have not been removed
	etk::Set<RigidBody*>::Iterator itRigidBodies;
	for (itRigidBodies = m_rigidBodies.begin(); itRigidBodies != m_rigidBodies.end();) {
		etk::Set<RigidBody*>::Iterator itToRemove = itRigidBodies;
		++itRigidBodies;
		destroyRigidBody(*itToRemove);
	}
	// Release the memory allocated for the islands
	for (auto &it: m_islands) {
		// Call the island destructor
		ETK_DELETE(Island, it);
		it = nullptr;
	}
	m_islands.clear();
	// Release the memory allocated for the bodies velocity arrays
	if (m_numberBodiesCapacity > 0) {
		m_splitLinearVelocities.clear();
		m_splitAngularVelocities.clear();
		m_constrainedLinearVelocities.clear();
		m_constrainedAngularVelocities.clear();
		m_constrainedPositions.clear();
		m_constrainedOrientations.clear();
	}
	assert(m_joints.size() == 0);
	assert(m_rigidBodies.size() == 0);
#ifdef IS_PROFILING_ACTIVE
	// Print32_t the profiling report
	etk::Stream tmp;
	Profiler::print32_tReport(tmp);
	EPHY_PRINT(tmp.str());
	// Destroy the profiler (release the allocated memory)
	Profiler::destroy();
#endif
}

void ephysics::DynamicsWorld::update(float timeStep) {
	#ifdef IS_PROFILING_ACTIVE
		// Increment the frame counter of the profiler
		Profiler::incrementFrameCounter();
	#endif
	PROFILE("ephysics::DynamicsWorld::update()");
	m_timeStep = timeStep;
	// Notify the event listener about the beginning of an int32_ternal tick
	if (m_eventListener != nullptr) {
		m_eventListener->beginInternalTick();
	}
	// Reset all the contact manifolds lists of each body
	resetContactManifoldListsOfBodies();
	if (m_rigidBodies.size() == 0) {
		// no rigid body ==> no process to do ...
		return;
	}
	// Compute the collision detection
	m_collisionDetection.computeCollisionDetection();
	// Compute the islands (separate groups of bodies with constraints between each others)
	computeIslands();
	// Integrate the velocities
	integrateRigidBodiesVelocities();
	// Solve the contacts and constraints
	solveContactsAndConstraints();
	// Integrate the position and orientation of each body
	integrateRigidBodiesPositions();
	// Solve the position correction for constraints
	solvePositionCorrection();
	// Update the state (positions and velocities) of the bodies
	updateBodiesState();
	if (m_isSleepingEnabled) {
		updateSleepingBodies();
	}
	// Notify the event listener about the end of an int32_ternal tick
	if (m_eventListener != nullptr) {
		m_eventListener->endInternalTick();
	}
	// Reset the external force and torque applied to the bodies
	resetBodiesForceAndTorque();
}

void ephysics::DynamicsWorld::integrateRigidBodiesPositions() {
	PROFILE("ephysics::DynamicsWorld::integrateRigidBodiesPositions()");
	// For each island of the world
	for (uint32_t i=0; i < m_islands.size(); i++) {
		RigidBody** bodies = m_islands[i]->getBodies();
		// For each body of the island
		for (uint32_t b=0; b < m_islands[i]->getNbBodies(); b++) {
			// Get the constrained velocity
			uint32_t indexArray = m_mapBodyToConstrainedVelocityIndex.find(bodies[b])->second;
			vec3 newLinVelocity = m_constrainedLinearVelocities[indexArray];
			vec3 newAngVelocity = m_constrainedAngularVelocities[indexArray];
			// Add the split impulse velocity from Contact Solver (only used
			// to update the position)
			if (m_contactSolver.isSplitImpulseActive()) {
				newLinVelocity += m_splitLinearVelocities[indexArray];
				newAngVelocity += m_splitAngularVelocities[indexArray];
			}
			// Get current position and orientation of the body
			const vec3& currentPosition = bodies[b]->m_centerOfMassWorld;
			const etk::Quaternion& currentOrientation = bodies[b]->getTransform().getOrientation();
			// Update the new constrained position and orientation of the body
			m_constrainedPositions[indexArray] = currentPosition + newLinVelocity * m_timeStep;
			m_constrainedOrientations[indexArray] = currentOrientation;
			m_constrainedOrientations[indexArray] +=   etk::Quaternion(0, newAngVelocity)
			                                         * currentOrientation
			                                         * 0.5f
			                                         * m_timeStep;
		}
	}
}

void ephysics::DynamicsWorld::updateBodiesState() {
	PROFILE("ephysics::DynamicsWorld::updateBodiesState()");
	// For each island of the world
	for (uint32_t islandIndex = 0; islandIndex < m_islands.size(); islandIndex++) {
		// For each body of the island
		RigidBody** bodies = m_islands[islandIndex]->getBodies();
		for (uint32_t b=0; b < m_islands[islandIndex]->getNbBodies(); b++) {
			uint32_t index = m_mapBodyToConstrainedVelocityIndex.find(bodies[b])->second;
			// Update the linear and angular velocity of the body
			bodies[b]->m_linearVelocity = m_constrainedLinearVelocities[index];
			bodies[b]->m_angularVelocity = m_constrainedAngularVelocities[index];
			// Update the position of the center of mass of the body
			bodies[b]->m_centerOfMassWorld = m_constrainedPositions[index];
			// Update the orientation of the body
			bodies[b]->m_transform.setOrientation(m_constrainedOrientations[index].safeNormalized());
			// Update the transform of the body (using the new center of mass and new orientation)
			bodies[b]->updateTransformWithCenterOfMass();
			// Update the broad-phase state of the body
			bodies[b]->updateBroadPhaseState();
		}
	}
}

void ephysics::DynamicsWorld::initVelocityArrays() {
	// Allocate memory for the bodies velocity arrays
	uint32_t nbBodies = m_rigidBodies.size();
	if (m_numberBodiesCapacity != nbBodies && nbBodies > 0) {
		if (m_numberBodiesCapacity > 0) {
			m_splitLinearVelocities.clear();
			m_splitAngularVelocities.clear();
		}
		m_numberBodiesCapacity = nbBodies;
		m_splitLinearVelocities.clear();
		m_splitAngularVelocities.clear();
		m_constrainedLinearVelocities.clear();
		m_constrainedAngularVelocities.clear();
		m_constrainedPositions.clear();
		m_constrainedOrientations.clear();
		m_splitLinearVelocities.resize(m_numberBodiesCapacity, vec3(0,0,0));
		m_splitAngularVelocities.resize(m_numberBodiesCapacity, vec3(0,0,0));
		m_constrainedLinearVelocities.resize(m_numberBodiesCapacity, vec3(0,0,0));
		m_constrainedAngularVelocities.resize(m_numberBodiesCapacity, vec3(0,0,0));
		m_constrainedPositions.resize(m_numberBodiesCapacity, vec3(0,0,0));
		m_constrainedOrientations.resize(m_numberBodiesCapacity, etk::Quaternion::identity());
	}
	// Reset the velocities arrays
	for (uint32_t i=0; i<m_numberBodiesCapacity; i++) {
		m_splitLinearVelocities[i].setZero();
		m_splitAngularVelocities[i].setZero();
	}
	// Initialize the map of body indexes in the velocity arrays
	m_mapBodyToConstrainedVelocityIndex.clear();
	etk::Set<RigidBody*>::Iterator it;
	uint32_t indexBody = 0;
	for (it = m_rigidBodies.begin(); it != m_rigidBodies.end(); ++it) {
		// Add the body int32_to the map
		m_mapBodyToConstrainedVelocityIndex.add(*it, indexBody);
		indexBody++;
	}
}

void ephysics::DynamicsWorld::integrateRigidBodiesVelocities() {
	PROFILE("ephysics::DynamicsWorld::integrateRigidBodiesVelocities()");
	// Initialize the bodies velocity arrays
	initVelocityArrays();
	// For each island of the world
	for (uint32_t i=0; i < m_islands.size(); i++) {
		RigidBody** bodies = m_islands[i]->getBodies();
		// For each body of the island
		for (uint32_t b=0; b < m_islands[i]->getNbBodies(); b++) {
			// Insert the body int32_to the map of constrained velocities
			uint32_t indexBody = m_mapBodyToConstrainedVelocityIndex.find(bodies[b])->second;
			assert(m_splitLinearVelocities[indexBody] == vec3(0, 0, 0));
			assert(m_splitAngularVelocities[indexBody] == vec3(0, 0, 0));
			// Integrate the external force to get the new velocity of the body
			m_constrainedLinearVelocities[indexBody] = bodies[b]->getLinearVelocity();
			m_constrainedLinearVelocities[indexBody] += bodies[b]->m_massInverse * bodies[b]->m_externalForce * m_timeStep;
			m_constrainedAngularVelocities[indexBody] = bodies[b]->getAngularVelocity();
			m_constrainedAngularVelocities[indexBody] += bodies[b]->getInertiaTensorInverseWorld() * bodies[b]->m_externalTorque * m_timeStep;
			// If the gravity has to be applied to this rigid body
			if (bodies[b]->isGravityEnabled() && m_isGravityEnabled) {
				// Integrate the gravity force
				m_constrainedLinearVelocities[indexBody] += m_timeStep * bodies[b]->m_massInverse * bodies[b]->getMass() * m_gravity;
			}
			// Apply the velocity damping
			// Damping force : F_c = -c' * v (c=damping factor)
			// Equation	  : m * dv/dt = -c' * v
			//				 => dv/dt = -c * v (with c=c'/m)
			//				 => dv/dt + c * v = 0
			// Solution	  : v(t) = v0 * e^(-c * t)
			//				 => v(t + dt) = v0 * e^(-c(t + dt))
			//							  = v0 * e^(-ct) * e^(-c * dt)
			//							  = v(t) * e^(-c * dt)
			//				 => v2 = v1 * e^(-c * dt)
			// Using Taylor Serie for e^(-x) : e^x ~ 1 + x + x^2/2! + ...
			//							  => e^(-x) ~ 1 - x
			//				 => v2 = v1 * (1 - c * dt)
			float linDampingFactor = bodies[b]->getLinearDamping();
			float angDampingFactor = bodies[b]->getAngularDamping();
			float linearDamping = pow(1.0f - linDampingFactor, m_timeStep);
			float angularDamping = pow(1.0f - angDampingFactor, m_timeStep);
			m_constrainedLinearVelocities[indexBody] *= linearDamping;
			m_constrainedAngularVelocities[indexBody] *= angularDamping;
			indexBody++;
		}
	}
}

void ephysics::DynamicsWorld::solveContactsAndConstraints() {
	PROFILE("ephysics::DynamicsWorld::solveContactsAndConstraints()");
	// Set the velocities arrays
	m_contactSolver.setSplitVelocitiesArrays(&m_splitLinearVelocities[0], &m_splitAngularVelocities[0]);
	m_contactSolver.setConstrainedVelocitiesArrays(&m_constrainedLinearVelocities[0],
	                                               &m_constrainedAngularVelocities[0]);
	m_constraintSolver.setConstrainedVelocitiesArrays(&m_constrainedLinearVelocities[0],
	                                                  &m_constrainedAngularVelocities[0]);
	m_constraintSolver.setConstrainedPositionsArrays(&m_constrainedPositions[0],
	                                                 &m_constrainedOrientations[0]);
	// ---------- Solve velocity constraints for joints and contacts ---------- //
	// For each island of the world
	for (uint32_t islandIndex = 0; islandIndex < m_islands.size(); islandIndex++) {
		// Check if there are contacts and constraints to solve
		bool isConstraintsToSolve = m_islands[islandIndex]->getNbJoints() > 0;
		bool isContactsToSolve = m_islands[islandIndex]->getNbContactManifolds() > 0;
		if (!isConstraintsToSolve && !isContactsToSolve) {
			continue;
		}
		// If there are contacts in the current island
		if (isContactsToSolve) {
			// Initialize the solver
			m_contactSolver.initializeForIsland(m_timeStep, m_islands[islandIndex]);
			// Warm start the contact solver
			m_contactSolver.warmStart();
		}
		// If there are constraints
		if (isConstraintsToSolve) {
			// Initialize the constraint solver
			m_constraintSolver.initializeForIsland(m_timeStep, m_islands[islandIndex]);
		}
		// For each iteration of the velocity solver
		for (uint32_t i=0; i<m_nbVelocitySolverIterations; i++) {
			// Solve the constraints
			if (isConstraintsToSolve) {
				m_constraintSolver.solveVelocityConstraints(m_islands[islandIndex]);
			}
			// Solve the contacts
			if (isContactsToSolve) m_contactSolver.solve();
		}
		// Cache the lambda values in order to use them in the next
		// step and cleanup the contact solver
		if (isContactsToSolve) {
			m_contactSolver.storeImpulses();
			m_contactSolver.cleanup();
		}
	}
}

void ephysics::DynamicsWorld::solvePositionCorrection() {
	PROFILE("ephysics::DynamicsWorld::solvePositionCorrection()");
	// Do not continue if there is no constraints
	if (m_joints.empty()) {
		return;
	}
	// For each island of the world
	for (uint32_t islandIndex = 0; islandIndex < m_islands.size(); islandIndex++) {
		// ---------- Solve the position error correction for the constraints ---------- //
		// For each iteration of the position (error correction) solver
		for (uint32_t i=0; i<m_nbPositionSolverIterations; i++) {
			// Solve the position constraints
			m_constraintSolver.solvePositionConstraints(m_islands[islandIndex]);
		}
	}
}

ephysics::RigidBody* ephysics::DynamicsWorld::createRigidBody(const etk::Transform3D& _transform) {
	// Compute the body ID
	ephysics::bodyindex bodyID = computeNextAvailableBodyID();
	// Largest index cannot be used (it is used for invalid index)
	assert(bodyID < UINT64_MAX);
	// Create the rigid body
	ephysics::RigidBody* rigidBody = ETK_NEW(RigidBody, _transform, *this, bodyID);
	assert(rigidBody != nullptr);
	// Add the rigid body to the physics world
	m_bodies.add(rigidBody);
	m_rigidBodies.add(rigidBody);
	// Return the pointer to the rigid body
	return rigidBody;
}

void ephysics::DynamicsWorld::destroyRigidBody(RigidBody* _rigidBody) {
	// Remove all the collision shapes of the body
	_rigidBody->removeAllCollisionShapes();
	// Add the body ID to the list of free IDs
	m_freeBodiesIDs.pushBack(_rigidBody->getID());
	// Destroy all the joints in which the rigid body to be destroyed is involved
	for (ephysics::JointListElement* element = _rigidBody->m_jointsList;
	     element != nullptr;
	     element = element->next) {
		destroyJoint(element->joint);
	}
	// Reset the contact manifold list of the body
	_rigidBody->resetContactManifoldsList();
	// Remove the rigid body from the list of rigid bodies
	m_bodies.erase(m_bodies.find(_rigidBody));
	m_rigidBodies.erase(m_rigidBodies.find(_rigidBody));
	// Call the destructor of the rigid body
	ETK_DELETE(RigidBody, _rigidBody);
	_rigidBody = nullptr;
}

ephysics::Joint* ephysics::DynamicsWorld::createJoint(const ephysics::JointInfo& _jointInfo) {
	Joint* newJoint = nullptr;
	// Allocate memory to create the new joint
	switch(_jointInfo.type) {
		// Ball-and-Socket joint
		case BALLSOCKETJOINT:
			newJoint = ETK_NEW(BallAndSocketJoint, static_cast<const ephysics::BallAndSocketJointInfo&>(_jointInfo));
			break;
		// Slider joint
		case SLIDERJOINT:
			newJoint = ETK_NEW(SliderJoint, static_cast<const ephysics::SliderJointInfo&>(_jointInfo));
			break;
		// Hinge joint
		case HINGEJOINT:
			newJoint = ETK_NEW(HingeJoint, static_cast<const ephysics::HingeJointInfo&>(_jointInfo));
			break;
		// Fixed joint
		case FIXEDJOINT:
			newJoint = ETK_NEW(FixedJoint, static_cast<const ephysics::FixedJointInfo&>(_jointInfo));
			break;
		default:
			assert(false);
			return nullptr;
	}
	// If the collision between the two bodies of the constraint is disabled
	if (!_jointInfo.isCollisionEnabled) {
		// Add the pair of bodies in the set of body pairs that cannot collide with each other
		m_collisionDetection.addNoCollisionPair(_jointInfo.body1, _jointInfo.body2);
	}
	// Add the joint int32_to the world
	m_joints.add(newJoint);
	// Add the joint int32_to the joint list of the bodies involved in the joint
	addJointToBody(newJoint);
	// Return the pointer to the created joint
	return newJoint;
}

void ephysics::DynamicsWorld::destroyJoint(Joint* _joint) {
	if (_joint == nullptr) {
		EPHY_WARNING("Request destroy nullptr joint");
		return;
	}
	// If the collision between the two bodies of the constraint was disabled
	if (!_joint->isCollisionEnabled()) {
		// Remove the pair of bodies from the set of body pairs that cannot collide with each other
		m_collisionDetection.removeNoCollisionPair(_joint->getBody1(), _joint->getBody2());
	}
	// Wake up the two bodies of the joint
	_joint->getBody1()->setIsSleeping(false);
	_joint->getBody2()->setIsSleeping(false);
	// Remove the joint from the world
	m_joints.erase(m_joints.find(_joint));
	// Remove the joint from the joint list of the bodies involved in the joint
	_joint->m_body1->removeJointFrom_jointsList(_joint);
	_joint->m_body2->removeJointFrom_jointsList(_joint);
	size_t nbBytes = _joint->getSizeInBytes();
	// Call the destructor of the joint
	ETK_DELETE(Joint, _joint);
	_joint = nullptr;
}

void ephysics::DynamicsWorld::addJointToBody(ephysics::Joint* _joint) {
	if (_joint == nullptr) {
		EPHY_WARNING("Request add nullptr joint");
		return;
	}
	// Add the joint at the beginning of the linked list of joints of the first body
	_joint->m_body1->m_jointsList = ETK_NEW(JointListElement, _joint, _joint->m_body1->m_jointsList);
	// Add the joint at the beginning of the linked list of joints of the second body
	_joint->m_body2->m_jointsList = ETK_NEW(JointListElement, _joint, _joint->m_body2->m_jointsList);
}

void ephysics::DynamicsWorld::computeIslands() {
	PROFILE("ephysics::DynamicsWorld::computeIslands()");
	uint32_t nbBodies = m_rigidBodies.size();
	// Clear all the islands
	for (auto &it: m_islands) {
		ETK_DELETE(Island, it);
		it = nullptr;
	}
	// Call the island destructor
	m_islands.clear();
	int32_t nbContactManifolds = 0;
	// Reset all the isAlreadyInIsland variables of bodies, joints and contact manifolds
	for (etk::Set<ephysics::RigidBody*>::Iterator it = m_rigidBodies.begin(); it != m_rigidBodies.end(); ++it) {
		int32_t nbBodyManifolds = (*it)->resetIsAlreadyInIslandAndCountManifolds();
		nbContactManifolds += nbBodyManifolds;
	}
	for (etk::Set<ephysics::Joint*>::Iterator it = m_joints.begin(); it != m_joints.end(); ++it) {
		(*it)->m_isAlreadyInIsland = false;
	}
	// Create a stack (using an array) for the rigid bodies to visit during the Depth First Search
	etk::Vector<ephysics::RigidBody*> stackBodiesToVisit;
	stackBodiesToVisit.resize(nbBodies, nullptr);
	// For each rigid body of the world
	for (etk::Set<ephysics::RigidBody*>::Iterator it = m_rigidBodies.begin(); it != m_rigidBodies.end(); ++it) {
		ephysics::RigidBody* body = *it;
		// If the body has already been added to an island, we go to the next body
		if (body->m_isAlreadyInIsland) {
			continue;
		}
		// If the body is static, we go to the next body
		if (body->getType() == STATIC) {
			continue;
		}
		// If the body is sleeping or inactive, we go to the next body
		if (body->isSleeping() || !body->isActive()) {
			continue;
		}
		// Reset the stack of bodies to visit
		uint32_t stackIndex = 0;
		stackBodiesToVisit[stackIndex] = body;
		stackIndex++;
		body->m_isAlreadyInIsland = true;
		// Create the new island
		m_islands.pushBack(ETK_NEW(Island, nbBodies, nbContactManifolds, m_joints.size()));
		// While there are still some bodies to visit in the stack
		while (stackIndex > 0) {
			// Get the next body to visit from the stack
			stackIndex--;
			ephysics::RigidBody* bodyToVisit = stackBodiesToVisit[stackIndex];
			assert(bodyToVisit->isActive());
			// Awake the body if it is slepping
			bodyToVisit->setIsSleeping(false);
			// Add the body int32_to the island
			m_islands.back()->addBody(bodyToVisit);
			// If the current body is static, we do not want to perform the DFS
			// search across that body
			if (bodyToVisit->getType() == STATIC) {
				continue;
			}
			// For each contact manifold in which the current body is involded
			ephysics::ContactManifoldListElement* contactElement;
			for (contactElement = bodyToVisit->m_contactManifoldsList;
			     contactElement != nullptr;
			     contactElement = contactElement->next) {
				ephysics::ContactManifold* contactManifold = contactElement->contactManifold;
				assert(contactManifold->getNbContactPoints() > 0);
				// Check if the current contact manifold has already been added int32_to an island
				if (contactManifold->isAlreadyInIsland()) {
					continue;
				}
				// Add the contact manifold int32_to the island
				m_islands.back()->addContactManifold(contactManifold);
				contactManifold->m_isAlreadyInIsland = true;
				// Get the other body of the contact manifold
				ephysics::RigidBody* body1 = static_cast<ephysics::RigidBody*>(contactManifold->getBody1());
				ephysics::RigidBody* body2 = static_cast<ephysics::RigidBody*>(contactManifold->getBody2());
				ephysics::RigidBody* otherBody = (body1->getID() == bodyToVisit->getID()) ? body2 : body1;
				// Check if the other body has already been added to the island
				if (otherBody->m_isAlreadyInIsland) {
					continue;
				}
				// Insert the other body int32_to the stack of bodies to visit
				stackBodiesToVisit[stackIndex] = otherBody;
				stackIndex++;
				otherBody->m_isAlreadyInIsland = true;
			}
			// For each joint in which the current body is involved
			ephysics::JointListElement* jointElement;
			for (jointElement = bodyToVisit->m_jointsList;
			     jointElement != nullptr;
			     jointElement = jointElement->next) {
				ephysics::Joint* joint = jointElement->joint;
				// Check if the current joint has already been added int32_to an island
				if (joint->isAlreadyInIsland()) continue;
				// Add the joint int32_to the island
				m_islands.back()->addJoint(joint);
				joint->m_isAlreadyInIsland = true;
				// Get the other body of the contact manifold
				ephysics::RigidBody* body1 = static_cast<ephysics::RigidBody*>(joint->getBody1());
				ephysics::RigidBody* body2 = static_cast<ephysics::RigidBody*>(joint->getBody2());
				ephysics::RigidBody* otherBody = (body1->getID() == bodyToVisit->getID()) ? body2 : body1;
				// Check if the other body has already been added to the island
				if (otherBody->m_isAlreadyInIsland) continue;
				// Insert the other body int32_to the stack of bodies to visit
				stackBodiesToVisit[stackIndex] = otherBody;
				stackIndex++;
				otherBody->m_isAlreadyInIsland = true;
			}
		}
		m_islands.back()->resetStaticBobyNotInIsland();
	}
}

void ephysics::DynamicsWorld::updateSleepingBodies() {
	PROFILE("ephysics::DynamicsWorld::updateSleepingBodies()");
	const float sleepLinearVelocitySquare = m_sleepLinearVelocity * m_sleepLinearVelocity;
	const float sleepAngularVelocitySquare = m_sleepAngularVelocity * m_sleepAngularVelocity;
	// For each island of the world
	for (uint32_t i=0; i<m_islands.size(); i++) {
		float minSleepTime = FLT_MAX;
		// For each body of the island
		ephysics::RigidBody** bodies = m_islands[i]->getBodies();
		for (uint32_t b=0; b < m_islands[i]->getNbBodies(); b++) {
			// Skip static bodies
			if (bodies[b]->getType() == STATIC) continue;
			// If the body is velocity is large enough to stay awake
			if (bodies[b]->getLinearVelocity().length2() > sleepLinearVelocitySquare ||
				bodies[b]->getAngularVelocity().length2() > sleepAngularVelocitySquare ||
				!bodies[b]->isAllowedToSleep()) {
				// Reset the sleep time of the body
				bodies[b]->m_sleepTime = 0.0f;
				minSleepTime = 0.0f;
			} else {  // If the body velocity is bellow the sleeping velocity threshold
				// Increase the sleep time
				bodies[b]->m_sleepTime += m_timeStep;
				if (bodies[b]->m_sleepTime < minSleepTime) {
					minSleepTime = bodies[b]->m_sleepTime;
				}
			}
		}
		// If the velocity of all the bodies of the island is under the
		// sleeping velocity threshold for a period of time larger than
		// the time required to become a sleeping body
		if (minSleepTime >= m_timeBeforeSleep) {
			// Put all the bodies of the island to sleep
			for (uint32_t b=0; b < m_islands[i]->getNbBodies(); b++) {
				bodies[b]->setIsSleeping(true);
			}
		}
	}
}

void ephysics::DynamicsWorld::enableSleeping(bool _isSleepingEnabled) {
	m_isSleepingEnabled = _isSleepingEnabled;
	if (!m_isSleepingEnabled) {
		// For each body of the world
		etk::Set<ephysics::RigidBody*>::Iterator it;
		for (it = m_rigidBodies.begin(); it != m_rigidBodies.end(); ++it) {
			// Wake up the rigid body
			(*it)->setIsSleeping(false);
		}
	}
}

void ephysics::DynamicsWorld::testCollision(const ephysics::ProxyShape* _shape, ephysics::CollisionCallback* _callback) {
	// Create the sets of shapes
	etk::Set<uint32_t> shapes;
	shapes.add(_shape->m_broadPhaseID);
	etk::Set<uint32_t> emptySet;
	// Perform the collision detection and report contacts
	m_collisionDetection.reportCollisionBetweenShapes(_callback, shapes, emptySet);
}

void ephysics::DynamicsWorld::testCollision(const ephysics::ProxyShape* _shape1, const ephysics::ProxyShape* _shape2, ephysics::CollisionCallback* _callback) {
	// Create the sets of shapes
	etk::Set<uint32_t> shapes1;
	shapes1.add(_shape1->m_broadPhaseID);
	etk::Set<uint32_t> shapes2;
	shapes2.add(_shape2->m_broadPhaseID);
	// Perform the collision detection and report contacts
	m_collisionDetection.reportCollisionBetweenShapes(_callback, shapes1, shapes2);
}

void ephysics::DynamicsWorld::testCollision(const ephysics::CollisionBody* _body, ephysics::CollisionCallback* _callback) {
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
	m_collisionDetection.reportCollisionBetweenShapes(_callback, shapes1, emptySet);
}

void ephysics::DynamicsWorld::testCollision(const ephysics::CollisionBody* _body1, const ephysics::CollisionBody* _body2, ephysics::CollisionCallback* _callback) {
	// Create the sets of shapes
	etk::Set<uint32_t> shapes1;
	for (const ProxyShape* shape=_body1->getProxyShapesList(); shape != nullptr;
		 shape = shape->getNext()) {
		shapes1.add(shape->m_broadPhaseID);
	}
	etk::Set<uint32_t> shapes2;
	for (const ProxyShape* shape=_body2->getProxyShapesList(); shape != nullptr;
		 shape = shape->getNext()) {
		shapes2.add(shape->m_broadPhaseID);
	}
	// Perform the collision detection and report contacts
	m_collisionDetection.reportCollisionBetweenShapes(_callback, shapes1, shapes2);
}

void ephysics::DynamicsWorld::testCollision(ephysics::CollisionCallback* _callback) {
	etk::Set<uint32_t> emptySet;
	// Perform the collision detection and report contacts
	m_collisionDetection.reportCollisionBetweenShapes(_callback, emptySet, emptySet);
}

etk::Vector<const ephysics::ContactManifold*> ephysics::DynamicsWorld::getContactsList() const {
	etk::Vector<const ephysics::ContactManifold*> contactManifolds;
	// For each currently overlapping pair of bodies
	etk::Map<ephysics::overlappingpairid, ephysics::OverlappingPair*>::Iterator it;
	for (it = m_collisionDetection.m_overlappingPairs.begin();
	     it != m_collisionDetection.m_overlappingPairs.end();
	     ++it) {
		ephysics::OverlappingPair* pair = it->second;
		// For each contact manifold of the pair
		const ephysics::ContactManifoldSet& manifoldSet = pair->getContactManifoldSet();
		for (int32_t i=0; i<manifoldSet.getNbContactManifolds(); i++) {
			ContactManifold* manifold = manifoldSet.getContactManifold(i);
			// Get the contact manifold
			contactManifolds.pushBack(manifold);
		}
	}
	// Return all the contact manifold
	return contactManifolds;
}

void ephysics::DynamicsWorld::resetBodiesForceAndTorque() {
	// For each body of the world
	etk::Set<ephysics::RigidBody*>::Iterator it;
	for (it = m_rigidBodies.begin(); it != m_rigidBodies.end(); ++it) {
		(*it)->m_externalForce.setZero();
		(*it)->m_externalTorque.setZero();
	}
}

uint32_t ephysics::DynamicsWorld::getNbIterationsVelocitySolver() const {
	return m_nbVelocitySolverIterations;
}

void ephysics::DynamicsWorld::setNbIterationsVelocitySolver(uint32_t _nbIterations) {
	m_nbVelocitySolverIterations = _nbIterations;
}

uint32_t ephysics::DynamicsWorld::getNbIterationsPositionSolver() const {
	return m_nbPositionSolverIterations;
}

void ephysics::DynamicsWorld::setNbIterationsPositionSolver(uint32_t _nbIterations) {
	m_nbPositionSolverIterations = _nbIterations;
}

void ephysics::DynamicsWorld::setContactsPositionCorrectionTechnique(ephysics::ContactsPositionCorrectionTechnique _technique) {
	if (_technique == BAUMGARTE_CONTACTS) {
		m_contactSolver.setIsSplitImpulseActive(false);
	} else {
		m_contactSolver.setIsSplitImpulseActive(true);
	}
}

void ephysics::DynamicsWorld::setJointsPositionCorrectionTechnique(ephysics::JointsPositionCorrectionTechnique _technique) {
	if (_technique == BAUMGARTE_JOINTS) {
		m_constraintSolver.setIsNonLinearGaussSeidelPositionCorrectionActive(false);
	} else {
		m_constraintSolver.setIsNonLinearGaussSeidelPositionCorrectionActive(true);
	}
}

void ephysics::DynamicsWorld::setIsSolveFrictionAtContactManifoldCenterActive(bool _isActive) {
	m_contactSolver.setIsSolveFrictionAtContactManifoldCenterActive(_isActive);
}

vec3 ephysics::DynamicsWorld::getGravity() const {
	return m_gravity;
}

void ephysics::DynamicsWorld::setGravity(vec3& _gravity) {
	m_gravity = _gravity;
}

bool ephysics::DynamicsWorld::isGravityEnabled() const {
	return m_isGravityEnabled;
}

void ephysics::DynamicsWorld::setIsGratityEnabled(bool _isGravityEnabled) {
	m_isGravityEnabled = _isGravityEnabled;
}

uint32_t ephysics::DynamicsWorld::getNbRigidBodies() const {
	return m_rigidBodies.size();
}

uint32_t ephysics::DynamicsWorld::getNbJoints() const {
	return m_joints.size();
}

etk::Set<ephysics::RigidBody*>::Iterator ephysics::DynamicsWorld::getRigidBodiesBeginIterator() {
	return m_rigidBodies.begin();
}

etk::Set<ephysics::RigidBody*>::Iterator ephysics::DynamicsWorld::getRigidBodiesEndIterator() {
	return m_rigidBodies.end();
}

bool ephysics::DynamicsWorld::isSleepingEnabled() const {
	return m_isSleepingEnabled;
}

float ephysics::DynamicsWorld::getSleepLinearVelocity() const {
	return m_sleepLinearVelocity;
}

void ephysics::DynamicsWorld::setSleepLinearVelocity(float _sleepLinearVelocity) {
	if(_sleepLinearVelocity < 0.0f) {
		EPHY_ERROR("Can not set _sleepLinearVelocity=" << _sleepLinearVelocity << " < 0 ");
		return;
	}
	m_sleepLinearVelocity = _sleepLinearVelocity;
}

float ephysics::DynamicsWorld::getSleepAngularVelocity() const {
	return m_sleepAngularVelocity;
}

void ephysics::DynamicsWorld::setSleepAngularVelocity(float _sleepAngularVelocity) {
	if(_sleepAngularVelocity < 0.0f) {
		EPHY_ERROR("Can not set _sleepAngularVelocity=" << _sleepAngularVelocity << " < 0 ");
		return;
	}
	m_sleepAngularVelocity = _sleepAngularVelocity;
}

float ephysics::DynamicsWorld::getTimeBeforeSleep() const {
	return m_timeBeforeSleep;
}

void ephysics::DynamicsWorld::setTimeBeforeSleep(float _timeBeforeSleep) {
	if(_timeBeforeSleep < 0.0f) {
		EPHY_ERROR("Can not set _timeBeforeSleep=" << _timeBeforeSleep << " < 0 ");
		return;
	}
	m_timeBeforeSleep = _timeBeforeSleep;
}

void ephysics::DynamicsWorld::setEventListener(ephysics::EventListener* _eventListener) {
	m_eventListener = _eventListener;
}

