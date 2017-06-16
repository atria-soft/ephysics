/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/engine/DynamicsWorld.h>
#include <ephysics/constraint/BallAndSocketJoint.h>
#include <ephysics/constraint/SliderJoint.h>
#include <ephysics/constraint/HingeJoint.h>
#include <ephysics/constraint/FixedJoint.h>

// Namespaces
using namespace reactphysics3d;
using namespace std;

// Constructor
/**
 * @param gravity Gravity vector in the world (in meters per second squared)
 */
DynamicsWorld::DynamicsWorld(const vec3 &gravity)
			  : CollisionWorld(),
				m_contactSolver(m_mapBodyToConstrainedVelocityIndex),
				m_constraintSolver(m_mapBodyToConstrainedVelocityIndex),
				m_nbVelocitySolverIterations(DEFAULT_VELOCITY_SOLVER_NB_ITERATIONS),
				m_nbPositionSolverIterations(DEFAULT_POSITION_SOLVER_NB_ITERATIONS),
				m_isSleepingEnabled(SPLEEPING_ENABLED), m_gravity(gravity),
				m_isGravityEnabled(true), m_constrainedLinearVelocities(nullptr),
				m_constrainedAngularVelocities(nullptr), m_splitLinearVelocities(nullptr),
				m_splitAngularVelocities(nullptr), m_constrainedPositions(nullptr),
				m_constrainedOrientations(nullptr), m_numberIslands(0),
				m_numberIslandsCapacity(0), m_islands(nullptr), m_numberBodiesCapacity(0),
				m_sleepLinearVelocity(DEFAULT_SLEEP_LINEAR_VELOCITY),
				m_sleepAngularVelocity(DEFAULT_SLEEP_ANGULAR_VELOCITY),
				m_timeBeforeSleep(DEFAULT_TIME_BEFORE_SLEEP) {

}

// Destructor
DynamicsWorld::~DynamicsWorld() {

	// Destroy all the joints that have not been removed
	std::set<Joint*>::iterator itJoints;
	for (itJoints = m_joints.begin(); itJoints != m_joints.end();) {
		std::set<Joint*>::iterator itToRemove = itJoints;
		++itJoints;
		destroyJoint(*itToRemove);
	}

	// Destroy all the rigid bodies that have not been removed
	std::set<RigidBody*>::iterator itRigidBodies;
	for (itRigidBodies = m_rigidBodies.begin(); itRigidBodies != m_rigidBodies.end();) {
		std::set<RigidBody*>::iterator itToRemove = itRigidBodies;
		++itRigidBodies;
		destroyRigidBody(*itToRemove);
	}

	// Release the memory allocated for the islands
	for (uint32_t i=0; i<m_numberIslands; i++) {

		// Call the island destructor
		m_islands[i]->~Island();

		// Release the allocated memory for the island
		m_memoryAllocator.release(m_islands[i], sizeof(Island));
	}
	if (m_numberIslandsCapacity > 0) {
		m_memoryAllocator.release(m_islands, sizeof(Island*) * m_numberIslandsCapacity);
	}

	// Release the memory allocated for the bodies velocity arrays
	if (m_numberBodiesCapacity > 0) {
		delete[] m_splitLinearVelocities;
		delete[] m_splitAngularVelocities;
		delete[] m_constrainedLinearVelocities;
		delete[] m_constrainedAngularVelocities;
		delete[] m_constrainedPositions;
		delete[] m_constrainedOrientations;
	}

	assert(m_joints.size() == 0);
	assert(m_rigidBodies.size() == 0);

#ifdef IS_PROFILING_ACTIVE

	// Print32_t the profiling report
	Profiler::print32_tReport(std::cout);

	// Destroy the profiler (release the allocated memory)
	Profiler::destroy();
#endif

}

// Update the physics simulation
/**
 * @param timeStep The amount of time to step the simulation by (in seconds)
 */
void DynamicsWorld::update(float timeStep) {

	#ifdef IS_PROFILING_ACTIVE
		// Increment the frame counter of the profiler
		Profiler::incrementFrameCounter();
	#endif

	PROFILE("DynamicsWorld::update()");
	
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
	int32_tegrateRigidBodiesVelocities();
	
	// Solve the contacts and constraints
	solveContactsAndConstraints();
	
	// Integrate the position and orientation of each body
	int32_tegrateRigidBodiesPositions();
	
	// Solve the position correction for constraints
	solvePositionCorrection();
	
	// Update the state (positions and velocities) of the bodies
	updateBodiesState();
	
	if (m_isSleepingEnabled) updateSleepingBodies();
	
	// Notify the event listener about the end of an int32_ternal tick
	if (m_eventListener != nullptr) m_eventListener->endInternalTick();
	
	// Reset the external force and torque applied to the bodies
	resetBodiesForceAndTorque();
}

// Integrate position and orientation of the rigid bodies.
/// The positions and orientations of the bodies are int32_tegrated using
/// the sympletic Euler time stepping scheme.
void DynamicsWorld::int32_tegrateRigidBodiesPositions() {

	PROFILE("DynamicsWorld::int32_tegrateRigidBodiesPositions()");
	
	// For each island of the world
	for (uint32_t i=0; i < m_numberIslands; i++) {

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

// Update the postion/orientation of the bodies
void DynamicsWorld::updateBodiesState() {

	PROFILE("DynamicsWorld::updateBodiesState()");

	// For each island of the world
	for (uint32_t islandIndex = 0; islandIndex < m_numberIslands; islandIndex++) {

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

// Initialize the bodies velocities arrays for the next simulation step.
void DynamicsWorld::initVelocityArrays() {

	// Allocate memory for the bodies velocity arrays
	uint32_t nbBodies = m_rigidBodies.size();
	if (m_numberBodiesCapacity != nbBodies && nbBodies > 0) {
		if (m_numberBodiesCapacity > 0) {
			delete[] m_splitLinearVelocities;
			delete[] m_splitAngularVelocities;
		}
		m_numberBodiesCapacity = nbBodies;
		// TODO : Use better memory allocation here
		m_splitLinearVelocities = new vec3[m_numberBodiesCapacity];
		m_splitAngularVelocities = new vec3[m_numberBodiesCapacity];
		m_constrainedLinearVelocities = new vec3[m_numberBodiesCapacity];
		m_constrainedAngularVelocities = new vec3[m_numberBodiesCapacity];
		m_constrainedPositions = new vec3[m_numberBodiesCapacity];
		m_constrainedOrientations = new etk::Quaternion[m_numberBodiesCapacity];
		assert(m_splitLinearVelocities != nullptr);
		assert(m_splitAngularVelocities != nullptr);
		assert(m_constrainedLinearVelocities != nullptr);
		assert(m_constrainedAngularVelocities != nullptr);
		assert(m_constrainedPositions != nullptr);
		assert(m_constrainedOrientations != nullptr);
	}

	// Reset the velocities arrays
	for (uint32_t i=0; i<m_numberBodiesCapacity; i++) {
		m_splitLinearVelocities[i].setZero();
		m_splitAngularVelocities[i].setZero();
	}

	// Initialize the map of body indexes in the velocity arrays
	m_mapBodyToConstrainedVelocityIndex.clear();
	std::set<RigidBody*>::const_iterator it;
	uint32_t indexBody = 0;
	for (it = m_rigidBodies.begin(); it != m_rigidBodies.end(); ++it) {

		// Add the body int32_to the map
		m_mapBodyToConstrainedVelocityIndex.insert(std::make_pair(*it, indexBody));
		indexBody++;
	}
}

// Integrate the velocities of rigid bodies.
/// This method only set the temporary velocities but does not update
/// the actual velocitiy of the bodies. The velocities updated in this method
/// might violate the constraints and will be corrected in the constraint and
/// contact solver.
void DynamicsWorld::int32_tegrateRigidBodiesVelocities() {

	PROFILE("DynamicsWorld::int32_tegrateRigidBodiesVelocities()");

	// Initialize the bodies velocity arrays
	initVelocityArrays();

	// For each island of the world
	for (uint32_t i=0; i < m_numberIslands; i++) {

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
				m_constrainedLinearVelocities[indexBody] += m_timeStep * bodies[b]->m_massInverse *
						bodies[b]->getMass() * m_gravity;
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

// Solve the contacts and constraints
void DynamicsWorld::solveContactsAndConstraints() {

	PROFILE("DynamicsWorld::solveContactsAndConstraints()");

	// Set the velocities arrays
	m_contactSolver.setSplitVelocitiesArrays(m_splitLinearVelocities, m_splitAngularVelocities);
	m_contactSolver.setConstrainedVelocitiesArrays(m_constrainedLinearVelocities,
												  m_constrainedAngularVelocities);
	m_constraintSolver.setConstrainedVelocitiesArrays(m_constrainedLinearVelocities,
													 m_constrainedAngularVelocities);
	m_constraintSolver.setConstrainedPositionsArrays(m_constrainedPositions,
													m_constrainedOrientations);

	// ---------- Solve velocity constraints for joints and contacts ---------- //

	// For each island of the world
	for (uint32_t islandIndex = 0; islandIndex < m_numberIslands; islandIndex++) {

		// Check if there are contacts and constraints to solve
		bool isConstraintsToSolve = m_islands[islandIndex]->getNbJoints() > 0;
		bool isContactsToSolve = m_islands[islandIndex]->getNbContactManifolds() > 0;
		if (!isConstraintsToSolve && !isContactsToSolve) continue;

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

// Solve the position error correction of the constraints
void DynamicsWorld::solvePositionCorrection() {

	PROFILE("DynamicsWorld::solvePositionCorrection()");

	// Do not continue if there is no constraints
	if (m_joints.empty()) return;

	// For each island of the world
	for (uint32_t islandIndex = 0; islandIndex < m_numberIslands; islandIndex++) {

		// ---------- Solve the position error correction for the constraints ---------- //

		// For each iteration of the position (error correction) solver
		for (uint32_t i=0; i<m_nbPositionSolverIterations; i++) {

			// Solve the position constraints
			m_constraintSolver.solvePositionConstraints(m_islands[islandIndex]);
		}
	}
}

// Create a rigid body int32_to the physics world
/**
 * @param transform etk::Transform3Dation from body local-space to world-space
 * @return A pointer to the body that has been created in the world
 */
RigidBody* DynamicsWorld::createRigidBody(const etk::Transform3D& transform) {

	// Compute the body ID
	bodyindex bodyID = computeNextAvailableBodyID();

	// Largest index cannot be used (it is used for invalid index)
	assert(bodyID < std::numeric_limits<reactphysics3d::bodyindex>::max());

	// Create the rigid body
	RigidBody* rigidBody = new (m_memoryAllocator.allocate(sizeof(RigidBody))) RigidBody(transform,
																				*this, bodyID);
	assert(rigidBody != nullptr);

	// Add the rigid body to the physics world
	m_bodies.insert(rigidBody);
	m_rigidBodies.insert(rigidBody);

	// Return the pointer to the rigid body
	return rigidBody;
}

// Destroy a rigid body and all the joints which it belongs
/**
 * @param rigidBody Pointer to the body you want to destroy
 */
void DynamicsWorld::destroyRigidBody(RigidBody* rigidBody) {

	// Remove all the collision shapes of the body
	rigidBody->removeAllCollisionShapes();

	// Add the body ID to the list of free IDs
	m_freeBodiesIDs.push_back(rigidBody->getID());

	// Destroy all the joints in which the rigid body to be destroyed is involved
	JointListElement* element;
	for (element = rigidBody->m_jointsList; element != nullptr; element = element->next) {
		destroyJoint(element->joint);
	}

	// Reset the contact manifold list of the body
	rigidBody->resetContactManifoldsList();

	// Call the destructor of the rigid body
	rigidBody->~RigidBody();

	// Remove the rigid body from the list of rigid bodies
	m_bodies.erase(rigidBody);
	m_rigidBodies.erase(rigidBody);

	// Free the object from the memory allocator
	m_memoryAllocator.release(rigidBody, sizeof(RigidBody));
}

// Create a joint between two bodies in the world and return a pointer to the new joint
/**
 * @param jointInfo The information that is necessary to create the joint
 * @return A pointer to the joint that has been created in the world
 */
Joint* DynamicsWorld::createJoint(const JointInfo& jointInfo) {

	Joint* newJoint = nullptr;

	// Allocate memory to create the new joint
	switch(jointInfo.type) {

		// Ball-and-Socket joint
		case BALLSOCKETJOINT:
		{
			void* allocatedMemory = m_memoryAllocator.allocate(sizeof(BallAndSocketJoint));
			const BallAndSocketJointInfo& info = static_cast<const BallAndSocketJointInfo&>(
																						jointInfo);
			newJoint = new (allocatedMemory) BallAndSocketJoint(info);
			break;
		}

		// Slider joint
		case SLIDERJOINT:
		{
			void* allocatedMemory = m_memoryAllocator.allocate(sizeof(SliderJoint));
			const SliderJointInfo& info = static_cast<const SliderJointInfo&>(jointInfo);
			newJoint = new (allocatedMemory) SliderJoint(info);
			break;
		}

		// Hinge joint
		case HINGEJOINT:
		{
			void* allocatedMemory = m_memoryAllocator.allocate(sizeof(HingeJoint));
			const HingeJointInfo& info = static_cast<const HingeJointInfo&>(jointInfo);
			newJoint = new (allocatedMemory) HingeJoint(info);
			break;
		}

		// Fixed joint
		case FIXEDJOINT:
		{
			void* allocatedMemory = m_memoryAllocator.allocate(sizeof(FixedJoint));
			const FixedJointInfo& info = static_cast<const FixedJointInfo&>(jointInfo);
			newJoint = new (allocatedMemory) FixedJoint(info);
			break;
		}

		default:
		{
			assert(false);
			return nullptr;
		}
	}

	// If the collision between the two bodies of the constraint is disabled
	if (!jointInfo.isCollisionEnabled) {

		// Add the pair of bodies in the set of body pairs that cannot collide with each other
		m_collisionDetection.addNoCollisionPair(jointInfo.body1, jointInfo.body2);
	}

	// Add the joint int32_to the world
	m_joints.insert(newJoint);

	// Add the joint int32_to the joint list of the bodies involved in the joint
	addJointToBody(newJoint);

	// Return the pointer to the created joint
	return newJoint;
}

// Destroy a joint
/**
 * @param joint Pointer to the joint you want to destroy
 */
void DynamicsWorld::destroyJoint(Joint* joint) {

	assert(joint != nullptr);

	// If the collision between the two bodies of the constraint was disabled
	if (!joint->isCollisionEnabled()) {

		// Remove the pair of bodies from the set of body pairs that cannot collide with each other
		m_collisionDetection.removeNoCollisionPair(joint->getBody1(), joint->getBody2());
	}

	// Wake up the two bodies of the joint
	joint->getBody1()->setIsSleeping(false);
	joint->getBody2()->setIsSleeping(false);

	// Remove the joint from the world
	m_joints.erase(joint);

	// Remove the joint from the joint list of the bodies involved in the joint
	joint->m_body1->removeJointFrom_jointsList(m_memoryAllocator, joint);
	joint->m_body2->removeJointFrom_jointsList(m_memoryAllocator, joint);

	size_t nbBytes = joint->getSizeInBytes();

	// Call the destructor of the joint
	joint->~Joint();

	// Release the allocated memory
	m_memoryAllocator.release(joint, nbBytes);
}

// Add the joint to the list of joints of the two bodies involved in the joint
void DynamicsWorld::addJointToBody(Joint* joint) {

	assert(joint != nullptr);

	// Add the joint at the beginning of the linked list of joints of the first body
	void* allocatedMemory1 = m_memoryAllocator.allocate(sizeof(JointListElement));
	JointListElement* jointListElement1 = new (allocatedMemory1) JointListElement(joint,
																	 joint->m_body1->m_jointsList);
	joint->m_body1->m_jointsList = jointListElement1;

	// Add the joint at the beginning of the linked list of joints of the second body
	void* allocatedMemory2 = m_memoryAllocator.allocate(sizeof(JointListElement));
	JointListElement* jointListElement2 = new (allocatedMemory2) JointListElement(joint,
																	 joint->m_body2->m_jointsList);
	joint->m_body2->m_jointsList = jointListElement2;
}

// Compute the islands of awake bodies.
/// An island is an isolated group of rigid bodies that have constraints (joints or contacts)
/// between each other. This method computes the islands at each time step as follows: For each
/// awake rigid body, we run a Depth First Search (DFS) through the constraint graph of that body
/// (graph where nodes are the bodies and where the edges are the constraints between the bodies) to
/// find all the bodies that are connected with it (the bodies that share joints or contacts with
/// it). Then, we create an island with this group of connected bodies.
void DynamicsWorld::computeIslands() {

	PROFILE("DynamicsWorld::computeIslands()");

	uint32_t nbBodies = m_rigidBodies.size();

	// Clear all the islands
	for (uint32_t i=0; i<m_numberIslands; i++) {

		// Call the island destructor
		m_islands[i]->~Island();

		// Release the allocated memory for the island
		m_memoryAllocator.release(m_islands[i], sizeof(Island));
	}

	// Allocate and create the array of islands
	if (m_numberIslandsCapacity != nbBodies && nbBodies > 0) {
		if (m_numberIslandsCapacity > 0) {
			m_memoryAllocator.release(m_islands, sizeof(Island*) * m_numberIslandsCapacity);
		}
		m_numberIslandsCapacity = nbBodies;
		m_islands = (Island**)m_memoryAllocator.allocate(sizeof(Island*) * m_numberIslandsCapacity);
	}
	m_numberIslands = 0;

	int32_t nbContactManifolds = 0;

	// Reset all the isAlreadyInIsland variables of bodies, joints and contact manifolds
	for (std::set<RigidBody*>::iterator it = m_rigidBodies.begin(); it != m_rigidBodies.end(); ++it) {
		int32_t nbBodyManifolds = (*it)->resetIsAlreadyInIslandAndCountManifolds();
		nbContactManifolds += nbBodyManifolds;
	}
	for (std::set<Joint*>::iterator it = m_joints.begin(); it != m_joints.end(); ++it) {
		(*it)->m_isAlreadyInIsland = false;
	}

	// Create a stack (using an array) for the rigid bodies to visit during the Depth First Search
	size_t nbBytesStack = sizeof(RigidBody*) * nbBodies;
	RigidBody** stackBodiesToVisit = (RigidBody**)m_memoryAllocator.allocate(nbBytesStack);

	// For each rigid body of the world
	for (std::set<RigidBody*>::iterator it = m_rigidBodies.begin(); it != m_rigidBodies.end(); ++it) {

		RigidBody* body = *it;

		// If the body has already been added to an island, we go to the next body
		if (body->m_isAlreadyInIsland) continue;

		// If the body is static, we go to the next body
		if (body->getType() == STATIC) continue;

		// If the body is sleeping or inactive, we go to the next body
		if (body->isSleeping() || !body->isActive()) continue;

		// Reset the stack of bodies to visit
		uint32_t stackIndex = 0;
		stackBodiesToVisit[stackIndex] = body;
		stackIndex++;
		body->m_isAlreadyInIsland = true;

		// Create the new island
		void* allocatedMemoryIsland = m_memoryAllocator.allocate(sizeof(Island));
		m_islands[m_numberIslands] = new (allocatedMemoryIsland) Island(nbBodies,
																  nbContactManifolds,
																  m_joints.size(), m_memoryAllocator);

		// While there are still some bodies to visit in the stack
		while (stackIndex > 0) {

			// Get the next body to visit from the stack
			stackIndex--;
			RigidBody* bodyToVisit = stackBodiesToVisit[stackIndex];
			assert(bodyToVisit->isActive());

			// Awake the body if it is slepping
			bodyToVisit->setIsSleeping(false);

			// Add the body int32_to the island
			m_islands[m_numberIslands]->addBody(bodyToVisit);

			// If the current body is static, we do not want to perform the DFS
			// search across that body
			if (bodyToVisit->getType() == STATIC) continue;

			// For each contact manifold in which the current body is involded
			ContactManifoldListElement* contactElement;
			for (contactElement = bodyToVisit->m_contactManifoldsList; contactElement != nullptr;
				 contactElement = contactElement->next) {

				ContactManifold* contactManifold = contactElement->contactManifold;

				assert(contactManifold->getNbContactPoints() > 0);

				// Check if the current contact manifold has already been added int32_to an island
				if (contactManifold->isAlreadyInIsland()) continue;

				// Add the contact manifold int32_to the island
				m_islands[m_numberIslands]->addContactManifold(contactManifold);
				contactManifold->m_isAlreadyInIsland = true;

				// Get the other body of the contact manifold
				RigidBody* body1 = static_cast<RigidBody*>(contactManifold->getBody1());
				RigidBody* body2 = static_cast<RigidBody*>(contactManifold->getBody2());
				RigidBody* otherBody = (body1->getID() == bodyToVisit->getID()) ? body2 : body1;

				// Check if the other body has already been added to the island
				if (otherBody->m_isAlreadyInIsland) continue;

				// Insert the other body int32_to the stack of bodies to visit
				stackBodiesToVisit[stackIndex] = otherBody;
				stackIndex++;
				otherBody->m_isAlreadyInIsland = true;
			}

			// For each joint in which the current body is involved
			JointListElement* jointElement;
			for (jointElement = bodyToVisit->m_jointsList; jointElement != nullptr;
				 jointElement = jointElement->next) {

				Joint* joint = jointElement->joint;

				// Check if the current joint has already been added int32_to an island
				if (joint->isAlreadyInIsland()) continue;

				// Add the joint int32_to the island
				m_islands[m_numberIslands]->addJoint(joint);
				joint->m_isAlreadyInIsland = true;

				// Get the other body of the contact manifold
				RigidBody* body1 = static_cast<RigidBody*>(joint->getBody1());
				RigidBody* body2 = static_cast<RigidBody*>(joint->getBody2());
				RigidBody* otherBody = (body1->getID() == bodyToVisit->getID()) ? body2 : body1;

				// Check if the other body has already been added to the island
				if (otherBody->m_isAlreadyInIsland) continue;

				// Insert the other body int32_to the stack of bodies to visit
				stackBodiesToVisit[stackIndex] = otherBody;
				stackIndex++;
				otherBody->m_isAlreadyInIsland = true;
			}
		}

		// Reset the isAlreadyIsland variable of the static bodies so that they
		// can also be included in the other islands
		for (uint32_t i=0; i < m_islands[m_numberIslands]->m_numberBodies; i++) {

			if (m_islands[m_numberIslands]->m_bodies[i]->getType() == STATIC) {
				m_islands[m_numberIslands]->m_bodies[i]->m_isAlreadyInIsland = false;
			}
		}

		m_numberIslands++;
	 }

	// Release the allocated memory for the stack of bodies to visit
	m_memoryAllocator.release(stackBodiesToVisit, nbBytesStack);
}

// Put bodies to sleep if needed.
/// For each island, if all the bodies have been almost still for a long enough period of
/// time, we put all the bodies of the island to sleep.
void DynamicsWorld::updateSleepingBodies() {

	PROFILE("DynamicsWorld::updateSleepingBodies()");

	const float sleepLinearVelocitySquare = m_sleepLinearVelocity * m_sleepLinearVelocity;
	const float sleepAngularVelocitySquare = m_sleepAngularVelocity * m_sleepAngularVelocity;

	// For each island of the world
	for (uint32_t i=0; i<m_numberIslands; i++) {

		float minSleepTime = DECIMAL_LARGEST;

		// For each body of the island
		RigidBody** bodies = m_islands[i]->getBodies();
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
			}
			else {  // If the body velocity is bellow the sleeping velocity threshold

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

// Enable/Disable the sleeping technique.
/// The sleeping technique is used to put bodies that are not moving int32_to sleep
/// to speed up the simulation.
/**
 * @param isSleepingEnabled True if you want to enable the sleeping technique
 *						  and false otherwise
 */
void DynamicsWorld::enableSleeping(bool isSleepingEnabled) {
	m_isSleepingEnabled = isSleepingEnabled;

	if (!m_isSleepingEnabled) {

		// For each body of the world
		std::set<RigidBody*>::iterator it;
		for (it = m_rigidBodies.begin(); it != m_rigidBodies.end(); ++it) {

			// Wake up the rigid body
			(*it)->setIsSleeping(false);
		}
	}
}

// Test and report collisions between a given shape and all the others
// shapes of the world.
/// This method should be called after calling the
/// DynamicsWorld::update() method that will compute the collisions.
/**
 * @param shape Pointer to the proxy shape to test
 * @param callback Pointer to the object with the callback method
 */
void DynamicsWorld::testCollision(const ProxyShape* shape,
								   CollisionCallback* callback) {

	// Create the sets of shapes
	std::set<uint32_t> shapes;
	shapes.insert(shape->m_broadPhaseID);
	std::set<uint32_t> emptySet;

	// Perform the collision detection and report contacts
	m_collisionDetection.reportCollisionBetweenShapes(callback, shapes, emptySet);
}

// Test and report collisions between two given shapes.
/// This method should be called after calling the
/// DynamicsWorld::update() method that will compute the collisions.
/**
 * @param shape1 Pointer to the first proxy shape to test
 * @param shape2 Pointer to the second proxy shape to test
 * @param callback Pointer to the object with the callback method
 */
void DynamicsWorld::testCollision(const ProxyShape* shape1,
								   const ProxyShape* shape2,
								   CollisionCallback* callback) {

	// Create the sets of shapes
	std::set<uint32_t> shapes1;
	shapes1.insert(shape1->m_broadPhaseID);
	std::set<uint32_t> shapes2;
	shapes2.insert(shape2->m_broadPhaseID);

	// Perform the collision detection and report contacts
	m_collisionDetection.reportCollisionBetweenShapes(callback, shapes1, shapes2);
}

// Test and report collisions between a body and all the others bodies of the
// world.
/// This method should be called after calling the
/// DynamicsWorld::update() method that will compute the collisions.
/**
 * @param body Pointer to the first body to test
 * @param callback Pointer to the object with the callback method
 */
void DynamicsWorld::testCollision(const CollisionBody* body,
								   CollisionCallback* callback) {

	// Create the sets of shapes
	std::set<uint32_t> shapes1;

	// For each shape of the body
	for (const ProxyShape* shape=body->getProxyShapesList(); shape != nullptr;
		 shape = shape->getNext()) {
		shapes1.insert(shape->m_broadPhaseID);
	}

	std::set<uint32_t> emptySet;

	// Perform the collision detection and report contacts
	m_collisionDetection.reportCollisionBetweenShapes(callback, shapes1, emptySet);
}

// Test and report collisions between two bodies.
/// This method should be called after calling the
/// DynamicsWorld::update() method that will compute the collisions.
/**
 * @param body1 Pointer to the first body to test
 * @param body2 Pointer to the second body to test
 * @param callback Pointer to the object with the callback method
 */
void DynamicsWorld::testCollision(const CollisionBody* body1,
								   const CollisionBody* body2,
								   CollisionCallback* callback) {

	// Create the sets of shapes
	std::set<uint32_t> shapes1;
	for (const ProxyShape* shape=body1->getProxyShapesList(); shape != nullptr;
		 shape = shape->getNext()) {
		shapes1.insert(shape->m_broadPhaseID);
	}

	std::set<uint32_t> shapes2;
	for (const ProxyShape* shape=body2->getProxyShapesList(); shape != nullptr;
		 shape = shape->getNext()) {
		shapes2.insert(shape->m_broadPhaseID);
	}

	// Perform the collision detection and report contacts
	m_collisionDetection.reportCollisionBetweenShapes(callback, shapes1, shapes2);
}

// Test and report collisions between all shapes of the world.
/// This method should be called after calling the
/// DynamicsWorld::update() method that will compute the collisions.
/**
 * @param callback Pointer to the object with the callback method
 */
void DynamicsWorld::testCollision(CollisionCallback* callback) {

	std::set<uint32_t> emptySet;

	// Perform the collision detection and report contacts
	m_collisionDetection.reportCollisionBetweenShapes(callback, emptySet, emptySet);
}

/// Return the list of all contacts of the world
std::vector<const ContactManifold*> DynamicsWorld::getContactsList() const {

	std::vector<const ContactManifold*> contactManifolds;

	// For each currently overlapping pair of bodies
	std::map<overlappingpairid, OverlappingPair*>::const_iterator it;
	for (it = m_collisionDetection.m_overlappingPairs.begin();
		 it != m_collisionDetection.m_overlappingPairs.end(); ++it) {

		OverlappingPair* pair = it->second;

		// For each contact manifold of the pair
		const ContactManifoldSet& manifoldSet = pair->getContactManifoldSet();
		for (int32_t i=0; i<manifoldSet.getNbContactManifolds(); i++) {

			ContactManifold* manifold = manifoldSet.getContactManifold(i);

			// Get the contact manifold
			contactManifolds.push_back(manifold);
		}
	}

	// Return all the contact manifold
	return contactManifolds;
}
