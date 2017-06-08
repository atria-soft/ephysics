/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/engine/CollisionWorld.h>
#include <ephysics/collision/CollisionDetection.h>
#include <ephysics/engine/ContactSolver.h>
#include <ephysics/engine/ConstraintSolver.h>
#include <ephysics/body/RigidBody.h>
#include <ephysics/engine/Island.h>
#include <ephysics/configuration.h>

/// Namespace ReactPhysics3D
namespace reactphysics3d {

// Class DynamicsWorld
/**
 * This class represents a dynamics world. This class inherits from
 * the CollisionWorld class. In a dynamics world, bodies can collide
 * and their movements are simulated using the laws of physics.
 */
class DynamicsWorld : public CollisionWorld {

	protected :

		// -------------------- Attributes -------------------- //

		/// Contact solver
		ContactSolver mContactSolver;

		/// Constraint solver
		ConstraintSolver mConstraintSolver;

		/// Number of iterations for the velocity solver of the Sequential Impulses technique
		uint32_t mNbVelocitySolverIterations;

		/// Number of iterations for the position solver of the Sequential Impulses technique
		uint32_t mNbPositionSolverIterations;

		/// True if the spleeping technique for inactive bodies is enabled
		bool m_isSleepingEnabled;

		/// All the rigid bodies of the physics world
		std::set<RigidBody*> mRigidBodies;

		/// All the joints of the world
		std::set<Joint*> mJoints;

		/// Gravity vector of the world
		Vector3 mGravity;

		/// Current frame time step (in seconds)
		float mTimeStep;

		/// True if the gravity force is on
		bool mIsGravityEnabled;

		/// Array of constrained linear velocities (state of the linear velocities
		/// after solving the constraints)
		Vector3* mConstrainedLinearVelocities;

		/// Array of constrained angular velocities (state of the angular velocities
		/// after solving the constraints)
		Vector3* mConstrainedAngularVelocities;

		/// Split linear velocities for the position contact solver (split impulse)
		Vector3* mSplitLinearVelocities;

		/// Split angular velocities for the position contact solver (split impulse)
		Vector3* mSplitAngularVelocities;

		/// Array of constrained rigid bodies position (for position error correction)
		Vector3* mConstrainedPositions;

		/// Array of constrained rigid bodies orientation (for position error correction)
		Quaternion* mConstrainedOrientations;

		/// Map body to their index in the constrained velocities array
		std::map<RigidBody*, uint32_t> mMapBodyToConstrainedVelocityIndex;

		/// Number of islands in the world
		uint32_t mNbIslands;

		/// Current allocated capacity for the islands
		uint32_t mNbIslandsCapacity;

		/// Array with all the islands of awaken bodies
		Island** mIslands;

		/// Current allocated capacity for the bodies
		uint32_t mNbBodiesCapacity;

		/// Sleep linear velocity threshold
		float mSleepLinearVelocity;

		/// Sleep angular velocity threshold
		float mSleepAngularVelocity;

		/// Time (in seconds) before a body is put to sleep if its velocity
		/// becomes smaller than the sleep velocity.
		float mTimeBeforeSleep;

		// -------------------- Methods -------------------- //

		/// Private copy-constructor
		DynamicsWorld(const DynamicsWorld& world);

		/// Private assignment operator
		DynamicsWorld& operator=(const DynamicsWorld& world);

		/// Integrate the positions and orientations of rigid bodies.
		void int32_tegrateRigidBodiesPositions();

		/// Update the AABBs of the bodies
		void updateRigidBodiesAABB();

		/// Reset the external force and torque applied to the bodies
		void resetBodiesForceAndTorque();

		/// Update the position and orientation of a body
		void updatePositionAndOrientationOfBody(RigidBody* body, Vector3 newLinVelocity,
												Vector3 newAngVelocity);

		/// Compute and set the int32_terpolation factor to all bodies
		void setInterpolationFactorToAllBodies();

		/// Initialize the bodies velocities arrays for the next simulation step.
		void initVelocityArrays();

		/// Integrate the velocities of rigid bodies.
		void int32_tegrateRigidBodiesVelocities();

		/// Solve the contacts and constraints
		void solveContactsAndConstraints();

		/// Solve the position error correction of the constraints
		void solvePositionCorrection();

		/// Cleanup the constrained velocities array at each step
		void cleanupConstrainedVelocitiesArray();

		/// Compute the islands of awake bodies.
		void computeIslands();

		/// Update the postion/orientation of the bodies
		void updateBodiesState();

		/// Put bodies to sleep if needed.
		void updateSleepingBodies();

		/// Add the joint to the list of joints of the two bodies involved in the joint
		void addJointToBody(Joint* joint);

	public :

		// -------------------- Methods -------------------- //

		/// Constructor
		DynamicsWorld(const Vector3& mGravity);

		/// Destructor
		virtual ~DynamicsWorld();

		/// Update the physics simulation
		void update(float timeStep);

		/// Get the number of iterations for the velocity constraint solver
		uint32_t getNbIterationsVelocitySolver() const;

		/// Set the number of iterations for the velocity constraint solver
		void setNbIterationsVelocitySolver(uint32_t nbIterations);

		/// Get the number of iterations for the position constraint solver
		uint32_t getNbIterationsPositionSolver() const;

		/// Set the number of iterations for the position constraint solver
		void setNbIterationsPositionSolver(uint32_t nbIterations);

		/// Set the position correction technique used for contacts
		void setContactsPositionCorrectionTechnique(ContactsPositionCorrectionTechnique technique);

		/// Set the position correction technique used for joints
		void setJointsPositionCorrectionTechnique(JointsPositionCorrectionTechnique technique);

		/// Activate or deactivate the solving of friction constraints at the center of
		/// the contact manifold instead of solving them at each contact point
		void setIsSolveFrictionAtContactManifoldCenterActive(bool isActive);

		/// Create a rigid body int32_to the physics world.
		RigidBody* createRigidBody(const Transform& transform);

		/// Destroy a rigid body and all the joints which it belongs
		void destroyRigidBody(RigidBody* rigidBody);

		/// Create a joint between two bodies in the world and return a pointer to the new joint
		Joint* createJoint(const JointInfo& jointInfo);

		/// Destroy a joint
		void destroyJoint(Joint* joint);

		/// Return the gravity vector of the world
		Vector3 getGravity() const;

		/// Set the gravity vector of the world
		void setGravity(Vector3& gravity);

		/// Return if the gravity is on
		bool isGravityEnabled() const;

		/// Enable/Disable the gravity
		void setIsGratityEnabled(bool isGravityEnabled);

		/// Return the number of rigid bodies in the world
		uint32_t getNbRigidBodies() const;

		/// Return the number of joints in the world
		uint32_t getNbJoints() const;

		/// Return an iterator to the beginning of the rigid bodies of the physics world
		std::set<RigidBody*>::iterator getRigidBodiesBeginIterator();

		/// Return an iterator to the end of the rigid bodies of the physics world
		std::set<RigidBody*>::iterator getRigidBodiesEndIterator();

		/// Return true if the sleeping technique is enabled
		bool isSleepingEnabled() const;

		/// Enable/Disable the sleeping technique
		void enableSleeping(bool isSleepingEnabled);

		/// Return the current sleep linear velocity
		float getSleepLinearVelocity() const;

		/// Set the sleep linear velocity.
		void setSleepLinearVelocity(float sleepLinearVelocity);

		/// Return the current sleep angular velocity
		float getSleepAngularVelocity() const;

		/// Set the sleep angular velocity.
		void setSleepAngularVelocity(float sleepAngularVelocity);

		/// Return the time a body is required to stay still before sleeping
		float getTimeBeforeSleep() const;

		/// Set the time a body is required to stay still before sleeping
		void setTimeBeforeSleep(float timeBeforeSleep);

		/// Set an event listener object to receive events callbacks.
		void setEventListener(EventListener* eventListener);

		/// Test and report collisions between a given shape and all the others
		/// shapes of the world
		virtual void testCollision(const ProxyShape* shape,
								   CollisionCallback* callback);

		/// Test and report collisions between two given shapes
		virtual void testCollision(const ProxyShape* shape1,
								   const ProxyShape* shape2,
								   CollisionCallback* callback);

		/// Test and report collisions between a body and all
		/// the others bodies of the world
		virtual void testCollision(const CollisionBody* body,
								   CollisionCallback* callback);

		/// Test and report collisions between two bodies
		virtual void testCollision(const CollisionBody* body1,
								   const CollisionBody* body2,
								   CollisionCallback* callback);

		/// Test and report collisions between all shapes of the world
		virtual void testCollision(CollisionCallback* callback);

		/// Return the list of all contacts of the world
		std::vector<const ContactManifold*> getContactsList() const;

		// -------------------- Friendship -------------------- //

		friend class RigidBody;
};

// Reset the external force and torque applied to the bodies
inline void DynamicsWorld::resetBodiesForceAndTorque() {

	// For each body of the world
	std::set<RigidBody*>::iterator it;
	for (it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it) {
		(*it)->mExternalForce.setToZero();
		(*it)->mExternalTorque.setToZero();
	}
}

// Get the number of iterations for the velocity constraint solver
inline uint32_t DynamicsWorld::getNbIterationsVelocitySolver() const {
	return mNbVelocitySolverIterations;
}

// Set the number of iterations for the velocity constraint solver
/**
 * @param nbIterations Number of iterations for the velocity solver
 */
inline void DynamicsWorld::setNbIterationsVelocitySolver(uint32_t nbIterations) {
	mNbVelocitySolverIterations = nbIterations;
}

// Get the number of iterations for the position constraint solver
inline uint32_t DynamicsWorld::getNbIterationsPositionSolver() const {
	return mNbPositionSolverIterations;
}

// Set the number of iterations for the position constraint solver
/**
 * @param nbIterations Number of iterations for the position solver
 */
inline void DynamicsWorld::setNbIterationsPositionSolver(uint32_t nbIterations) {
	mNbPositionSolverIterations = nbIterations;
}

// Set the position correction technique used for contacts
/**
 * @param technique Technique used for the position correction (Baumgarte or Split Impulses)
 */
inline void DynamicsWorld::setContactsPositionCorrectionTechnique(
							  ContactsPositionCorrectionTechnique technique) {
	if (technique == BAUMGARTE_CONTACTS) {
		mContactSolver.setIsSplitImpulseActive(false);
	}
	else {
		mContactSolver.setIsSplitImpulseActive(true);
	}
}

// Set the position correction technique used for joints
/**
 * @param technique Technique used for the joins position correction (Baumgarte or Non Linear Gauss Seidel)
 */
inline void DynamicsWorld::setJointsPositionCorrectionTechnique(
							  JointsPositionCorrectionTechnique technique) {
	if (technique == BAUMGARTE_JOINTS) {
		mConstraintSolver.setIsNonLinearGaussSeidelPositionCorrectionActive(false);
	}
	else {
		mConstraintSolver.setIsNonLinearGaussSeidelPositionCorrectionActive(true);
	}
}

// Activate or deactivate the solving of friction constraints at the center of
// the contact manifold instead of solving them at each contact point
/**
 * @param isActive True if you want the friction to be solved at the center of
 *				 the contact manifold and false otherwise
 */
inline void DynamicsWorld::setIsSolveFrictionAtContactManifoldCenterActive(bool isActive) {
	mContactSolver.setIsSolveFrictionAtContactManifoldCenterActive(isActive);
}

// Return the gravity vector of the world
/**
 * @return The current gravity vector (in meter per seconds squared)
 */
inline Vector3 DynamicsWorld::getGravity() const {
	return mGravity;
}

// Set the gravity vector of the world
/**
 * @param gravity The gravity vector (in meter per seconds squared)
 */
inline void DynamicsWorld::setGravity(Vector3& gravity) {
	mGravity = gravity;
}

// Return if the gravity is enaled
/**
 * @return True if the gravity is enabled in the world
 */
inline bool DynamicsWorld::isGravityEnabled() const {
	return mIsGravityEnabled;
}

// Enable/Disable the gravity
/**
 * @param isGravityEnabled True if you want to enable the gravity in the world
 *						 and false otherwise
 */
inline void DynamicsWorld::setIsGratityEnabled(bool isGravityEnabled) {
	mIsGravityEnabled = isGravityEnabled;
}

// Return the number of rigid bodies in the world
/**
 * @return Number of rigid bodies in the world
 */
inline uint32_t DynamicsWorld::getNbRigidBodies() const {
	return mRigidBodies.size();
}

/// Return the number of joints in the world
/**
 * @return Number of joints in the world
 */
inline uint32_t DynamicsWorld::getNbJoints() const {
	return mJoints.size();
}

// Return an iterator to the beginning of the bodies of the physics world
/**
 * @return Starting iterator of the set of rigid bodies
 */
inline std::set<RigidBody*>::iterator DynamicsWorld::getRigidBodiesBeginIterator() {
	return mRigidBodies.begin();
}

// Return an iterator to the end of the bodies of the physics world
/**
 * @return Ending iterator of the set of rigid bodies
 */
inline std::set<RigidBody*>::iterator DynamicsWorld::getRigidBodiesEndIterator() {
	return mRigidBodies.end();
}

// Return true if the sleeping technique is enabled
/**
 * @return True if the sleeping technique is enabled and false otherwise
 */
inline bool DynamicsWorld::isSleepingEnabled() const {
	return m_isSleepingEnabled;
}

// Return the current sleep linear velocity
/**
 * @return The sleep linear velocity (in meters per second)
 */
inline float DynamicsWorld::getSleepLinearVelocity() const {
	return mSleepLinearVelocity;
}

// Set the sleep linear velocity.
/// When the velocity of a body becomes smaller than the sleep linear/angular
/// velocity for a given amount of time, the body starts sleeping and does not need
/// to be simulated anymore.
/**
 * @param sleepLinearVelocity The sleep linear velocity (in meters per second)
 */
inline void DynamicsWorld::setSleepLinearVelocity(float sleepLinearVelocity) {
	assert(sleepLinearVelocity >= float(0.0));
	mSleepLinearVelocity = sleepLinearVelocity;
}

// Return the current sleep angular velocity
/**
 * @return The sleep angular velocity (in radian per second)
 */
inline float DynamicsWorld::getSleepAngularVelocity() const {
	return mSleepAngularVelocity;
}

// Set the sleep angular velocity.
/// When the velocity of a body becomes smaller than the sleep linear/angular
/// velocity for a given amount of time, the body starts sleeping and does not need
/// to be simulated anymore.
/**
 * @param sleepAngularVelocity The sleep angular velocity (in radian per second)
 */
inline void DynamicsWorld::setSleepAngularVelocity(float sleepAngularVelocity) {
	assert(sleepAngularVelocity >= float(0.0));
	mSleepAngularVelocity = sleepAngularVelocity;
}

// Return the time a body is required to stay still before sleeping
/**
 * @return Time a body is required to stay still before sleeping (in seconds)
 */
inline float DynamicsWorld::getTimeBeforeSleep() const {
	return mTimeBeforeSleep;
}


// Set the time a body is required to stay still before sleeping
/**
 * @param timeBeforeSleep Time a body is required to stay still before sleeping (in seconds)
 */
inline void DynamicsWorld::setTimeBeforeSleep(float timeBeforeSleep) {
	assert(timeBeforeSleep >= float(0.0));
	mTimeBeforeSleep = timeBeforeSleep;
}

// Set an event listener object to receive events callbacks.
/// If you use NULL as an argument, the events callbacks will be disabled.
/**
 * @param eventListener Pointer to the event listener object that will receive
 *					  event callbacks during the simulation
 */
inline void DynamicsWorld::setEventListener(EventListener* eventListener) {
	mEventListener = eventListener;
}


}

