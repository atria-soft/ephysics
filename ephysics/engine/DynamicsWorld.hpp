/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

#include <ephysics/engine/CollisionWorld.hpp>
#include <ephysics/collision/CollisionDetection.hpp>
#include <ephysics/engine/ContactSolver.hpp>
#include <ephysics/engine/ConstraintSolver.hpp>
#include <ephysics/body/RigidBody.hpp>
#include <ephysics/engine/Island.hpp>
#include <ephysics/configuration.hpp>

namespace ephysics {
	/**
	 * @brief This class represents a dynamics world. This class inherits from
	 * the CollisionWorld class. In a dynamics world, bodies can collide
	 * and their movements are simulated using the laws of physics.
	 */
	class DynamicsWorld : public CollisionWorld {
		protected :
			ContactSolver m_contactSolver; //!< Contact solver
			ConstraintSolver m_constraintSolver; //!< Constraint solver
			uint32_t m_nbVelocitySolverIterations; //!< Number of iterations for the velocity solver of the Sequential Impulses technique
			uint32_t m_nbPositionSolverIterations; //!< Number of iterations for the position solver of the Sequential Impulses technique
			bool m_isSleepingEnabled; //!< True if the spleeping technique for inactive bodies is enabled
			etk::Set<RigidBody*> m_rigidBodies; //!< All the rigid bodies of the physics world
			etk::Set<Joint*> m_joints; //!< All the joints of the world
			vec3 m_gravity; //!< Gravity vector of the world
			float m_timeStep; //!< Current frame time step (in seconds)
			bool m_isGravityEnabled; //!< True if the gravity force is on
			vec3* m_constrainedLinearVelocities; //!< Array of constrained linear velocities (state of the linear velocities after solving the constraints)
			vec3* m_constrainedAngularVelocities; //!< Array of constrained angular velocities (state of the angular velocities after solving the constraints)
			vec3* m_splitLinearVelocities; //!< Split linear velocities for the position contact solver (split impulse)
			vec3* m_splitAngularVelocities; //!< Split angular velocities for the position contact solver (split impulse)
			vec3* m_constrainedPositions; //!< Array of constrained rigid bodies position (for position error correction)
			etk::Quaternion* m_constrainedOrientations; //!< Array of constrained rigid bodies orientation (for position error correction)
			etk::Map<RigidBody*, uint32_t> m_mapBodyToConstrainedVelocityIndex; //!< Map body to their index in the constrained velocities array
			uint32_t m_numberIslands; //!< Number of islands in the world
			uint32_t m_numberIslandsCapacity; //!< Current allocated capacity for the islands
			Island** m_islands; //!< Array with all the islands of awaken bodies
			uint32_t m_numberBodiesCapacity; //!< Current allocated capacity for the bodies
			float m_sleepLinearVelocity; //!< Sleep linear velocity threshold
			float m_sleepAngularVelocity; //!< Sleep angular velocity threshold
			float m_timeBeforeSleep; //!< Time (in seconds) before a body is put to sleep if its velocity becomes smaller than the sleep velocity.
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
			void updatePositionAndOrientationOfBody(RigidBody* body, vec3 newLinVelocity,
													vec3 newAngVelocity);
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
			/// Constructor
			DynamicsWorld(const vec3& m_gravity);
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
			RigidBody* createRigidBody(const etk::Transform3D& transform);
			/// Destroy a rigid body and all the joints which it belongs
			void destroyRigidBody(RigidBody* rigidBody);
			/// Create a joint between two bodies in the world and return a pointer to the new joint
			Joint* createJoint(const JointInfo& jointInfo);
			/// Destroy a joint
			void destroyJoint(Joint* joint);
			/// Return the gravity vector of the world
			vec3 getGravity() const;
			/// Set the gravity vector of the world
			void setGravity(vec3& gravity);
			/// Return if the gravity is on
			bool isGravityEnabled() const;
			/// Enable/Disable the gravity
			void setIsGratityEnabled(bool isGravityEnabled);
			/// Return the number of rigid bodies in the world
			uint32_t getNbRigidBodies() const;
			/// Return the number of joints in the world
			uint32_t getNbJoints() const;
			/// Return an iterator to the beginning of the rigid bodies of the physics world
			etk::Set<RigidBody*>::Iterator getRigidBodiesBeginIterator();
			/// Return an iterator to the end of the rigid bodies of the physics world
			etk::Set<RigidBody*>::Iterator getRigidBodiesEndIterator();
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
			void testCollision(const ProxyShape* _shape,
			                   CollisionCallback* _callback) override;
			virtual void testCollision(const ProxyShape* _shape1,
									   const ProxyShape* _shape2,
									   CollisionCallback* _callback) override;
			virtual void testCollision(const CollisionBody* _body,
									   CollisionCallback* _callback) override;
			virtual void testCollision(const CollisionBody* _body1,
									   const CollisionBody* _body2,
									   CollisionCallback* _callback) override;
			/// Test and report collisions between all shapes of the world
			virtual void testCollision(CollisionCallback* _callback) override;
			/// Return the list of all contacts of the world
			etk::Vector<const ContactManifold*> getContactsList() const;
			friend class RigidBody;
	};



}

