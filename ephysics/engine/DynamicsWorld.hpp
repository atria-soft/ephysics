/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
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
			etk::Vector<vec3> m_constrainedLinearVelocities; //!< Array of constrained linear velocities (state of the linear velocities after solving the constraints)
			etk::Vector<vec3> m_constrainedAngularVelocities; //!< Array of constrained angular velocities (state of the angular velocities after solving the constraints)
			etk::Vector<vec3> m_splitLinearVelocities; //!< Split linear velocities for the position contact solver (split impulse)
			etk::Vector<vec3> m_splitAngularVelocities; //!< Split angular velocities for the position contact solver (split impulse)
			etk::Vector<vec3> m_constrainedPositions; //!< Array of constrained rigid bodies position (for position error correction)
			etk::Vector<etk::Quaternion> m_constrainedOrientations; //!< Array of constrained rigid bodies orientation (for position error correction)
			etk::Map<RigidBody*, uint32_t> m_mapBodyToConstrainedVelocityIndex; //!< Map body to their index in the constrained velocities array
			etk::Vector<Island*> m_islands; //!< Array with all the islands of awaken bodies
			uint32_t m_numberBodiesCapacity; //!< Current allocated capacity for the bodies
			float m_sleepLinearVelocity; //!< Sleep linear velocity threshold
			float m_sleepAngularVelocity; //!< Sleep angular velocity threshold
			float m_timeBeforeSleep; //!< Time (in seconds) before a body is put to sleep if its velocity becomes smaller than the sleep velocity.
			/// Private copy-constructor
			DynamicsWorld(const DynamicsWorld& world) = delete;
			/// Private assignment operator
			DynamicsWorld& operator=(const DynamicsWorld& world) = delete;
			/**
			 * @brief Integrate position and orientation of the rigid bodies.
			 * The positions and orientations of the bodies are int32_tegrated using
			 * the sympletic Euler time stepping scheme.
			 */
			void integrateRigidBodiesPositions();
			/**
			 * @brief Reset the external force and torque applied to the bodies
			 */
			void resetBodiesForceAndTorque();
			/**
			 * @brief Initialize the bodies velocities arrays for the next simulation step.
			 */
			void initVelocityArrays();
			/**
			 * @brief Integrate the velocities of rigid bodies.
			 * This method only set the temporary velocities but does not update
			 * the actual velocitiy of the bodies. The velocities updated in this method
			 * might violate the constraints and will be corrected in the constraint and
			 * contact solver.
			 */
			void integrateRigidBodiesVelocities();
			/**
			 * @brief Solve the contacts and constraints
			 */
			void solveContactsAndConstraints();
			/**
			 * @brief Solve the position error correction of the constraints
			 */
			void solvePositionCorrection();
			/**
			 * @brief Compute the islands of awake bodies.
			 * An island is an isolated group of rigid bodies that have constraints (joints or contacts)
			 * between each other. This method computes the islands at each time step as follows: For each
			 * awake rigid body, we run a Depth First Search (DFS) through the constraint graph of that body
			 * (graph where nodes are the bodies and where the edges are the constraints between the bodies) to
			 * find all the bodies that are connected with it (the bodies that share joints or contacts with
			 * it). Then, we create an island with this group of connected bodies.
			 */
			void computeIslands();
			/**
			 * @brief Update the postion/orientation of the bodies
			 */
			void updateBodiesState();
			/**
			 * @brief Put bodies to sleep if needed.
			 * For each island, if all the bodies have been almost still for a long enough period of
			 * time, we put all the bodies of the island to sleep.
			 */
			void updateSleepingBodies();
			/**
			 * @brief Add the joint to the list of joints of the two bodies involved in the joint
			 * @param[in,out] _joint Joint to add at the body.
			 */
			void addJointToBody(Joint* _joint);
		public :
			/**
			 * @brief Constructor
			 * @param gravity Gravity vector in the world (in meters per second squared)
			 */
			DynamicsWorld(const vec3& _gravity);
			/**
			 * @brief Vitualize the Destructor
			 */
			virtual ~DynamicsWorld();
			/**
			 * @brief Update the physics simulation
			 * @param timeStep The amount of time to step the simulation by (in seconds)
			 */
			void update(float _timeStep);
			/**
			 * @brief Get the number of iterations for the velocity constraint solver
			 * @return Number if iteration.
			 */
			uint32_t getNbIterationsVelocitySolver() const;
			/**
			 * @brief Set the number of iterations for the velocity constraint solver
			 * @param[in] _nbIterations Number of iterations for the velocity solver
			 */
			void setNbIterationsVelocitySolver(uint32_t _nbIterations);
			/**
			 * @brief Get the number of iterations for the position constraint solver
			 */
			uint32_t getNbIterationsPositionSolver() const;
			/**
			 * @brief Set the number of iterations for the position constraint solver
			 * @param[in] _nbIterations Number of iterations for the position solver
			 */
			void setNbIterationsPositionSolver(uint32_t _nbIterations);
			/**
			 * @brief Set the position correction technique used for contacts
			 * @param[in] _technique Technique used for the position correction (Baumgarte or Split Impulses)
			 */
			void setContactsPositionCorrectionTechnique(ContactsPositionCorrectionTechnique _technique);
			/**
			 * @brief Set the position correction technique used for joints
			 * @param[in] _technique Technique used for the joins position correction (Baumgarte or Non Linear Gauss Seidel)
			 */
			void setJointsPositionCorrectionTechnique(JointsPositionCorrectionTechnique _technique);
			/**
			 * @brief Activate or deactivate the solving of friction constraints at the center of the contact
			 * manifold instead of solving them at each contact point
			 * @param[in] _isActive True if you want the friction to be solved at the center of
			 * the contact manifold and false otherwise
			 */
			void setIsSolveFrictionAtContactManifoldCenterActive(bool _isActive);
			/**
			 * @brief Create a rigid body int32_to the physics world
			 * @param[in] _transform etk::Transform3Dation from body local-space to world-space
			 * @return A pointer to the body that has been created in the world
			 */
			RigidBody* createRigidBody(const etk::Transform3D& _transform);
			/**
			 * @brief Destroy a rigid body and all the joints which it belongs
			 * @param[in,out] _rigidBody Pointer to the body you want to destroy
			 */
			void destroyRigidBody(RigidBody* _rigidBody);
			/**
			 * @brief Create a joint between two bodies in the world and return a pointer to the new joint
			 * @param[in] _jointInfo The information that is necessary to create the joint
			 * @return A pointer to the joint that has been created in the world
			 */
			Joint* createJoint(const JointInfo& _jointInfo);
			/**
			 * @brief Destroy a joint
			 * @param[in,out] _joint Pointer to the joint you want to destroy
			 */
			void destroyJoint(Joint* _joint);
			/**
			 * @brief Get the gravity vector of the world
			 * @return The current gravity vector (in meter per seconds squared)
			 */
			vec3 getGravity() const;
			/**
			 * @brief Set the gravity vector of the world
			 * @param[in] _gravity The gravity vector (in meter per seconds squared)
			 */
			void setGravity(vec3& _gravity);
			/**
			 * @brief Get if the gravity is enaled
			 * @return True if the gravity is enabled in the world
			 */
			bool isGravityEnabled() const;
			/**
			 * @brief Enable/Disable the gravity
			 * @param[in] _isGravityEnabled True if you want to enable the gravity in the world
			 * and false otherwise
			 */
			void setIsGratityEnabled(bool _isGravityEnabled);
			/**
			 * @brief Get the number of rigid bodies in the world
			 * @return Number of rigid bodies in the world
			 */
			uint32_t getNbRigidBodies() const;
			/**
			 * @brief Get the number of all joints
			 * @return Number of joints in the world
			 */
			uint32_t getNbJoints() const;
			/**
			 * @brief Get an iterator to the beginning of the bodies of the physics world
			 * @return Starting iterator of the set of rigid bodies
			 */
			etk::Set<RigidBody*>::Iterator getRigidBodiesBeginIterator();
			/**
			 * @brief Get an iterator to the end of the bodies of the physics world
			 * @return Ending iterator of the set of rigid bodies
			 */
			etk::Set<RigidBody*>::Iterator getRigidBodiesEndIterator();
			/**
			 * @brief Get if the sleeping technique is enabled
			 * @return True if the sleeping technique is enabled and false otherwise
			 */
			bool isSleepingEnabled() const;
			 /**
			 * @brief Enable/Disable the sleeping technique.
			 * The sleeping technique is used to put bodies that are not moving int32_to sleep
			 * to speed up the simulation.
			 * @param[in] _isSleepingEnabled True if you want to enable the sleeping technique and false otherwise
			 */
			void enableSleeping(bool _isSleepingEnabled);
			/**
			 * @brief Get the sleep linear velocity
			 * @return The current sleep linear velocity (in meters per second)
			 */
			float getSleepLinearVelocity() const;
			/**
			 * @brief Set the sleep linear velocity
			 * @param[in] _sleepLinearVelocity The new sleep linear velocity (in meters per second)
			 */
			void setSleepLinearVelocity(float _sleepLinearVelocity);
			/**
			 * @brief Return the current sleep angular velocity
			 * @return The sleep angular velocity (in radian per second)
			 */
			float getSleepAngularVelocity() const;
			/**
			 * @brief Set the sleep angular velocity.
			 * When the velocity of a body becomes smaller than the sleep linear/angular
			 * velocity for a given amount of time, the body starts sleeping and does not need
			 * to be simulated anymore.
			 * @param[in] _sleepAngularVelocity The sleep angular velocity (in radian per second)
			 */
			void setSleepAngularVelocity(float _sleepAngularVelocity);
			/**
			 * @brief Get the time a body is required to stay still before sleeping
			 * @return Time a body is required to stay still before sleeping (in seconds)
			 */
			float getTimeBeforeSleep() const;
			/**
			 * @brief Set the time a body is required to stay still before sleeping
			 * @param timeBeforeSleep Time a body is required to stay still before sleeping (in seconds)
			 */
			void setTimeBeforeSleep(float timeBeforeSleep);
			/**
			 * @brief Set an event listener object to receive events callbacks.
			 * @note If you use null as an argument, the events callbacks will be disabled.
			 * @param[in] _eventListener Pointer to the event listener object that will receive
			 * event callbacks during the simulation
			 */
			void setEventListener(EventListener* _eventListener);
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
			/**
			 * @brief Get list of all contacts.
			 * @return The list of all contacts of the world
			 */
			etk::Vector<const ContactManifold*> getContactsList() const;
			friend class RigidBody;
	};



}

