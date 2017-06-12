/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/configuration.h>
#include <ephysics/body/RigidBody.h>
#include <ephysics/mathematics/mathematics.h>

// ReactPhysics3D namespace
namespace reactphysics3d {

/// Enumeration for the type of a constraint
enum JointType {BALLSOCKETJOINT, SLIDERJOINT, HINGEJOINT, FIXEDJOINT};

// Class declarations
struct ConstraintSolverData;
class Joint;

// Structure JointListElement
/**
 * This structure represents a single element of a linked list of joints
 */
struct JointListElement {

	public:

		// -------------------- Attributes -------------------- //

		/// Pointer to the actual joint
		Joint* joint;

		/// Next element of the list
		JointListElement* next;

		// -------------------- Methods -------------------- //

		/// Constructor
		JointListElement(Joint* initJoint, JointListElement* initNext)
						:joint(initJoint), next(initNext){

		}
};

// Structure JointInfo
/**
 * This structure is used to gather the information needed to create a joint.
 */
struct JointInfo {

	public :

		// -------------------- Attributes -------------------- //

		/// First rigid body of the joint
		RigidBody* body1;

		/// Second rigid body of the joint
		RigidBody* body2;

		/// Type of the joint
		JointType type;

		/// Position correction technique used for the constraint (used for joints).
		/// By default, the BAUMGARTE technique is used
		JointsPositionCorrectionTechnique positionCorrectionTechnique;

		/// True if the two bodies of the joint are allowed to collide with each other
		bool isCollisionEnabled;

		/// Constructor
		JointInfo(JointType constraintType)
					  : body1(NULL), body2(NULL), type(constraintType),
						positionCorrectionTechnique(NON_LINEAR_GAUSS_SEIDEL),
						isCollisionEnabled(true) {}

		/// Constructor
		JointInfo(RigidBody* rigidBody1, RigidBody* rigidBody2, JointType constraintType)
					  : body1(rigidBody1), body2(rigidBody2), type(constraintType),
						positionCorrectionTechnique(NON_LINEAR_GAUSS_SEIDEL),
						isCollisionEnabled(true) {
		}

		/// Destructor
		virtual ~JointInfo() {}

};

// Class Joint
/**
 * This abstract class represents a joint between two bodies.
 */
class Joint {

	protected :

		// -------------------- Attributes -------------------- //

		/// Pointer to the first body of the joint
		RigidBody* const m_body1;

		/// Pointer to the second body of the joint
		RigidBody* const m_body2;

		/// Type of the joint
		const JointType m_type;

		/// Body 1 index in the velocity array to solve the constraint
		uint32_t m_indexBody1;

		/// Body 2 index in the velocity array to solve the constraint
		uint32_t m_indexBody2;

		/// Position correction technique used for the constraint (used for joints)
		JointsPositionCorrectionTechnique m_positionCorrectionTechnique;

		/// True if the two bodies of the constraint are allowed to collide with each other
		bool m_isCollisionEnabled;

		/// True if the joint has already been added int32_to an island
		bool m_isAlreadyInIsland;

		// -------------------- Methods -------------------- //

		/// Private copy-constructor
		Joint(const Joint& constraint);

		/// Private assignment operator
		Joint& operator=(const Joint& constraint);

		/// Return true if the joint has already been added int32_to an island
		bool isAlreadyInIsland() const;

		/// Return the number of bytes used by the joint
		virtual size_t getSizeInBytes() const = 0;

		/// Initialize before solving the joint
		virtual void initBeforeSolve(const ConstraintSolverData& constraintSolverData) = 0;

		/// Warm start the joint (apply the previous impulse at the beginning of the step)
		virtual void warmstart(const ConstraintSolverData& constraintSolverData) = 0;

		/// Solve the velocity constraint
		virtual void solveVelocityConstraint(const ConstraintSolverData& constraintSolverData) = 0;

		/// Solve the position constraint
		virtual void solvePositionConstraint(const ConstraintSolverData& constraintSolverData) = 0;

	public :

		// -------------------- Methods -------------------- //

		/// Constructor
		Joint(const JointInfo& jointInfo);

		/// Destructor
		virtual ~Joint();

		/// Return the reference to the body 1
		RigidBody* getBody1() const;

		/// Return the reference to the body 2
		RigidBody* getBody2() const;

		/// Return true if the constraint is active
		bool isActive() const;

		/// Return the type of the constraint
		JointType getType() const;

		/// Return true if the collision between the two bodies of the joint is enabled
		bool isCollisionEnabled() const;

		// -------------------- Friendship -------------------- //

		friend class DynamicsWorld;
		friend class Island;
		friend class ConstraintSolver;
};

// Return the reference to the body 1
/**
 * @return The first body involved in the joint
 */
inline RigidBody* Joint::getBody1() const {
	return m_body1;
}

// Return the reference to the body 2
/**
 * @return The second body involved in the joint
 */
inline RigidBody* Joint::getBody2() const {
	return m_body2;
}

// Return true if the joint is active
/**
 * @return True if the joint is active
 */
inline bool Joint::isActive() const {
	return (m_body1->isActive() && m_body2->isActive());
}

// Return the type of the joint
/**
 * @return The type of the joint
 */
inline JointType Joint::getType() const {
	return m_type;
}

// Return true if the collision between the two bodies of the joint is enabled
/**
 * @return True if the collision is enabled between the two bodies of the joint
 *			  is enabled and false otherwise
 */
inline bool Joint::isCollisionEnabled() const {
	return m_isCollisionEnabled;
}

// Return true if the joint has already been added int32_to an island
inline bool Joint::isAlreadyInIsland() const {
	return m_isAlreadyInIsland;
}

}
