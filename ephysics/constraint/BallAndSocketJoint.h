/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/constraint/Joint.h>
#include <ephysics/mathematics/mathematics.h>

namespace reactphysics3d {

// Structure BallAndSocketJointInfo
/**
 * This structure is used to gather the information needed to create a ball-and-socket
 * joint. This structure will be used to create the actual ball-and-socket joint.
 */
struct BallAndSocketJointInfo : public JointInfo {

	public :

		// -------------------- Attributes -------------------- //

		/// Anchor point (in world-space coordinates)
		Vector3 m_anchorPointWorldSpace;

		/// Constructor
		/**
		 * @param rigidBody1 Pointer to the first body of the joint
		 * @param rigidBody2 Pointer to the second body of the joint
		 * @param initAnchorPointWorldSpace The anchor point in world-space
		 *								  coordinates
		 */
		BallAndSocketJointInfo(RigidBody* rigidBody1, RigidBody* rigidBody2,
							   const Vector3& initAnchorPointWorldSpace)
							  : JointInfo(rigidBody1, rigidBody2, BALLSOCKETJOINT),
								m_anchorPointWorldSpace(initAnchorPointWorldSpace) {}
};

// Class BallAndSocketJoint
/**
 * This class represents a ball-and-socket joint that allows arbitrary rotation
 * between two bodies. This joint has three degrees of freedom. It can be used to
 * create a chain of bodies for instance.
 */
class BallAndSocketJoint : public Joint {

	private :

		// -------------------- Constants -------------------- //

		// Beta value for the bias factor of position correction
		static const float BETA;

		// -------------------- Attributes -------------------- //

		/// Anchor point of body 1 (in local-space coordinates of body 1)
		Vector3 m_localAnchorPointBody1;

		/// Anchor point of body 2 (in local-space coordinates of body 2)
		Vector3 m_localAnchorPointBody2;

		/// Vector from center of body 2 to anchor point in world-space
		Vector3 m_r1World;

		/// Vector from center of body 2 to anchor point in world-space
		Vector3 m_r2World;

		/// Inertia tensor of body 1 (in world-space coordinates)
		Matrix3x3 m_i1;

		/// Inertia tensor of body 2 (in world-space coordinates)
		Matrix3x3 m_i2;

		/// Bias vector for the constraint
		Vector3 m_biasVector;

		/// Inverse mass matrix K=JM^-1J^-t of the constraint
		Matrix3x3 m_inverseMassMatrix;

		/// Accumulated impulse
		Vector3 m_impulse;

		// -------------------- Methods -------------------- //

		/// Private copy-constructor
		BallAndSocketJoint(const BallAndSocketJoint& constraint);

		/// Private assignment operator
		BallAndSocketJoint& operator=(const BallAndSocketJoint& constraint);

		/// Return the number of bytes used by the joint
		virtual size_t getSizeInBytes() const;

		/// Initialize before solving the constraint
		virtual void initBeforeSolve(const ConstraintSolverData& constraintSolverData);

		/// Warm start the constraint (apply the previous impulse at the beginning of the step)
		virtual void warmstart(const ConstraintSolverData& constraintSolverData);

		/// Solve the velocity constraint
		virtual void solveVelocityConstraint(const ConstraintSolverData& constraintSolverData);

		/// Solve the position constraint (for position error correction)
		virtual void solvePositionConstraint(const ConstraintSolverData& constraintSolverData);

	public :

		// -------------------- Methods -------------------- //

		/// Constructor
		BallAndSocketJoint(const BallAndSocketJointInfo& jointInfo);

		/// Destructor
		virtual ~BallAndSocketJoint();
};

// Return the number of bytes used by the joint
inline size_t BallAndSocketJoint::getSizeInBytes() const {
	return sizeof(BallAndSocketJoint);
}

}
