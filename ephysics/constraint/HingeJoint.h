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

// Structure HingeJointInfo
/**
 * This structure is used to gather the information needed to create a hinge joint.
 * This structure will be used to create the actual hinge joint.
 */
struct HingeJointInfo : public JointInfo {

	public :

		// -------------------- Attributes -------------------- //

		/// Anchor point (in world-space coordinates)
		Vector3 m_m_m_m_anchorPointWorldSpace;

		/// Hinge rotation axis (in world-space coordinates)
		Vector3 rotationAxisWorld;

		/// True if the hinge joint limits are enabled
		bool isLimitEnabled;

		/// True if the hinge joint motor is enabled
		bool isMotorEnabled;

		/// Minimum allowed rotation angle (in radian) if limits are enabled.
		/// The angle must be in the range [-2*pi, 0]
		float minAngleLimit;

		/// Maximum allowed rotation angle (in radian) if limits are enabled.
		/// The angle must be in the range [0, 2*pi]
		float maxAngleLimit;

		/// Motor speed (in radian/second)
		float motorSpeed;

		/// Maximum motor torque (in Newtons * meters) that can be applied to reach
		/// to desired motor speed
		float maxMotorTorque;

		/// Constructor without limits and without motor
		/**
		 * @param rigidBody1 The first body of the joint
		 * @param rigidBody2 The second body of the joint
		 * @param initAnchorPointWorldSpace The initial anchor point in world-space
		 *								  coordinates
		 * @param initRotationAxisWorld The initial rotation axis in world-space
		 *							  coordinates
		 */
		HingeJointInfo(RigidBody* rigidBody1, RigidBody* rigidBody2,
							   const Vector3& initAnchorPointWorldSpace,
							   const Vector3& initRotationAxisWorld)
							  : JointInfo(rigidBody1, rigidBody2, HINGEJOINT),
								m_m_m_m_anchorPointWorldSpace(initAnchorPointWorldSpace),
								rotationAxisWorld(initRotationAxisWorld), isLimitEnabled(false),
								isMotorEnabled(false), minAngleLimit(-1), maxAngleLimit(1),
								motorSpeed(0), maxMotorTorque(0) {}

		/// Constructor with limits but without motor
		/**
		 * @param rigidBody1 The first body of the joint
		 * @param rigidBody2 The second body of the joint
		 * @param initAnchorPointWorldSpace The initial anchor point in world-space coordinates
		 * @param initRotationAxisWorld The int32_tial rotation axis in world-space coordinates
		 * @param initMinAngleLimit The initial minimum limit angle (in radian)
		 * @param initMaxAngleLimit The initial maximum limit angle (in radian)
		 */
		HingeJointInfo(RigidBody* rigidBody1, RigidBody* rigidBody2,
							   const Vector3& initAnchorPointWorldSpace,
							   const Vector3& initRotationAxisWorld,
							   float initMinAngleLimit, float initMaxAngleLimit)
							  : JointInfo(rigidBody1, rigidBody2, HINGEJOINT),
								m_m_m_m_anchorPointWorldSpace(initAnchorPointWorldSpace),
								rotationAxisWorld(initRotationAxisWorld), isLimitEnabled(true),
								isMotorEnabled(false), minAngleLimit(initMinAngleLimit),
								maxAngleLimit(initMaxAngleLimit), motorSpeed(0),
								maxMotorTorque(0) {}

		/// Constructor with limits and motor
		/**
		 * @param rigidBody1 The first body of the joint
		 * @param rigidBody2 The second body of the joint
		 * @param initAnchorPointWorldSpace The initial anchor point in world-space
		 * @param initRotationAxisWorld The initial rotation axis in world-space
		 * @param initMinAngleLimit The initial minimum limit angle (in radian)
		 * @param initMaxAngleLimit The initial maximum limit angle (in radian)
		 * @param initMotorSpeed The initial motor speed of the joint (in radian per second)
		 * @param initMaxMotorTorque The initial maximum motor torque (in Newtons)
		 */
		HingeJointInfo(RigidBody* rigidBody1, RigidBody* rigidBody2,
							   const Vector3& initAnchorPointWorldSpace,
							   const Vector3& initRotationAxisWorld,
							   float initMinAngleLimit, float initMaxAngleLimit,
							   float initMotorSpeed, float initMaxMotorTorque)
							  : JointInfo(rigidBody1, rigidBody2, HINGEJOINT),
								m_m_m_m_anchorPointWorldSpace(initAnchorPointWorldSpace),
								rotationAxisWorld(initRotationAxisWorld), isLimitEnabled(true),
								isMotorEnabled(false), minAngleLimit(initMinAngleLimit),
								maxAngleLimit(initMaxAngleLimit), motorSpeed(initMotorSpeed),
								maxMotorTorque(initMaxMotorTorque) {}
};

// Class HingeJoint
/**
 * This class represents a hinge joint that allows arbitrary rotation
 * between two bodies around a single axis. This joint has one degree of freedom. It
 * can be useful to simulate doors or pendulumns.
 */
class HingeJoint : public Joint {

	private :

		// -------------------- Constants -------------------- //

		// Beta value for the bias factor of position correction
		static const float BETA;

		// -------------------- Attributes -------------------- //

		/// Anchor point of body 1 (in local-space coordinates of body 1)
		Vector3 m_localAnchorPointBody1;

		/// Anchor point of body 2 (in local-space coordinates of body 2)
		Vector3 m_localAnchorPointBody2;

		/// Hinge rotation axis (in local-space coordinates of body 1)
		Vector3 mHingeLocalAxisBody1;

		/// Hinge rotation axis (in local-space coordiantes of body 2)
		Vector3 mHingeLocalAxisBody2;

		/// Inertia tensor of body 1 (in world-space coordinates)
		Matrix3x3 m_i1;

		/// Inertia tensor of body 2 (in world-space coordinates)
		Matrix3x3 m_i2;

		/// Hinge rotation axis (in world-space coordinates) computed from body 1
		Vector3 mA1;

		/// Vector from center of body 2 to anchor point in world-space
		Vector3 m_r1World;

		/// Vector from center of body 2 to anchor point in world-space
		Vector3 m_r2World;

		/// Cross product of vector b2 and a1
		Vector3 mB2CrossA1;

		/// Cross product of vector c2 and a1;
		Vector3 mC2CrossA1;

		/// Impulse for the 3 translation constraints
		Vector3 m_impulseTranslation;

		/// Impulse for the 2 rotation constraints
		Vector2 m_impulseRotation;

		/// Accumulated impulse for the lower limit constraint
		float m_impulseLowerLimit;

		/// Accumulated impulse for the upper limit constraint
		float m_impulseUpperLimit;

		/// Accumulated impulse for the motor constraint;
		float m_impulseMotor;

		/// Inverse mass matrix K=JM^-1J^t for the 3 translation constraints
		Matrix3x3 m_inverseMassMatrixTranslation;

		/// Inverse mass matrix K=JM^-1J^t for the 2 rotation constraints
		Matrix2x2 m_inverseMassMatrixRotation;

		/// Inverse of mass matrix K=JM^-1J^t for the limits and motor constraints (1x1 matrix)
		float m_inverseMassMatrixLimitMotor;

		/// Inverse of mass matrix K=JM^-1J^t for the motor
		float m_inverseMassMatrixMotor;

		/// Bias vector for the error correction for the translation constraints
		Vector3 mBTranslation;

		/// Bias vector for the error correction for the rotation constraints
		Vector2 mBRotation;

		/// Bias of the lower limit constraint
		float mBLowerLimit;

		/// Bias of the upper limit constraint
		float mBUpperLimit;

		/// Inverse of the initial orientation difference between the bodies
		Quaternion mInitOrientationDifferenceInv;

		/// True if the joint limits are enabled
		bool mIsLimitEnabled;

		/// True if the motor of the joint in enabled
		bool mIsMotorEnabled;

		/// Lower limit (minimum allowed rotation angle in radian)
		float mLowerLimit;

		/// Upper limit (maximum translation distance)
		float mUpperLimit;

		/// True if the lower limit is violated
		bool mIsLowerLimitViolated;

		/// True if the upper limit is violated
		bool mIsUpperLimitViolated;

		/// Motor speed (in rad/s)
		float mMotorSpeed;

		/// Maximum motor torque (in Newtons) that can be applied to reach to desired motor speed
		float mMaxMotorTorque;

		// -------------------- Methods -------------------- //

		/// Private copy-constructor
		HingeJoint(const HingeJoint& constraint);

		/// Private assignment operator
		HingeJoint& operator=(const HingeJoint& constraint);

		/// Reset the limits
		void resetLimits();

		/// Given an angle in radian, this method returns the corresponding
		/// angle in the range [-pi; pi]
		float computeNormalizedAngle(float angle) const;

		/// Given an "inputAngle" in the range [-pi, pi], this method returns an
		/// angle (modulo 2*pi) in the range [-2*pi; 2*pi] that is closest to one of the
		/// two angle limits in arguments.
		float computeCorrespondingAngleNearLimits(float inputAngle, float lowerLimitAngle,
													float upperLimitAngle) const;

		/// Compute the current angle around the hinge axis
		float computeCurrentHingeAngle(const Quaternion& orientationBody1,
										 const Quaternion& orientationBody2);

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
		HingeJoint(const HingeJointInfo& jointInfo);

		/// Destructor
		virtual ~HingeJoint();

		/// Return true if the limits or the joint are enabled
		bool isLimitEnabled() const;

		/// Return true if the motor of the joint is enabled
		bool isMotorEnabled() const;

		/// Enable/Disable the limits of the joint
		void enableLimit(bool isLimitEnabled);

		/// Enable/Disable the motor of the joint
		void enableMotor(bool isMotorEnabled);

		/// Return the minimum angle limit
		float getMinAngleLimit() const;

		/// Set the minimum angle limit
		void setMinAngleLimit(float lowerLimit);

		/// Return the maximum angle limit
		float getMaxAngleLimit() const;

		/// Set the maximum angle limit
		void setMaxAngleLimit(float upperLimit);

		/// Return the motor speed
		float getMotorSpeed() const;

		/// Set the motor speed
		void setMotorSpeed(float motorSpeed);

		/// Return the maximum motor torque
		float getMaxMotorTorque() const;

		/// Set the maximum motor torque
		void setMaxMotorTorque(float maxMotorTorque);

		/// Return the int32_tensity of the current torque applied for the joint motor
		float getMotorTorque(float timeStep) const;
};

// Return true if the limits of the joint are enabled
/**
 * @return True if the limits of the joint are enabled and false otherwise
 */
inline bool HingeJoint::isLimitEnabled() const {
	return mIsLimitEnabled;
}

// Return true if the motor of the joint is enabled
/**
 * @return True if the motor of joint is enabled and false otherwise
 */
inline bool HingeJoint::isMotorEnabled() const {
	return mIsMotorEnabled;
}

// Return the minimum angle limit
/**
 * @return The minimum limit angle of the joint (in radian)
 */
inline float HingeJoint::getMinAngleLimit() const {
	return mLowerLimit;
}

// Return the maximum angle limit
/**
 * @return The maximum limit angle of the joint (in radian)
 */
inline float HingeJoint::getMaxAngleLimit() const {
	return mUpperLimit;
}

// Return the motor speed
/**
 * @return The current speed of the joint motor (in radian per second)
 */
inline float HingeJoint::getMotorSpeed() const {
	return mMotorSpeed;
}

// Return the maximum motor torque
/**
 * @return The maximum torque of the joint motor (in Newtons)
 */
inline float HingeJoint::getMaxMotorTorque() const {
	return mMaxMotorTorque;
}

// Return the int32_tensity of the current torque applied for the joint motor
/**
 * @param timeStep The current time step (in seconds)
 * @return The int32_tensity of the current torque (in Newtons) of the joint motor
 */
inline float HingeJoint::getMotorTorque(float timeStep) const {
	return m_impulseMotor / timeStep;
}

// Return the number of bytes used by the joint
inline size_t HingeJoint::getSizeInBytes() const {
	return sizeof(HingeJoint);
}

}

