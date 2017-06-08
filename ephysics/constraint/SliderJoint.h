/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/mathematics/mathematics.h>
#include <ephysics/engine/ConstraintSolver.h>

namespace reactphysics3d {

// Structure SliderJointInfo
/**
 * This structure is used to gather the information needed to create a slider
 * joint. This structure will be used to create the actual slider joint.
 */
struct SliderJointInfo : public JointInfo {

	public :

		// -------------------- Attributes -------------------- //

		/// Anchor point (in world-space coordinates)
		Vector3 anchorPointWorldSpace;

		/// Slider axis (in world-space coordinates)
		Vector3 sliderAxisWorldSpace;

		/// True if the slider limits are enabled
		bool isLimitEnabled;

		/// True if the slider motor is enabled
		bool isMotorEnabled;

		/// Mininum allowed translation if limits are enabled
		float minTranslationLimit;

		/// Maximum allowed translation if limits are enabled
		float maxTranslationLimit;

		/// Motor speed
		float motorSpeed;

		/// Maximum motor force (in Newtons) that can be applied to reach to desired motor speed
		float maxMotorForce;

		/// Constructor without limits and without motor
		/**
		 * @param rigidBody1 The first body of the joint
		 * @param rigidBody2 The second body of the joint
		 * @param initAnchorPointWorldSpace The initial anchor point in world-space
		 * @param initSliderAxisWorldSpace The initial slider axis in world-space
		 */
		SliderJointInfo(RigidBody* rigidBody1, RigidBody* rigidBody2,
						const Vector3& initAnchorPointWorldSpace,
						const Vector3& initSliderAxisWorldSpace)
					   : JointInfo(rigidBody1, rigidBody2, SLIDERJOINT),
						 anchorPointWorldSpace(initAnchorPointWorldSpace),
						 sliderAxisWorldSpace(initSliderAxisWorldSpace),
						 isLimitEnabled(false), isMotorEnabled(false), minTranslationLimit(-1.0),
						 maxTranslationLimit(1.0), motorSpeed(0), maxMotorForce(0) {}

		/// Constructor with limits and no motor
		/**
		 * @param rigidBody1 The first body of the joint
		 * @param rigidBody2 The second body of the joint
		 * @param initAnchorPointWorldSpace The initial anchor point in world-space
		 * @param initSliderAxisWorldSpace The initial slider axis in world-space
		 * @param initMinTranslationLimit The initial minimum translation limit (in meters)
		 * @param initMaxTranslationLimit The initial maximum translation limit (in meters)
		 */
		SliderJointInfo(RigidBody* rigidBody1, RigidBody* rigidBody2,
						const Vector3& initAnchorPointWorldSpace,
						const Vector3& initSliderAxisWorldSpace,
						float initMinTranslationLimit, float initMaxTranslationLimit)
					   : JointInfo(rigidBody1, rigidBody2, SLIDERJOINT),
						 anchorPointWorldSpace(initAnchorPointWorldSpace),
						 sliderAxisWorldSpace(initSliderAxisWorldSpace),
						 isLimitEnabled(true), isMotorEnabled(false),
						 minTranslationLimit(initMinTranslationLimit),
						 maxTranslationLimit(initMaxTranslationLimit), motorSpeed(0),
						 maxMotorForce(0) {}

		/// Constructor with limits and motor
		/**
		 * @param rigidBody1 The first body of the joint
		 * @param rigidBody2 The second body of the joint
		 * @param initAnchorPointWorldSpace The initial anchor point in world-space
		 * @param initSliderAxisWorldSpace The initial slider axis in world-space
		 * @param initMinTranslationLimit The initial minimum translation limit (in meters)
		 * @param initMaxTranslationLimit The initial maximum translation limit (in meters)
		 * @param initMotorSpeed The initial speed of the joint motor (in meters per second)
		 * @param initMaxMotorForce The initial maximum motor force of the joint (in Newtons x meters)
		 */
		SliderJointInfo(RigidBody* rigidBody1, RigidBody* rigidBody2,
						const Vector3& initAnchorPointWorldSpace,
						const Vector3& initSliderAxisWorldSpace,
						float initMinTranslationLimit, float initMaxTranslationLimit,
						float initMotorSpeed, float initMaxMotorForce)
					   : JointInfo(rigidBody1, rigidBody2, SLIDERJOINT),
						 anchorPointWorldSpace(initAnchorPointWorldSpace),
						 sliderAxisWorldSpace(initSliderAxisWorldSpace),
						 isLimitEnabled(true), isMotorEnabled(true),
						 minTranslationLimit(initMinTranslationLimit),
						 maxTranslationLimit(initMaxTranslationLimit), motorSpeed(initMotorSpeed),
						 maxMotorForce(initMaxMotorForce) {}
};

// Class SliderJoint
/**
 * This class represents a slider joint. This joint has a one degree of freedom.
 * It only allows relative translation of the bodies along a single direction and no
 * rotation.
 */
class SliderJoint : public Joint {

	private :

		// -------------------- Constants -------------------- //

		// Beta value for the position correction bias factor
		static const float BETA;

		// -------------------- Attributes -------------------- //

		/// Anchor point of body 1 (in local-space coordinates of body 1)
		Vector3 mLocalAnchorPointBody1;

		/// Anchor point of body 2 (in local-space coordinates of body 2)
		Vector3 mLocalAnchorPointBody2;

		/// Slider axis (in local-space coordinates of body 1)
		Vector3 mSliderAxisBody1;

		/// Inertia tensor of body 1 (in world-space coordinates)
		Matrix3x3 mI1;

		/// Inertia tensor of body 2 (in world-space coordinates)
		Matrix3x3 mI2;

		/// Inverse of the initial orientation difference between the two bodies
		Quaternion mInitOrientationDifferenceInv;

		/// First vector orthogonal to the slider axis local-space of body 1
		Vector3 mN1;

		/// Second vector orthogonal to the slider axis and mN1 in local-space of body 1
		Vector3 mN2;

		/// Vector r1 in world-space coordinates
		Vector3 mR1;

		/// Vector r2 in world-space coordinates
		Vector3 mR2;

		/// Cross product of r2 and n1
		Vector3 mR2CrossN1;

		/// Cross product of r2 and n2
		Vector3 mR2CrossN2;

		/// Cross product of r2 and the slider axis
		Vector3 mR2CrossSliderAxis;

		/// Cross product of vector (r1 + u) and n1
		Vector3 mR1PlusUCrossN1;

		/// Cross product of vector (r1 + u) and n2
		Vector3 mR1PlusUCrossN2;

		/// Cross product of vector (r1 + u) and the slider axis
		Vector3 mR1PlusUCrossSliderAxis;

		/// Bias of the 2 translation constraints
		Vector2 mBTranslation;

		/// Bias of the 3 rotation constraints
		Vector3 mBRotation;

		/// Bias of the lower limit constraint
		float mBLowerLimit;

		/// Bias of the upper limit constraint
		float mBUpperLimit;

		/// Inverse of mass matrix K=JM^-1J^t for the translation constraint (2x2 matrix)
		Matrix2x2 mInverseMassMatrixTranslationConstraint;

		/// Inverse of mass matrix K=JM^-1J^t for the rotation constraint (3x3 matrix)
		Matrix3x3 mInverseMassMatrixRotationConstraint;

		/// Inverse of mass matrix K=JM^-1J^t for the upper and lower limit constraints (1x1 matrix)
		float mInverseMassMatrixLimit;

		/// Inverse of mass matrix K=JM^-1J^t for the motor
		float mInverseMassMatrixMotor;

		/// Accumulated impulse for the 2 translation constraints
		Vector2 mImpulseTranslation;

		/// Accumulated impulse for the 3 rotation constraints
		Vector3 mImpulseRotation;

		/// Accumulated impulse for the lower limit constraint
		float mImpulseLowerLimit;

		/// Accumulated impulse for the upper limit constraint
		float mImpulseUpperLimit;

		/// Accumulated impulse for the motor
		float mImpulseMotor;

		/// True if the slider limits are enabled
		bool mIsLimitEnabled;

		/// True if the motor of the joint in enabled
		bool mIsMotorEnabled;

		/// Slider axis in world-space coordinates
		Vector3 mSliderAxisWorld;

		/// Lower limit (minimum translation distance)
		float mLowerLimit;

		/// Upper limit (maximum translation distance)
		float mUpperLimit;

		/// True if the lower limit is violated
		bool mIsLowerLimitViolated;

		/// True if the upper limit is violated
		bool mIsUpperLimitViolated;

		/// Motor speed (in m/s)
		float mMotorSpeed;

		/// Maximum motor force (in Newtons) that can be applied to reach to desired motor speed
		float mMaxMotorForce;

		// -------------------- Methods -------------------- //

		/// Private copy-constructor
		SliderJoint(const SliderJoint& constraint);

		/// Private assignment operator
		SliderJoint& operator=(const SliderJoint& constraint);

		/// Reset the limits
		void resetLimits();

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
		SliderJoint(const SliderJointInfo& jointInfo);

		/// Destructor
		virtual ~SliderJoint();

		/// Return true if the limits or the joint are enabled
		bool isLimitEnabled() const;

		/// Return true if the motor of the joint is enabled
		bool isMotorEnabled() const;

		/// Enable/Disable the limits of the joint
		void enableLimit(bool isLimitEnabled);

		/// Enable/Disable the motor of the joint
		void enableMotor(bool isMotorEnabled);

		/// Return the current translation value of the joint
		float getTranslation() const;

		/// Return the minimum translation limit
		float getMinTranslationLimit() const;

		/// Set the minimum translation limit
		void setMinTranslationLimit(float lowerLimit);

		/// Return the maximum translation limit
		float getMaxTranslationLimit() const;

		/// Set the maximum translation limit
		void setMaxTranslationLimit(float upperLimit);

		/// Return the motor speed
		float getMotorSpeed() const;

		/// Set the motor speed
		void setMotorSpeed(float motorSpeed);

		/// Return the maximum motor force
		float getMaxMotorForce() const;

		/// Set the maximum motor force
		void setMaxMotorForce(float maxMotorForce);

		/// Return the int32_tensity of the current force applied for the joint motor
		float getMotorForce(float timeStep) const;
};

// Return true if the limits or the joint are enabled
/**
 * @return True if the joint limits are enabled
 */
inline bool SliderJoint::isLimitEnabled() const {
	return mIsLimitEnabled;
}

// Return true if the motor of the joint is enabled
/**
 * @return True if the joint motor is enabled
 */
inline bool SliderJoint::isMotorEnabled() const {
	return mIsMotorEnabled;
}

// Return the minimum translation limit
/**
 * @return The minimum translation limit of the joint (in meters)
 */
inline float SliderJoint::getMinTranslationLimit() const {
	return mLowerLimit;
}

// Return the maximum translation limit
/**
 * @return The maximum translation limit of the joint (in meters)
 */
inline float SliderJoint::getMaxTranslationLimit() const {
	return mUpperLimit;
}

// Return the motor speed
/**
 * @return The current motor speed of the joint (in meters per second)
 */
inline float SliderJoint::getMotorSpeed() const {
	return mMotorSpeed;
}

// Return the maximum motor force
/**
 * @return The maximum force of the joint motor (in Newton x meters)
 */
inline float SliderJoint::getMaxMotorForce() const {
	return mMaxMotorForce;
}

// Return the int32_tensity of the current force applied for the joint motor
/**
 * @param timeStep Time step (in seconds)
 * @return The current force of the joint motor (in Newton x meters)
 */
inline float SliderJoint::getMotorForce(float timeStep) const {
	return mImpulseMotor / timeStep;
}

// Return the number of bytes used by the joint
inline size_t SliderJoint::getSizeInBytes() const {
	return sizeof(SliderJoint);
}

}
