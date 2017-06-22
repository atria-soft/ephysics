/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

#include <ephysics/constraint/Joint.hpp>
#include <ephysics/mathematics/mathematics.hpp>

namespace ephysics {
	/**
	 * @brief It is used to gather the information needed to create a hinge joint.
	 * This structure will be used to create the actual hinge joint.
	 */
	struct HingeJointInfo : public JointInfo {
		public :
			vec3 m_anchorPointWorldSpace; //!< Anchor point (in world-space coordinates)
			vec3 rotationAxisWorld; //!< Hinge rotation axis (in world-space coordinates)
			bool isLimitEnabled; //!< True if the hinge joint limits are enabled
			bool isMotorEnabled; //!< True if the hinge joint motor is enabled
			float minAngleLimit; //!< Minimum allowed rotation angle (in radian) if limits are enabled. The angle must be in the range [-2*pi, 0]
			float maxAngleLimit; //!< Maximum allowed rotation angle (in radian) if limits are enabled. The angle must be in the range [0, 2*pi]
			float motorSpeed; //!< Motor speed (in radian/second)
			float maxMotorTorque; //!< Maximum motor torque (in Newtons * meters) that can be applied to reach to desired motor speed
			/**
			 * @brief Constructor without limits and without motor
			 * @param[in] _rigidBody1 The first body of the joint
			 * @param[in] _rigidBody2 The second body of the joint
			 * @param[in] _initAnchorPointWorldSpace The initial anchor point in world-space coordinates
			 * @param[in] _initRotationAxisWorld The initial rotation axis in world-space coordinates
			 */
			HingeJointInfo(RigidBody* _rigidBody1,
			               RigidBody* _rigidBody2,
			               const vec3& _initAnchorPointWorldSpace,
			               const vec3& _initRotationAxisWorld):
			  JointInfo(_rigidBody1, _rigidBody2, HINGEJOINT),
			  m_anchorPointWorldSpace(_initAnchorPointWorldSpace),
			  rotationAxisWorld(_initRotationAxisWorld),
			  isLimitEnabled(false),
			  isMotorEnabled(false),
			  minAngleLimit(-1),
			  maxAngleLimit(1),
			  motorSpeed(0),
			  maxMotorTorque(0) {
				
			}
			/**
			 * @brief Constructor with limits but without motor
			 * @param[in] _rigidBody1 The first body of the joint
			 * @param[in] _rigidBody2 The second body of the joint
			 * @param[in] _initAnchorPointWorldSpace The initial anchor point in world-space coordinates
			 * @param[in] _initRotationAxisWorld The int32_tial rotation axis in world-space coordinates
			 * @param[in] _initMinAngleLimit The initial minimum limit angle (in radian)
			 * @param[in] _initMaxAngleLimit The initial maximum limit angle (in radian)
			 */
			HingeJointInfo(RigidBody* _rigidBody1,
			               RigidBody* _rigidBody2,
			               const vec3& _initAnchorPointWorldSpace,
			               const vec3& _initRotationAxisWorld,
			               float _initMinAngleLimit,
			               float _initMaxAngleLimit):
			  JointInfo(_rigidBody1, _rigidBody2, HINGEJOINT),
			  m_anchorPointWorldSpace(_initAnchorPointWorldSpace),
			  rotationAxisWorld(_initRotationAxisWorld),
			  isLimitEnabled(true),
			  isMotorEnabled(false),
			  minAngleLimit(_initMinAngleLimit),
			  maxAngleLimit(_initMaxAngleLimit),
			  motorSpeed(0),
			  maxMotorTorque(0) {
				
			}
			/**
			 * @brief Constructor with limits and motor
			 * @param[in] _rigidBody1 The first body of the joint
			 * @param[in] _rigidBody2 The second body of the joint
			 * @param[in] _initAnchorPointWorldSpace The initial anchor point in world-space
			 * @param[in] _initRotationAxisWorld The initial rotation axis in world-space
			 * @param[in] _initMinAngleLimit The initial minimum limit angle (in radian)
			 * @param[in] _initMaxAngleLimit The initial maximum limit angle (in radian)
			 * @param[in] _initMotorSpeed The initial motor speed of the joint (in radian per second)
			 * @param[in] _initMaxMotorTorque The initial maximum motor torque (in Newtons)
			 */
			HingeJointInfo(RigidBody* _rigidBody1,
			               RigidBody* _rigidBody2,
			               const vec3& _initAnchorPointWorldSpace,
			               const vec3& _initRotationAxisWorld,
			               float _initMinAngleLimit,
			               float _initMaxAngleLimit,
			               float _initMotorSpeed,
			               float _initMaxMotorTorque):
			  JointInfo(_rigidBody1, _rigidBody2, HINGEJOINT),
			  m_anchorPointWorldSpace(_initAnchorPointWorldSpace),
			  rotationAxisWorld(_initRotationAxisWorld),
			  isLimitEnabled(true),
			  isMotorEnabled(false),
			  minAngleLimit(_initMinAngleLimit),
			  maxAngleLimit(_initMaxAngleLimit),
			  motorSpeed(_initMotorSpeed),
			  maxMotorTorque(_initMaxMotorTorque) {
			
			}
	};

	/**
	 * @brief It represents a hinge joint that allows arbitrary rotation
	 * between two bodies around a single axis. This joint has one degree of freedom. It
	 * can be useful to simulate doors or pendulumns.
	 */
	class HingeJoint : public Joint {
		private :
			static const float BETA; //!< Beta value for the bias factor of position correction
			vec3 m_localAnchorPointBody1; //!< Anchor point of body 1 (in local-space coordinates of body 1)
			vec3 m_localAnchorPointBody2; //!< Anchor point of body 2 (in local-space coordinates of body 2)
			vec3 m_hingeLocalAxisBody1; //!< Hinge rotation axis (in local-space coordinates of body 1)
			vec3 m_hingeLocalAxisBody2; //!< Hinge rotation axis (in local-space coordiantes of body 2)
			etk::Matrix3x3 m_i1; //!< Inertia tensor of body 1 (in world-space coordinates)
			etk::Matrix3x3 m_i2; //!< Inertia tensor of body 2 (in world-space coordinates)
			vec3 mA1; //!< Hinge rotation axis (in world-space coordinates) computed from body 1
			vec3 m_r1World; //!< Vector from center of body 2 to anchor point in world-space
			vec3 m_r2World; //!< Vector from center of body 2 to anchor point in world-space
			vec3 m_b2CrossA1; //!< Cross product of vector b2 and a1
			vec3 m_c2CrossA1; //!< Cross product of vector c2 and a1;
			vec3 m_impulseTranslation; //!< Impulse for the 3 translation constraints
			vec2 m_impulseRotation; //!< Impulse for the 2 rotation constraints
			float m_impulseLowerLimit; //!< Accumulated impulse for the lower limit constraint
			float m_impulseUpperLimit; //!< Accumulated impulse for the upper limit constraint
			float m_impulseMotor; //!< Accumulated impulse for the motor constraint;
			etk::Matrix3x3 m_inverseMassMatrixTranslation; //!< Inverse mass matrix K=JM^-1J^t for the 3 translation constraints
			etk::Matrix2x2 m_inverseMassMatrixRotation; //!< Inverse mass matrix K=JM^-1J^t for the 2 rotation constraints
			float m_inverseMassMatrixLimitMotor; //!< Inverse of mass matrix K=JM^-1J^t for the limits and motor constraints (1x1 matrix)
			float m_inverseMassMatrixMotor; //!< Inverse of mass matrix K=JM^-1J^t for the motor
			vec3 m_bTranslation; //!< Bias vector for the error correction for the translation constraints
			vec2 m_bRotation; //!< Bias vector for the error correction for the rotation constraints
			float m_bLowerLimit; //!< Bias of the lower limit constraint
			float m_bUpperLimit; //!< Bias of the upper limit constraint
			etk::Quaternion m_initOrientationDifferenceInv; //!< Inverse of the initial orientation difference between the bodies
			bool m_isLimitEnabled; //!< True if the joint limits are enabled
			bool m_isMotorEnabled; //!< True if the motor of the joint in enabled
			float m_lowerLimit; //!< Lower limit (minimum allowed rotation angle in radian)
			float m_upperLimit; //!< Upper limit (maximum translation distance)
			bool m_isLowerLimitViolated; //!< True if the lower limit is violated
			bool m_isUpperLimitViolated; //!< True if the upper limit is violated
			float m_motorSpeed; //!< Motor speed (in rad/s)
			float m_maxMotorTorque; //!< Maximum motor torque (in Newtons) that can be applied to reach to desired motor speed
			/// Private copy-constructor
			HingeJoint(const HingeJoint& _constraint);
			/// Private assignment operator
			HingeJoint& operator=(const HingeJoint& _constraint);
			/// Reset the limits
			void resetLimits();
			/// Given an angle in radian, this method returns the corresponding
			/// angle in the range [-pi; pi]
			float computeNormalizedAngle(float _angle) const;
			/// Given an "inputAngle" in the range [-pi, pi], this method returns an
			/// angle (modulo 2*pi) in the range [-2*pi; 2*pi] that is closest to one of the
			/// two angle limits in arguments.
			float computeCorrespondingAngleNearLimits(float _inputAngle,
			                                          float _lowerLimitAngle,
			                                          float _upperLimitAngle) const;
			/// Compute the current angle around the hinge axis
			float computeCurrentHingeAngle(const etk::Quaternion& _orientationBody1,
											 const etk::Quaternion& _orientationBody2);
			/// Return the number of bytes used by the joint
			virtual size_t getSizeInBytes() const;
			/// Initialize before solving the constraint
			virtual void initBeforeSolve(const ConstraintSolverData& _constraintSolverData);
			/// Warm start the constraint (apply the previous impulse at the beginning of the step)
			virtual void warmstart(const ConstraintSolverData& _constraintSolverData);
			/// Solve the velocity constraint
			virtual void solveVelocityConstraint(const ConstraintSolverData& _constraintSolverData);
			/// Solve the position constraint (for position error correction)
			virtual void solvePositionConstraint(const ConstraintSolverData& _constraintSolverData);
		public :
			/// Constructor
			HingeJoint(const HingeJointInfo& _jointInfo);
			/// Destructor
			virtual ~HingeJoint();
			/// Return true if the limits or the joint are enabled
			bool isLimitEnabled() const;
			/// Return true if the motor of the joint is enabled
			bool isMotorEnabled() const;
			/// Enable/Disable the limits of the joint
			void enableLimit(bool _isLimitEnabled);
			/// Enable/Disable the motor of the joint
			void enableMotor(bool _isMotorEnabled);
			/// Return the minimum angle limit
			float getMinAngleLimit() const;
			/// Set the minimum angle limit
			void setMinAngleLimit(float _lowerLimit);
			/// Return the maximum angle limit
			float getMaxAngleLimit() const;
			/// Set the maximum angle limit
			void setMaxAngleLimit(float _upperLimit);
			/// Return the motor speed
			float getMotorSpeed() const;
			/// Set the motor speed
			void setMotorSpeed(float _motorSpeed);
			/// Return the maximum motor torque
			float getMaxMotorTorque() const;
			/// Set the maximum motor torque
			void setMaxMotorTorque(float _maxMotorTorque);
			/// Return the int32_tensity of the current torque applied for the joint motor
			float getMotorTorque(float _timeStep) const;
	};

}

