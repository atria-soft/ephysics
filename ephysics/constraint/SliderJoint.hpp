/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

#include <ephysics/mathematics/mathematics.hpp>
#include <ephysics/engine/ConstraintSolver.hpp>

namespace ephysics {
	/**
	 * This structure is used to gather the information needed to create a slider
	 * joint. This structure will be used to create the actual slider joint.
	 */
	struct SliderJointInfo : public JointInfo {
	
		public :
			vec3 m_anchorPointWorldSpace; //!< Anchor point (in world-space coordinates)
			vec3 sliderAxisWorldSpace; //!< Slider axis (in world-space coordinates)
			bool isLimitEnabled; //!< True if the slider limits are enabled
			bool isMotorEnabled; //!< True if the slider motor is enabled
			float minTranslationLimit; //!< Mininum allowed translation if limits are enabled
			float maxTranslationLimit; //!< Maximum allowed translation if limits are enabled
			float motorSpeed; //!< Motor speed
			float maxMotorForce; //!< Maximum motor force (in Newtons) that can be applied to reach to desired motor speed
			/** 
			 * @brief Constructor without limits and without motor
			 * @param[in] _rigidBody1 The first body of the joint
			 * @param[in] _rigidBody2 The second body of the joint
			 * @param[in] _initAnchorPointWorldSpace The initial anchor point in world-space
			 * @param[in] _initSliderAxisWorldSpace The initial slider axis in world-space
			 */
			SliderJointInfo(RigidBody* _rigidBody1,
			                RigidBody* _rigidBody2,
			                const vec3& _initAnchorPointWorldSpace,
			                const vec3& _initSliderAxisWorldSpace):
			  JointInfo(_rigidBody1, _rigidBody2, SLIDERJOINT),
			  m_anchorPointWorldSpace(_initAnchorPointWorldSpace),
			  sliderAxisWorldSpace(_initSliderAxisWorldSpace),
			  isLimitEnabled(false),
			  isMotorEnabled(false),
			  minTranslationLimit(-1.0),
			  maxTranslationLimit(1.0),
			  motorSpeed(0),
			  maxMotorForce(0) {
				
			}
			/**
			 * @brief Constructor with limits and no motor
			 * @param[in] _rigidBody1 The first body of the joint
			 * @param[in] _rigidBody2 The second body of the joint
			 * @param[in] _initAnchorPointWorldSpace The initial anchor point in world-space
			 * @param[in] _initSliderAxisWorldSpace The initial slider axis in world-space
			 * @param[in] _initMinTranslationLimit The initial minimum translation limit (in meters)
			 * @param[in] _initMaxTranslationLimit The initial maximum translation limit (in meters)
			 */
			SliderJointInfo(RigidBody* _rigidBody1,
			                RigidBody* _rigidBody2,
			                const vec3& _initAnchorPointWorldSpace,
			                const vec3& _initSliderAxisWorldSpace,
			                float _initMinTranslationLimit,
			                float _initMaxTranslationLimit):
			  JointInfo(_rigidBody1, _rigidBody2, SLIDERJOINT),
			  m_anchorPointWorldSpace(_initAnchorPointWorldSpace),
			  sliderAxisWorldSpace(_initSliderAxisWorldSpace),
			  isLimitEnabled(true),
			  isMotorEnabled(false),
			  minTranslationLimit(_initMinTranslationLimit),
			  maxTranslationLimit(_initMaxTranslationLimit),
			  motorSpeed(0),
			  maxMotorForce(0) {
				
			}
			/**
			 * @brief Constructor with limits and motor
			 * @param[in] _rigidBody1 The first body of the joint
			 * @param[in] _rigidBody2 The second body of the joint
			 * @param[in] _initAnchorPointWorldSpace The initial anchor point in world-space
			 * @param[in] _initSliderAxisWorldSpace The initial slider axis in world-space
			 * @param[in] _initMinTranslationLimit The initial minimum translation limit (in meters)
			 * @param[in] _initMaxTranslationLimit The initial maximum translation limit (in meters)
			 * @param[in] _initMotorSpeed The initial speed of the joint motor (in meters per second)
			 * @param[in] _initMaxMotorForce The initial maximum motor force of the joint (in Newtons x meters)
			 */
			SliderJointInfo(RigidBody* _rigidBody1,
			                RigidBody* _rigidBody2,
			                const vec3& _initAnchorPointWorldSpace,
			                const vec3& _initSliderAxisWorldSpace,
			                float _initMinTranslationLimit,
			                float _initMaxTranslationLimit,
			                float _initMotorSpeed,
			                float _initMaxMotorForce):
			  JointInfo(_rigidBody1, _rigidBody2, SLIDERJOINT),
			  m_anchorPointWorldSpace(_initAnchorPointWorldSpace),
			  sliderAxisWorldSpace(_initSliderAxisWorldSpace),
			  isLimitEnabled(true),
			  isMotorEnabled(true),
			  minTranslationLimit(_initMinTranslationLimit),
			  maxTranslationLimit(_initMaxTranslationLimit),
			  motorSpeed(_initMotorSpeed),
			  maxMotorForce(_initMaxMotorForce) {
				
			}
	};
	
	/**
	 * @brief This class represents a slider joint. This joint has a one degree of freedom.
	 * It only allows relative translation of the bodies along a single direction and no
	 * rotation.
	 */
	class SliderJoint: public Joint {
		private:
			static const float BETA; //!< Beta value for the position correction bias factor
			vec3 m_localAnchorPointBody1; //!< Anchor point of body 1 (in local-space coordinates of body 1)
			vec3 m_localAnchorPointBody2; //!< Anchor point of body 2 (in local-space coordinates of body 2)
			vec3 m_sliderAxisBody1; //!< Slider axis (in local-space coordinates of body 1)
			etk::Matrix3x3 m_i1; //!< Inertia tensor of body 1 (in world-space coordinates)
			etk::Matrix3x3 m_i2; //!< Inertia tensor of body 2 (in world-space coordinates)
			etk::Quaternion m_initOrientationDifferenceInv; //!< Inverse of the initial orientation difference between the two bodies
			vec3 m_N1; //!< First vector orthogonal to the slider axis local-space of body 1
			vec3 m_N2; //!< Second vector orthogonal to the slider axis and m_N1 in local-space of body 1
			vec3 m_R1; //!< Vector r1 in world-space coordinates
			vec3 m_R2; //!< Vector r2 in world-space coordinates
			vec3 m_R2CrossN1; //!< Cross product of r2 and n1
			vec3 m_R2CrossN2; //!< Cross product of r2 and n2
			vec3 m_R2CrossSliderAxis; //!< Cross product of r2 and the slider axis
			vec3 m_R1PlusUCrossN1; //!< Cross product of vector (r1 + u) and n1
			vec3 m_R1PlusUCrossN2; //!< Cross product of vector (r1 + u) and n2
			vec3 m_R1PlusUCrossSliderAxis; //!< Cross product of vector (r1 + u) and the slider axis
			vec2 m_bTranslation; //!< Bias of the 2 translation constraints
			vec3 m_bRotation; //!< Bias of the 3 rotation constraints
			float m_bLowerLimit; //!< Bias of the lower limit constraint
			float m_bUpperLimit; //!< Bias of the upper limit constraint
			etk::Matrix2x2 m_inverseMassMatrixTranslationConstraint; //!< Inverse of mass matrix K=JM^-1J^t for the translation constraint (2x2 matrix)
			etk::Matrix3x3 m_inverseMassMatrixRotationConstraint; //!< Inverse of mass matrix K=JM^-1J^t for the rotation constraint (3x3 matrix)
			float m_inverseMassMatrixLimit; //!< Inverse of mass matrix K=JM^-1J^t for the upper and lower limit constraints (1x1 matrix)
			float m_inverseMassMatrixMotor; //!< Inverse of mass matrix K=JM^-1J^t for the motor
			vec2 m_impulseTranslation; //!< Accumulated impulse for the 2 translation constraints
			vec3 m_impulseRotation; //!< Accumulated impulse for the 3 rotation constraints
			float m_impulseLowerLimit; //!< Accumulated impulse for the lower limit constraint
			float m_impulseUpperLimit; //!< Accumulated impulse for the upper limit constraint
			float m_impulseMotor; //!< Accumulated impulse for the motor
			bool m_isLimitEnabled; //!< True if the slider limits are enabled
			bool m_isMotorEnabled; //!< True if the motor of the joint in enabled
			vec3 m_sliderAxisWorld; //!< Slider axis in world-space coordinates
			float m_lowerLimit; //!< Lower limit (minimum translation distance)
			float m_upperLimit; //!< Upper limit (maximum translation distance)
			bool m_isLowerLimitViolated; //!< True if the lower limit is violated
			bool m_isUpperLimitViolated; //!< True if the upper limit is violated
			float m_motorSpeed; //!< Motor speed (in m/s)
			float m_maxMotorForce; //!< Maximum motor force (in Newtons) that can be applied to reach to desired motor speed
			/// Private copy-constructor
			SliderJoint(const SliderJoint& _constraint);
			/// Private assignment operator
			SliderJoint& operator=(const SliderJoint& _constraint);
			/// Reset the limits
			void resetLimits();
			size_t getSizeInBytes() const override;
			void initBeforeSolve(const ConstraintSolverData& _constraintSolverData) override;
			void warmstart(const ConstraintSolverData& _constraintSolverData) override;
			void solveVelocityConstraint(const ConstraintSolverData& _constraintSolverData) override;
			void solvePositionConstraint(const ConstraintSolverData& _constraintSolverData) override;
		public :
			/// Constructor
			SliderJoint(const SliderJointInfo& _jointInfo);
			/// Destructor
			virtual ~SliderJoint();
			/// Return true if the limits or the joint are enabled
			bool isLimitEnabled() const;
			/// Return true if the motor of the joint is enabled
			bool isMotorEnabled() const;
			/// Enable/Disable the limits of the joint
			void enableLimit(bool _isLimitEnabled);
			/// Enable/Disable the motor of the joint
			void enableMotor(bool _isMotorEnabled);
			/// Return the current translation value of the joint
			float getTranslation() const;
			/// Return the minimum translation limit
			float getMinTranslationLimit() const;
			/// Set the minimum translation limit
			void setMinTranslationLimit(float _lowerLimit);
			/// Return the maximum translation limit
			float getMaxTranslationLimit() const;
			/// Set the maximum translation limit
			void setMaxTranslationLimit(float _upperLimit);
			/// Return the motor speed
			float getMotorSpeed() const;
			/// Set the motor speed
			void setMotorSpeed(float _motorSpeed);
			/// Return the maximum motor force
			float getMaxMotorForce() const;
			/// Set the maximum motor force
			void setMaxMotorForce(float _maxMotorForce);
			/// Return the int32_tensity of the current force applied for the joint motor
			float getMotorForce(float _timeStep) const;
	};


}
