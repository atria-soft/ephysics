/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/constraint/Joint.hpp>
#include <ephysics/mathematics/mathematics.hpp>

namespace ephysics {

	/**
	 * This structure is used to gather the information needed to create a fixed
	 * joint. This structure will be used to create the actual fixed joint.
	 */
	struct FixedJointInfo : public JointInfo {
		public :
			vec3 m_anchorPointWorldSpace; //!< Anchor point (in world-space coordinates)
			/**
			 * @breif Contructor
			 * @param _rigidBody1 The first body of the joint
			 * @param _rigidBody2 The second body of the joint
			 * @param _initAnchorPointWorldSpace The initial anchor point of the joint in world-space coordinates
			 */
			FixedJointInfo(RigidBody* _rigidBody1,
			               RigidBody* _rigidBody2,
			               const vec3& _initAnchorPointWorldSpace):
			  JointInfo(_rigidBody1, _rigidBody2, FIXEDJOINT),
			  m_anchorPointWorldSpace(_initAnchorPointWorldSpace){
				
			}
	};
	
	/**
	 * @breif It represents a fixed joint that is used to forbid any translation or rotation
	 * between two bodies.
	 */
	class FixedJoint : public Joint {
		private :
			static const float BETA; //!< Beta value for the bias factor of position correction
			vec3 m_localAnchorPointBody1; //!< Anchor point of body 1 (in local-space coordinates of body 1)
			vec3 m_localAnchorPointBody2; //!< Anchor point of body 2 (in local-space coordinates of body 2)
			vec3 m_r1World; //!< Vector from center of body 2 to anchor point in world-space
			vec3 m_r2World; //!< Vector from center of body 2 to anchor point in world-space
			etk::Matrix3x3 m_i1; //!< Inertia tensor of body 1 (in world-space coordinates)
			etk::Matrix3x3 m_i2; //!< Inertia tensor of body 2 (in world-space coordinates)
			vec3 m_impulseTranslation; //!< Accumulated impulse for the 3 translation constraints
			vec3 m_impulseRotation; //!< Accumulate impulse for the 3 rotation constraints
			etk::Matrix3x3 m_inverseMassMatrixTranslation; //!< Inverse mass matrix K=JM^-1J^-t of the 3 translation constraints (3x3 matrix)
			etk::Matrix3x3 m_inverseMassMatrixRotation; //!< Inverse mass matrix K=JM^-1J^-t of the 3 rotation constraints (3x3 matrix)
			vec3 m_biasTranslation; //!< Bias vector for the 3 translation constraints
			vec3 m_biasRotation; //!< Bias vector for the 3 rotation constraints
			etk::Quaternion m_initOrientationDifferenceInv; //!< Inverse of the initial orientation difference between the two bodies
			/// Private copy-constructor
			FixedJoint(const FixedJoint& constraint);
			/// Private assignment operator
			FixedJoint& operator=(const FixedJoint& constraint);
			/// Return the number of bytes used by the joint
			virtual size_t getSizeInBytes() const override {
				return sizeof(FixedJoint);
			}
			/// Initialize before solving the constraint
			virtual void initBeforeSolve(const ConstraintSolverData& constraintSolverData);
			/// Warm start the constraint (apply the previous impulse at the beginning of the step)
			virtual void warmstart(const ConstraintSolverData& constraintSolverData);
			/// Solve the velocity constraint
			virtual void solveVelocityConstraint(const ConstraintSolverData& constraintSolverData);
			/// Solve the position constraint (for position error correction)
			virtual void solvePositionConstraint(const ConstraintSolverData& constraintSolverData);
		public :
			/// Constructor
			FixedJoint(const FixedJointInfo& jointInfo);
			/// Destructor
			virtual ~FixedJoint();
	};
}

