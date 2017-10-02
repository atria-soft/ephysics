/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#pragma once

#include <ephysics/constraint/Joint.hpp>
#include <ephysics/mathematics/mathematics.hpp>

namespace ephysics {
	/**
	 * @brief It is used to gather the information needed to create a ball-and-socket
	 * joint. This structure will be used to create the actual ball-and-socket joint.
	 */
	struct BallAndSocketJointInfo : public JointInfo {
		public :
			vec3 m_anchorPointWorldSpace; //!< Anchor point (in world-space coordinates)
			/**
			 * @brief Constructor
			 * @param _rigidBody1 Pointer to the first body of the joint
			 * @param _rigidBody2 Pointer to the second body of the joint
			 * @param _initAnchorPointWorldSpace The anchor point in world-space coordinates
			 */
			BallAndSocketJointInfo(RigidBody* _rigidBody1,
			                       RigidBody* _rigidBody2,
			                       const vec3& _initAnchorPointWorldSpace):
			  JointInfo(_rigidBody1, _rigidBody2, BALLSOCKETJOINT),
			  m_anchorPointWorldSpace(_initAnchorPointWorldSpace) {
				
			}
	};
	/**
	 * @brief Represents a ball-and-socket joint that allows arbitrary rotation
	 * between two bodies. This joint has three degrees of freedom. It can be used to
	 * create a chain of bodies for instance.
	 */
	class BallAndSocketJoint : public Joint {
		private:
			static const float BETA; //!< Beta value for the bias factor of position correction
			vec3 m_localAnchorPointBody1; //!< Anchor point of body 1 (in local-space coordinates of body 1)
			vec3 m_localAnchorPointBody2; //!< Anchor point of body 2 (in local-space coordinates of body 2)
			vec3 m_r1World; //!< Vector from center of body 2 to anchor point in world-space
			vec3 m_r2World; //!< Vector from center of body 2 to anchor point in world-space
			etk::Matrix3x3 m_i1; //!< Inertia tensor of body 1 (in world-space coordinates)
			etk::Matrix3x3 m_i2; //!< Inertia tensor of body 2 (in world-space coordinates)
			vec3 m_biasVector; //!< Bias vector for the constraint
			etk::Matrix3x3 m_inverseMassMatrix; //!< Inverse mass matrix K=JM^-1J^-t of the constraint
			vec3 m_impulse; //!< Accumulated impulse
			/// Private copy-constructor
			BallAndSocketJoint(const BallAndSocketJoint& constraint);
			/// Private assignment operator
			BallAndSocketJoint& operator=(const BallAndSocketJoint& constraint);
			size_t getSizeInBytes() const override {
				return sizeof(BallAndSocketJoint);
			}
			void initBeforeSolve(const ConstraintSolverData& _constraintSolverData) override;
			void warmstart(const ConstraintSolverData& _constraintSolverData) override;
			void solveVelocityConstraint(const ConstraintSolverData& _constraintSolverData) override;
			void solvePositionConstraint(const ConstraintSolverData& _constraintSolverData) override;
		public:
			/// Constructor
			BallAndSocketJoint(const BallAndSocketJointInfo& _jointInfo);
	};
}
