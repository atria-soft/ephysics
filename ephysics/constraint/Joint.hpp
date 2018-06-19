/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#pragma once

#include <ephysics/configuration.hpp>
#include <ephysics/body/RigidBody.hpp>
#include <ephysics/mathematics/mathematics.hpp>

namespace ephysics {

	/// Enumeration for the type of a constraint
	enum JointType {BALLSOCKETJOINT, SLIDERJOINT, HINGEJOINT, FIXEDJOINT};
	
	struct ConstraintSolverData;
	class Joint;
	
	/**
	 * @brief It represents a single element of a linked list of joints
	 */
	struct JointListElement {
		public:
			Joint* joint; //!< Pointer to the actual joint
			JointListElement* next; //!< Next element of the list
			/**
			 * @breif Constructor
			 */
			JointListElement(Joint* _initJoint,
			                 JointListElement* _initNext):
			  joint(_initJoint),
			  next(_initNext) {
				
			}
	};
	
	/**
	 * @brief It is used to gather the information needed to create a joint.
	 */
	struct JointInfo {
		public :
			RigidBody* body1; //!< First rigid body of the joint
			RigidBody* body2; //!< Second rigid body of the joint
			JointType type; //!< Type of the joint
			JointsPositionCorrectionTechnique positionCorrectionTechnique; //!< Position correction technique used for the constraint (used for joints). By default, the BAUMGARTE technique is used
			bool isCollisionEnabled; //!< True if the two bodies of the joint are allowed to collide with each other
			/// Constructor
			JointInfo(JointType _constraintType):
			  body1(null),
			  body2(null),
			  type(_constraintType),
			  positionCorrectionTechnique(NON_LINEAR_GAUSS_SEIDEL),
			  isCollisionEnabled(true) {
				
			}
			/// Constructor
			JointInfo(RigidBody* _rigidBody1,
			          RigidBody* _rigidBody2,
			          JointType _constraintType):
			  body1(_rigidBody1),
			  body2(_rigidBody2),
			  type(_constraintType),
			  positionCorrectionTechnique(NON_LINEAR_GAUSS_SEIDEL),
			  isCollisionEnabled(true) {
				
			}
			/// Destructor
			virtual ~JointInfo() = default;
	};
	
	/**
	 * @brief It represents a joint between two bodies.
	 */
	class Joint {
		protected :
			RigidBody* const m_body1; //!< Pointer to the first body of the joint
			RigidBody* const m_body2; //!< Pointer to the second body of the joint
			const JointType m_type; //!< Type of the joint
			uint32_t m_indexBody1; //!< Body 1 index in the velocity array to solve the constraint
			uint32_t m_indexBody2; //!< Body 2 index in the velocity array to solve the constraint
			JointsPositionCorrectionTechnique m_positionCorrectionTechnique; //!< Position correction technique used for the constraint (used for joints)
			bool m_isCollisionEnabled; //!< True if the two bodies of the constraint are allowed to collide with each other
			bool m_isAlreadyInIsland; //!< True if the joint has already been added int32_to an island
			/// Private copy-constructor
			Joint(const Joint& _constraint);
			/// Private assignment operator
			Joint& operator=(const Joint& _constraint);
			/// Return true if the joint has already been added int32_to an island
			bool isAlreadyInIsland() const;
			/// Return the number of bytes used by the joint
			virtual size_t getSizeInBytes() const = 0;
			/// Initialize before solving the joint
			virtual void initBeforeSolve(const ConstraintSolverData& _constraintSolverData) = 0;
			/// Warm start the joint (apply the previous impulse at the beginning of the step)
			virtual void warmstart(const ConstraintSolverData& _constraintSolverData) = 0;
			/// Solve the velocity constraint
			virtual void solveVelocityConstraint(const ConstraintSolverData& _constraintSolverData) = 0;
			/// Solve the position constraint
			virtual void solvePositionConstraint(const ConstraintSolverData& _constraintSolverData) = 0;
		public :
			/// Constructor
			Joint(const JointInfo& _jointInfo);
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
			friend class DynamicsWorld;
			friend class Island;
			friend class ConstraintSolver;
	};

}
