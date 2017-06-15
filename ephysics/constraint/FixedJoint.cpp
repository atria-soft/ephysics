/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/constraint/FixedJoint.h>
#include <ephysics/engine/ConstraintSolver.h>

using namespace reactphysics3d;

// Static variables definition
const float FixedJoint::BETA = float(0.2);

// Constructor
FixedJoint::FixedJoint(const FixedJointInfo& jointInfo)
		   : Joint(jointInfo), m_impulseTranslation(0, 0, 0), m_impulseRotation(0, 0, 0) {

	// Compute the local-space anchor point for each body
	const etk::Transform3D& transform1 = m_body1->getTransform();
	const etk::Transform3D& transform2 = m_body2->getTransform();
	m_localAnchorPointBody1 = transform1.getInverse() * jointInfo.m_anchorPointWorldSpace;
	m_localAnchorPointBody2 = transform2.getInverse() * jointInfo.m_anchorPointWorldSpace;

	// Compute the inverse of the initial orientation difference between the two bodies
	m_initOrientationDifferenceInv = transform2.getOrientation() *
									transform1.getOrientation().getInverse();
	m_initOrientationDifferenceInv.normalize();
	m_initOrientationDifferenceInv.inverse();
}

// Destructor
FixedJoint::~FixedJoint() {

}

// Initialize before solving the constraint
void FixedJoint::initBeforeSolve(const ConstraintSolverData& constraintSolverData) {

	// Initialize the bodies index in the velocity array
	m_indexBody1 = constraintSolverData.mapBodyToConstrainedVelocityIndex.find(m_body1)->second;
	m_indexBody2 = constraintSolverData.mapBodyToConstrainedVelocityIndex.find(m_body2)->second;

	// Get the bodies positions and orientations
	const vec3& x1 = m_body1->m_centerOfMassWorld;
	const vec3& x2 = m_body2->m_centerOfMassWorld;
	const etk::Quaternion& orientationBody1 = m_body1->getTransform().getOrientation();
	const etk::Quaternion& orientationBody2 = m_body2->getTransform().getOrientation();

	// Get the inertia tensor of bodies
	m_i1 = m_body1->getInertiaTensorInverseWorld();
	m_i2 = m_body2->getInertiaTensorInverseWorld();

	// Compute the vector from body center to the anchor point in world-space
	m_r1World = orientationBody1 * m_localAnchorPointBody1;
	m_r2World = orientationBody2 * m_localAnchorPointBody2;

	// Compute the corresponding skew-symmetric matrices
	etk::Matrix3x3 skewSymmetricMatrixU1= etk::Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(m_r1World);
	etk::Matrix3x3 skewSymmetricMatrixU2= etk::Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(m_r2World);

	// Compute the matrix K=JM^-1J^t (3x3 matrix) for the 3 translation constraints
	float inverseMassBodies = m_body1->m_massInverse + m_body2->m_massInverse;
	etk::Matrix3x3 massMatrix = etk::Matrix3x3(inverseMassBodies, 0, 0,
	                                           0, inverseMassBodies, 0,
	                                           0, 0, inverseMassBodies)
	                            + skewSymmetricMatrixU1 * m_i1 * skewSymmetricMatrixU1.getTranspose()
	                            + skewSymmetricMatrixU2 * m_i2 * skewSymmetricMatrixU2.getTranspose();

	// Compute the inverse mass matrix K^-1 for the 3 translation constraints
	m_inverseMassMatrixTranslation.setZero();
	if (m_body1->getType() == DYNAMIC || m_body2->getType() == DYNAMIC) {
		m_inverseMassMatrixTranslation = massMatrix.getInverse();
	}

	// Compute the bias "b" of the constraint for the 3 translation constraints
	float biasFactor = (BETA / constraintSolverData.timeStep);
	m_biasTranslation.setZero();
	if (m_positionCorrectionTechnique == BAUMGARTE_JOINTS) {
		m_biasTranslation = biasFactor * (x2 + m_r2World - x1 - m_r1World);
	}

	// Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
	// contraints (3x3 matrix)
	m_inverseMassMatrixRotation = m_i1 + m_i2;
	if (m_body1->getType() == DYNAMIC || m_body2->getType() == DYNAMIC) {
		m_inverseMassMatrixRotation = m_inverseMassMatrixRotation.getInverse();
	}

	// Compute the bias "b" for the 3 rotation constraints
	m_biasRotation.setZero();
	if (m_positionCorrectionTechnique == BAUMGARTE_JOINTS) {
		etk::Quaternion currentOrientationDifference = orientationBody2 * orientationBody1.getInverse();
		currentOrientationDifference.normalize();
		const etk::Quaternion qError = currentOrientationDifference * m_initOrientationDifferenceInv;
		m_biasRotation = biasFactor * float(2.0) * qError.getVectorV();
	}

	// If warm-starting is not enabled
	if (!constraintSolverData.isWarmStartingActive) {

		// Reset the accumulated impulses
		m_impulseTranslation.setZero();
		m_impulseRotation.setZero();
	}
}

// Warm start the constraint (apply the previous impulse at the beginning of the step)
void FixedJoint::warmstart(const ConstraintSolverData& constraintSolverData) {

	// Get the velocities
	vec3& v1 = constraintSolverData.linearVelocities[m_indexBody1];
	vec3& v2 = constraintSolverData.linearVelocities[m_indexBody2];
	vec3& w1 = constraintSolverData.angularVelocities[m_indexBody1];
	vec3& w2 = constraintSolverData.angularVelocities[m_indexBody2];

	// Get the inverse mass of the bodies
	const float inverseMassBody1 = m_body1->m_massInverse;
	const float inverseMassBody2 = m_body2->m_massInverse;

	// Compute the impulse P=J^T * lambda for the 3 translation constraints for body 1
	vec3 linearImpulseBody1 = -m_impulseTranslation;
	vec3 angularImpulseBody1 = m_impulseTranslation.cross(m_r1World);

	// Compute the impulse P=J^T * lambda for the 3 rotation constraints for body 1
	angularImpulseBody1 += -m_impulseRotation;

	// Apply the impulse to the body 1
	v1 += inverseMassBody1 * linearImpulseBody1;
	w1 += m_i1 * angularImpulseBody1;

	// Compute the impulse P=J^T * lambda for the 3 translation constraints for body 2
	vec3 angularImpulseBody2 = -m_impulseTranslation.cross(m_r2World);

	// Compute the impulse P=J^T * lambda for the 3 rotation constraints for body 2
	angularImpulseBody2 += m_impulseRotation;

	// Apply the impulse to the body 2
	v2 += inverseMassBody2 * m_impulseTranslation;
	w2 += m_i2 * angularImpulseBody2;
}

// Solve the velocity constraint
void FixedJoint::solveVelocityConstraint(const ConstraintSolverData& constraintSolverData) {

	// Get the velocities
	vec3& v1 = constraintSolverData.linearVelocities[m_indexBody1];
	vec3& v2 = constraintSolverData.linearVelocities[m_indexBody2];
	vec3& w1 = constraintSolverData.angularVelocities[m_indexBody1];
	vec3& w2 = constraintSolverData.angularVelocities[m_indexBody2];

	// Get the inverse mass of the bodies
	float inverseMassBody1 = m_body1->m_massInverse;
	float inverseMassBody2 = m_body2->m_massInverse;

	// --------------- Translation Constraints --------------- //

	// Compute J*v for the 3 translation constraints
	const vec3 JvTranslation = v2 + w2.cross(m_r2World) - v1 - w1.cross(m_r1World);

	// Compute the Lagrange multiplier lambda
	const vec3 deltaLambda = m_inverseMassMatrixTranslation *
							   (-JvTranslation - m_biasTranslation);
	m_impulseTranslation += deltaLambda;

	// Compute the impulse P=J^T * lambda for body 1
	const vec3 linearImpulseBody1 = -deltaLambda;
	vec3 angularImpulseBody1 = deltaLambda.cross(m_r1World);

	// Apply the impulse to the body 1
	v1 += inverseMassBody1 * linearImpulseBody1;
	w1 += m_i1 * angularImpulseBody1;

	// Compute the impulse P=J^T * lambda  for body 2
	const vec3 angularImpulseBody2 = -deltaLambda.cross(m_r2World);

	// Apply the impulse to the body 2
	v2 += inverseMassBody2 * deltaLambda;
	w2 += m_i2 * angularImpulseBody2;

	// --------------- Rotation Constraints --------------- //

	// Compute J*v for the 3 rotation constraints
	const vec3 JvRotation = w2 - w1;

	// Compute the Lagrange multiplier lambda for the 3 rotation constraints
	vec3 deltaLambda2 = m_inverseMassMatrixRotation * (-JvRotation - m_biasRotation);
	m_impulseRotation += deltaLambda2;

	// Compute the impulse P=J^T * lambda for the 3 rotation constraints for body 1
	angularImpulseBody1 = -deltaLambda2;

	// Apply the impulse to the body 1
	w1 += m_i1 * angularImpulseBody1;

	// Apply the impulse to the body 2
	w2 += m_i2 * deltaLambda2;
}

// Solve the position constraint (for position error correction)
void FixedJoint::solvePositionConstraint(const ConstraintSolverData& constraintSolverData) {

	// If the error position correction technique is not the non-linear-gauss-seidel, we do
	// do not execute this method
	if (m_positionCorrectionTechnique != NON_LINEAR_GAUSS_SEIDEL) return;

	// Get the bodies positions and orientations
	vec3& x1 = constraintSolverData.positions[m_indexBody1];
	vec3& x2 = constraintSolverData.positions[m_indexBody2];
	etk::Quaternion& q1 = constraintSolverData.orientations[m_indexBody1];
	etk::Quaternion& q2 = constraintSolverData.orientations[m_indexBody2];

	// Get the inverse mass and inverse inertia tensors of the bodies
	float inverseMassBody1 = m_body1->m_massInverse;
	float inverseMassBody2 = m_body2->m_massInverse;

	// Recompute the inverse inertia tensors
	m_i1 = m_body1->getInertiaTensorInverseWorld();
	m_i2 = m_body2->getInertiaTensorInverseWorld();

	// Compute the vector from body center to the anchor point in world-space
	m_r1World = q1 * m_localAnchorPointBody1;
	m_r2World = q2 * m_localAnchorPointBody2;

	// Compute the corresponding skew-symmetric matrices
	etk::Matrix3x3 skewSymmetricMatrixU1= etk::Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(m_r1World);
	etk::Matrix3x3 skewSymmetricMatrixU2= etk::Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(m_r2World);

	// --------------- Translation Constraints --------------- //

	// Compute the matrix K=JM^-1J^t (3x3 matrix) for the 3 translation constraints
	float inverseMassBodies = m_body1->m_massInverse + m_body2->m_massInverse;
	etk::Matrix3x3 massMatrix = etk::Matrix3x3(inverseMassBodies, 0, 0,
	                                           0, inverseMassBodies, 0,
	                                           0, 0, inverseMassBodies)
	                            + skewSymmetricMatrixU1 * m_i1 * skewSymmetricMatrixU1.getTranspose()
	                            + skewSymmetricMatrixU2 * m_i2 * skewSymmetricMatrixU2.getTranspose();
	m_inverseMassMatrixTranslation.setZero();
	if (m_body1->getType() == DYNAMIC || m_body2->getType() == DYNAMIC) {
		m_inverseMassMatrixTranslation = massMatrix.getInverse();
	}

	// Compute position error for the 3 translation constraints
	const vec3 errorTranslation = x2 + m_r2World - x1 - m_r1World;

	// Compute the Lagrange multiplier lambda
	const vec3 lambdaTranslation = m_inverseMassMatrixTranslation * (-errorTranslation);

	// Compute the impulse of body 1
	vec3 linearImpulseBody1 = -lambdaTranslation;
	vec3 angularImpulseBody1 = lambdaTranslation.cross(m_r1World);

	// Compute the pseudo velocity of body 1
	const vec3 v1 = inverseMassBody1 * linearImpulseBody1;
	vec3 w1 = m_i1 * angularImpulseBody1;

	// Update the body position/orientation of body 1
	x1 += v1;
	q1 += etk::Quaternion(0, w1) * q1 * 0.5f;
	q1.normalize();

	// Compute the impulse of body 2
	vec3 angularImpulseBody2 = -lambdaTranslation.cross(m_r2World);

	// Compute the pseudo velocity of body 2
	const vec3 v2 = inverseMassBody2 * lambdaTranslation;
	vec3 w2 = m_i2 * angularImpulseBody2;

	// Update the body position/orientation of body 2
	x2 += v2;
	q2 += etk::Quaternion(0, w2) * q2 * 0.5f;
	q2.normalize();

	// --------------- Rotation Constraints --------------- //

	// Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
	// contraints (3x3 matrix)
	m_inverseMassMatrixRotation = m_i1 + m_i2;
	if (m_body1->getType() == DYNAMIC || m_body2->getType() == DYNAMIC) {
		m_inverseMassMatrixRotation = m_inverseMassMatrixRotation.getInverse();
	}

	// Compute the position error for the 3 rotation constraints
	etk::Quaternion currentOrientationDifference = q2 * q1.getInverse();
	currentOrientationDifference.normalize();
	const etk::Quaternion qError = currentOrientationDifference * m_initOrientationDifferenceInv;
	const vec3 errorRotation = float(2.0) * qError.getVectorV();

	// Compute the Lagrange multiplier lambda for the 3 rotation constraints
	vec3 lambdaRotation = m_inverseMassMatrixRotation * (-errorRotation);

	// Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
	angularImpulseBody1 = -lambdaRotation;

	// Compute the pseudo velocity of body 1
	w1 = m_i1 * angularImpulseBody1;

	// Update the body position/orientation of body 1
	q1 += etk::Quaternion(0, w1) * q1 * 0.5f;
	q1.normalize();

	// Compute the pseudo velocity of body 2
	w2 = m_i2 * lambdaRotation;

	// Update the body position/orientation of body 2
	q2 += etk::Quaternion(0, w2) * q2 * 0.5f;
	q2.normalize();
}

