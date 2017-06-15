/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/constraint/BallAndSocketJoint.h>
#include <ephysics/engine/ConstraintSolver.h>

using namespace reactphysics3d;

// Static variables definition
const float BallAndSocketJoint::BETA = float(0.2);

// Constructor
BallAndSocketJoint::BallAndSocketJoint(const BallAndSocketJointInfo& jointInfo)
				   : Joint(jointInfo), m_impulse(vec3(0, 0, 0)) {

	// Compute the local-space anchor point for each body
	m_localAnchorPointBody1 = m_body1->getTransform().getInverse() * jointInfo.m_anchorPointWorldSpace;
	m_localAnchorPointBody2 = m_body2->getTransform().getInverse() * jointInfo.m_anchorPointWorldSpace;
}

// Destructor
BallAndSocketJoint::~BallAndSocketJoint() {

}

// Initialize before solving the constraint
void BallAndSocketJoint::initBeforeSolve(const ConstraintSolverData& constraintSolverData) {

	// Initialize the bodies index in the velocity array
	m_indexBody1 = constraintSolverData.mapBodyToConstrainedVelocityIndex.find(m_body1)->second;
	m_indexBody2 = constraintSolverData.mapBodyToConstrainedVelocityIndex.find(m_body2)->second;

	// Get the bodies center of mass and orientations
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

	// Compute the matrix K=JM^-1J^t (3x3 matrix)
	float inverseMassBodies = m_body1->m_massInverse + m_body2->m_massInverse;
	etk::Matrix3x3 massMatrix = etk::Matrix3x3(inverseMassBodies, 0, 0,
									0, inverseMassBodies, 0,
									0, 0, inverseMassBodies) +
						   skewSymmetricMatrixU1 * m_i1 * skewSymmetricMatrixU1.getTranspose() +
						   skewSymmetricMatrixU2 * m_i2 * skewSymmetricMatrixU2.getTranspose();

	// Compute the inverse mass matrix K^-1
	m_inverseMassMatrix.setZero();
	if (m_body1->getType() == DYNAMIC || m_body2->getType() == DYNAMIC) {
		m_inverseMassMatrix = massMatrix.getInverse();
	}

	// Compute the bias "b" of the constraint
	m_biasVector.setZero();
	if (m_positionCorrectionTechnique == BAUMGARTE_JOINTS) {
		float biasFactor = (BETA / constraintSolverData.timeStep);
		m_biasVector = biasFactor * (x2 + m_r2World - x1 - m_r1World);
	}

	// If warm-starting is not enabled
	if (!constraintSolverData.isWarmStartingActive) {

		// Reset the accumulated impulse
		m_impulse.setZero();
	}
}

// Warm start the constraint (apply the previous impulse at the beginning of the step)
void BallAndSocketJoint::warmstart(const ConstraintSolverData& constraintSolverData) {

	// Get the velocities
	vec3& v1 = constraintSolverData.linearVelocities[m_indexBody1];
	vec3& v2 = constraintSolverData.linearVelocities[m_indexBody2];
	vec3& w1 = constraintSolverData.angularVelocities[m_indexBody1];
	vec3& w2 = constraintSolverData.angularVelocities[m_indexBody2];

	// Compute the impulse P=J^T * lambda for the body 1
	const vec3 linearImpulseBody1 = -m_impulse;
	const vec3 angularImpulseBody1 = m_impulse.cross(m_r1World);

	// Apply the impulse to the body 1
	v1 += m_body1->m_massInverse * linearImpulseBody1;
	w1 += m_i1 * angularImpulseBody1;

	// Compute the impulse P=J^T * lambda for the body 2
	const vec3 angularImpulseBody2 = -m_impulse.cross(m_r2World);

	// Apply the impulse to the body to the body 2
	v2 += m_body2->m_massInverse * m_impulse;
	w2 += m_i2 * angularImpulseBody2;
}

// Solve the velocity constraint
void BallAndSocketJoint::solveVelocityConstraint(const ConstraintSolverData& constraintSolverData) {

	// Get the velocities
	vec3& v1 = constraintSolverData.linearVelocities[m_indexBody1];
	vec3& v2 = constraintSolverData.linearVelocities[m_indexBody2];
	vec3& w1 = constraintSolverData.angularVelocities[m_indexBody1];
	vec3& w2 = constraintSolverData.angularVelocities[m_indexBody2];

	// Compute J*v
	const vec3 Jv = v2 + w2.cross(m_r2World) - v1 - w1.cross(m_r1World);

	// Compute the Lagrange multiplier lambda
	const vec3 deltaLambda = m_inverseMassMatrix * (-Jv - m_biasVector);
	m_impulse += deltaLambda;

	// Compute the impulse P=J^T * lambda for the body 1
	const vec3 linearImpulseBody1 = -deltaLambda;
	const vec3 angularImpulseBody1 = deltaLambda.cross(m_r1World);

	// Apply the impulse to the body 1
	v1 += m_body1->m_massInverse * linearImpulseBody1;
	w1 += m_i1 * angularImpulseBody1;

	// Compute the impulse P=J^T * lambda for the body 2
	const vec3 angularImpulseBody2 = -deltaLambda.cross(m_r2World);

	// Apply the impulse to the body 2
	v2 += m_body2->m_massInverse * deltaLambda;
	w2 += m_i2 * angularImpulseBody2;
}

// Solve the position constraint (for position error correction)
void BallAndSocketJoint::solvePositionConstraint(const ConstraintSolverData& constraintSolverData) {

	// If the error position correction technique is not the non-linear-gauss-seidel, we do
	// do not execute this method
	if (m_positionCorrectionTechnique != NON_LINEAR_GAUSS_SEIDEL) return;

	// Get the bodies center of mass and orientations
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

	// Recompute the inverse mass matrix K=J^TM^-1J of of the 3 translation constraints
	float inverseMassBodies = inverseMassBody1 + inverseMassBody2;
	etk::Matrix3x3 massMatrix = etk::Matrix3x3(inverseMassBodies, 0, 0,
									0, inverseMassBodies, 0,
									0, 0, inverseMassBodies) +
						   skewSymmetricMatrixU1 * m_i1 * skewSymmetricMatrixU1.getTranspose() +
						   skewSymmetricMatrixU2 * m_i2 * skewSymmetricMatrixU2.getTranspose();
	m_inverseMassMatrix.setZero();
	if (m_body1->getType() == DYNAMIC || m_body2->getType() == DYNAMIC) {
		m_inverseMassMatrix = massMatrix.getInverse();
	}

	// Compute the constraint error (value of the C(x) function)
	const vec3 constraintError = (x2 + m_r2World - x1 - m_r1World);

	// Compute the Lagrange multiplier lambda
	// TODO : Do not solve the system by computing the inverse each time and multiplying with the
	//		right-hand side vector but instead use a method to directly solve the linear system.
	const vec3 lambda = m_inverseMassMatrix * (-constraintError);

	// Compute the impulse of body 1
	const vec3 linearImpulseBody1 = -lambda;
	const vec3 angularImpulseBody1 = lambda.cross(m_r1World);

	// Compute the pseudo velocity of body 1
	const vec3 v1 = inverseMassBody1 * linearImpulseBody1;
	const vec3 w1 = m_i1 * angularImpulseBody1;

	// Update the body center of mass and orientation of body 1
	x1 += v1;
	q1 += etk::Quaternion(0, w1) * q1 * 0.5f;
	q1.normalize();

	// Compute the impulse of body 2
	const vec3 angularImpulseBody2 = -lambda.cross(m_r2World);

	// Compute the pseudo velocity of body 2
	const vec3 v2 = inverseMassBody2 * lambda;
	const vec3 w2 = m_i2 * angularImpulseBody2;

	// Update the body position/orientation of body 2
	x2 += v2;
	q2 += etk::Quaternion(0, w2) * q2 * 0.5f;
	q2.normalize();
}

