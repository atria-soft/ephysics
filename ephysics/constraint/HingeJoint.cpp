/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/constraint/HingeJoint.h>
#include <ephysics/engine/ConstraintSolver.h>
#include <cmath>

using namespace reactphysics3d;

// Static variables definition
const float HingeJoint::BETA = float(0.2);

// Constructor
HingeJoint::HingeJoint(const HingeJointInfo& jointInfo)
		   : Joint(jointInfo), m_impulseTranslation(0, 0, 0), m_impulseRotation(0, 0),
			 m_impulseLowerLimit(0), m_impulseUpperLimit(0), m_impulseMotor(0),
			 mIsLimitEnabled(jointInfo.isLimitEnabled), mIsMotorEnabled(jointInfo.isMotorEnabled),
			 mLowerLimit(jointInfo.minAngleLimit), mUpperLimit(jointInfo.maxAngleLimit),
			 mIsLowerLimitViolated(false), mIsUpperLimitViolated(false),
			 mMotorSpeed(jointInfo.motorSpeed), mMaxMotorTorque(jointInfo.maxMotorTorque) {

	assert(mLowerLimit <= 0 && mLowerLimit >= -2.0 * PI);
	assert(mUpperLimit >= 0 && mUpperLimit <= 2.0 * PI);

	// Compute the local-space anchor point for each body
	Transform transform1 = m_body1->getTransform();
	Transform transform2 = m_body2->getTransform();
	m_localAnchorPointBody1 = transform1.getInverse() * jointInfo.m_anchorPointWorldSpace;
	m_localAnchorPointBody2 = transform2.getInverse() * jointInfo.m_anchorPointWorldSpace;

	// Compute the local-space hinge axis
	mHingeLocalAxisBody1 = transform1.getOrientation().getInverse() * jointInfo.rotationAxisWorld;
	mHingeLocalAxisBody2 = transform2.getOrientation().getInverse() * jointInfo.rotationAxisWorld;
	mHingeLocalAxisBody1.normalize();
	mHingeLocalAxisBody2.normalize();

	// Compute the inverse of the initial orientation difference between the two bodies
	m_initOrientationDifferenceInv = transform2.getOrientation() *
									transform1.getOrientation().getInverse();
	m_initOrientationDifferenceInv.normalize();
	m_initOrientationDifferenceInv.inverse();
}

// Destructor
HingeJoint::~HingeJoint() {

}

// Initialize before solving the constraint
void HingeJoint::initBeforeSolve(const ConstraintSolverData& constraintSolverData) {

	// Initialize the bodies index in the velocity array
	m_indexBody1 = constraintSolverData.mapBodyToConstrainedVelocityIndex.find(m_body1)->second;
	m_indexBody2 = constraintSolverData.mapBodyToConstrainedVelocityIndex.find(m_body2)->second;

	// Get the bodies positions and orientations
	const Vector3& x1 = m_body1->m_centerOfMassWorld;
	const Vector3& x2 = m_body2->m_centerOfMassWorld;
	const Quaternion& orientationBody1 = m_body1->getTransform().getOrientation();
	const Quaternion& orientationBody2 = m_body2->getTransform().getOrientation();

	// Get the inertia tensor of bodies
	m_i1 = m_body1->getInertiaTensorInverseWorld();
	m_i2 = m_body2->getInertiaTensorInverseWorld();

	// Compute the vector from body center to the anchor point in world-space
	m_r1World = orientationBody1 * m_localAnchorPointBody1;
	m_r2World = orientationBody2 * m_localAnchorPointBody2;

	// Compute the current angle around the hinge axis
	float hingeAngle = computeCurrentHingeAngle(orientationBody1, orientationBody2);

	// Check if the limit constraints are violated or not
	float lowerLimitError = hingeAngle - mLowerLimit;
	float upperLimitError = mUpperLimit - hingeAngle;
	bool oldIsLowerLimitViolated = mIsLowerLimitViolated;
	mIsLowerLimitViolated = lowerLimitError <= 0;
	if (mIsLowerLimitViolated != oldIsLowerLimitViolated) {
		m_impulseLowerLimit = 0.0;
	}
	bool oldIsUpperLimitViolated = mIsUpperLimitViolated;
	mIsUpperLimitViolated = upperLimitError <= 0;
	if (mIsUpperLimitViolated != oldIsUpperLimitViolated) {
		m_impulseUpperLimit = 0.0;
	}

	// Compute vectors needed in the Jacobian
	mA1 = orientationBody1 * mHingeLocalAxisBody1;
	Vector3 a2 = orientationBody2 * mHingeLocalAxisBody2;
	mA1.normalize();
	a2.normalize();
	const Vector3 b2 = a2.getOneUnitOrthogonalVector();
	const Vector3 c2 = a2.cross(b2);
	mB2CrossA1 = b2.cross(mA1);
	mC2CrossA1 = c2.cross(mA1);

	// Compute the corresponding skew-symmetric matrices
	Matrix3x3 skewSymmetricMatrixU1= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(m_r1World);
	Matrix3x3 skewSymmetricMatrixU2= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(m_r2World);

	// Compute the inverse mass matrix K=JM^-1J^t for the 3 translation constraints (3x3 matrix)
	float inverseMassBodies = m_body1->m_massInverse + m_body2->m_massInverse;
	Matrix3x3 massMatrix = Matrix3x3(inverseMassBodies, 0, 0,
									0, inverseMassBodies, 0,
									0, 0, inverseMassBodies) +
						   skewSymmetricMatrixU1 * m_i1 * skewSymmetricMatrixU1.getTranspose() +
						   skewSymmetricMatrixU2 * m_i2 * skewSymmetricMatrixU2.getTranspose();
	m_inverseMassMatrixTranslation.setToZero();
	if (m_body1->getType() == DYNAMIC || m_body2->getType() == DYNAMIC) {
		m_inverseMassMatrixTranslation = massMatrix.getInverse();
	}

	// Compute the bias "b" of the translation constraints
	mBTranslation.setToZero();
	float biasFactor = (BETA / constraintSolverData.timeStep);
	if (m_positionCorrectionTechnique == BAUMGARTE_JOINTS) {
		mBTranslation = biasFactor * (x2 + m_r2World - x1 - m_r1World);
	}

	// Compute the inverse mass matrix K=JM^-1J^t for the 2 rotation constraints (2x2 matrix)
	Vector3 I1B2CrossA1 = m_i1 * mB2CrossA1;
	Vector3 I1C2CrossA1 = m_i1 * mC2CrossA1;
	Vector3 I2B2CrossA1 = m_i2 * mB2CrossA1;
	Vector3 I2C2CrossA1 = m_i2 * mC2CrossA1;
	const float el11 = mB2CrossA1.dot(I1B2CrossA1) +
						 mB2CrossA1.dot(I2B2CrossA1);
	const float el12 = mB2CrossA1.dot(I1C2CrossA1) +
						 mB2CrossA1.dot(I2C2CrossA1);
	const float el21 = mC2CrossA1.dot(I1B2CrossA1) +
						 mC2CrossA1.dot(I2B2CrossA1);
	const float el22 = mC2CrossA1.dot(I1C2CrossA1) +
						 mC2CrossA1.dot(I2C2CrossA1);
	const Matrix2x2 matrixKRotation(el11, el12, el21, el22);
	m_inverseMassMatrixRotation.setToZero();
	if (m_body1->getType() == DYNAMIC || m_body2->getType() == DYNAMIC) {
		m_inverseMassMatrixRotation = matrixKRotation.getInverse();
	}

	// Compute the bias "b" of the rotation constraints
	mBRotation.setToZero();
	if (m_positionCorrectionTechnique == BAUMGARTE_JOINTS) {
		mBRotation = biasFactor * Vector2(mA1.dot(b2), mA1.dot(c2));
	}

	// If warm-starting is not enabled
	if (!constraintSolverData.isWarmStartingActive) {

		// Reset all the accumulated impulses
		m_impulseTranslation.setToZero();
		m_impulseRotation.setToZero();
		m_impulseLowerLimit = 0.0;
		m_impulseUpperLimit = 0.0;
		m_impulseMotor = 0.0;
	}

	// If the motor or limits are enabled
	if (mIsMotorEnabled || (mIsLimitEnabled && (mIsLowerLimitViolated || mIsUpperLimitViolated))) {

		// Compute the inverse of the mass matrix K=JM^-1J^t for the limits and motor (1x1 matrix)
		m_inverseMassMatrixLimitMotor = mA1.dot(m_i1 * mA1) + mA1.dot(m_i2 * mA1);
		m_inverseMassMatrixLimitMotor = (m_inverseMassMatrixLimitMotor > 0.0) ?
								  float(1.0) / m_inverseMassMatrixLimitMotor : float(0.0);

		if (mIsLimitEnabled) {

			// Compute the bias "b" of the lower limit constraint
			mBLowerLimit = 0.0;
			if (m_positionCorrectionTechnique == BAUMGARTE_JOINTS) {
				mBLowerLimit = biasFactor * lowerLimitError;
			}

			// Compute the bias "b" of the upper limit constraint
			mBUpperLimit = 0.0;
			if (m_positionCorrectionTechnique == BAUMGARTE_JOINTS) {
				mBUpperLimit = biasFactor * upperLimitError;
			}
		}
	}
}

// Warm start the constraint (apply the previous impulse at the beginning of the step)
void HingeJoint::warmstart(const ConstraintSolverData& constraintSolverData) {

	// Get the velocities
	Vector3& v1 = constraintSolverData.linearVelocities[m_indexBody1];
	Vector3& v2 = constraintSolverData.linearVelocities[m_indexBody2];
	Vector3& w1 = constraintSolverData.angularVelocities[m_indexBody1];
	Vector3& w2 = constraintSolverData.angularVelocities[m_indexBody2];

	// Get the inverse mass and inverse inertia tensors of the bodies
	const float inverseMassBody1 = m_body1->m_massInverse;
	const float inverseMassBody2 = m_body2->m_massInverse;

	// Compute the impulse P=J^T * lambda for the 2 rotation constraints
	Vector3 rotationImpulse = -mB2CrossA1 * m_impulseRotation.x - mC2CrossA1 * m_impulseRotation.y;

	// Compute the impulse P=J^T * lambda for the lower and upper limits constraints
	const Vector3 limitsImpulse = (m_impulseUpperLimit - m_impulseLowerLimit) * mA1;

	// Compute the impulse P=J^T * lambda for the motor constraint
	const Vector3 motorImpulse = -m_impulseMotor * mA1;

	// Compute the impulse P=J^T * lambda for the 3 translation constraints of body 1
	Vector3 linearImpulseBody1 = -m_impulseTranslation;
	Vector3 angularImpulseBody1 = m_impulseTranslation.cross(m_r1World);

	// Compute the impulse P=J^T * lambda for the 2 rotation constraints of body 1
	angularImpulseBody1 += rotationImpulse;

	// Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 1
	angularImpulseBody1 += limitsImpulse;

	// Compute the impulse P=J^T * lambda for the motor constraint of body 1
	angularImpulseBody1 += motorImpulse;

	// Apply the impulse to the body 1
	v1 += inverseMassBody1 * linearImpulseBody1;
	w1 += m_i1 * angularImpulseBody1;

	// Compute the impulse P=J^T * lambda for the 3 translation constraints of body 2
	Vector3 angularImpulseBody2 = -m_impulseTranslation.cross(m_r2World);

	// Compute the impulse P=J^T * lambda for the 2 rotation constraints of body 2
	angularImpulseBody2 += -rotationImpulse;

	// Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 2
	angularImpulseBody2 += -limitsImpulse;

	// Compute the impulse P=J^T * lambda for the motor constraint of body 2
	angularImpulseBody2 += -motorImpulse;

	// Apply the impulse to the body 2
	v2 += inverseMassBody2 * m_impulseTranslation;
	w2 += m_i2 * angularImpulseBody2;
}

// Solve the velocity constraint
void HingeJoint::solveVelocityConstraint(const ConstraintSolverData& constraintSolverData) {

	// Get the velocities
	Vector3& v1 = constraintSolverData.linearVelocities[m_indexBody1];
	Vector3& v2 = constraintSolverData.linearVelocities[m_indexBody2];
	Vector3& w1 = constraintSolverData.angularVelocities[m_indexBody1];
	Vector3& w2 = constraintSolverData.angularVelocities[m_indexBody2];

	// Get the inverse mass and inverse inertia tensors of the bodies
	float inverseMassBody1 = m_body1->m_massInverse;
	float inverseMassBody2 = m_body2->m_massInverse;

	// --------------- Translation Constraints --------------- //

	// Compute J*v
	const Vector3 JvTranslation = v2 + w2.cross(m_r2World) - v1 - w1.cross(m_r1World);

	// Compute the Lagrange multiplier lambda
	const Vector3 deltaLambdaTranslation = m_inverseMassMatrixTranslation *
										  (-JvTranslation - mBTranslation);
	m_impulseTranslation += deltaLambdaTranslation;

	// Compute the impulse P=J^T * lambda of body 1
	const Vector3 linearImpulseBody1 = -deltaLambdaTranslation;
	Vector3 angularImpulseBody1 = deltaLambdaTranslation.cross(m_r1World);

	// Apply the impulse to the body 1
	v1 += inverseMassBody1 * linearImpulseBody1;
	w1 += m_i1 * angularImpulseBody1;

	// Compute the impulse P=J^T * lambda of body 2
	Vector3 angularImpulseBody2 = -deltaLambdaTranslation.cross(m_r2World);

	// Apply the impulse to the body 2
	v2 += inverseMassBody2 * deltaLambdaTranslation;
	w2 += m_i2 * angularImpulseBody2;

	// --------------- Rotation Constraints --------------- //

	// Compute J*v for the 2 rotation constraints
	const Vector2 JvRotation(-mB2CrossA1.dot(w1) + mB2CrossA1.dot(w2),
							 -mC2CrossA1.dot(w1) + mC2CrossA1.dot(w2));

	// Compute the Lagrange multiplier lambda for the 2 rotation constraints
	Vector2 deltaLambdaRotation = m_inverseMassMatrixRotation * (-JvRotation - mBRotation);
	m_impulseRotation += deltaLambdaRotation;

	// Compute the impulse P=J^T * lambda for the 2 rotation constraints of body 1
	angularImpulseBody1 = -mB2CrossA1 * deltaLambdaRotation.x -
										mC2CrossA1 * deltaLambdaRotation.y;

	// Apply the impulse to the body 1
	w1 += m_i1 * angularImpulseBody1;

	// Compute the impulse P=J^T * lambda for the 2 rotation constraints of body 2
	angularImpulseBody2 = mB2CrossA1 * deltaLambdaRotation.x +
			mC2CrossA1 * deltaLambdaRotation.y;

	// Apply the impulse to the body 2
	w2 += m_i2 * angularImpulseBody2;

	// --------------- Limits Constraints --------------- //

	if (mIsLimitEnabled) {

		// If the lower limit is violated
		if (mIsLowerLimitViolated) {

			// Compute J*v for the lower limit constraint
			const float JvLowerLimit = (w2 - w1).dot(mA1);

			// Compute the Lagrange multiplier lambda for the lower limit constraint
			float deltaLambdaLower = m_inverseMassMatrixLimitMotor * (-JvLowerLimit -mBLowerLimit);
			float lambdaTemp = m_impulseLowerLimit;
			m_impulseLowerLimit = std::max(m_impulseLowerLimit + deltaLambdaLower, float(0.0));
			deltaLambdaLower = m_impulseLowerLimit - lambdaTemp;

			// Compute the impulse P=J^T * lambda for the lower limit constraint of body 1
			const Vector3 angularImpulseBody1 = -deltaLambdaLower * mA1;

			// Apply the impulse to the body 1
			w1 += m_i1 * angularImpulseBody1;

			// Compute the impulse P=J^T * lambda for the lower limit constraint of body 2
			const Vector3 angularImpulseBody2 = deltaLambdaLower * mA1;

			// Apply the impulse to the body 2
			w2 += m_i2 * angularImpulseBody2;
		}

		// If the upper limit is violated
		if (mIsUpperLimitViolated) {

			// Compute J*v for the upper limit constraint
			const float JvUpperLimit = -(w2 - w1).dot(mA1);

			// Compute the Lagrange multiplier lambda for the upper limit constraint
			float deltaLambdaUpper = m_inverseMassMatrixLimitMotor * (-JvUpperLimit -mBUpperLimit);
			float lambdaTemp = m_impulseUpperLimit;
			m_impulseUpperLimit = std::max(m_impulseUpperLimit + deltaLambdaUpper, float(0.0));
			deltaLambdaUpper = m_impulseUpperLimit - lambdaTemp;

			// Compute the impulse P=J^T * lambda for the upper limit constraint of body 1
			const Vector3 angularImpulseBody1 = deltaLambdaUpper * mA1;

			// Apply the impulse to the body 1
			w1 += m_i1 * angularImpulseBody1;

			// Compute the impulse P=J^T * lambda for the upper limit constraint of body 2
			const Vector3 angularImpulseBody2 = -deltaLambdaUpper * mA1;

			// Apply the impulse to the body 2
			w2 += m_i2 * angularImpulseBody2;
		}
	}

	// --------------- Motor --------------- //

	// If the motor is enabled
	if (mIsMotorEnabled) {

		// Compute J*v for the motor
		const float JvMotor = mA1.dot(w1 - w2);

		// Compute the Lagrange multiplier lambda for the motor
		const float maxMotorImpulse = mMaxMotorTorque * constraintSolverData.timeStep;
		float deltaLambdaMotor = m_inverseMassMatrixLimitMotor * (-JvMotor - mMotorSpeed);
		float lambdaTemp = m_impulseMotor;
		m_impulseMotor = clamp(m_impulseMotor + deltaLambdaMotor, -maxMotorImpulse, maxMotorImpulse);
		deltaLambdaMotor = m_impulseMotor - lambdaTemp;

		// Compute the impulse P=J^T * lambda for the motor of body 1
		const Vector3 angularImpulseBody1 = -deltaLambdaMotor * mA1;

		// Apply the impulse to the body 1
		w1 += m_i1 * angularImpulseBody1;

		// Compute the impulse P=J^T * lambda for the motor of body 2
		const Vector3 angularImpulseBody2 = deltaLambdaMotor * mA1;

		// Apply the impulse to the body 2
		w2 += m_i2 * angularImpulseBody2;
	}
}

// Solve the position constraint (for position error correction)
void HingeJoint::solvePositionConstraint(const ConstraintSolverData& constraintSolverData) {

	// If the error position correction technique is not the non-linear-gauss-seidel, we do
	// do not execute this method
	if (m_positionCorrectionTechnique != NON_LINEAR_GAUSS_SEIDEL) return;

	// Get the bodies positions and orientations
	Vector3& x1 = constraintSolverData.positions[m_indexBody1];
	Vector3& x2 = constraintSolverData.positions[m_indexBody2];
	Quaternion& q1 = constraintSolverData.orientations[m_indexBody1];
	Quaternion& q2 = constraintSolverData.orientations[m_indexBody2];

	// Get the inverse mass and inverse inertia tensors of the bodies
	float inverseMassBody1 = m_body1->m_massInverse;
	float inverseMassBody2 = m_body2->m_massInverse;

	// Recompute the inverse inertia tensors
	m_i1 = m_body1->getInertiaTensorInverseWorld();
	m_i2 = m_body2->getInertiaTensorInverseWorld();

	// Compute the vector from body center to the anchor point in world-space
	m_r1World = q1 * m_localAnchorPointBody1;
	m_r2World = q2 * m_localAnchorPointBody2;

	// Compute the current angle around the hinge axis
	float hingeAngle = computeCurrentHingeAngle(q1, q2);

	// Check if the limit constraints are violated or not
	float lowerLimitError = hingeAngle - mLowerLimit;
	float upperLimitError = mUpperLimit - hingeAngle;
	mIsLowerLimitViolated = lowerLimitError <= 0;
	mIsUpperLimitViolated = upperLimitError <= 0;

	// Compute vectors needed in the Jacobian
	mA1 = q1 * mHingeLocalAxisBody1;
	Vector3 a2 = q2 * mHingeLocalAxisBody2;
	mA1.normalize();
	a2.normalize();
	const Vector3 b2 = a2.getOneUnitOrthogonalVector();
	const Vector3 c2 = a2.cross(b2);
	mB2CrossA1 = b2.cross(mA1);
	mC2CrossA1 = c2.cross(mA1);

	// Compute the corresponding skew-symmetric matrices
	Matrix3x3 skewSymmetricMatrixU1= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(m_r1World);
	Matrix3x3 skewSymmetricMatrixU2= Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(m_r2World);

	// --------------- Translation Constraints --------------- //

	// Compute the matrix K=JM^-1J^t (3x3 matrix) for the 3 translation constraints
	float inverseMassBodies = m_body1->m_massInverse + m_body2->m_massInverse;
	Matrix3x3 massMatrix = Matrix3x3(inverseMassBodies, 0, 0,
									0, inverseMassBodies, 0,
									0, 0, inverseMassBodies) +
						   skewSymmetricMatrixU1 * m_i1 * skewSymmetricMatrixU1.getTranspose() +
						   skewSymmetricMatrixU2 * m_i2 * skewSymmetricMatrixU2.getTranspose();
	m_inverseMassMatrixTranslation.setToZero();
	if (m_body1->getType() == DYNAMIC || m_body2->getType() == DYNAMIC) {
		m_inverseMassMatrixTranslation = massMatrix.getInverse();
	}

	// Compute position error for the 3 translation constraints
	const Vector3 errorTranslation = x2 + m_r2World - x1 - m_r1World;

	// Compute the Lagrange multiplier lambda
	const Vector3 lambdaTranslation = m_inverseMassMatrixTranslation * (-errorTranslation);

	// Compute the impulse of body 1
	Vector3 linearImpulseBody1 = -lambdaTranslation;
	Vector3 angularImpulseBody1 = lambdaTranslation.cross(m_r1World);

	// Compute the pseudo velocity of body 1
	const Vector3 v1 = inverseMassBody1 * linearImpulseBody1;
	Vector3 w1 = m_i1 * angularImpulseBody1;

	// Update the body position/orientation of body 1
	x1 += v1;
	q1 += Quaternion(0, w1) * q1 * float(0.5);
	q1.normalize();

	// Compute the impulse of body 2
	Vector3 angularImpulseBody2 = -lambdaTranslation.cross(m_r2World);

	// Compute the pseudo velocity of body 2
	const Vector3 v2 = inverseMassBody2 * lambdaTranslation;
	Vector3 w2 = m_i2 * angularImpulseBody2;

	// Update the body position/orientation of body 2
	x2 += v2;
	q2 += Quaternion(0, w2) * q2 * float(0.5);
	q2.normalize();

	// --------------- Rotation Constraints --------------- //

	// Compute the inverse mass matrix K=JM^-1J^t for the 2 rotation constraints (2x2 matrix)
	Vector3 I1B2CrossA1 = m_i1 * mB2CrossA1;
	Vector3 I1C2CrossA1 = m_i1 * mC2CrossA1;
	Vector3 I2B2CrossA1 = m_i2 * mB2CrossA1;
	Vector3 I2C2CrossA1 = m_i2 * mC2CrossA1;
	const float el11 = mB2CrossA1.dot(I1B2CrossA1) +
						 mB2CrossA1.dot(I2B2CrossA1);
	const float el12 = mB2CrossA1.dot(I1C2CrossA1) +
						 mB2CrossA1.dot(I2C2CrossA1);
	const float el21 = mC2CrossA1.dot(I1B2CrossA1) +
						 mC2CrossA1.dot(I2B2CrossA1);
	const float el22 = mC2CrossA1.dot(I1C2CrossA1) +
						 mC2CrossA1.dot(I2C2CrossA1);
	const Matrix2x2 matrixKRotation(el11, el12, el21, el22);
	m_inverseMassMatrixRotation.setToZero();
	if (m_body1->getType() == DYNAMIC || m_body2->getType() == DYNAMIC) {
		m_inverseMassMatrixRotation = matrixKRotation.getInverse();
	}

	// Compute the position error for the 3 rotation constraints
	const Vector2 errorRotation = Vector2(mA1.dot(b2), mA1.dot(c2));

	// Compute the Lagrange multiplier lambda for the 3 rotation constraints
	Vector2 lambdaRotation = m_inverseMassMatrixRotation * (-errorRotation);

	// Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
	angularImpulseBody1 = -mB2CrossA1 * lambdaRotation.x - mC2CrossA1 * lambdaRotation.y;

	// Compute the pseudo velocity of body 1
	w1 = m_i1 * angularImpulseBody1;

	// Update the body position/orientation of body 1
	q1 += Quaternion(0, w1) * q1 * float(0.5);
	q1.normalize();

	// Compute the impulse of body 2
	angularImpulseBody2 = mB2CrossA1 * lambdaRotation.x + mC2CrossA1 * lambdaRotation.y;

	// Compute the pseudo velocity of body 2
	w2 = m_i2 * angularImpulseBody2;

	// Update the body position/orientation of body 2
	q2 += Quaternion(0, w2) * q2 * float(0.5);
	q2.normalize();

	// --------------- Limits Constraints --------------- //

	if (mIsLimitEnabled) {

		if (mIsLowerLimitViolated || mIsUpperLimitViolated) {

			// Compute the inverse of the mass matrix K=JM^-1J^t for the limits (1x1 matrix)
			m_inverseMassMatrixLimitMotor = mA1.dot(m_i1 * mA1) + mA1.dot(m_i2 * mA1);
			m_inverseMassMatrixLimitMotor = (m_inverseMassMatrixLimitMotor > 0.0) ?
									  float(1.0) / m_inverseMassMatrixLimitMotor : float(0.0);
		}

		// If the lower limit is violated
		if (mIsLowerLimitViolated) {

			// Compute the Lagrange multiplier lambda for the lower limit constraint
			float lambdaLowerLimit = m_inverseMassMatrixLimitMotor * (-lowerLimitError );

			// Compute the impulse P=J^T * lambda of body 1
			const Vector3 angularImpulseBody1 = -lambdaLowerLimit * mA1;

			// Compute the pseudo velocity of body 1
			const Vector3 w1 = m_i1 * angularImpulseBody1;

			// Update the body position/orientation of body 1
			q1 += Quaternion(0, w1) * q1 * float(0.5);
			q1.normalize();

			// Compute the impulse P=J^T * lambda of body 2
			const Vector3 angularImpulseBody2 = lambdaLowerLimit * mA1;

			// Compute the pseudo velocity of body 2
			const Vector3 w2 = m_i2 * angularImpulseBody2;

			// Update the body position/orientation of body 2
			q2 += Quaternion(0, w2) * q2 * float(0.5);
			q2.normalize();
		}

		// If the upper limit is violated
		if (mIsUpperLimitViolated) {

			// Compute the Lagrange multiplier lambda for the upper limit constraint
			float lambdaUpperLimit = m_inverseMassMatrixLimitMotor * (-upperLimitError);

			// Compute the impulse P=J^T * lambda of body 1
			const Vector3 angularImpulseBody1 = lambdaUpperLimit * mA1;

			// Compute the pseudo velocity of body 1
			const Vector3 w1 = m_i1 * angularImpulseBody1;

			// Update the body position/orientation of body 1
			q1 += Quaternion(0, w1) * q1 * float(0.5);
			q1.normalize();

			// Compute the impulse P=J^T * lambda of body 2
			const Vector3 angularImpulseBody2 = -lambdaUpperLimit * mA1;

			// Compute the pseudo velocity of body 2
			const Vector3 w2 = m_i2 * angularImpulseBody2;

			// Update the body position/orientation of body 2
			q2 += Quaternion(0, w2) * q2 * float(0.5);
			q2.normalize();
		}
	}
}


// Enable/Disable the limits of the joint
/**
 * @param isLimitEnabled True if you want to enable the limits of the joint and
 *					   false otherwise
 */
void HingeJoint::enableLimit(bool isLimitEnabled) {

	if (isLimitEnabled != mIsLimitEnabled) {

		mIsLimitEnabled = isLimitEnabled;

		// Reset the limits
		resetLimits();
	}
}

// Enable/Disable the motor of the joint
/**
 * @param isMotorEnabled True if you want to enable the motor of the joint and
 *					   false otherwise
 */
void HingeJoint::enableMotor(bool isMotorEnabled) {

	mIsMotorEnabled = isMotorEnabled;
	m_impulseMotor = 0.0;

	// Wake up the two bodies of the joint
	m_body1->setIsSleeping(false);
	m_body2->setIsSleeping(false);
}

// Set the minimum angle limit
/**
 * @param lowerLimit The minimum limit angle of the joint (in radian)
 */
void HingeJoint::setMinAngleLimit(float lowerLimit) {

	assert(mLowerLimit <= 0 && mLowerLimit >= -2.0 * PI);

	if (lowerLimit != mLowerLimit) {

		mLowerLimit = lowerLimit;

		// Reset the limits
		resetLimits();
	}
}

// Set the maximum angle limit
/**
 * @param upperLimit The maximum limit angle of the joint (in radian)
 */
void HingeJoint::setMaxAngleLimit(float upperLimit) {

	assert(upperLimit >= 0 && upperLimit <= 2.0 * PI);

	if (upperLimit != mUpperLimit) {

		mUpperLimit = upperLimit;

		// Reset the limits
		resetLimits();
	}
}

// Reset the limits
void HingeJoint::resetLimits() {

	// Reset the accumulated impulses for the limits
	m_impulseLowerLimit = 0.0;
	m_impulseUpperLimit = 0.0;

	// Wake up the two bodies of the joint
	m_body1->setIsSleeping(false);
	m_body2->setIsSleeping(false);
}

// Set the motor speed
void HingeJoint::setMotorSpeed(float motorSpeed) {

	if (motorSpeed != mMotorSpeed) {

		mMotorSpeed = motorSpeed;

		// Wake up the two bodies of the joint
		m_body1->setIsSleeping(false);
		m_body2->setIsSleeping(false);
	}
}

// Set the maximum motor torque
/**
 * @param maxMotorTorque The maximum torque (in Newtons) of the joint motor
 */
void HingeJoint::setMaxMotorTorque(float maxMotorTorque) {

	if (maxMotorTorque != mMaxMotorTorque) {

		assert(mMaxMotorTorque >= 0.0);
		mMaxMotorTorque = maxMotorTorque;

		// Wake up the two bodies of the joint
		m_body1->setIsSleeping(false);
		m_body2->setIsSleeping(false);
	}
}

// Given an angle in radian, this method returns the corresponding angle in the range [-pi; pi]
float HingeJoint::computeNormalizedAngle(float angle) const {

	// Convert it int32_to the range [-2*pi; 2*pi]
	angle = fmod(angle, PI_TIMES_2);

	// Convert it int32_to the range [-pi; pi]
	if (angle < -PI) {
		return angle + PI_TIMES_2;
	}
	else if (angle > PI) {
		return angle - PI_TIMES_2;
	}
	else {
		return angle;
	}
}

// Given an "inputAngle" in the range [-pi, pi], this method returns an
// angle (modulo 2*pi) in the range [-2*pi; 2*pi] that is closest to one of the
// two angle limits in arguments.
float HingeJoint::computeCorrespondingAngleNearLimits(float inputAngle, float lowerLimitAngle,
														float upperLimitAngle) const {
	if (upperLimitAngle <= lowerLimitAngle) {
		return inputAngle;
	}
	else if (inputAngle > upperLimitAngle) {
		float diffToUpperLimit = fabs(computeNormalizedAngle(inputAngle - upperLimitAngle));
		float diffToLowerLimit = fabs(computeNormalizedAngle(inputAngle - lowerLimitAngle));
		return (diffToUpperLimit > diffToLowerLimit) ? (inputAngle - PI_TIMES_2) : inputAngle;
	}
	else if (inputAngle < lowerLimitAngle) {
		float diffToUpperLimit = fabs(computeNormalizedAngle(upperLimitAngle - inputAngle));
		float diffToLowerLimit = fabs(computeNormalizedAngle(lowerLimitAngle - inputAngle));
		return (diffToUpperLimit > diffToLowerLimit) ? inputAngle : (inputAngle + PI_TIMES_2);
	}
	else {
		return inputAngle;
	}
}

// Compute the current angle around the hinge axis
float HingeJoint::computeCurrentHingeAngle(const Quaternion& orientationBody1,
											 const Quaternion& orientationBody2) {

	float hingeAngle;

	// Compute the current orientation difference between the two bodies
	Quaternion currentOrientationDiff = orientationBody2 * orientationBody1.getInverse();
	currentOrientationDiff.normalize();

	// Compute the relative rotation considering the initial orientation difference
	Quaternion relativeRotation = currentOrientationDiff * m_initOrientationDifferenceInv;
	relativeRotation.normalize();

	// A quaternion q = [cos(theta/2); sin(theta/2) * rotAxis] where rotAxis is a unit
	// length vector. We can extract cos(theta/2) with q.w and we can extract |sin(theta/2)| with :
	// |sin(theta/2)| = q.getVectorV().length() since rotAxis is unit length. Note that any
	// rotation can be represented by a quaternion q and -q. Therefore, if the relative rotation
	// axis is not pointing in the same direction as the hinge axis, we use the rotation -q which
	// has the same |sin(theta/2)| value but the value cos(theta/2) is sign inverted. Some details
	// about this trick is explained in the source code of OpenTissue (http://www.opentissue.org).
	float cosHalfAngle = relativeRotation.w;
	float sinHalfAngleAbs = relativeRotation.getVectorV().length();

	// Compute the dot product of the relative rotation axis and the hinge axis
	float dotProduct = relativeRotation.getVectorV().dot(mA1);

	// If the relative rotation axis and the hinge axis are pointing the same direction
	if (dotProduct >= float(0.0)) {
		hingeAngle = float(2.0) * std::atan2(sinHalfAngleAbs, cosHalfAngle);
	}
	else {
		hingeAngle = float(2.0) * std::atan2(sinHalfAngleAbs, -cosHalfAngle);
	}

	// Convert the angle from range [-2*pi; 2*pi] int32_to the range [-pi; pi]
	hingeAngle = computeNormalizedAngle(hingeAngle);

	// Compute and return the corresponding angle near one the two limits
	return computeCorrespondingAngleNearLimits(hingeAngle, mLowerLimit, mUpperLimit);
}

