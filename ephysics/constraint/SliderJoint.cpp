/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/constraint/SliderJoint.h>

using namespace reactphysics3d;

// Static variables definition
const float SliderJoint::BETA = float(0.2);

// Constructor
SliderJoint::SliderJoint(const SliderJointInfo& jointInfo)
			: Joint(jointInfo), m_impulseTranslation(0, 0), m_impulseRotation(0, 0, 0),
			  m_impulseLowerLimit(0), m_impulseUpperLimit(0), m_impulseMotor(0),
			  mIsLimitEnabled(jointInfo.isLimitEnabled), mIsMotorEnabled(jointInfo.isMotorEnabled),
			  mLowerLimit(jointInfo.minTranslationLimit),
			  mUpperLimit(jointInfo.maxTranslationLimit), mIsLowerLimitViolated(false),
			  mIsUpperLimitViolated(false), mMotorSpeed(jointInfo.motorSpeed),
			  mMaxMotorForce(jointInfo.maxMotorForce){

	assert(mUpperLimit >= 0.0);
	assert(mLowerLimit <= 0.0);
	assert(mMaxMotorForce >= 0.0);

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

	// Compute the slider axis in local-space of body 1
	mSliderAxisBody1 = m_body1->getTransform().getOrientation().getInverse() *
					   jointInfo.sliderAxisWorldSpace;
	mSliderAxisBody1.normalize();
}

// Destructor
SliderJoint::~SliderJoint() {

}

// Initialize before solving the constraint
void SliderJoint::initBeforeSolve(const ConstraintSolverData& constraintSolverData) {

	// Initialize the bodies index in the veloc ity array
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

	// Vector from body center to the anchor point
	mR1 = orientationBody1 * m_localAnchorPointBody1;
	mR2 = orientationBody2 * m_localAnchorPointBody2;

	// Compute the vector u (difference between anchor points)
	const vec3 u = x2 + mR2 - x1 - mR1;

	// Compute the two orthogonal vectors to the slider axis in world-space
	mSliderAxisWorld = orientationBody1 * mSliderAxisBody1;
	mSliderAxisWorld.normalize();
	mN1 = mSliderAxisWorld.getOneUnitOrthogonalVector();
	mN2 = mSliderAxisWorld.cross(mN1);

	// Check if the limit constraints are violated or not
	float uDotSliderAxis = u.dot(mSliderAxisWorld);
	float lowerLimitError = uDotSliderAxis - mLowerLimit;
	float upperLimitError = mUpperLimit - uDotSliderAxis;
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

	// Compute the cross products used in the Jacobians
	mR2CrossN1 = mR2.cross(mN1);
	mR2CrossN2 = mR2.cross(mN2);
	mR2CrossSliderAxis = mR2.cross(mSliderAxisWorld);
	const vec3 r1PlusU = mR1 + u;
	mR1PlusUCrossN1 = (r1PlusU).cross(mN1);
	mR1PlusUCrossN2 = (r1PlusU).cross(mN2);
	mR1PlusUCrossSliderAxis = (r1PlusU).cross(mSliderAxisWorld);

	// Compute the inverse of the mass matrix K=JM^-1J^t for the 2 translation
	// constraints (2x2 matrix)
	float sumInverseMass = m_body1->m_massInverse + m_body2->m_massInverse;
	vec3 I1R1PlusUCrossN1 = m_i1 * mR1PlusUCrossN1;
	vec3 I1R1PlusUCrossN2 = m_i1 * mR1PlusUCrossN2;
	vec3 I2R2CrossN1 = m_i2 * mR2CrossN1;
	vec3 I2R2CrossN2 = m_i2 * mR2CrossN2;
	const float el11 = sumInverseMass + mR1PlusUCrossN1.dot(I1R1PlusUCrossN1) +
						 mR2CrossN1.dot(I2R2CrossN1);
	const float el12 = mR1PlusUCrossN1.dot(I1R1PlusUCrossN2) +
						 mR2CrossN1.dot(I2R2CrossN2);
	const float el21 = mR1PlusUCrossN2.dot(I1R1PlusUCrossN1) +
						 mR2CrossN2.dot(I2R2CrossN1);
	const float el22 = sumInverseMass + mR1PlusUCrossN2.dot(I1R1PlusUCrossN2) +
						 mR2CrossN2.dot(I2R2CrossN2);
	etk::Matrix2x2 matrixKTranslation(el11, el12, el21, el22);
	m_inverseMassMatrixTranslationConstraint.setZero();
	if (m_body1->getType() == DYNAMIC || m_body2->getType() == DYNAMIC) {
		m_inverseMassMatrixTranslationConstraint = matrixKTranslation.getInverse();
	}

	// Compute the bias "b" of the translation constraint
	mBTranslation.setZero();
	float biasFactor = (BETA / constraintSolverData.timeStep);
	if (m_positionCorrectionTechnique == BAUMGARTE_JOINTS) {
		mBTranslation.x() = u.dot(mN1);
		mBTranslation.y() = u.dot(mN2);
		mBTranslation *= biasFactor;
	}

	// Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
	// contraints (3x3 matrix)
	m_inverseMassMatrixRotationConstraint = m_i1 + m_i2;
	if (m_body1->getType() == DYNAMIC || m_body2->getType() == DYNAMIC) {
		m_inverseMassMatrixRotationConstraint = m_inverseMassMatrixRotationConstraint.getInverse();
	}

	// Compute the bias "b" of the rotation constraint
	mBRotation.setZero();
	if (m_positionCorrectionTechnique == BAUMGARTE_JOINTS) {
		etk::Quaternion currentOrientationDifference = orientationBody2 * orientationBody1.getInverse();
		currentOrientationDifference.normalize();
		const etk::Quaternion qError = currentOrientationDifference * m_initOrientationDifferenceInv;
		mBRotation = biasFactor * float(2.0) * qError.getVectorV();
	}

	// If the limits are enabled
	if (mIsLimitEnabled && (mIsLowerLimitViolated || mIsUpperLimitViolated)) {

		// Compute the inverse of the mass matrix K=JM^-1J^t for the limits (1x1 matrix)
		m_inverseMassMatrixLimit = m_body1->m_massInverse + m_body2->m_massInverse +
								  mR1PlusUCrossSliderAxis.dot(m_i1 * mR1PlusUCrossSliderAxis) +
								  mR2CrossSliderAxis.dot(m_i2 * mR2CrossSliderAxis);
		m_inverseMassMatrixLimit = (m_inverseMassMatrixLimit > 0.0) ?
								  1.0f / m_inverseMassMatrixLimit : 0.0f;

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

	// If the motor is enabled
	if (mIsMotorEnabled) {

		// Compute the inverse of mass matrix K=JM^-1J^t for the motor (1x1 matrix)
		m_inverseMassMatrixMotor = m_body1->m_massInverse + m_body2->m_massInverse;
		m_inverseMassMatrixMotor = (m_inverseMassMatrixMotor > 0.0) ?
					1.0f / m_inverseMassMatrixMotor : 0.0f;
	}

	// If warm-starting is not enabled
	if (!constraintSolverData.isWarmStartingActive) {

		// Reset all the accumulated impulses
		m_impulseTranslation.setZero();
		m_impulseRotation.setZero();
		m_impulseLowerLimit = 0.0;
		m_impulseUpperLimit = 0.0;
		m_impulseMotor = 0.0;
	}
}

// Warm start the constraint (apply the previous impulse at the beginning of the step)
void SliderJoint::warmstart(const ConstraintSolverData& constraintSolverData) {

	// Get the velocities
	vec3& v1 = constraintSolverData.linearVelocities[m_indexBody1];
	vec3& v2 = constraintSolverData.linearVelocities[m_indexBody2];
	vec3& w1 = constraintSolverData.angularVelocities[m_indexBody1];
	vec3& w2 = constraintSolverData.angularVelocities[m_indexBody2];

	// Get the inverse mass and inverse inertia tensors of the bodies
	const float inverseMassBody1 = m_body1->m_massInverse;
	const float inverseMassBody2 = m_body2->m_massInverse;

	// Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 1
	float impulseLimits = m_impulseUpperLimit - m_impulseLowerLimit;
	vec3 linearImpulseLimits = impulseLimits * mSliderAxisWorld;

	// Compute the impulse P=J^T * lambda for the motor constraint of body 1
	vec3 impulseMotor = m_impulseMotor * mSliderAxisWorld;

	// Compute the impulse P=J^T * lambda for the 2 translation constraints of body 1
	vec3 linearImpulseBody1 = -mN1 * m_impulseTranslation.x() - mN2 * m_impulseTranslation.y();
	vec3 angularImpulseBody1 = -mR1PlusUCrossN1 * m_impulseTranslation.x() -
			mR1PlusUCrossN2 * m_impulseTranslation.y();

	// Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
	angularImpulseBody1 += -m_impulseRotation;

	// Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 1
	linearImpulseBody1 += linearImpulseLimits;
	angularImpulseBody1 += impulseLimits * mR1PlusUCrossSliderAxis;

	// Compute the impulse P=J^T * lambda for the motor constraint of body 1
	linearImpulseBody1 += impulseMotor;

	// Apply the impulse to the body 1
	v1 += inverseMassBody1 * linearImpulseBody1;
	w1 += m_i1 * angularImpulseBody1;

	// Compute the impulse P=J^T * lambda for the 2 translation constraints of body 2
	vec3 linearImpulseBody2 = mN1 * m_impulseTranslation.x() + mN2 * m_impulseTranslation.y();
	vec3 angularImpulseBody2 = mR2CrossN1 * m_impulseTranslation.x() +
			mR2CrossN2 * m_impulseTranslation.y();

	// Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 2
	angularImpulseBody2 += m_impulseRotation;

	// Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 2
	linearImpulseBody2 += -linearImpulseLimits;
	angularImpulseBody2 += -impulseLimits * mR2CrossSliderAxis;

	// Compute the impulse P=J^T * lambda for the motor constraint of body 2
	linearImpulseBody2 += -impulseMotor;

	// Apply the impulse to the body 2
	v2 += inverseMassBody2 * linearImpulseBody2;
	w2 += m_i2 * angularImpulseBody2;
}

// Solve the velocity constraint
void SliderJoint::solveVelocityConstraint(const ConstraintSolverData& constraintSolverData) {

	// Get the velocities
	vec3& v1 = constraintSolverData.linearVelocities[m_indexBody1];
	vec3& v2 = constraintSolverData.linearVelocities[m_indexBody2];
	vec3& w1 = constraintSolverData.angularVelocities[m_indexBody1];
	vec3& w2 = constraintSolverData.angularVelocities[m_indexBody2];

	// Get the inverse mass and inverse inertia tensors of the bodies
	float inverseMassBody1 = m_body1->m_massInverse;
	float inverseMassBody2 = m_body2->m_massInverse;

	// --------------- Translation Constraints --------------- //

	// Compute J*v for the 2 translation constraints
	const float el1 = -mN1.dot(v1) - w1.dot(mR1PlusUCrossN1) +
						 mN1.dot(v2) + w2.dot(mR2CrossN1);
	const float el2 = -mN2.dot(v1) - w1.dot(mR1PlusUCrossN2) +
						 mN2.dot(v2) + w2.dot(mR2CrossN2);
	const vec2 JvTranslation(el1, el2);

	// Compute the Lagrange multiplier lambda for the 2 translation constraints
	vec2 deltaLambda = m_inverseMassMatrixTranslationConstraint * (-JvTranslation -mBTranslation);
	m_impulseTranslation += deltaLambda;

	// Compute the impulse P=J^T * lambda for the 2 translation constraints of body 1
	const vec3 linearImpulseBody1 = -mN1 * deltaLambda.x() - mN2 * deltaLambda.y();
	vec3 angularImpulseBody1 = -mR1PlusUCrossN1 * deltaLambda.x() -
			mR1PlusUCrossN2 * deltaLambda.y();

	// Apply the impulse to the body 1
	v1 += inverseMassBody1 * linearImpulseBody1;
	w1 += m_i1 * angularImpulseBody1;

	// Compute the impulse P=J^T * lambda for the 2 translation constraints of body 2
	const vec3 linearImpulseBody2 = mN1 * deltaLambda.x() + mN2 * deltaLambda.y();
	vec3 angularImpulseBody2 = mR2CrossN1 * deltaLambda.x() + mR2CrossN2 * deltaLambda.y();

	// Apply the impulse to the body 2
	v2 += inverseMassBody2 * linearImpulseBody2;
	w2 += m_i2 * angularImpulseBody2;

	// --------------- Rotation Constraints --------------- //

	// Compute J*v for the 3 rotation constraints
	const vec3 JvRotation = w2 - w1;

	// Compute the Lagrange multiplier lambda for the 3 rotation constraints
	vec3 deltaLambda2 = m_inverseMassMatrixRotationConstraint * (-JvRotation - mBRotation);
	m_impulseRotation += deltaLambda2;

	// Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
	angularImpulseBody1 = -deltaLambda2;

	// Apply the impulse to the body to body 1
	w1 += m_i1 * angularImpulseBody1;

	// Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 2
	angularImpulseBody2 = deltaLambda2;

	// Apply the impulse to the body 2
	w2 += m_i2 * angularImpulseBody2;

	// --------------- Limits Constraints --------------- //

	if (mIsLimitEnabled) {

		// If the lower limit is violated
		if (mIsLowerLimitViolated) {

			// Compute J*v for the lower limit constraint
			const float JvLowerLimit = mSliderAxisWorld.dot(v2) + mR2CrossSliderAxis.dot(w2) -
										 mSliderAxisWorld.dot(v1) - mR1PlusUCrossSliderAxis.dot(w1);

			// Compute the Lagrange multiplier lambda for the lower limit constraint
			float deltaLambdaLower = m_inverseMassMatrixLimit * (-JvLowerLimit -mBLowerLimit);
			float lambdaTemp = m_impulseLowerLimit;
			m_impulseLowerLimit = std::max(m_impulseLowerLimit + deltaLambdaLower, 0.0f);
			deltaLambdaLower = m_impulseLowerLimit - lambdaTemp;

			// Compute the impulse P=J^T * lambda for the lower limit constraint of body 1
			const vec3 linearImpulseBody1 = -deltaLambdaLower * mSliderAxisWorld;
			const vec3 angularImpulseBody1 = -deltaLambdaLower * mR1PlusUCrossSliderAxis;

			// Apply the impulse to the body 1
			v1 += inverseMassBody1 * linearImpulseBody1;
			w1 += m_i1 * angularImpulseBody1;

			// Compute the impulse P=J^T * lambda for the lower limit constraint of body 2
			const vec3 linearImpulseBody2 = deltaLambdaLower * mSliderAxisWorld;
			const vec3 angularImpulseBody2 = deltaLambdaLower * mR2CrossSliderAxis;

			// Apply the impulse to the body 2
			v2 += inverseMassBody2 * linearImpulseBody2;
			w2 += m_i2 * angularImpulseBody2;
		}

		// If the upper limit is violated
		if (mIsUpperLimitViolated) {

			// Compute J*v for the upper limit constraint
			const float JvUpperLimit = mSliderAxisWorld.dot(v1) + mR1PlusUCrossSliderAxis.dot(w1)
										- mSliderAxisWorld.dot(v2) - mR2CrossSliderAxis.dot(w2);

			// Compute the Lagrange multiplier lambda for the upper limit constraint
			float deltaLambdaUpper = m_inverseMassMatrixLimit * (-JvUpperLimit -mBUpperLimit);
			float lambdaTemp = m_impulseUpperLimit;
			m_impulseUpperLimit = std::max(m_impulseUpperLimit + deltaLambdaUpper, 0.0f);
			deltaLambdaUpper = m_impulseUpperLimit - lambdaTemp;

			// Compute the impulse P=J^T * lambda for the upper limit constraint of body 1
			const vec3 linearImpulseBody1 = deltaLambdaUpper * mSliderAxisWorld;
			const vec3 angularImpulseBody1 = deltaLambdaUpper * mR1PlusUCrossSliderAxis;

			// Apply the impulse to the body 1
			v1 += inverseMassBody1 * linearImpulseBody1;
			w1 += m_i1 * angularImpulseBody1;

			// Compute the impulse P=J^T * lambda for the upper limit constraint of body 2
			const vec3 linearImpulseBody2 = -deltaLambdaUpper * mSliderAxisWorld;
			const vec3 angularImpulseBody2 = -deltaLambdaUpper * mR2CrossSliderAxis;

			// Apply the impulse to the body 2
			v2 += inverseMassBody2 * linearImpulseBody2;
			w2 += m_i2 * angularImpulseBody2;
		}
	}

	// --------------- Motor --------------- //

	if (mIsMotorEnabled) {

		// Compute J*v for the motor
		const float JvMotor = mSliderAxisWorld.dot(v1) - mSliderAxisWorld.dot(v2);

		// Compute the Lagrange multiplier lambda for the motor
		const float maxMotorImpulse = mMaxMotorForce * constraintSolverData.timeStep;
		float deltaLambdaMotor = m_inverseMassMatrixMotor * (-JvMotor - mMotorSpeed);
		float lambdaTemp = m_impulseMotor;
		m_impulseMotor = clamp(m_impulseMotor + deltaLambdaMotor, -maxMotorImpulse, maxMotorImpulse);
		deltaLambdaMotor = m_impulseMotor - lambdaTemp;

		// Compute the impulse P=J^T * lambda for the motor of body 1
		const vec3 linearImpulseBody1 = deltaLambdaMotor * mSliderAxisWorld;

		// Apply the impulse to the body 1
		v1 += inverseMassBody1 * linearImpulseBody1;

		// Compute the impulse P=J^T * lambda for the motor of body 2
		const vec3 linearImpulseBody2 = -deltaLambdaMotor * mSliderAxisWorld;

		// Apply the impulse to the body 2
		v2 += inverseMassBody2 * linearImpulseBody2;
	}
}

// Solve the position constraint (for position error correction)
void SliderJoint::solvePositionConstraint(const ConstraintSolverData& constraintSolverData) {

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

	// Recompute the inertia tensor of bodies
	m_i1 = m_body1->getInertiaTensorInverseWorld();
	m_i2 = m_body2->getInertiaTensorInverseWorld();

	// Vector from body center to the anchor point
	mR1 = q1 * m_localAnchorPointBody1;
	mR2 = q2 * m_localAnchorPointBody2;

	// Compute the vector u (difference between anchor points)
	const vec3 u = x2 + mR2 - x1 - mR1;

	// Compute the two orthogonal vectors to the slider axis in world-space
	mSliderAxisWorld = q1 * mSliderAxisBody1;
	mSliderAxisWorld.normalize();
	mN1 = mSliderAxisWorld.getOneUnitOrthogonalVector();
	mN2 = mSliderAxisWorld.cross(mN1);

	// Check if the limit constraints are violated or not
	float uDotSliderAxis = u.dot(mSliderAxisWorld);
	float lowerLimitError = uDotSliderAxis - mLowerLimit;
	float upperLimitError = mUpperLimit - uDotSliderAxis;
	mIsLowerLimitViolated = lowerLimitError <= 0;
	mIsUpperLimitViolated = upperLimitError <= 0;

	// Compute the cross products used in the Jacobians
	mR2CrossN1 = mR2.cross(mN1);
	mR2CrossN2 = mR2.cross(mN2);
	mR2CrossSliderAxis = mR2.cross(mSliderAxisWorld);
	const vec3 r1PlusU = mR1 + u;
	mR1PlusUCrossN1 = (r1PlusU).cross(mN1);
	mR1PlusUCrossN2 = (r1PlusU).cross(mN2);
	mR1PlusUCrossSliderAxis = (r1PlusU).cross(mSliderAxisWorld);

	// --------------- Translation Constraints --------------- //

	// Recompute the inverse of the mass matrix K=JM^-1J^t for the 2 translation
	// constraints (2x2 matrix)
	float sumInverseMass = m_body1->m_massInverse + m_body2->m_massInverse;
	vec3 I1R1PlusUCrossN1 = m_i1 * mR1PlusUCrossN1;
	vec3 I1R1PlusUCrossN2 = m_i1 * mR1PlusUCrossN2;
	vec3 I2R2CrossN1 = m_i2 * mR2CrossN1;
	vec3 I2R2CrossN2 = m_i2 * mR2CrossN2;
	const float el11 = sumInverseMass + mR1PlusUCrossN1.dot(I1R1PlusUCrossN1) +
						 mR2CrossN1.dot(I2R2CrossN1);
	const float el12 = mR1PlusUCrossN1.dot(I1R1PlusUCrossN2) +
						 mR2CrossN1.dot(I2R2CrossN2);
	const float el21 = mR1PlusUCrossN2.dot(I1R1PlusUCrossN1) +
						 mR2CrossN2.dot(I2R2CrossN1);
	const float el22 = sumInverseMass + mR1PlusUCrossN2.dot(I1R1PlusUCrossN2) +
						 mR2CrossN2.dot(I2R2CrossN2);
	etk::Matrix2x2 matrixKTranslation(el11, el12, el21, el22);
	m_inverseMassMatrixTranslationConstraint.setZero();
	if (m_body1->getType() == DYNAMIC || m_body2->getType() == DYNAMIC) {
		m_inverseMassMatrixTranslationConstraint = matrixKTranslation.getInverse();
	}

	// Compute the position error for the 2 translation constraints
	const vec2 translationError(u.dot(mN1), u.dot(mN2));

	// Compute the Lagrange multiplier lambda for the 2 translation constraints
	vec2 lambdaTranslation = m_inverseMassMatrixTranslationConstraint * (-translationError);

	// Compute the impulse P=J^T * lambda for the 2 translation constraints of body 1
	const vec3 linearImpulseBody1 = -mN1 * lambdaTranslation.x() - mN2 * lambdaTranslation.y();
	vec3 angularImpulseBody1 = -mR1PlusUCrossN1 * lambdaTranslation.x() -
										mR1PlusUCrossN2 * lambdaTranslation.y();

	// Apply the impulse to the body 1
	const vec3 v1 = inverseMassBody1 * linearImpulseBody1;
	vec3 w1 = m_i1 * angularImpulseBody1;

	// Update the body position/orientation of body 1
	x1 += v1;
	q1 += etk::Quaternion(0, w1) * q1 * 0.5f;
	q1.normalize();

	// Compute the impulse P=J^T * lambda for the 2 translation constraints of body 2
	const vec3 linearImpulseBody2 = mN1 * lambdaTranslation.x() + mN2 * lambdaTranslation.y();
	vec3 angularImpulseBody2 = mR2CrossN1 * lambdaTranslation.x() +
			mR2CrossN2 * lambdaTranslation.y();

	// Apply the impulse to the body 2
	const vec3 v2 = inverseMassBody2 * linearImpulseBody2;
	vec3 w2 = m_i2 * angularImpulseBody2;

	// Update the body position/orientation of body 2
	x2 += v2;
	q2 += etk::Quaternion(0, w2) * q2 * 0.5f;
	q2.normalize();

	// --------------- Rotation Constraints --------------- //

	// Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
	// contraints (3x3 matrix)
	m_inverseMassMatrixRotationConstraint = m_i1 + m_i2;
	if (m_body1->getType() == DYNAMIC || m_body2->getType() == DYNAMIC) {
		m_inverseMassMatrixRotationConstraint = m_inverseMassMatrixRotationConstraint.getInverse();
	}

	// Compute the position error for the 3 rotation constraints
	etk::Quaternion currentOrientationDifference = q2 * q1.getInverse();
	currentOrientationDifference.normalize();
	const etk::Quaternion qError = currentOrientationDifference * m_initOrientationDifferenceInv;
	const vec3 errorRotation = float(2.0) * qError.getVectorV();

	// Compute the Lagrange multiplier lambda for the 3 rotation constraints
	vec3 lambdaRotation = m_inverseMassMatrixRotationConstraint * (-errorRotation);

	// Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
	angularImpulseBody1 = -lambdaRotation;

	// Apply the impulse to the body 1
	w1 = m_i1 * angularImpulseBody1;

	// Update the body position/orientation of body 1
	q1 += etk::Quaternion(0, w1) * q1 * 0.5f;
	q1.normalize();

	// Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 2
	angularImpulseBody2 = lambdaRotation;

	// Apply the impulse to the body 2
	w2 = m_i2 * angularImpulseBody2;

	// Update the body position/orientation of body 2
	q2 += etk::Quaternion(0, w2) * q2 * 0.5f;
	q2.normalize();

	// --------------- Limits Constraints --------------- //

	if (mIsLimitEnabled) {

		if (mIsLowerLimitViolated || mIsUpperLimitViolated) {

			// Compute the inverse of the mass matrix K=JM^-1J^t for the limits (1x1 matrix)
			m_inverseMassMatrixLimit = m_body1->m_massInverse + m_body2->m_massInverse +
									mR1PlusUCrossSliderAxis.dot(m_i1 * mR1PlusUCrossSliderAxis) +
									mR2CrossSliderAxis.dot(m_i2 * mR2CrossSliderAxis);
			m_inverseMassMatrixLimit = (m_inverseMassMatrixLimit > 0.0) ?
									  1.0f / m_inverseMassMatrixLimit : 0.0f;
		}

		// If the lower limit is violated
		if (mIsLowerLimitViolated) {

			// Compute the Lagrange multiplier lambda for the lower limit constraint
			float lambdaLowerLimit = m_inverseMassMatrixLimit * (-lowerLimitError);

			// Compute the impulse P=J^T * lambda for the lower limit constraint of body 1
			const vec3 linearImpulseBody1 = -lambdaLowerLimit * mSliderAxisWorld;
			const vec3 angularImpulseBody1 = -lambdaLowerLimit * mR1PlusUCrossSliderAxis;

			// Apply the impulse to the body 1
			const vec3 v1 = inverseMassBody1 * linearImpulseBody1;
			const vec3 w1 = m_i1 * angularImpulseBody1;

			// Update the body position/orientation of body 1
			x1 += v1;
			q1 += etk::Quaternion(0, w1) * q1 * 0.5f;
			q1.normalize();

			// Compute the impulse P=J^T * lambda for the lower limit constraint of body 2
			const vec3 linearImpulseBody2 = lambdaLowerLimit * mSliderAxisWorld;
			const vec3 angularImpulseBody2 = lambdaLowerLimit * mR2CrossSliderAxis;

			// Apply the impulse to the body 2
			const vec3 v2 = inverseMassBody2 * linearImpulseBody2;
			const vec3 w2 = m_i2 * angularImpulseBody2;

			// Update the body position/orientation of body 2
			x2 += v2;
			q2 += etk::Quaternion(0, w2) * q2 * 0.5f;
			q2.normalize();
		}

		// If the upper limit is violated
		if (mIsUpperLimitViolated) {

			// Compute the Lagrange multiplier lambda for the upper limit constraint
			float lambdaUpperLimit = m_inverseMassMatrixLimit * (-upperLimitError);

			// Compute the impulse P=J^T * lambda for the upper limit constraint of body 1
			const vec3 linearImpulseBody1 = lambdaUpperLimit * mSliderAxisWorld;
			const vec3 angularImpulseBody1 = lambdaUpperLimit * mR1PlusUCrossSliderAxis;

			// Apply the impulse to the body 1
			const vec3 v1 = inverseMassBody1 * linearImpulseBody1;
			const vec3 w1 = m_i1 * angularImpulseBody1;

			// Update the body position/orientation of body 1
			x1 += v1;
			q1 += etk::Quaternion(0, w1) * q1 * 0.5f;
			q1.normalize();

			// Compute the impulse P=J^T * lambda for the upper limit constraint of body 2
			const vec3 linearImpulseBody2 = -lambdaUpperLimit * mSliderAxisWorld;
			const vec3 angularImpulseBody2 = -lambdaUpperLimit * mR2CrossSliderAxis;

			// Apply the impulse to the body 2
			const vec3 v2 = inverseMassBody2 * linearImpulseBody2;
			const vec3 w2 = m_i2 * angularImpulseBody2;

			// Update the body position/orientation of body 2
			x2 += v2;
			q2 += etk::Quaternion(0, w2) * q2 * 0.5f;
			q2.normalize();
		}
	}
}

// Enable/Disable the limits of the joint
/**
 * @param isLimitEnabled True if you want to enable the joint limits and false
 *					   otherwise
 */
void SliderJoint::enableLimit(bool isLimitEnabled) {

	if (isLimitEnabled != mIsLimitEnabled) {

		mIsLimitEnabled = isLimitEnabled;

		// Reset the limits
		resetLimits();
	}
}

// Enable/Disable the motor of the joint
/**
 * @param isMotorEnabled True if you want to enable the joint motor and false
 *					   otherwise
 */
void SliderJoint::enableMotor(bool isMotorEnabled) {

	mIsMotorEnabled = isMotorEnabled;
	m_impulseMotor = 0.0;

	// Wake up the two bodies of the joint
	m_body1->setIsSleeping(false);
	m_body2->setIsSleeping(false);
}

// Return the current translation value of the joint
/**
 * @return The current translation distance of the joint (in meters)
 */
float SliderJoint::getTranslation() const {

	// TODO : Check if we need to compare rigid body position or center of mass here

	// Get the bodies positions and orientations
	const vec3& x1 = m_body1->getTransform().getPosition();
	const vec3& x2 = m_body2->getTransform().getPosition();
	const etk::Quaternion& q1 = m_body1->getTransform().getOrientation();
	const etk::Quaternion& q2 = m_body2->getTransform().getOrientation();

	// Compute the two anchor points in world-space coordinates
	const vec3 anchorBody1 = x1 + q1 * m_localAnchorPointBody1;
	const vec3 anchorBody2 = x2 + q2 * m_localAnchorPointBody2;

	// Compute the vector u (difference between anchor points)
	const vec3 u = anchorBody2 - anchorBody1;

	// Compute the slider axis in world-space
	vec3 sliderAxisWorld = q1 * mSliderAxisBody1;
	sliderAxisWorld.normalize();

	// Compute and return the translation value
	return u.dot(sliderAxisWorld);
}

// Set the minimum translation limit
/**
 * @param lowerLimit The minimum translation limit of the joint (in meters)
 */
void SliderJoint::setMinTranslationLimit(float lowerLimit) {

	assert(lowerLimit <= mUpperLimit);

	if (lowerLimit != mLowerLimit) {

		mLowerLimit = lowerLimit;

		// Reset the limits
		resetLimits();
	}
}

// Set the maximum translation limit
/**
 * @param lowerLimit The maximum translation limit of the joint (in meters)
 */
void SliderJoint::setMaxTranslationLimit(float upperLimit) {

	assert(mLowerLimit <= upperLimit);

	if (upperLimit != mUpperLimit) {

		mUpperLimit = upperLimit;

		// Reset the limits
		resetLimits();
	}
}

// Reset the limits
void SliderJoint::resetLimits() {

	// Reset the accumulated impulses for the limits
	m_impulseLowerLimit = 0.0;
	m_impulseUpperLimit = 0.0;

	// Wake up the two bodies of the joint
	m_body1->setIsSleeping(false);
	m_body2->setIsSleeping(false);
}

// Set the motor speed
/**
 * @param motorSpeed The speed of the joint motor (in meters per second)
 */
void SliderJoint::setMotorSpeed(float motorSpeed) {

	if (motorSpeed != mMotorSpeed) {

		mMotorSpeed = motorSpeed;

		// Wake up the two bodies of the joint
		m_body1->setIsSleeping(false);
		m_body2->setIsSleeping(false);
	}
}

// Set the maximum motor force
/**
 * @param maxMotorForce The maximum force of the joint motor (in Newton x meters)
 */
void SliderJoint::setMaxMotorForce(float maxMotorForce) {

	if (maxMotorForce != mMaxMotorForce) {

		assert(mMaxMotorForce >= 0.0);
		mMaxMotorForce = maxMotorForce;

		// Wake up the two bodies of the joint
		m_body1->setIsSleeping(false);
		m_body2->setIsSleeping(false);
	}
}
