/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/constraint/SliderJoint.hpp>

using namespace ephysics;

// Static variables definition
const float SliderJoint::BETA = float(0.2);

// Constructor
SliderJoint::SliderJoint(const SliderJointInfo& jointInfo)
			: Joint(jointInfo), m_impulseTranslation(0, 0), m_impulseRotation(0, 0, 0),
			  m_impulseLowerLimit(0), m_impulseUpperLimit(0), m_impulseMotor(0),
			  m_isLimitEnabled(jointInfo.isLimitEnabled), m_isMotorEnabled(jointInfo.isMotorEnabled),
			  m_lowerLimit(jointInfo.minTranslationLimit),
			  m_upperLimit(jointInfo.maxTranslationLimit), m_isLowerLimitViolated(false),
			  m_isUpperLimitViolated(false), m_motorSpeed(jointInfo.motorSpeed),
			  m_maxMotorForce(jointInfo.maxMotorForce){

	assert(m_upperLimit >= 0.0);
	assert(m_lowerLimit <= 0.0);
	assert(m_maxMotorForce >= 0.0);

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
	m_sliderAxisBody1 = m_body1->getTransform().getOrientation().getInverse() *
					   jointInfo.sliderAxisWorldSpace;
	m_sliderAxisBody1.normalize();
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
	m_R1 = orientationBody1 * m_localAnchorPointBody1;
	m_R2 = orientationBody2 * m_localAnchorPointBody2;

	// Compute the vector u (difference between anchor points)
	const vec3 u = x2 + m_R2 - x1 - m_R1;

	// Compute the two orthogonal vectors to the slider axis in world-space
	m_sliderAxisWorld = orientationBody1 * m_sliderAxisBody1;
	m_sliderAxisWorld.normalize();
	m_N1 = m_sliderAxisWorld.getOrthoVector();
	m_N2 = m_sliderAxisWorld.cross(m_N1);

	// Check if the limit constraints are violated or not
	float uDotSliderAxis = u.dot(m_sliderAxisWorld);
	float lowerLimitError = uDotSliderAxis - m_lowerLimit;
	float upperLimitError = m_upperLimit - uDotSliderAxis;
	bool oldIsLowerLimitViolated = m_isLowerLimitViolated;
	m_isLowerLimitViolated = lowerLimitError <= 0;
	if (m_isLowerLimitViolated != oldIsLowerLimitViolated) {
		m_impulseLowerLimit = 0.0;
	}
	bool oldIsUpperLimitViolated = m_isUpperLimitViolated;
	m_isUpperLimitViolated = upperLimitError <= 0;
	if (m_isUpperLimitViolated != oldIsUpperLimitViolated) {
		m_impulseUpperLimit = 0.0;
	}

	// Compute the cross products used in the Jacobians
	m_R2CrossN1 = m_R2.cross(m_N1);
	m_R2CrossN2 = m_R2.cross(m_N2);
	m_R2CrossSliderAxis = m_R2.cross(m_sliderAxisWorld);
	const vec3 r1PlusU = m_R1 + u;
	m_R1PlusUCrossN1 = (r1PlusU).cross(m_N1);
	m_R1PlusUCrossN2 = (r1PlusU).cross(m_N2);
	m_R1PlusUCrossSliderAxis = (r1PlusU).cross(m_sliderAxisWorld);

	// Compute the inverse of the mass matrix K=JM^-1J^t for the 2 translation
	// constraints (2x2 matrix)
	float sumInverseMass = m_body1->m_massInverse + m_body2->m_massInverse;
	vec3 I1R1PlusUCrossN1 = m_i1 * m_R1PlusUCrossN1;
	vec3 I1R1PlusUCrossN2 = m_i1 * m_R1PlusUCrossN2;
	vec3 I2R2CrossN1 = m_i2 * m_R2CrossN1;
	vec3 I2R2CrossN2 = m_i2 * m_R2CrossN2;
	const float el11 = sumInverseMass + m_R1PlusUCrossN1.dot(I1R1PlusUCrossN1) +
						 m_R2CrossN1.dot(I2R2CrossN1);
	const float el12 = m_R1PlusUCrossN1.dot(I1R1PlusUCrossN2) +
						 m_R2CrossN1.dot(I2R2CrossN2);
	const float el21 = m_R1PlusUCrossN2.dot(I1R1PlusUCrossN1) +
						 m_R2CrossN2.dot(I2R2CrossN1);
	const float el22 = sumInverseMass + m_R1PlusUCrossN2.dot(I1R1PlusUCrossN2) +
						 m_R2CrossN2.dot(I2R2CrossN2);
	etk::Matrix2x2 matrixKTranslation(el11, el12, el21, el22);
	m_inverseMassMatrixTranslationConstraint.setZero();
	if (m_body1->getType() == DYNAMIC || m_body2->getType() == DYNAMIC) {
		m_inverseMassMatrixTranslationConstraint = matrixKTranslation.getInverse();
	}

	// Compute the bias "b" of the translation constraint
	m_bTranslation.setZero();
	float biasFactor = (BETA / constraintSolverData.timeStep);
	if (m_positionCorrectionTechnique == BAUMGARTE_JOINTS) {
		m_bTranslation.setX(u.dot(m_N1));
		m_bTranslation.setY(u.dot(m_N2));
		m_bTranslation *= biasFactor;
	}

	// Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
	// contraints (3x3 matrix)
	m_inverseMassMatrixRotationConstraint = m_i1 + m_i2;
	if (m_body1->getType() == DYNAMIC || m_body2->getType() == DYNAMIC) {
		m_inverseMassMatrixRotationConstraint = m_inverseMassMatrixRotationConstraint.getInverse();
	}

	// Compute the bias "b" of the rotation constraint
	m_bRotation.setZero();
	if (m_positionCorrectionTechnique == BAUMGARTE_JOINTS) {
		etk::Quaternion currentOrientationDifference = orientationBody2 * orientationBody1.getInverse();
		currentOrientationDifference.normalize();
		const etk::Quaternion qError = currentOrientationDifference * m_initOrientationDifferenceInv;
		m_bRotation = biasFactor * float(2.0) * qError.getVectorV();
	}

	// If the limits are enabled
	if (m_isLimitEnabled && (m_isLowerLimitViolated || m_isUpperLimitViolated)) {

		// Compute the inverse of the mass matrix K=JM^-1J^t for the limits (1x1 matrix)
		m_inverseMassMatrixLimit = m_body1->m_massInverse + m_body2->m_massInverse +
								  m_R1PlusUCrossSliderAxis.dot(m_i1 * m_R1PlusUCrossSliderAxis) +
								  m_R2CrossSliderAxis.dot(m_i2 * m_R2CrossSliderAxis);
		m_inverseMassMatrixLimit = (m_inverseMassMatrixLimit > 0.0) ?
								  1.0f / m_inverseMassMatrixLimit : 0.0f;

		// Compute the bias "b" of the lower limit constraint
		m_bLowerLimit = 0.0;
		if (m_positionCorrectionTechnique == BAUMGARTE_JOINTS) {
			m_bLowerLimit = biasFactor * lowerLimitError;
		}

		// Compute the bias "b" of the upper limit constraint
		m_bUpperLimit = 0.0;
		if (m_positionCorrectionTechnique == BAUMGARTE_JOINTS) {
			m_bUpperLimit = biasFactor * upperLimitError;
		}
	}

	// If the motor is enabled
	if (m_isMotorEnabled) {

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
	vec3 linearImpulseLimits = impulseLimits * m_sliderAxisWorld;

	// Compute the impulse P=J^T * lambda for the motor constraint of body 1
	vec3 impulseMotor = m_impulseMotor * m_sliderAxisWorld;

	// Compute the impulse P=J^T * lambda for the 2 translation constraints of body 1
	vec3 linearImpulseBody1 = -m_N1 * m_impulseTranslation.x() - m_N2 * m_impulseTranslation.y();
	vec3 angularImpulseBody1 = -m_R1PlusUCrossN1 * m_impulseTranslation.x() -
			m_R1PlusUCrossN2 * m_impulseTranslation.y();

	// Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
	angularImpulseBody1 += -m_impulseRotation;

	// Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 1
	linearImpulseBody1 += linearImpulseLimits;
	angularImpulseBody1 += impulseLimits * m_R1PlusUCrossSliderAxis;

	// Compute the impulse P=J^T * lambda for the motor constraint of body 1
	linearImpulseBody1 += impulseMotor;

	// Apply the impulse to the body 1
	v1 += inverseMassBody1 * linearImpulseBody1;
	w1 += m_i1 * angularImpulseBody1;

	// Compute the impulse P=J^T * lambda for the 2 translation constraints of body 2
	vec3 linearImpulseBody2 = m_N1 * m_impulseTranslation.x() + m_N2 * m_impulseTranslation.y();
	vec3 angularImpulseBody2 = m_R2CrossN1 * m_impulseTranslation.x() +
			m_R2CrossN2 * m_impulseTranslation.y();

	// Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 2
	angularImpulseBody2 += m_impulseRotation;

	// Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 2
	linearImpulseBody2 += -linearImpulseLimits;
	angularImpulseBody2 += -impulseLimits * m_R2CrossSliderAxis;

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
	const float el1 = -m_N1.dot(v1) - w1.dot(m_R1PlusUCrossN1) +
						 m_N1.dot(v2) + w2.dot(m_R2CrossN1);
	const float el2 = -m_N2.dot(v1) - w1.dot(m_R1PlusUCrossN2) +
						 m_N2.dot(v2) + w2.dot(m_R2CrossN2);
	const vec2 JvTranslation(el1, el2);

	// Compute the Lagrange multiplier lambda for the 2 translation constraints
	vec2 deltaLambda = m_inverseMassMatrixTranslationConstraint * (-JvTranslation -m_bTranslation);
	m_impulseTranslation += deltaLambda;

	// Compute the impulse P=J^T * lambda for the 2 translation constraints of body 1
	const vec3 linearImpulseBody1 = -m_N1 * deltaLambda.x() - m_N2 * deltaLambda.y();
	vec3 angularImpulseBody1 = -m_R1PlusUCrossN1 * deltaLambda.x() -
			m_R1PlusUCrossN2 * deltaLambda.y();

	// Apply the impulse to the body 1
	v1 += inverseMassBody1 * linearImpulseBody1;
	w1 += m_i1 * angularImpulseBody1;

	// Compute the impulse P=J^T * lambda for the 2 translation constraints of body 2
	const vec3 linearImpulseBody2 = m_N1 * deltaLambda.x() + m_N2 * deltaLambda.y();
	vec3 angularImpulseBody2 = m_R2CrossN1 * deltaLambda.x() + m_R2CrossN2 * deltaLambda.y();

	// Apply the impulse to the body 2
	v2 += inverseMassBody2 * linearImpulseBody2;
	w2 += m_i2 * angularImpulseBody2;

	// --------------- Rotation Constraints --------------- //

	// Compute J*v for the 3 rotation constraints
	const vec3 JvRotation = w2 - w1;

	// Compute the Lagrange multiplier lambda for the 3 rotation constraints
	vec3 deltaLambda2 = m_inverseMassMatrixRotationConstraint * (-JvRotation - m_bRotation);
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

	if (m_isLimitEnabled) {

		// If the lower limit is violated
		if (m_isLowerLimitViolated) {

			// Compute J*v for the lower limit constraint
			const float JvLowerLimit = m_sliderAxisWorld.dot(v2) + m_R2CrossSliderAxis.dot(w2) -
										 m_sliderAxisWorld.dot(v1) - m_R1PlusUCrossSliderAxis.dot(w1);

			// Compute the Lagrange multiplier lambda for the lower limit constraint
			float deltaLambdaLower = m_inverseMassMatrixLimit * (-JvLowerLimit -m_bLowerLimit);
			float lambdaTemp = m_impulseLowerLimit;
			m_impulseLowerLimit = std::max(m_impulseLowerLimit + deltaLambdaLower, 0.0f);
			deltaLambdaLower = m_impulseLowerLimit - lambdaTemp;

			// Compute the impulse P=J^T * lambda for the lower limit constraint of body 1
			const vec3 linearImpulseBody1 = -deltaLambdaLower * m_sliderAxisWorld;
			const vec3 angularImpulseBody1 = -deltaLambdaLower * m_R1PlusUCrossSliderAxis;

			// Apply the impulse to the body 1
			v1 += inverseMassBody1 * linearImpulseBody1;
			w1 += m_i1 * angularImpulseBody1;

			// Compute the impulse P=J^T * lambda for the lower limit constraint of body 2
			const vec3 linearImpulseBody2 = deltaLambdaLower * m_sliderAxisWorld;
			const vec3 angularImpulseBody2 = deltaLambdaLower * m_R2CrossSliderAxis;

			// Apply the impulse to the body 2
			v2 += inverseMassBody2 * linearImpulseBody2;
			w2 += m_i2 * angularImpulseBody2;
		}

		// If the upper limit is violated
		if (m_isUpperLimitViolated) {

			// Compute J*v for the upper limit constraint
			const float JvUpperLimit = m_sliderAxisWorld.dot(v1) + m_R1PlusUCrossSliderAxis.dot(w1)
										- m_sliderAxisWorld.dot(v2) - m_R2CrossSliderAxis.dot(w2);

			// Compute the Lagrange multiplier lambda for the upper limit constraint
			float deltaLambdaUpper = m_inverseMassMatrixLimit * (-JvUpperLimit -m_bUpperLimit);
			float lambdaTemp = m_impulseUpperLimit;
			m_impulseUpperLimit = std::max(m_impulseUpperLimit + deltaLambdaUpper, 0.0f);
			deltaLambdaUpper = m_impulseUpperLimit - lambdaTemp;

			// Compute the impulse P=J^T * lambda for the upper limit constraint of body 1
			const vec3 linearImpulseBody1 = deltaLambdaUpper * m_sliderAxisWorld;
			const vec3 angularImpulseBody1 = deltaLambdaUpper * m_R1PlusUCrossSliderAxis;

			// Apply the impulse to the body 1
			v1 += inverseMassBody1 * linearImpulseBody1;
			w1 += m_i1 * angularImpulseBody1;

			// Compute the impulse P=J^T * lambda for the upper limit constraint of body 2
			const vec3 linearImpulseBody2 = -deltaLambdaUpper * m_sliderAxisWorld;
			const vec3 angularImpulseBody2 = -deltaLambdaUpper * m_R2CrossSliderAxis;

			// Apply the impulse to the body 2
			v2 += inverseMassBody2 * linearImpulseBody2;
			w2 += m_i2 * angularImpulseBody2;
		}
	}

	// --------------- Motor --------------- //

	if (m_isMotorEnabled) {

		// Compute J*v for the motor
		const float JvMotor = m_sliderAxisWorld.dot(v1) - m_sliderAxisWorld.dot(v2);

		// Compute the Lagrange multiplier lambda for the motor
		const float maxMotorImpulse = m_maxMotorForce * constraintSolverData.timeStep;
		float deltaLambdaMotor = m_inverseMassMatrixMotor * (-JvMotor - m_motorSpeed);
		float lambdaTemp = m_impulseMotor;
		m_impulseMotor = clamp(m_impulseMotor + deltaLambdaMotor, -maxMotorImpulse, maxMotorImpulse);
		deltaLambdaMotor = m_impulseMotor - lambdaTemp;

		// Compute the impulse P=J^T * lambda for the motor of body 1
		const vec3 linearImpulseBody1 = deltaLambdaMotor * m_sliderAxisWorld;

		// Apply the impulse to the body 1
		v1 += inverseMassBody1 * linearImpulseBody1;

		// Compute the impulse P=J^T * lambda for the motor of body 2
		const vec3 linearImpulseBody2 = -deltaLambdaMotor * m_sliderAxisWorld;

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
	m_R1 = q1 * m_localAnchorPointBody1;
	m_R2 = q2 * m_localAnchorPointBody2;

	// Compute the vector u (difference between anchor points)
	const vec3 u = x2 + m_R2 - x1 - m_R1;

	// Compute the two orthogonal vectors to the slider axis in world-space
	m_sliderAxisWorld = q1 * m_sliderAxisBody1;
	m_sliderAxisWorld.normalize();
	m_N1 = m_sliderAxisWorld.getOrthoVector();
	m_N2 = m_sliderAxisWorld.cross(m_N1);

	// Check if the limit constraints are violated or not
	float uDotSliderAxis = u.dot(m_sliderAxisWorld);
	float lowerLimitError = uDotSliderAxis - m_lowerLimit;
	float upperLimitError = m_upperLimit - uDotSliderAxis;
	m_isLowerLimitViolated = lowerLimitError <= 0;
	m_isUpperLimitViolated = upperLimitError <= 0;

	// Compute the cross products used in the Jacobians
	m_R2CrossN1 = m_R2.cross(m_N1);
	m_R2CrossN2 = m_R2.cross(m_N2);
	m_R2CrossSliderAxis = m_R2.cross(m_sliderAxisWorld);
	const vec3 r1PlusU = m_R1 + u;
	m_R1PlusUCrossN1 = (r1PlusU).cross(m_N1);
	m_R1PlusUCrossN2 = (r1PlusU).cross(m_N2);
	m_R1PlusUCrossSliderAxis = (r1PlusU).cross(m_sliderAxisWorld);

	// --------------- Translation Constraints --------------- //

	// Recompute the inverse of the mass matrix K=JM^-1J^t for the 2 translation
	// constraints (2x2 matrix)
	float sumInverseMass = m_body1->m_massInverse + m_body2->m_massInverse;
	vec3 I1R1PlusUCrossN1 = m_i1 * m_R1PlusUCrossN1;
	vec3 I1R1PlusUCrossN2 = m_i1 * m_R1PlusUCrossN2;
	vec3 I2R2CrossN1 = m_i2 * m_R2CrossN1;
	vec3 I2R2CrossN2 = m_i2 * m_R2CrossN2;
	const float el11 = sumInverseMass + m_R1PlusUCrossN1.dot(I1R1PlusUCrossN1) +
						 m_R2CrossN1.dot(I2R2CrossN1);
	const float el12 = m_R1PlusUCrossN1.dot(I1R1PlusUCrossN2) +
						 m_R2CrossN1.dot(I2R2CrossN2);
	const float el21 = m_R1PlusUCrossN2.dot(I1R1PlusUCrossN1) +
						 m_R2CrossN2.dot(I2R2CrossN1);
	const float el22 = sumInverseMass + m_R1PlusUCrossN2.dot(I1R1PlusUCrossN2) +
						 m_R2CrossN2.dot(I2R2CrossN2);
	etk::Matrix2x2 matrixKTranslation(el11, el12, el21, el22);
	m_inverseMassMatrixTranslationConstraint.setZero();
	if (m_body1->getType() == DYNAMIC || m_body2->getType() == DYNAMIC) {
		m_inverseMassMatrixTranslationConstraint = matrixKTranslation.getInverse();
	}

	// Compute the position error for the 2 translation constraints
	const vec2 translationError(u.dot(m_N1), u.dot(m_N2));

	// Compute the Lagrange multiplier lambda for the 2 translation constraints
	vec2 lambdaTranslation = m_inverseMassMatrixTranslationConstraint * (-translationError);

	// Compute the impulse P=J^T * lambda for the 2 translation constraints of body 1
	const vec3 linearImpulseBody1 = -m_N1 * lambdaTranslation.x() - m_N2 * lambdaTranslation.y();
	vec3 angularImpulseBody1 = -m_R1PlusUCrossN1 * lambdaTranslation.x() -
										m_R1PlusUCrossN2 * lambdaTranslation.y();

	// Apply the impulse to the body 1
	const vec3 v1 = inverseMassBody1 * linearImpulseBody1;
	vec3 w1 = m_i1 * angularImpulseBody1;

	// Update the body position/orientation of body 1
	x1 += v1;
	q1 += etk::Quaternion(0, w1) * q1 * 0.5f;
	q1.normalize();

	// Compute the impulse P=J^T * lambda for the 2 translation constraints of body 2
	const vec3 linearImpulseBody2 = m_N1 * lambdaTranslation.x() + m_N2 * lambdaTranslation.y();
	vec3 angularImpulseBody2 = m_R2CrossN1 * lambdaTranslation.x() +
			m_R2CrossN2 * lambdaTranslation.y();

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

	if (m_isLimitEnabled) {

		if (m_isLowerLimitViolated || m_isUpperLimitViolated) {

			// Compute the inverse of the mass matrix K=JM^-1J^t for the limits (1x1 matrix)
			m_inverseMassMatrixLimit = m_body1->m_massInverse + m_body2->m_massInverse +
									m_R1PlusUCrossSliderAxis.dot(m_i1 * m_R1PlusUCrossSliderAxis) +
									m_R2CrossSliderAxis.dot(m_i2 * m_R2CrossSliderAxis);
			m_inverseMassMatrixLimit = (m_inverseMassMatrixLimit > 0.0) ?
									  1.0f / m_inverseMassMatrixLimit : 0.0f;
		}

		// If the lower limit is violated
		if (m_isLowerLimitViolated) {

			// Compute the Lagrange multiplier lambda for the lower limit constraint
			float lambdaLowerLimit = m_inverseMassMatrixLimit * (-lowerLimitError);

			// Compute the impulse P=J^T * lambda for the lower limit constraint of body 1
			const vec3 linearImpulseBody1 = -lambdaLowerLimit * m_sliderAxisWorld;
			const vec3 angularImpulseBody1 = -lambdaLowerLimit * m_R1PlusUCrossSliderAxis;

			// Apply the impulse to the body 1
			const vec3 v1 = inverseMassBody1 * linearImpulseBody1;
			const vec3 w1 = m_i1 * angularImpulseBody1;

			// Update the body position/orientation of body 1
			x1 += v1;
			q1 += etk::Quaternion(0, w1) * q1 * 0.5f;
			q1.normalize();

			// Compute the impulse P=J^T * lambda for the lower limit constraint of body 2
			const vec3 linearImpulseBody2 = lambdaLowerLimit * m_sliderAxisWorld;
			const vec3 angularImpulseBody2 = lambdaLowerLimit * m_R2CrossSliderAxis;

			// Apply the impulse to the body 2
			const vec3 v2 = inverseMassBody2 * linearImpulseBody2;
			const vec3 w2 = m_i2 * angularImpulseBody2;

			// Update the body position/orientation of body 2
			x2 += v2;
			q2 += etk::Quaternion(0, w2) * q2 * 0.5f;
			q2.normalize();
		}

		// If the upper limit is violated
		if (m_isUpperLimitViolated) {

			// Compute the Lagrange multiplier lambda for the upper limit constraint
			float lambdaUpperLimit = m_inverseMassMatrixLimit * (-upperLimitError);

			// Compute the impulse P=J^T * lambda for the upper limit constraint of body 1
			const vec3 linearImpulseBody1 = lambdaUpperLimit * m_sliderAxisWorld;
			const vec3 angularImpulseBody1 = lambdaUpperLimit * m_R1PlusUCrossSliderAxis;

			// Apply the impulse to the body 1
			const vec3 v1 = inverseMassBody1 * linearImpulseBody1;
			const vec3 w1 = m_i1 * angularImpulseBody1;

			// Update the body position/orientation of body 1
			x1 += v1;
			q1 += etk::Quaternion(0, w1) * q1 * 0.5f;
			q1.normalize();

			// Compute the impulse P=J^T * lambda for the upper limit constraint of body 2
			const vec3 linearImpulseBody2 = -lambdaUpperLimit * m_sliderAxisWorld;
			const vec3 angularImpulseBody2 = -lambdaUpperLimit * m_R2CrossSliderAxis;

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

	if (isLimitEnabled != m_isLimitEnabled) {

		m_isLimitEnabled = isLimitEnabled;

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

	m_isMotorEnabled = isMotorEnabled;
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
	vec3 sliderAxisWorld = q1 * m_sliderAxisBody1;
	sliderAxisWorld.normalize();

	// Compute and return the translation value
	return u.dot(sliderAxisWorld);
}

// Set the minimum translation limit
/**
 * @param lowerLimit The minimum translation limit of the joint (in meters)
 */
void SliderJoint::setMinTranslationLimit(float lowerLimit) {

	assert(lowerLimit <= m_upperLimit);

	if (lowerLimit != m_lowerLimit) {

		m_lowerLimit = lowerLimit;

		// Reset the limits
		resetLimits();
	}
}

// Set the maximum translation limit
/**
 * @param lowerLimit The maximum translation limit of the joint (in meters)
 */
void SliderJoint::setMaxTranslationLimit(float upperLimit) {

	assert(m_lowerLimit <= upperLimit);

	if (upperLimit != m_upperLimit) {

		m_upperLimit = upperLimit;

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

	if (motorSpeed != m_motorSpeed) {

		m_motorSpeed = motorSpeed;

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

	if (maxMotorForce != m_maxMotorForce) {

		assert(m_maxMotorForce >= 0.0);
		m_maxMotorForce = maxMotorForce;

		// Wake up the two bodies of the joint
		m_body1->setIsSleeping(false);
		m_body2->setIsSleeping(false);
	}
}

// Return true if the limits or the joint are enabled
/**
 * @return True if the joint limits are enabled
 */
bool SliderJoint::isLimitEnabled() const {
	return m_isLimitEnabled;
}

// Return true if the motor of the joint is enabled
/**
 * @return True if the joint motor is enabled
 */
bool SliderJoint::isMotorEnabled() const {
	return m_isMotorEnabled;
}

// Return the minimum translation limit
/**
 * @return The minimum translation limit of the joint (in meters)
 */
float SliderJoint::getMinTranslationLimit() const {
	return m_lowerLimit;
}

// Return the maximum translation limit
/**
 * @return The maximum translation limit of the joint (in meters)
 */
float SliderJoint::getMaxTranslationLimit() const {
	return m_upperLimit;
}

// Return the motor speed
/**
 * @return The current motor speed of the joint (in meters per second)
 */
float SliderJoint::getMotorSpeed() const {
	return m_motorSpeed;
}

// Return the maximum motor force
/**
 * @return The maximum force of the joint motor (in Newton x meters)
 */
float SliderJoint::getMaxMotorForce() const {
	return m_maxMotorForce;
}

// Return the int32_tensity of the current force applied for the joint motor
/**
 * @param timeStep Time step (in seconds)
 * @return The current force of the joint motor (in Newton x meters)
 */
float SliderJoint::getMotorForce(float timeStep) const {
	return m_impulseMotor / timeStep;
}

// Return the number of bytes used by the joint
size_t SliderJoint::getSizeInBytes() const {
	return sizeof(SliderJoint);
}

