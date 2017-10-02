/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#include <ephysics/engine/ContactSolver.hpp>
#include <ephysics/engine/DynamicsWorld.hpp>
#include <ephysics/body/RigidBody.hpp>
#include <ephysics/engine/Profiler.hpp>

using namespace ephysics;
using namespace std;

// Constants initialization
const float ContactSolver::BETA = float(0.2);
const float ContactSolver::BETA_SPLIT_IMPULSE = float(0.2);
const float ContactSolver::SLOP= float(0.01);

// Constructor
ContactSolver::ContactSolver(const etk::Map<RigidBody*, uint32_t>& mapBodyToVelocityIndex)
			  :m_splitLinearVelocities(nullptr), m_splitAngularVelocities(nullptr),
			   m_contactConstraints(nullptr), m_linearVelocities(nullptr), m_angularVelocities(nullptr),
			   m_mapBodyToConstrainedVelocityIndex(mapBodyToVelocityIndex),
			   m_isWarmStartingActive(true), m_isSplitImpulseActive(true),
			   m_isSolveFrictionAtContactManifoldCenterActive(true) {

}

// Destructor
ContactSolver::~ContactSolver() {

}

// Initialize the constraint solver for a given island
void ContactSolver::initializeForIsland(float dt, Island* island) {

	PROFILE("ContactSolver::initializeForIsland()");

	assert(island != nullptr);
	assert(island->getNbBodies() > 0);
	assert(island->getNbContactManifolds() > 0);
	assert(m_splitLinearVelocities != nullptr);
	assert(m_splitAngularVelocities != nullptr);

	// Set the current time step
	m_timeStep = dt;

	m_numberContactManifolds = island->getNbContactManifolds();

	m_contactConstraints = new ContactManifoldSolver[m_numberContactManifolds];
	assert(m_contactConstraints != nullptr);

	// For each contact manifold of the island
	ContactManifold** contactManifolds = island->getContactManifold();
	for (uint32_t i=0; i<m_numberContactManifolds; i++) {

		ContactManifold* externalManifold = contactManifolds[i];

		ContactManifoldSolver& int32_ternalManifold = m_contactConstraints[i];

		assert(externalManifold->getNbContactPoints() > 0);

		// Get the two bodies of the contact
		RigidBody* body1 = static_cast<RigidBody*>(externalManifold->getContactPoint(0)->getBody1());
		RigidBody* body2 = static_cast<RigidBody*>(externalManifold->getContactPoint(0)->getBody2());
		assert(body1 != nullptr);
		assert(body2 != nullptr);

		// Get the position of the two bodies
		const vec3& x1 = body1->m_centerOfMassWorld;
		const vec3& x2 = body2->m_centerOfMassWorld;

		// Initialize the int32_ternal contact manifold structure using the external
		// contact manifold
		int32_ternalManifold.indexBody1 = m_mapBodyToConstrainedVelocityIndex.find(body1)->second;
		int32_ternalManifold.indexBody2 = m_mapBodyToConstrainedVelocityIndex.find(body2)->second;
		int32_ternalManifold.inverseInertiaTensorBody1 = body1->getInertiaTensorInverseWorld();
		int32_ternalManifold.inverseInertiaTensorBody2 = body2->getInertiaTensorInverseWorld();
		int32_ternalManifold.massInverseBody1 = body1->m_massInverse;
		int32_ternalManifold.massInverseBody2 = body2->m_massInverse;
		int32_ternalManifold.nbContacts = externalManifold->getNbContactPoints();
		int32_ternalManifold.restitutionFactor = computeMixedRestitutionFactor(body1, body2);
		int32_ternalManifold.frictionCoefficient = computeMixedFrictionCoefficient(body1, body2);
		int32_ternalManifold.rollingResistanceFactor = computeMixedRollingResistance(body1, body2);
		int32_ternalManifold.externalContactManifold = externalManifold;
		int32_ternalManifold.isBody1DynamicType = body1->getType() == DYNAMIC;
		int32_ternalManifold.isBody2DynamicType = body2->getType() == DYNAMIC;

		// If we solve the friction constraints at the center of the contact manifold
		if (m_isSolveFrictionAtContactManifoldCenterActive) {
			int32_ternalManifold.frictionPointBody1 = vec3(0.0f,0.0f,0.0f);
			int32_ternalManifold.frictionPointBody2 = vec3(0.0f,0.0f,0.0f);
		}

		// For each  contact point of the contact manifold
		for (uint32_t c=0; c<externalManifold->getNbContactPoints(); c++) {

			ContactPointSolver& contactPoint = int32_ternalManifold.contacts[c];

			// Get a contact point
			ContactPoint* externalContact = externalManifold->getContactPoint(c);

			// Get the contact point on the two bodies
			vec3 p1 = externalContact->getWorldPointOnBody1();
			vec3 p2 = externalContact->getWorldPointOnBody2();

			contactPoint.externalContact = externalContact;
			contactPoint.normal = externalContact->getNormal();
			contactPoint.r1 = p1 - x1;
			contactPoint.r2 = p2 - x2;
			contactPoint.penetrationDepth = externalContact->getPenetrationDepth();
			contactPoint.isRestingContact = externalContact->getIsRestingContact();
			externalContact->setIsRestingContact(true);
			contactPoint.oldFrictionVector1 = externalContact->getFrictionVector1();
			contactPoint.oldFrictionvec2 = externalContact->getFrictionvec2();
			contactPoint.penetrationImpulse = 0.0;
			contactPoint.friction1Impulse = 0.0;
			contactPoint.friction2Impulse = 0.0;
			contactPoint.rollingResistanceImpulse = vec3(0.0f,0.0f,0.0f);

			// If we solve the friction constraints at the center of the contact manifold
			if (m_isSolveFrictionAtContactManifoldCenterActive) {
				int32_ternalManifold.frictionPointBody1 += p1;
				int32_ternalManifold.frictionPointBody2 += p2;
			}
		}

		// If we solve the friction constraints at the center of the contact manifold
		if (m_isSolveFrictionAtContactManifoldCenterActive) {

			int32_ternalManifold.frictionPointBody1 /=static_cast<float>(int32_ternalManifold.nbContacts);
			int32_ternalManifold.frictionPointBody2 /=static_cast<float>(int32_ternalManifold.nbContacts);
			int32_ternalManifold.r1Friction = int32_ternalManifold.frictionPointBody1 - x1;
			int32_ternalManifold.r2Friction = int32_ternalManifold.frictionPointBody2 - x2;
			int32_ternalManifold.oldFrictionVector1 = externalManifold->getFrictionVector1();
			int32_ternalManifold.oldFrictionvec2 = externalManifold->getFrictionvec2();

			// If warm starting is active
			if (m_isWarmStartingActive) {

				// Initialize the accumulated impulses with the previous step accumulated impulses
				int32_ternalManifold.friction1Impulse = externalManifold->getFrictionImpulse1();
				int32_ternalManifold.friction2Impulse = externalManifold->getFrictionImpulse2();
				int32_ternalManifold.frictionTwistImpulse = externalManifold->getFrictionTwistImpulse();
			}
			else {

				// Initialize the accumulated impulses to zero
				int32_ternalManifold.friction1Impulse = 0.0;
				int32_ternalManifold.friction2Impulse = 0.0;
				int32_ternalManifold.frictionTwistImpulse = 0.0;
				int32_ternalManifold.rollingResistanceImpulse = vec3(0, 0, 0);
			}
		}
	}

	// Fill-in all the matrices needed to solve the LCP problem
	initializeContactConstraints();
}

// Initialize the contact constraints before solving the system
void ContactSolver::initializeContactConstraints() {
	
	// For each contact constraint
	for (uint32_t c=0; c<m_numberContactManifolds; c++) {

		ContactManifoldSolver& manifold = m_contactConstraints[c];

		// Get the inertia tensors of both bodies
		etk::Matrix3x3& I1 = manifold.inverseInertiaTensorBody1;
		etk::Matrix3x3& I2 = manifold.inverseInertiaTensorBody2;

		// If we solve the friction constraints at the center of the contact manifold
		if (m_isSolveFrictionAtContactManifoldCenterActive) {
			manifold.normal = vec3(0.0, 0.0, 0.0);
		}

		// Get the velocities of the bodies
		const vec3& v1 = m_linearVelocities[manifold.indexBody1];
		const vec3& w1 = m_angularVelocities[manifold.indexBody1];
		const vec3& v2 = m_linearVelocities[manifold.indexBody2];
		const vec3& w2 = m_angularVelocities[manifold.indexBody2];

		// For each contact point constraint
		for (uint32_t i=0; i<manifold.nbContacts; i++) {

			ContactPointSolver& contactPoint = manifold.contacts[i];
			ContactPoint* externalContact = contactPoint.externalContact;

			// Compute the velocity difference
			vec3 deltaV = v2 + w2.cross(contactPoint.r2) - v1 - w1.cross(contactPoint.r1);

			contactPoint.r1CrossN = contactPoint.r1.cross(contactPoint.normal);
			contactPoint.r2CrossN = contactPoint.r2.cross(contactPoint.normal);

			// Compute the inverse mass matrix K for the penetration constraint
			float massPenetration = manifold.massInverseBody1 + manifold.massInverseBody2 +
					((I1 * contactPoint.r1CrossN).cross(contactPoint.r1)).dot(contactPoint.normal) +
					((I2 * contactPoint.r2CrossN).cross(contactPoint.r2)).dot(contactPoint.normal);
			massPenetration > 0.0 ? contactPoint.inversePenetrationMass = 1.0f /
																		  massPenetration :
																		  0.0f;

			// If we do not solve the friction constraints at the center of the contact manifold
			if (!m_isSolveFrictionAtContactManifoldCenterActive) {

				// Compute the friction vectors
				computeFrictionVectors(deltaV, contactPoint);

				contactPoint.r1CrossT1 = contactPoint.r1.cross(contactPoint.frictionVector1);
				contactPoint.r1CrossT2 = contactPoint.r1.cross(contactPoint.frictionvec2);
				contactPoint.r2CrossT1 = contactPoint.r2.cross(contactPoint.frictionVector1);
				contactPoint.r2CrossT2 = contactPoint.r2.cross(contactPoint.frictionvec2);

				// Compute the inverse mass matrix K for the friction
				// constraints at each contact point
				float friction1Mass = manifold.massInverseBody1 + manifold.massInverseBody2 +
										((I1 * contactPoint.r1CrossT1).cross(contactPoint.r1)).dot(
										contactPoint.frictionVector1) +
										((I2 * contactPoint.r2CrossT1).cross(contactPoint.r2)).dot(
										contactPoint.frictionVector1);
				float friction2Mass = manifold.massInverseBody1 + manifold.massInverseBody2 +
										((I1 * contactPoint.r1CrossT2).cross(contactPoint.r1)).dot(
										contactPoint.frictionvec2) +
										((I2 * contactPoint.r2CrossT2).cross(contactPoint.r2)).dot(
										contactPoint.frictionvec2);
				friction1Mass > 0.0 ? contactPoint.inverseFriction1Mass = 1.0f /
																		  friction1Mass :
																		  0.0f;
				friction2Mass > 0.0 ? contactPoint.inverseFriction2Mass = 1.0f /
																		  friction2Mass :
																		  0.0f;
			}

			// Compute the restitution velocity bias "b". We compute this here instead
			// of inside the solve() method because we need to use the velocity difference
			// at the beginning of the contact. Note that if it is a resting contact (normal
			// velocity bellow a given threshold), we do not add a restitution velocity bias
			contactPoint.restitutionBias = 0.0;
			float deltaVDotN = deltaV.dot(contactPoint.normal);
			if (deltaVDotN < -RESTITUTION_VELOCITY_THRESHOLD) {
				contactPoint.restitutionBias = manifold.restitutionFactor * deltaVDotN;
			}

			// If the warm starting of the contact solver is active
			if (m_isWarmStartingActive) {

				// Get the cached accumulated impulses from the previous step
				contactPoint.penetrationImpulse = externalContact->getPenetrationImpulse();
				contactPoint.friction1Impulse = externalContact->getFrictionImpulse1();
				contactPoint.friction2Impulse = externalContact->getFrictionImpulse2();
				contactPoint.rollingResistanceImpulse = externalContact->getRollingResistanceImpulse();
			}

			// Initialize the split impulses to zero
			contactPoint.penetrationSplitImpulse = 0.0;

			// If we solve the friction constraints at the center of the contact manifold
			if (m_isSolveFrictionAtContactManifoldCenterActive) {
				manifold.normal += contactPoint.normal;
			}
		}

		// Compute the inverse K matrix for the rolling resistance constraint
		manifold.inverseRollingResistance.setZero();
		if (manifold.rollingResistanceFactor > 0 && (manifold.isBody1DynamicType || manifold.isBody2DynamicType)) {
			manifold.inverseRollingResistance = manifold.inverseInertiaTensorBody1 + manifold.inverseInertiaTensorBody2;
			manifold.inverseRollingResistance = manifold.inverseRollingResistance.getInverse();
		}

		// If we solve the friction constraints at the center of the contact manifold
		if (m_isSolveFrictionAtContactManifoldCenterActive) {

			manifold.normal.normalize();

			vec3 deltaVFrictionPoint = v2 + w2.cross(manifold.r2Friction) -
										  v1 - w1.cross(manifold.r1Friction);

			// Compute the friction vectors
			computeFrictionVectors(deltaVFrictionPoint, manifold);

			// Compute the inverse mass matrix K for the friction constraints at the center of
			// the contact manifold
			manifold.r1CrossT1 = manifold.r1Friction.cross(manifold.frictionVector1);
			manifold.r1CrossT2 = manifold.r1Friction.cross(manifold.frictionvec2);
			manifold.r2CrossT1 = manifold.r2Friction.cross(manifold.frictionVector1);
			manifold.r2CrossT2 = manifold.r2Friction.cross(manifold.frictionvec2);
			float friction1Mass = manifold.massInverseBody1 + manifold.massInverseBody2 +
									((I1 * manifold.r1CrossT1).cross(manifold.r1Friction)).dot(
									manifold.frictionVector1) +
									((I2 * manifold.r2CrossT1).cross(manifold.r2Friction)).dot(
									manifold.frictionVector1);
			float friction2Mass = manifold.massInverseBody1 + manifold.massInverseBody2 +
									((I1 * manifold.r1CrossT2).cross(manifold.r1Friction)).dot(
									manifold.frictionvec2) +
									((I2 * manifold.r2CrossT2).cross(manifold.r2Friction)).dot(
									manifold.frictionvec2);
			float frictionTwistMass = manifold.normal.dot(manifold.inverseInertiaTensorBody1 *
										   manifold.normal) +
										manifold.normal.dot(manifold.inverseInertiaTensorBody2 *
										   manifold.normal);
			friction1Mass > 0.0 ? manifold.inverseFriction1Mass = 1.0f/friction1Mass
																		 : 0.0f;
			friction2Mass > 0.0 ? manifold.inverseFriction2Mass = 1.0f/friction2Mass
																		 : 0.0f;
			frictionTwistMass > 0.0 ? manifold.inverseTwistFrictionMass = 1.0f /
																				 frictionTwistMass :
																				 0.0f;
		}
	}
}

// Warm start the solver.
/// For each constraint, we apply the previous impulse (from the previous step)
/// at the beginning. With this technique, we will converge faster towards
/// the solution of the linear system
void ContactSolver::warmStart() {

	// Check that warm starting is active
	if (!m_isWarmStartingActive) return;

	// For each constraint
	for (uint32_t c=0; c<m_numberContactManifolds; c++) {

		ContactManifoldSolver& contactManifold = m_contactConstraints[c];

		bool atLeastOneRestingContactPoint = false;

		for (uint32_t i=0; i<contactManifold.nbContacts; i++) {

			ContactPointSolver& contactPoint = contactManifold.contacts[i];

			// If it is not a new contact (this contact was already existing at last time step)
			if (contactPoint.isRestingContact) {

				atLeastOneRestingContactPoint = true;

				// --------- Penetration --------- //

				// Compute the impulse P = J^T * lambda
				const Impulse impulsePenetration = computePenetrationImpulse(
													 contactPoint.penetrationImpulse, contactPoint);

				// Apply the impulse to the bodies of the constraint
				applyImpulse(impulsePenetration, contactManifold);

				// If we do not solve the friction constraints at the center of the contact manifold
				if (!m_isSolveFrictionAtContactManifoldCenterActive) {

					// Project the old friction impulses (with old friction vectors) int32_to
					// the new friction vectors to get the new friction impulses
					vec3 oldFrictionImpulse = contactPoint.friction1Impulse *
												 contactPoint.oldFrictionVector1 +
												 contactPoint.friction2Impulse *
												 contactPoint.oldFrictionvec2;
					contactPoint.friction1Impulse = oldFrictionImpulse.dot(
													   contactPoint.frictionVector1);
					contactPoint.friction2Impulse = oldFrictionImpulse.dot(
													   contactPoint.frictionvec2);

					// --------- Friction 1 --------- //

					// Compute the impulse P = J^T * lambda
					const Impulse impulseFriction1 = computeFriction1Impulse(
													   contactPoint.friction1Impulse, contactPoint);

					// Apply the impulses to the bodies of the constraint
					applyImpulse(impulseFriction1, contactManifold);

					// --------- Friction 2 --------- //

					// Compute the impulse P=J^T * lambda
				   const Impulse impulseFriction2 = computeFriction2Impulse(
													   contactPoint.friction2Impulse, contactPoint);

					// Apply the impulses to the bodies of the constraint
					applyImpulse(impulseFriction2, contactManifold);

					// ------ Rolling resistance------ //

					if (contactManifold.rollingResistanceFactor > 0) {

						// Compute the impulse P = J^T * lambda
						const Impulse impulseRollingResistance(vec3(0.0f,0.0f,0.0f), -contactPoint.rollingResistanceImpulse,
															   vec3(0.0f,0.0f,0.0f), contactPoint.rollingResistanceImpulse);

						// Apply the impulses to the bodies of the constraint
						applyImpulse(impulseRollingResistance, contactManifold);
					}
				}
			}
			else {  // If it is a new contact point

				// Initialize the accumulated impulses to zero
				contactPoint.penetrationImpulse = 0.0;
				contactPoint.friction1Impulse = 0.0;
				contactPoint.friction2Impulse = 0.0;
				contactPoint.rollingResistanceImpulse = vec3(0.0f,0.0f,0.0f);
			}
		}

		// If we solve the friction constraints at the center of the contact manifold and there is
		// at least one resting contact point in the contact manifold
		if (m_isSolveFrictionAtContactManifoldCenterActive && atLeastOneRestingContactPoint) {

			// Project the old friction impulses (with old friction vectors) int32_to the new friction
			// vectors to get the new friction impulses
			vec3 oldFrictionImpulse = contactManifold.friction1Impulse *
										 contactManifold.oldFrictionVector1 +
										 contactManifold.friction2Impulse *
										 contactManifold.oldFrictionvec2;
			contactManifold.friction1Impulse = oldFrictionImpulse.dot(
												  contactManifold.frictionVector1);
			contactManifold.friction2Impulse = oldFrictionImpulse.dot(
												  contactManifold.frictionvec2);

			// ------ First friction constraint at the center of the contact manifold ------ //

			// Compute the impulse P = J^T * lambda
			vec3 linearImpulseBody1 = -contactManifold.frictionVector1 *
										  contactManifold.friction1Impulse;
			vec3 angularImpulseBody1 = -contactManifold.r1CrossT1 *
										   contactManifold.friction1Impulse;
			vec3 linearImpulseBody2 = contactManifold.frictionVector1 *
										 contactManifold.friction1Impulse;
			vec3 angularImpulseBody2 = contactManifold.r2CrossT1 *
										  contactManifold.friction1Impulse;
			const Impulse impulseFriction1(linearImpulseBody1, angularImpulseBody1,
										   linearImpulseBody2, angularImpulseBody2);

			// Apply the impulses to the bodies of the constraint
			applyImpulse(impulseFriction1, contactManifold);

			// ------ Second friction constraint at the center of the contact manifold ----- //

			// Compute the impulse P = J^T * lambda
			linearImpulseBody1 = -contactManifold.frictionvec2 *
								  contactManifold.friction2Impulse;
			angularImpulseBody1 = -contactManifold.r1CrossT2 *
								   contactManifold.friction2Impulse;
			linearImpulseBody2 = contactManifold.frictionvec2 *
								 contactManifold.friction2Impulse;
			angularImpulseBody2 = contactManifold.r2CrossT2 *
								  contactManifold.friction2Impulse;
			const Impulse impulseFriction2(linearImpulseBody1, angularImpulseBody1,
										   linearImpulseBody2, angularImpulseBody2);

			// Apply the impulses to the bodies of the constraint
			applyImpulse(impulseFriction2, contactManifold);

			// ------ Twist friction constraint at the center of the contact manifold ------ //

			// Compute the impulse P = J^T * lambda
			linearImpulseBody1 = vec3(0.0, 0.0, 0.0);
			angularImpulseBody1 = -contactManifold.normal * contactManifold.frictionTwistImpulse;
			linearImpulseBody2 = vec3(0.0, 0.0, 0.0);
			angularImpulseBody2 = contactManifold.normal * contactManifold.frictionTwistImpulse;
			const Impulse impulseTwistFriction(linearImpulseBody1, angularImpulseBody1,
											   linearImpulseBody2, angularImpulseBody2);

			// Apply the impulses to the bodies of the constraint
			applyImpulse(impulseTwistFriction, contactManifold);

			// ------ Rolling resistance at the center of the contact manifold ------ //

			// Compute the impulse P = J^T * lambda
			angularImpulseBody1 = -contactManifold.rollingResistanceImpulse;
			angularImpulseBody2 = contactManifold.rollingResistanceImpulse;
			const Impulse impulseRollingResistance(vec3(0.0f,0.0f,0.0f), angularImpulseBody1,
												   vec3(0.0f,0.0f,0.0f), angularImpulseBody2);

			// Apply the impulses to the bodies of the constraint
			applyImpulse(impulseRollingResistance, contactManifold);
		}
		else {  // If it is a new contact manifold

			// Initialize the accumulated impulses to zero
			contactManifold.friction1Impulse = 0.0;
			contactManifold.friction2Impulse = 0.0;
			contactManifold.frictionTwistImpulse = 0.0;
			contactManifold.rollingResistanceImpulse = vec3(0.0f,0.0f,0.0f);
		}
	}
}

// Solve the contacts
void ContactSolver::solve() {

	PROFILE("ContactSolver::solve()");

	float deltaLambda;
	float lambdaTemp;

	// For each contact manifold
	for (uint32_t c=0; c<m_numberContactManifolds; c++) {

		ContactManifoldSolver& contactManifold = m_contactConstraints[c];

		float sum_penetrationImpulse = 0.0;

		// Get the constrained velocities
		const vec3& v1 = m_linearVelocities[contactManifold.indexBody1];
		const vec3& w1 = m_angularVelocities[contactManifold.indexBody1];
		const vec3& v2 = m_linearVelocities[contactManifold.indexBody2];
		const vec3& w2 = m_angularVelocities[contactManifold.indexBody2];

		for (uint32_t i=0; i<contactManifold.nbContacts; i++) {

			ContactPointSolver& contactPoint = contactManifold.contacts[i];

			// --------- Penetration --------- //

			// Compute J*v
			vec3 deltaV = v2 + w2.cross(contactPoint.r2) - v1 - w1.cross(contactPoint.r1);
			float deltaVDotN = deltaV.dot(contactPoint.normal);
			float Jv = deltaVDotN;

			// Compute the bias "b" of the constraint
			float beta = m_isSplitImpulseActive ? BETA_SPLIT_IMPULSE : BETA;
			float biasPenetrationDepth = 0.0;
			if (contactPoint.penetrationDepth > SLOP) biasPenetrationDepth = -(beta/m_timeStep) *
					etk::max(0.0f, float(contactPoint.penetrationDepth - SLOP));
			float b = biasPenetrationDepth + contactPoint.restitutionBias;

			// Compute the Lagrange multiplier lambda
			if (m_isSplitImpulseActive) {
				deltaLambda = - (Jv + contactPoint.restitutionBias) *
						contactPoint.inversePenetrationMass;
			}
			else {
				deltaLambda = - (Jv + b) * contactPoint.inversePenetrationMass;
			}
			lambdaTemp = contactPoint.penetrationImpulse;
			contactPoint.penetrationImpulse = etk::max(contactPoint.penetrationImpulse +
													   deltaLambda, 0.0f);
			deltaLambda = contactPoint.penetrationImpulse - lambdaTemp;

			// Compute the impulse P=J^T * lambda
			const Impulse impulsePenetration = computePenetrationImpulse(deltaLambda,
																		 contactPoint);

			// Apply the impulse to the bodies of the constraint
			applyImpulse(impulsePenetration, contactManifold);

			sum_penetrationImpulse += contactPoint.penetrationImpulse;

			// If the split impulse position correction is active
			if (m_isSplitImpulseActive) {

				// Split impulse (position correction)
				const vec3& v1Split = m_splitLinearVelocities[contactManifold.indexBody1];
				const vec3& w1Split = m_splitAngularVelocities[contactManifold.indexBody1];
				const vec3& v2Split = m_splitLinearVelocities[contactManifold.indexBody2];
				const vec3& w2Split = m_splitAngularVelocities[contactManifold.indexBody2];
				vec3 deltaVSplit = v2Split + w2Split.cross(contactPoint.r2) -
						v1Split - w1Split.cross(contactPoint.r1);
				float JvSplit = deltaVSplit.dot(contactPoint.normal);
				float deltaLambdaSplit = - (JvSplit + biasPenetrationDepth) *
						contactPoint.inversePenetrationMass;
				float lambdaTempSplit = contactPoint.penetrationSplitImpulse;
				contactPoint.penetrationSplitImpulse = etk::max(
							contactPoint.penetrationSplitImpulse +
							deltaLambdaSplit, 0.0f);
				deltaLambda = contactPoint.penetrationSplitImpulse - lambdaTempSplit;

				// Compute the impulse P=J^T * lambda
				const Impulse splitImpulsePenetration = computePenetrationImpulse(
							deltaLambdaSplit, contactPoint);

				applySplitImpulse(splitImpulsePenetration, contactManifold);
			}

			// If we do not solve the friction constraints at the center of the contact manifold
			if (!m_isSolveFrictionAtContactManifoldCenterActive) {

				// --------- Friction 1 --------- //

				// Compute J*v
				deltaV = v2 + w2.cross(contactPoint.r2) - v1 - w1.cross(contactPoint.r1);
				Jv = deltaV.dot(contactPoint.frictionVector1);

				// Compute the Lagrange multiplier lambda
				deltaLambda = -Jv;
				deltaLambda *= contactPoint.inverseFriction1Mass;
				float frictionLimit = contactManifold.frictionCoefficient *
						contactPoint.penetrationImpulse;
				lambdaTemp = contactPoint.friction1Impulse;
				contactPoint.friction1Impulse = etk::max(-frictionLimit,
														 etk::min(contactPoint.friction1Impulse
																  + deltaLambda, frictionLimit));
				deltaLambda = contactPoint.friction1Impulse - lambdaTemp;

				// Compute the impulse P=J^T * lambda
				const Impulse impulseFriction1 = computeFriction1Impulse(deltaLambda,
																		 contactPoint);

				// Apply the impulses to the bodies of the constraint
				applyImpulse(impulseFriction1, contactManifold);

				// --------- Friction 2 --------- //

				// Compute J*v
				deltaV = v2 + w2.cross(contactPoint.r2) - v1 - w1.cross(contactPoint.r1);
				Jv = deltaV.dot(contactPoint.frictionvec2);

				// Compute the Lagrange multiplier lambda
				deltaLambda = -Jv;
				deltaLambda *= contactPoint.inverseFriction2Mass;
				frictionLimit = contactManifold.frictionCoefficient *
						contactPoint.penetrationImpulse;
				lambdaTemp = contactPoint.friction2Impulse;
				contactPoint.friction2Impulse = etk::max(-frictionLimit,
														 etk::min(contactPoint.friction2Impulse
																  + deltaLambda, frictionLimit));
				deltaLambda = contactPoint.friction2Impulse - lambdaTemp;

				// Compute the impulse P=J^T * lambda
				const Impulse impulseFriction2 = computeFriction2Impulse(deltaLambda,
																		 contactPoint);

				// Apply the impulses to the bodies of the constraint
				applyImpulse(impulseFriction2, contactManifold);

				// --------- Rolling resistance constraint --------- //

				if (contactManifold.rollingResistanceFactor > 0) {

					// Compute J*v
					const vec3 JvRolling = w2 - w1;

					// Compute the Lagrange multiplier lambda
					vec3 deltaLambdaRolling = contactManifold.inverseRollingResistance * (-JvRolling);
					float rollingLimit = contactManifold.rollingResistanceFactor * contactPoint.penetrationImpulse;
					vec3 lambdaTempRolling = contactPoint.rollingResistanceImpulse;
					contactPoint.rollingResistanceImpulse = clamp(contactPoint.rollingResistanceImpulse +
																		 deltaLambdaRolling, rollingLimit);
					deltaLambdaRolling = contactPoint.rollingResistanceImpulse - lambdaTempRolling;

					// Compute the impulse P=J^T * lambda
					const Impulse impulseRolling(vec3(0.0f,0.0f,0.0f), -deltaLambdaRolling,
												 vec3(0.0f,0.0f,0.0f), deltaLambdaRolling);

					// Apply the impulses to the bodies of the constraint
					applyImpulse(impulseRolling, contactManifold);
				}
			}
		}

		// If we solve the friction constraints at the center of the contact manifold
		if (m_isSolveFrictionAtContactManifoldCenterActive) {

			// ------ First friction constraint at the center of the contact manifol ------ //

			// Compute J*v
			vec3 deltaV = v2 + w2.cross(contactManifold.r2Friction)
					- v1 - w1.cross(contactManifold.r1Friction);
			float Jv = deltaV.dot(contactManifold.frictionVector1);

			// Compute the Lagrange multiplier lambda
			float deltaLambda = -Jv * contactManifold.inverseFriction1Mass;
			float frictionLimit = contactManifold.frictionCoefficient * sum_penetrationImpulse;
			lambdaTemp = contactManifold.friction1Impulse;
			contactManifold.friction1Impulse = etk::max(-frictionLimit,
														etk::min(contactManifold.friction1Impulse +
																 deltaLambda, frictionLimit));
			deltaLambda = contactManifold.friction1Impulse - lambdaTemp;

			// Compute the impulse P=J^T * lambda
			vec3 linearImpulseBody1 = -contactManifold.frictionVector1 * deltaLambda;
			vec3 angularImpulseBody1 = -contactManifold.r1CrossT1 * deltaLambda;
			vec3 linearImpulseBody2 = contactManifold.frictionVector1 * deltaLambda;
			vec3 angularImpulseBody2 = contactManifold.r2CrossT1 * deltaLambda;
			const Impulse impulseFriction1(linearImpulseBody1, angularImpulseBody1,
										   linearImpulseBody2, angularImpulseBody2);

			// Apply the impulses to the bodies of the constraint
			applyImpulse(impulseFriction1, contactManifold);

			// ------ Second friction constraint at the center of the contact manifol ----- //

			// Compute J*v
			deltaV = v2 + w2.cross(contactManifold.r2Friction)
					- v1 - w1.cross(contactManifold.r1Friction);
			Jv = deltaV.dot(contactManifold.frictionvec2);

			// Compute the Lagrange multiplier lambda
			deltaLambda = -Jv * contactManifold.inverseFriction2Mass;
			frictionLimit = contactManifold.frictionCoefficient * sum_penetrationImpulse;
			lambdaTemp = contactManifold.friction2Impulse;
			contactManifold.friction2Impulse = etk::max(-frictionLimit,
														etk::min(contactManifold.friction2Impulse +
																 deltaLambda, frictionLimit));
			deltaLambda = contactManifold.friction2Impulse - lambdaTemp;

			// Compute the impulse P=J^T * lambda
			linearImpulseBody1 = -contactManifold.frictionvec2 * deltaLambda;
			angularImpulseBody1 = -contactManifold.r1CrossT2 * deltaLambda;
			linearImpulseBody2 = contactManifold.frictionvec2 * deltaLambda;
			angularImpulseBody2 = contactManifold.r2CrossT2 * deltaLambda;
			const Impulse impulseFriction2(linearImpulseBody1, angularImpulseBody1,
										   linearImpulseBody2, angularImpulseBody2);

			// Apply the impulses to the bodies of the constraint
			applyImpulse(impulseFriction2, contactManifold);

			// ------ Twist friction constraint at the center of the contact manifol ------ //

			// Compute J*v
			deltaV = w2 - w1;
			Jv = deltaV.dot(contactManifold.normal);

			deltaLambda = -Jv * (contactManifold.inverseTwistFrictionMass);
			frictionLimit = contactManifold.frictionCoefficient * sum_penetrationImpulse;
			lambdaTemp = contactManifold.frictionTwistImpulse;
			contactManifold.frictionTwistImpulse = etk::max(-frictionLimit,
															etk::min(contactManifold.frictionTwistImpulse
																	 + deltaLambda, frictionLimit));
			deltaLambda = contactManifold.frictionTwistImpulse - lambdaTemp;

			// Compute the impulse P=J^T * lambda
			linearImpulseBody1 = vec3(0.0, 0.0, 0.0);
			angularImpulseBody1 = -contactManifold.normal * deltaLambda;
			linearImpulseBody2 = vec3(0.0, 0.0, 0.0);;
			angularImpulseBody2 = contactManifold.normal * deltaLambda;
			const Impulse impulseTwistFriction(linearImpulseBody1, angularImpulseBody1,
											   linearImpulseBody2, angularImpulseBody2);

			// Apply the impulses to the bodies of the constraint
			applyImpulse(impulseTwistFriction, contactManifold);

			// --------- Rolling resistance constraint at the center of the contact manifold --------- //

			if (contactManifold.rollingResistanceFactor > 0) {

				// Compute J*v
				const vec3 JvRolling = w2 - w1;

				// Compute the Lagrange multiplier lambda
				vec3 deltaLambdaRolling = contactManifold.inverseRollingResistance * (-JvRolling);
				float rollingLimit = contactManifold.rollingResistanceFactor * sum_penetrationImpulse;
				vec3 lambdaTempRolling = contactManifold.rollingResistanceImpulse;
				contactManifold.rollingResistanceImpulse = clamp(contactManifold.rollingResistanceImpulse + deltaLambdaRolling,
				                                                 rollingLimit);
				deltaLambdaRolling = contactManifold.rollingResistanceImpulse - lambdaTempRolling;

				// Compute the impulse P=J^T * lambda
				angularImpulseBody1 = -deltaLambdaRolling;
				angularImpulseBody2 = deltaLambdaRolling;
				const Impulse impulseRolling(vec3(0.0f,0.0f,0.0f),
				                             angularImpulseBody1,
				                             vec3(0.0f,0.0f,0.0f),
				                             angularImpulseBody2);

				// Apply the impulses to the bodies of the constraint
				applyImpulse(impulseRolling, contactManifold);
			}
		}
	}
}

// Store the computed impulses to use them to
// warm start the solver at the next iteration
void ContactSolver::storeImpulses() {

	// For each contact manifold
	for (uint32_t c=0; c<m_numberContactManifolds; c++) {

		ContactManifoldSolver& manifold = m_contactConstraints[c];

		for (uint32_t i=0; i<manifold.nbContacts; i++) {

			ContactPointSolver& contactPoint = manifold.contacts[i];

			contactPoint.externalContact->setPenetrationImpulse(contactPoint.penetrationImpulse);
			contactPoint.externalContact->setFrictionImpulse1(contactPoint.friction1Impulse);
			contactPoint.externalContact->setFrictionImpulse2(contactPoint.friction2Impulse);
			contactPoint.externalContact->setRollingResistanceImpulse(contactPoint.rollingResistanceImpulse);

			contactPoint.externalContact->setFrictionVector1(contactPoint.frictionVector1);
			contactPoint.externalContact->setFrictionvec2(contactPoint.frictionvec2);
		}

		manifold.externalContactManifold->setFrictionImpulse1(manifold.friction1Impulse);
		manifold.externalContactManifold->setFrictionImpulse2(manifold.friction2Impulse);
		manifold.externalContactManifold->setFrictionTwistImpulse(manifold.frictionTwistImpulse);
		manifold.externalContactManifold->setRollingResistanceImpulse(manifold.rollingResistanceImpulse);
		manifold.externalContactManifold->setFrictionVector1(manifold.frictionVector1);
		manifold.externalContactManifold->setFrictionvec2(manifold.frictionvec2);
	}
}

// Apply an impulse to the two bodies of a constraint
void ContactSolver::applyImpulse(const Impulse& impulse,
								 const ContactManifoldSolver& manifold) {

	// Update the velocities of the body 1 by applying the impulse P
	m_linearVelocities[manifold.indexBody1] += manifold.massInverseBody1 *
											  impulse.linearImpulseBody1;
	m_angularVelocities[manifold.indexBody1] += manifold.inverseInertiaTensorBody1 *
											   impulse.angularImpulseBody1;

	// Update the velocities of the body 1 by applying the impulse P
	m_linearVelocities[manifold.indexBody2] += manifold.massInverseBody2 *
											  impulse.linearImpulseBody2;
	m_angularVelocities[manifold.indexBody2] += manifold.inverseInertiaTensorBody2 *
											   impulse.angularImpulseBody2;
}

// Apply an impulse to the two bodies of a constraint
void ContactSolver::applySplitImpulse(const Impulse& impulse,
									  const ContactManifoldSolver& manifold) {

	// Update the velocities of the body 1 by applying the impulse P
	m_splitLinearVelocities[manifold.indexBody1] += manifold.massInverseBody1 *
												   impulse.linearImpulseBody1;
	m_splitAngularVelocities[manifold.indexBody1] += manifold.inverseInertiaTensorBody1 *
													impulse.angularImpulseBody1;

	// Update the velocities of the body 1 by applying the impulse P
	m_splitLinearVelocities[manifold.indexBody2] += manifold.massInverseBody2 *
												   impulse.linearImpulseBody2;
	m_splitAngularVelocities[manifold.indexBody2] += manifold.inverseInertiaTensorBody2 *
													impulse.angularImpulseBody2;
}

// Compute the two unit orthogonal vectors "t1" and "t2" that span the tangential friction plane
// for a contact point. The two vectors have to be such that : t1 x t2 = contactNormal.
void ContactSolver::computeFrictionVectors(const vec3& deltaVelocity,
										   ContactPointSolver& contactPoint) const {

	assert(contactPoint.normal.length() > 0.0);

	// Compute the velocity difference vector in the tangential plane
	vec3 normalVelocity = deltaVelocity.dot(contactPoint.normal) * contactPoint.normal;
	vec3 tangentVelocity = deltaVelocity - normalVelocity;

	// If the velocty difference in the tangential plane is not zero
	float lengthTangenVelocity = tangentVelocity.length();
	if (lengthTangenVelocity > FLT_EPSILON) {

		// Compute the first friction vector in the direction of the tangent
		// velocity difference
		contactPoint.frictionVector1 = tangentVelocity / lengthTangenVelocity;
	}
	else {

		// Get any orthogonal vector to the normal as the first friction vector
		contactPoint.frictionVector1 = contactPoint.normal.getOrthoVector();
	}

	// The second friction vector is computed by the cross product of the firs
	// friction vector and the contact normal
	contactPoint.frictionvec2 =contactPoint.normal.cross(contactPoint.frictionVector1).safeNormalized();
}

// Compute the two unit orthogonal vectors "t1" and "t2" that span the tangential friction plane
// for a contact manifold. The two vectors have to be such that : t1 x t2 = contactNormal.
void ContactSolver::computeFrictionVectors(const vec3& deltaVelocity,
										   ContactManifoldSolver& contact) const {

	assert(contact.normal.length() > 0.0);

	// Compute the velocity difference vector in the tangential plane
	vec3 normalVelocity = deltaVelocity.dot(contact.normal) * contact.normal;
	vec3 tangentVelocity = deltaVelocity - normalVelocity;

	// If the velocty difference in the tangential plane is not zero
	float lengthTangenVelocity = tangentVelocity.length();
	if (lengthTangenVelocity > FLT_EPSILON) {

		// Compute the first friction vector in the direction of the tangent
		// velocity difference
		contact.frictionVector1 = tangentVelocity / lengthTangenVelocity;
	}
	else {

		// Get any orthogonal vector to the normal as the first friction vector
		contact.frictionVector1 = contact.normal.getOrthoVector();
	}

	// The second friction vector is computed by the cross product of the firs
	// friction vector and the contact normal
	contact.frictionvec2 = contact.normal.cross(contact.frictionVector1).safeNormalized();
}

// Clean up the constraint solver
void ContactSolver::cleanup() {

	if (m_contactConstraints != nullptr) {
		delete[] m_contactConstraints;
		m_contactConstraints = nullptr;
	}
}

// Set the split velocities arrays
void ContactSolver::setSplitVelocitiesArrays(vec3* splitLinearVelocities,
													vec3* splitAngularVelocities) {
	assert(splitLinearVelocities != NULL);
	assert(splitAngularVelocities != NULL);
	m_splitLinearVelocities = splitLinearVelocities;
	m_splitAngularVelocities = splitAngularVelocities;
}

// Set the constrained velocities arrays
void ContactSolver::setConstrainedVelocitiesArrays(vec3* constrainedLinearVelocities,
														  vec3* constrainedAngularVelocities) {
	assert(constrainedLinearVelocities != NULL);
	assert(constrainedAngularVelocities != NULL);
	m_linearVelocities = constrainedLinearVelocities;
	m_angularVelocities = constrainedAngularVelocities;
}

// Return true if the split impulses position correction technique is used for contacts
bool ContactSolver::isSplitImpulseActive() const {
	return m_isSplitImpulseActive;
}

// Activate or Deactivate the split impulses for contacts
void ContactSolver::setIsSplitImpulseActive(bool isActive) {
	m_isSplitImpulseActive = isActive;
}

// Activate or deactivate the solving of friction constraints at the center of
// the contact manifold instead of solving them at each contact point
void ContactSolver::setIsSolveFrictionAtContactManifoldCenterActive(bool isActive) {
	m_isSolveFrictionAtContactManifoldCenterActive = isActive;
}

// Compute the collision restitution factor from the restitution factor of each body
float ContactSolver::computeMixedRestitutionFactor(RigidBody* body1,
															RigidBody* body2) const {
	float restitution1 = body1->getMaterial().getBounciness();
	float restitution2 = body2->getMaterial().getBounciness();

	// Return the largest restitution factor
	return (restitution1 > restitution2) ? restitution1 : restitution2;
}

// Compute the mixed friction coefficient from the friction coefficient of each body
float ContactSolver::computeMixedFrictionCoefficient(RigidBody *body1,
															  RigidBody *body2) const {
	// Use the geometric mean to compute the mixed friction coefficient
	return sqrt(body1->getMaterial().getFrictionCoefficient() *
				body2->getMaterial().getFrictionCoefficient());
}

// Compute th mixed rolling resistance factor between two bodies
float ContactSolver::computeMixedRollingResistance(RigidBody* body1,
															RigidBody* body2) const {
	return float(0.5f) * (body1->getMaterial().getRollingResistance() + body2->getMaterial().getRollingResistance());
}

// Compute a penetration constraint impulse
const Impulse ContactSolver::computePenetrationImpulse(float deltaLambda,
														  const ContactPointSolver& contactPoint)
														  const {
	return Impulse(-contactPoint.normal * deltaLambda, -contactPoint.r1CrossN * deltaLambda,
				   contactPoint.normal * deltaLambda, contactPoint.r2CrossN * deltaLambda);
}

// Compute the first friction constraint impulse
const Impulse ContactSolver::computeFriction1Impulse(float deltaLambda,
														const ContactPointSolver& contactPoint)
														const {
	return Impulse(-contactPoint.frictionVector1 * deltaLambda,
				   -contactPoint.r1CrossT1 * deltaLambda,
				   contactPoint.frictionVector1 * deltaLambda,
				   contactPoint.r2CrossT1 * deltaLambda);
}

// Compute the second friction constraint impulse
const Impulse ContactSolver::computeFriction2Impulse(float deltaLambda,
														const ContactPointSolver& contactPoint)
														const {
	return Impulse(-contactPoint.frictionvec2 * deltaLambda,
				   -contactPoint.r1CrossT2 * deltaLambda,
				   contactPoint.frictionvec2 * deltaLambda,
				   contactPoint.r2CrossT2 * deltaLambda);
}
