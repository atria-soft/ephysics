/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

#include <ephysics/engine/ConstraintSolver.hpp>
#include <ephysics/engine/Profiler.hpp>

using namespace ephysics;

// Constructor
ConstraintSolver::ConstraintSolver(const std::map<RigidBody*, uint32_t>& mapBodyToVelocityIndex)
				 : m_mapBodyToConstrainedVelocityIndex(mapBodyToVelocityIndex),
				   m_isWarmStartingActive(true), m_constraintSolverData(mapBodyToVelocityIndex) {

}

// Initialize the constraint solver for a given island
void ConstraintSolver::initializeForIsland(float dt, Island* island) {

	PROFILE("ConstraintSolver::initializeForIsland()");

	assert(island != NULL);
	assert(island->getNbBodies() > 0);
	assert(island->getNbJoints() > 0);

	// Set the current time step
	m_timeStep = dt;

	// Initialize the constraint solver data used to initialize and solve the constraints
	m_constraintSolverData.timeStep = m_timeStep;
	m_constraintSolverData.isWarmStartingActive = m_isWarmStartingActive;

	// For each joint of the island
	Joint** joints = island->getJoints();
	for (uint32_t i=0; i<island->getNbJoints(); i++) {

		// Initialize the constraint before solving it
		joints[i]->initBeforeSolve(m_constraintSolverData);

		// Warm-start the constraint if warm-starting is enabled
		if (m_isWarmStartingActive) {
			joints[i]->warmstart(m_constraintSolverData);
		}
	}
}

// Solve the velocity constraints
void ConstraintSolver::solveVelocityConstraints(Island* island) {

	PROFILE("ConstraintSolver::solveVelocityConstraints()");

	assert(island != NULL);
	assert(island->getNbJoints() > 0);

	// For each joint of the island
	Joint** joints = island->getJoints();
	for (uint32_t i=0; i<island->getNbJoints(); i++) {

		// Solve the constraint
		joints[i]->solveVelocityConstraint(m_constraintSolverData);
	}
}

// Solve the position constraints
void ConstraintSolver::solvePositionConstraints(Island* island) {

	PROFILE("ConstraintSolver::solvePositionConstraints()");

	assert(island != NULL);
	assert(island->getNbJoints() > 0);

	// For each joint of the island
	Joint** joints = island->getJoints();
	for (uint32_t i=0; i < island->getNbJoints(); i++) {

		// Solve the constraint
		joints[i]->solvePositionConstraint(m_constraintSolverData);
	}
}
