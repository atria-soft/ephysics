/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/engine/ConstraintSolver.h>
#include <ephysics/engine/Profiler.h>

using namespace reactphysics3d;

// Constructor
ConstraintSolver::ConstraintSolver(const std::map<RigidBody*, uint32_t>& mapBodyToVelocityIndex)
				 : mMapBodyToConstrainedVelocityIndex(mapBodyToVelocityIndex),
				   mIsWarmStartingActive(true), mConstraintSolverData(mapBodyToVelocityIndex) {

}

// Destructor
ConstraintSolver::~ConstraintSolver() {

}

// Initialize the constraint solver for a given island
void ConstraintSolver::initializeForIsland(float dt, Island* island) {

	PROFILE("ConstraintSolver::initializeForIsland()");

	assert(island != NULL);
	assert(island->getNbBodies() > 0);
	assert(island->getNbJoints() > 0);

	// Set the current time step
	mTimeStep = dt;

	// Initialize the constraint solver data used to initialize and solve the constraints
	mConstraintSolverData.timeStep = mTimeStep;
	mConstraintSolverData.isWarmStartingActive = mIsWarmStartingActive;

	// For each joint of the island
	Joint** joints = island->getJoints();
	for (uint32_t i=0; i<island->getNbJoints(); i++) {

		// Initialize the constraint before solving it
		joints[i]->initBeforeSolve(mConstraintSolverData);

		// Warm-start the constraint if warm-starting is enabled
		if (mIsWarmStartingActive) {
			joints[i]->warmstart(mConstraintSolverData);
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
		joints[i]->solveVelocityConstraint(mConstraintSolverData);
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
		joints[i]->solvePositionConstraint(mConstraintSolverData);
	}
}
