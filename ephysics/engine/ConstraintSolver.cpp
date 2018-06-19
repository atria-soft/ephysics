/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#include <ephysics/engine/ConstraintSolver.hpp>
#include <ephysics/engine/Profiler.hpp>

using namespace ephysics;

ConstraintSolver::ConstraintSolver(const etk::Map<RigidBody*, uint32_t>& _mapBodyToVelocityIndex):
  m_mapBodyToConstrainedVelocityIndex(_mapBodyToVelocityIndex),
  m_isWarmStartingActive(true),
  m_constraintSolverData(_mapBodyToVelocityIndex) {
	
}

void ConstraintSolver::initializeForIsland(float _dt, Island* _island) {
	PROFILE("ConstraintSolver::initializeForIsland()");
	assert(_island != null);
	assert(_island->getNbBodies() > 0);
	assert(_island->getNbJoints() > 0);
	// Set the current time step
	m_timeStep = _dt;
	// Initialize the constraint solver data used to initialize and solve the constraints
	m_constraintSolverData.timeStep = m_timeStep;
	m_constraintSolverData.isWarmStartingActive = m_isWarmStartingActive;
	// For each joint of the island
	Joint** joints = _island->getJoints();
	for (uint32_t iii=0; iii<_island->getNbJoints(); ++iii) {
		// Initialize the constraint before solving it
		joints[iii]->initBeforeSolve(m_constraintSolverData);
		// Warm-start the constraint if warm-starting is enabled
		if (m_isWarmStartingActive) {
			joints[iii]->warmstart(m_constraintSolverData);
		}
	}
}

void ConstraintSolver::solveVelocityConstraints(Island* _island) {
	PROFILE("ConstraintSolver::solveVelocityConstraints()");
	assert(_island != null);
	assert(_island->getNbJoints() > 0);
	// For each joint of the island
	Joint** joints = _island->getJoints();
	for (uint32_t iii=0; iii<_island->getNbJoints(); ++iii) {
		joints[iii]->solveVelocityConstraint(m_constraintSolverData);
	}
}

void ConstraintSolver::solvePositionConstraints(Island* _island) {
	PROFILE("ConstraintSolver::solvePositionConstraints()");
	assert(_island != null);
	assert(_island->getNbJoints() > 0);
	Joint** joints = _island->getJoints();
	for (uint32_t iii=0; iii < _island->getNbJoints(); ++iii) {
		joints[iii]->solvePositionConstraint(m_constraintSolverData);
	}
}

void ConstraintSolver::setConstrainedVelocitiesArrays(vec3* _constrainedLinearVelocities,
                                                      vec3* _constrainedAngularVelocities) {
	assert(_constrainedLinearVelocities != null);
	assert(_constrainedAngularVelocities != null);
	m_constraintSolverData.linearVelocities = _constrainedLinearVelocities;
	m_constraintSolverData.angularVelocities = _constrainedAngularVelocities;
}

void ConstraintSolver::setConstrainedPositionsArrays(vec3* _constrainedPositions,
                                                     etk::Quaternion* _constrainedOrientations) {
	assert(_constrainedPositions != null);
	assert(_constrainedOrientations != null);
	m_constraintSolverData.positions = _constrainedPositions;
	m_constraintSolverData.orientations = _constrainedOrientations;
}