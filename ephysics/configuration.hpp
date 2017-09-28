/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <etk/types.hpp>
#include <etk/Pair.hpp>

/// Namespace ephysics
namespace ephysics {
	// ------------------- Type definitions ------------------- //
	typedef uint64_t bodyindex;
	typedef etk::Pair<bodyindex, bodyindex> bodyindexpair;
	
	// ------------------- Enumerations ------------------- //
	
	/// Position correction technique used in the constraint solver (for joints).
	/// BAUMGARTE_JOINTS : Faster but can be innacurate in some situations.
	/// NON_LINEAR_GAUSS_SEIDEL : Slower but more precise. This is the option used by default.
	enum JointsPositionCorrectionTechnique {BAUMGARTE_JOINTS, NON_LINEAR_GAUSS_SEIDEL};
	
	/// Position correction technique used in the contact solver (for contacts)
	/// BAUMGARTE_CONTACTS : Faster but can be innacurate and can lead to unexpected bounciness
	///                      in some situations (due to error correction factor being added to
	///                      the bodies momentum).
	/// SPLIT_IMPULSES : A bit slower but the error correction factor is not added to the
	///                 bodies momentum. This is the option used by default.
	enum ContactsPositionCorrectionTechnique {BAUMGARTE_CONTACTS, SPLIT_IMPULSES};
	
	// ------------------- Constants ------------------- //
	/// Pi constant
	const float PI = float(3.14159265);
	
	/// 2*Pi constant
	const float PI_TIMES_2 = float(6.28318530);
	
	/// Default friction coefficient for a rigid body
	const float DEFAULT_FRICTION_COEFFICIENT = float(0.3);
	
	/// Default bounciness factor for a rigid body
	const float DEFAULT_BOUNCINESS = 0.5f;
	
	/// Default rolling resistance
	const float DEFAULT_ROLLING_RESISTANCE = 0.0f;
	
	/// True if the spleeping technique is enabled
	const bool SPLEEPING_ENABLED = true;
	
	/// Object margin for collision detection in meters (for the GJK-EPA Algorithm)
	const float OBJECT_MARGIN = float(0.04);
	
	/// Distance threshold for two contact points for a valid persistent contact (in meters)
	const float PERSISTENT_CONTACT_DIST_THRESHOLD = float(0.03);
	
	/// Velocity threshold for contact velocity restitution
	const float RESTITUTION_VELOCITY_THRESHOLD = 1.0f;
	
	/// Number of iterations when solving the velocity constraints of the Sequential Impulse technique
	const uint32_t DEFAULT_VELOCITY_SOLVER_NB_ITERATIONS = 10;
	
	/// Number of iterations when solving the position constraints of the Sequential Impulse technique
	const uint32_t DEFAULT_POSITION_SOLVER_NB_ITERATIONS = 5;
	
	/// Time (in seconds) that a body must stay still to be considered sleeping
	const float DEFAULT_TIME_BEFORE_SLEEP = 1.0f;
	
	/// A body with a linear velocity smaller than the sleep linear velocity (in m/s)
	/// might enter sleeping mode.
	const float DEFAULT_SLEEP_LINEAR_VELOCITY = float(0.02);
	
	/// A body with angular velocity smaller than the sleep angular velocity (in rad/s)
	/// might enter sleeping mode
	const float DEFAULT_SLEEP_ANGULAR_VELOCITY = float(3.0 * (PI / 180.0));
	
	/// In the broad-phase collision detection (dynamic AABB tree), the AABBs are
	/// inflated with a constant gap to allow the collision shape to move a little bit
	/// without triggering a large modification of the tree which can be costly
	const float DYNAMIC_TREE_AABB_GAP = float(0.1);
	
	/// In the broad-phase collision detection (dynamic AABB tree), the AABBs are
	/// also inflated in direction of the linear motion of the body by mutliplying the
	/// followin constant with the linear velocity and the elapsed time between two frames.
	const float DYNAMIC_TREE_AABB_LIN_GAP_MULTIPLIER = float(1.7);
	
	/// Maximum number of contact manifolds in an overlapping pair that involves two
	/// convex collision shapes.
	const int32_t NB_MAX_CONTACT_MANIFOLDS_CONVEX_SHAPE = 1;
	
	/// Maximum number of contact manifolds in an overlapping pair that involves at
	/// least one concave collision shape.
	const int32_t NB_MAX_CONTACT_MANIFOLDS_CONCAVE_SHAPE = 3;

}
