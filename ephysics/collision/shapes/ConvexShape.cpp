/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/shapes/ConvexShape.hpp>


// We want to use the ReactPhysics3D namespace
using namespace ephysics;

// Constructor
ConvexShape::ConvexShape(CollisionShapeType type, float margin)
			: CollisionShape(type), m_margin(margin) {

}

// Destructor
ConvexShape::~ConvexShape() {

}

// Return a local support point in a given direction with the object margin
vec3 ConvexShape::getLocalSupportPointWithMargin(const vec3& direction,
													void** cachedCollisionData) const {

	// Get the support point without margin
	vec3 supportPoint = getLocalSupportPointWithoutMargin(direction, cachedCollisionData);

	if (m_margin != 0.0f) {

		// Add the margin to the support point
		vec3 unitVec(0.0, -1.0, 0.0);
		if (direction.length2() > FLT_EPSILON * FLT_EPSILON) {
			unitVec = direction.safeNormalized();
		}
		supportPoint += unitVec * m_margin;
	}

	return supportPoint;
}
