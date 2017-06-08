/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/shapes/ConvexShape.h>


// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
ConvexShape::ConvexShape(CollisionShapeType type, float margin)
			: CollisionShape(type), mMargin(margin) {

}

// Destructor
ConvexShape::~ConvexShape() {

}

// Return a local support point in a given direction with the object margin
Vector3 ConvexShape::getLocalSupportPointWithMargin(const Vector3& direction,
													void** cachedCollisionData) const {

	// Get the support point without margin
	Vector3 supportPoint = getLocalSupportPointWithoutMargin(direction, cachedCollisionData);

	if (mMargin != float(0.0)) {

		// Add the margin to the support point
		Vector3 unitVec(0.0, -1.0, 0.0);
		if (direction.lengthSquare() > MACHINE_EPSILON * MACHINE_EPSILON) {
			unitVec = direction.getUnit();
		}
		supportPoint += unitVec * mMargin;
	}

	return supportPoint;
}
