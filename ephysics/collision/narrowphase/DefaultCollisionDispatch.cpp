/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/narrowphase/DefaultCollisionDispatch.hpp>
#include <ephysics/collision/shapes/CollisionShape.hpp>

using namespace ephysics;

// Constructor
DefaultCollisionDispatch::DefaultCollisionDispatch() {

}

/// Initialize the collision dispatch configuration
void DefaultCollisionDispatch::init(CollisionDetection* collisionDetection,
									MemoryAllocator* memoryAllocator) {

	// Initialize the collision algorithms
	mSphereVsSphereAlgorithm.init(collisionDetection, memoryAllocator);
	mGJKAlgorithm.init(collisionDetection, memoryAllocator);
	mConcaveVsConvexAlgorithm.init(collisionDetection, memoryAllocator);
}

// Select and return the narrow-phase collision detection algorithm to
// use between two types of collision shapes.
NarrowPhaseAlgorithm* DefaultCollisionDispatch::selectAlgorithm(int32_t type1, int32_t type2) {
	CollisionShapeType shape1Type = static_cast<CollisionShapeType>(type1);
	CollisionShapeType shape2Type = static_cast<CollisionShapeType>(type2);
	// Sphere vs Sphere algorithm
	if (shape1Type == SPHERE && shape2Type == SPHERE) {
		return &mSphereVsSphereAlgorithm;
	} else if (    (!CollisionShape::isConvex(shape1Type) && CollisionShape::isConvex(shape2Type))
	            || (!CollisionShape::isConvex(shape2Type) && CollisionShape::isConvex(shape1Type))) {
		// Concave vs Convex algorithm
		return &mConcaveVsConvexAlgorithm;
	} else if (CollisionShape::isConvex(shape1Type) && CollisionShape::isConvex(shape2Type)) {
		// Convex vs Convex algorithm (GJK algorithm)
		return &mGJKAlgorithm;
	} else {
		return nullptr;
	}
}
