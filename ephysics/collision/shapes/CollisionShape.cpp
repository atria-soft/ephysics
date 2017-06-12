/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/shapes/CollisionShape.h>
#include <ephysics/engine/Profiler.h>
#include <ephysics/body/CollisionBody.h>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
CollisionShape::CollisionShape(CollisionShapeType type) : m_type(type), mScaling(1.0, 1.0, 1.0) {
	
}

// Destructor
CollisionShape::~CollisionShape() {

}

// Compute the world-space AABB of the collision shape given a transform
/**
 * @param[out] aabb The axis-aligned bounding box (AABB) of the collision shape
 *				  computed in world-space coordinates
 * @param transform Transform used to compute the AABB of the collision shape
 */
void CollisionShape::computeAABB(AABB& aabb, const Transform& transform) const {

	PROFILE("CollisionShape::computeAABB()");

	// Get the local bounds in x,y and z direction
	Vector3 minBounds;
	Vector3 maxBounds;
	getLocalBounds(minBounds, maxBounds);

	// Rotate the local bounds according to the orientation of the body
	Matrix3x3 worldAxis = transform.getOrientation().getMatrix().getAbsoluteMatrix();
	Vector3 worldMinBounds(worldAxis.getColumn(0).dot(minBounds),
						   worldAxis.getColumn(1).dot(minBounds),
						   worldAxis.getColumn(2).dot(minBounds));
	Vector3 worldMaxBounds(worldAxis.getColumn(0).dot(maxBounds),
						   worldAxis.getColumn(1).dot(maxBounds),
						   worldAxis.getColumn(2).dot(maxBounds));

	// Compute the minimum and maximum coordinates of the rotated extents
	Vector3 minCoordinates = transform.getPosition() + worldMinBounds;
	Vector3 maxCoordinates = transform.getPosition() + worldMaxBounds;

	// Update the AABB with the new minimum and maximum coordinates
	aabb.setMin(minCoordinates);
	aabb.setMax(maxCoordinates);
}
