/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/shapes/CollisionShape.hpp>
#include <ephysics/engine/Profiler.hpp>
#include <ephysics/body/CollisionBody.hpp>

// We want to use the ReactPhysics3D namespace
using namespace ephysics;

CollisionShape::CollisionShape(CollisionShapeType type) :
  m_type(type),
  m_scaling(1.0f, 1.0f, 1.0f) {
	
}

CollisionShape::~CollisionShape() {
	
}

// Compute the world-space AABB of the collision shape given a transform
/**
 * @param[out] aabb The axis-aligned bounding box (AABB) of the collision shape
 *				  computed in world-space coordinates
 * @param transform etk::Transform3D used to compute the AABB of the collision shape
 */
void CollisionShape::computeAABB(AABB& aabb, const etk::Transform3D& transform) const {

	PROFILE("CollisionShape::computeAABB()");

	// Get the local bounds in x,y and z direction
	vec3 minBounds(0,0,0);
	vec3 maxBounds(0,0,0);
	getLocalBounds(minBounds, maxBounds);

	// Rotate the local bounds according to the orientation of the body
	etk::Matrix3x3 worldAxis = transform.getOrientation().getMatrix().getAbsolute();
	vec3 worldMinBounds(worldAxis.getColumn(0).dot(minBounds),
	                    worldAxis.getColumn(1).dot(minBounds),
	                    worldAxis.getColumn(2).dot(minBounds));
	vec3 worldMaxBounds(worldAxis.getColumn(0).dot(maxBounds),
	                    worldAxis.getColumn(1).dot(maxBounds),
	                    worldAxis.getColumn(2).dot(maxBounds));

	// Compute the minimum and maximum coordinates of the rotated extents
	vec3 minCoordinates = transform.getPosition() + worldMinBounds;
	vec3 maxCoordinates = transform.getPosition() + worldMaxBounds;

	// Update the AABB with the new minimum and maximum coordinates
	aabb.setMin(minCoordinates);
	aabb.setMax(maxCoordinates);
}

int32_t CollisionShape::computeNbMaxContactManifolds(CollisionShapeType shapeType1,
														CollisionShapeType shapeType2) {
	// If both shapes are convex
	if (isConvex(shapeType1) && isConvex(shapeType2)) {
		return NB_MAX_CONTACT_MANIFOLDS_CONVEX_SHAPE;
	}   // If there is at least one concave shape
	else {
		return NB_MAX_CONTACT_MANIFOLDS_CONCAVE_SHAPE;
	}
}