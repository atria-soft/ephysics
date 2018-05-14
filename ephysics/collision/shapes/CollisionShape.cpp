/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#include <ephysics/collision/shapes/CollisionShape.hpp>
#include <ephysics/engine/Profiler.hpp>
#include <ephysics/body/CollisionBody.hpp>

// We want to use the ReactPhysics3D namespace
using namespace ephysics;

CollisionShape::CollisionShape(CollisionShapeType type) :
  m_type(type),
  m_scaling(1.0f, 1.0f, 1.0f) {
	
}

void CollisionShape::computeAABB(AABB& _aabb, const etk::Transform3D& _transform) const {
	PROFILE("CollisionShape::computeAABB()");
	// Get the local bounds in x,y and z direction
	vec3 minBounds(0,0,0);
	vec3 maxBounds(0,0,0);
	getLocalBounds(minBounds, maxBounds);
	// Rotate the local bounds according to the orientation of the body
	etk::Matrix3x3 worldAxis = _transform.getOrientation().getMatrix().getAbsolute();
	vec3 worldMinBounds(worldAxis.getColumn(0).dot(minBounds),
	                    worldAxis.getColumn(1).dot(minBounds),
	                    worldAxis.getColumn(2).dot(minBounds));
	vec3 worldMaxBounds(worldAxis.getColumn(0).dot(maxBounds),
	                    worldAxis.getColumn(1).dot(maxBounds),
	                    worldAxis.getColumn(2).dot(maxBounds));
	// Compute the minimum and maximum coordinates of the rotated extents
	vec3 minCoordinates = _transform.getPosition() + worldMinBounds;
	vec3 maxCoordinates = _transform.getPosition() + worldMaxBounds;
	// Update the AABB with the new minimum and maximum coordinates
	_aabb.setMin(minCoordinates);
	_aabb.setMax(maxCoordinates);
}

int32_t CollisionShape::computeNbMaxContactManifolds(CollisionShapeType _shapeType1,
                                                     CollisionShapeType _shapeType2) {
	// If both shapes are convex
	if (isConvex(_shapeType1) && isConvex(_shapeType2)) {
		return NB_MAX_CONTACT_MANIFOLDS_CONVEX_SHAPE;
	}
	// If there is at least one concave shape
	return NB_MAX_CONTACT_MANIFOLDS_CONCAVE_SHAPE;
}

