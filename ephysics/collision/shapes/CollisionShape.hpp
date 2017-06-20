/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

#include <cassert>
#include <typeinfo>
#include <etk/math/Vector3D.hpp>
#include <etk/math/Matrix3x3.hpp>
#include <ephysics/mathematics/Ray.hpp>
#include <ephysics/collision/shapes/AABB.hpp>
#include <ephysics/collision/RaycastInfo.hpp>
#include <ephysics/memory/MemoryAllocator.hpp>

namespace ephysics {
enum CollisionShapeType {TRIANGLE, BOX, SPHERE, CONE, CYLINDER,
						 CAPSULE, CONVEX_MESH, CONCAVE_MESH, HEIGHTFIELD};
const int32_t NB_COLLISION_SHAPE_TYPES = 9;

class ProxyShape;
class CollisionBody;

/**
 * This abstract class represents the collision shape associated with a
 * body that is used during the narrow-phase collision detection.
 */
class CollisionShape {
	protected :
		CollisionShapeType m_type; //!< Type of the collision shape
		vec3 m_scaling; //!< Scaling vector of the collision shape
		/// Private copy-constructor
		CollisionShape(const CollisionShape& shape) = delete;
		/// Private assignment operator
		CollisionShape& operator=(const CollisionShape& shape) = delete;
		/// Return true if a point is inside the collision shape
		virtual bool testPointInside(const vec3& worldPoint, ProxyShape* proxyShape) const=0;
		/// Raycast method with feedback information
		virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const=0;
		/// Return the number of bytes used by the collision shape
		virtual size_t getSizeInBytes() const = 0;
	public :
		/// Constructor
		CollisionShape(CollisionShapeType type);
		/// Destructor
		virtual ~CollisionShape();
		/// Return the type of the collision shapes
		CollisionShapeType getType() const;
		/// Return true if the collision shape is convex, false if it is concave
		virtual bool isConvex() const=0;
		/// Return the local bounds of the shape in x, y and z directions
		virtual void getLocalBounds(vec3& min, vec3& max) const=0;
		/// Return the scaling vector of the collision shape
		vec3 getScaling() const;
		/// Set the local scaling vector of the collision shape
		virtual void setLocalScaling(const vec3& scaling);
		/// Return the local inertia tensor of the collision shapes
		virtual void computeLocalInertiaTensor(etk::Matrix3x3& tensor, float mass) const=0;
		/// Compute the world-space AABB of the collision shape given a transform
		virtual void computeAABB(AABB& aabb, const etk::Transform3D& transform) const;
		/// Return true if the collision shape type is a convex shape
		static bool isConvex(CollisionShapeType shapeType);
		/// Return the maximum number of contact manifolds in an overlapping pair given two shape types
		static int32_t computeNbMaxContactManifolds(CollisionShapeType shapeType1,
												CollisionShapeType shapeType2);
		friend class ProxyShape;
		friend class CollisionWorld;
};

// Return the type of the collision shape
/**
 * @return The type of the collision shape (box, sphere, cylinder, ...)
 */
CollisionShapeType CollisionShape::getType() const {
	return m_type;
}

// Return true if the collision shape type is a convex shape
bool CollisionShape::isConvex(CollisionShapeType shapeType) {
	return shapeType != CONCAVE_MESH && shapeType != HEIGHTFIELD;
}

// Return the scaling vector of the collision shape
vec3 CollisionShape::getScaling() const {
	return m_scaling;
}

// Set the scaling vector of the collision shape
void CollisionShape::setLocalScaling(const vec3& scaling) {
	m_scaling = scaling;
}

// Return the maximum number of contact manifolds allowed in an overlapping
// pair wit the given two collision shape types
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

}

