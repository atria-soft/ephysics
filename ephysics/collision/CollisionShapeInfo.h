/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/collision/shapes/CollisionShape.h>

/// Namespace ReactPhysics3D
namespace reactphysics3d {

class OverlappingPair;

// Class CollisionShapeInfo
/**
 * This structure regroups different things about a collision shape. This is
 * used to pass information about a collision shape to a collision algorithm.
 */
struct CollisionShapeInfo {

	public:

		/// Broadphase overlapping pair
		OverlappingPair* overlappingPair;

		/// Proxy shape
		ProxyShape* proxyShape;

		/// Pointer to the collision shape
		const CollisionShape* collisionShape;

		/// Transform that maps from collision shape local-space to world-space
		Transform shapeToWorldTransform;

		/// Cached collision data of the proxy shape
		void** cachedCollisionData;

		/// Constructor
		CollisionShapeInfo(ProxyShape* proxyCollisionShape, const CollisionShape* shape,
						   const Transform& shapeLocalToWorldTransform, OverlappingPair* pair,
						   void** cachedData)
			  : overlappingPair(pair), proxyShape(proxyCollisionShape), collisionShape(shape),
				shapeToWorldTransform(shapeLocalToWorldTransform),
				cachedCollisionData(cachedData) {

		}
};

}


