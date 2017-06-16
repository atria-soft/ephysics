/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/collision/shapes/CollisionShape.hpp>

/// Namespace ReactPhysics3D
namespace ephysics {

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

		/// etk::Transform3D that maps from collision shape local-space to world-space
		etk::Transform3D shapeToWorldTransform;

		/// Cached collision data of the proxy shape
		void** cachedCollisionData;

		/// Constructor
		CollisionShapeInfo(ProxyShape* proxyCollisionShape, const CollisionShape* shape,
						   const etk::Transform3D& shapeLocalToWorldTransform, OverlappingPair* pair,
						   void** cachedData)
			  : overlappingPair(pair), proxyShape(proxyCollisionShape), collisionShape(shape),
				shapeToWorldTransform(shapeLocalToWorldTransform),
				cachedCollisionData(cachedData) {

		}
};

}


