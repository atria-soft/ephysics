/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

#include <ephysics/collision/shapes/CollisionShape.hpp>

namespace ephysics {
	class OverlappingPair;
	/**
	 * @brief It regroups different things about a collision shape. This is
	 * used to pass information about a collision shape to a collision algorithm.
	 */
	struct CollisionShapeInfo {
		public:
			OverlappingPair* overlappingPair; //!< Broadphase overlapping pair
			ProxyShape* proxyShape; //!< Proxy shape
			const CollisionShape* collisionShape; //!< Pointer to the collision shape
			etk::Transform3D shapeToWorldTransform; //!< etk::Transform3D that maps from collision shape local-space to world-space
			void** cachedCollisionData; //!< Cached collision data of the proxy shape
			/// Constructor
			CollisionShapeInfo(ProxyShape* _proxyCollisionShape,
			                   const CollisionShape* _shape,
			                   const etk::Transform3D& _shapeLocalToWorldTransform,
			                   OverlappingPair* _pair,
			                   void** _cachedData):
			  overlappingPair(_pair),
			  proxyShape(_proxyCollisionShape),
			  collisionShape(_shape),
			  shapeToWorldTransform(_shapeLocalToWorldTransform),
			  cachedCollisionData(_cachedData) {
				
			}
	};
}


