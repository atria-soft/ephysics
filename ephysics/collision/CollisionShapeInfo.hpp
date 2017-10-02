/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
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


