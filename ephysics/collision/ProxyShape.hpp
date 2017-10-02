/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#pragma once

#include <ephysics/body/CollisionBody.hpp>
#include <ephysics/collision/shapes/CollisionShape.hpp>

namespace  ephysics {
	/**
	 * @breif The CollisionShape instances are supposed to be unique for memory optimization. For instance,
	 * consider two rigid bodies with the same sphere collision shape. In this situation, we will have
	 * a unique instance of SphereShape but we need to differentiate between the two instances during
	 * the collision detection. They do not have the same position in the world and they do not
	 * belong to the same rigid body. The ProxyShape class is used for that purpose by attaching a
	 * rigid body with one of its collision shape. A body can have multiple proxy shapes (one for
	 * each collision shape attached to the body).
	 */
	class ProxyShape {
		protected:
			CollisionBody* m_body; //!< Pointer to the parent body
			CollisionShape* m_collisionShape; //!< Internal collision shape
			etk::Transform3D m_localToBodyTransform; //!< Local-space to parent body-space transform (does not change over time)
			float m_mass; //!< Mass (in kilogramms) of the corresponding collision shape
			ProxyShape* m_next; //!< Pointer to the next proxy shape of the body (linked list)
			int32_t m_broadPhaseID; //!< Broad-phase ID (node ID in the dynamic AABB tree)
			void* m_cachedCollisionData; //!< Cached collision data
			void* m_userData; //!< Pointer to user data
			/**
			 * @brief Bits used to define the collision category of this shape.
			 * You can set a single bit to one to define a category value for this
			 * shape. This value is one (0x0001) by default. This variable can be used
			 * together with the m_collideWithMaskBits variable so that given
			 * categories of shapes collide with each other and do not collide with
			 * other categories.
			 */
			unsigned short m_collisionCategoryBits;
			/**
			 * @brief Bits mask used to state which collision categories this shape can
			 * collide with. This value is 0xFFFF by default. It means that this
			 * proxy shape will collide with every collision categories by default.
			 */
			unsigned short m_collideWithMaskBits;
			/// Private copy-constructor
			ProxyShape(const ProxyShape&) = delete;
			/// Private assignment operator
			ProxyShape& operator=(const ProxyShape&) = delete;
		public:
			/// Constructor
			ProxyShape(CollisionBody* _body,
			           CollisionShape* _shape,
			           const etk::Transform3D& _transform,
			           float _mass);
	
			/// Destructor
			virtual ~ProxyShape();
	
			/// Return the collision shape
			const CollisionShape* getCollisionShape() const;
	
			/// Return the parent body
			CollisionBody* getBody() const;
	
			/// Return the mass of the collision shape
			float getMass() const;
	
			/// Return a pointer to the user data attached to this body
			void* getUserData() const;
	
			/// Attach user data to this body
			void setUserData(void* _userData);
	
			/// Return the local to parent body transform
			const etk::Transform3D& getLocalToBodyTransform() const;
	
			/// Set the local to parent body transform
			void setLocalToBodyTransform(const etk::Transform3D& _transform);
	
			/// Return the local to world transform
			const etk::Transform3D getLocalToWorldTransform() const;
	
			/// Return true if a point is inside the collision shape
			bool testPointInside(const vec3& _worldPoint);
	
			/// Raycast method with feedback information
			bool raycast(const Ray& _ray, RaycastInfo& _raycastInfo);
	
			/// Return the collision bits mask
			unsigned short getCollideWithMaskBits() const;
	
			/// Set the collision bits mask
			void setCollideWithMaskBits(unsigned short _collideWithMaskBits);
	
			/// Return the collision category bits
			unsigned short getCollisionCategoryBits() const;
	
			/// Set the collision category bits
			void setCollisionCategoryBits(unsigned short _collisionCategoryBits);
	
			/// Return the next proxy shape in the linked list of proxy shapes
			ProxyShape* getNext();
	
			/// Return the next proxy shape in the linked list of proxy shapes
			const ProxyShape* getNext() const;
	
			/// Return the pointer to the cached collision data
			void** getCachedCollisionData();
	
			/// Return the local scaling vector of the collision shape
			vec3 getLocalScaling() const;
	
			/// Set the local scaling vector of the collision shape
			virtual void setLocalScaling(const vec3& _scaling);
	
			friend class OverlappingPair;
			friend class CollisionBody;
			friend class RigidBody;
			friend class BroadPhaseAlgorithm;
			friend class DynamicAABBTree;
			friend class CollisionDetection;
			friend class CollisionWorld;
			friend class DynamicsWorld;
			friend class EPAAlgorithm;
			friend class GJKAlgorithm;
			friend class ConvexMeshShape;
	
	};

}

