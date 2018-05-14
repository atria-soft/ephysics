/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#pragma once

#include <ephysics/collision/shapes/ConvexShape.hpp>
#include <ephysics/body/CollisionBody.hpp>
#include <ephysics/mathematics/mathematics.hpp>

namespace ephysics {
	/**
	 * @brief It represents a capsule collision shape that is defined around the Y axis.
	 * A capsule shape can be seen as the convex hull of two spheres.
	 * The capsule shape is defined by its radius (radius of the two spheres of the capsule)
	 * and its height (distance between the centers of the two spheres). This collision shape
	 * does not have an explicit object margin distance. The margin is implicitly the radius
	 * and height of the shape. Therefore, no need to specify an object margin for a
	 * capsule shape.
	 */
	class CapsuleShape : public ConvexShape {
		public :
			/**
			 * @brief Constructor
			 * @param _radius The radius of the capsule (in meters)
			 * @param _height The height of the capsule (in meters)
			 */
			CapsuleShape(float _radius, float _height);
			/// DELETE copy-constructor
			CapsuleShape(const CapsuleShape& _shape) = delete;
			/// DELETE assignment operator
			CapsuleShape& operator=(const CapsuleShape& _shape) = delete;
			/**
			 * Get the radius of the capsule
			 * @return The radius of the capsule shape (in meters)
			 */
			float getRadius() const;
			/**
			 * @brief Return the height of the capsule
			 * @return The height of the capsule shape (in meters)
			 */
			float getHeight() const;
			void setLocalScaling(const vec3& _scaling) override;
			void getLocalBounds(vec3& _min, vec3& _max) const override;
			void computeLocalInertiaTensor(etk::Matrix3x3& _tensor, float _mass) const override;
		protected:
			float m_halfHeight; //!< Half height of the capsule (height = distance between the centers of the two spheres)
			vec3 getLocalSupportPointWithoutMargin(const vec3& _direction, void** _cachedCollisionData) const override;
			bool testPointInside(const vec3& _localPoint, ProxyShape* _proxyShape) const override;
			bool raycast(const Ray& _ray, RaycastInfo& _raycastInfo, ProxyShape* _proxyShape) const override;
			/**
			 * @brief Raycasting method between a ray one of the two spheres end cap of the capsule
			 */
			bool raycastWithSphereEndCap(const vec3& _point1,
			                             const vec3& _point2,
			                             const vec3& _sphereCenter,
			                             float _maxFraction,
			                             vec3& _hitLocalPoint,
			                             float& _hitFraction) const;
			size_t getSizeInBytes() const override;
	};
}
