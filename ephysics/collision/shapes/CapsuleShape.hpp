/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
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
		protected:
			float m_halfHeight; //!< Half height of the capsule (height = distance between the centers of the two spheres)
			/// Private copy-constructor
			CapsuleShape(const CapsuleShape& _shape);
			/// Private assignment operator
			CapsuleShape& operator=(const CapsuleShape& _shape);
			vec3 getLocalSupportPointWithoutMargin(const vec3& _direction, void** _cachedCollisionData) const override;
			bool testPointInside(const vec3& _localPoint, ProxyShape* _proxyShape) const override;
			bool raycast(const Ray& _ray, RaycastInfo& _raycastInfo, ProxyShape* _proxyShape) const override;
			/// Raycasting method between a ray one of the two spheres end cap of the capsule
			bool raycastWithSphereEndCap(const vec3& _point1, const vec3& _point2,
										 const vec3& _sphereCenter, float _maxFraction,
										 vec3& _hitLocalPoint, float& _hitFraction) const;
			size_t getSizeInBytes() const override;
		public :
			/// Constructor
			CapsuleShape(float _radius, float _height);
			/// Destructor
			virtual ~CapsuleShape();
			/// Return the radius of the capsule
			float getRadius() const;
			/// Return the height of the capsule
			float getHeight() const;
			void setLocalScaling(const vec3& _scaling) override;
			void getLocalBounds(vec3& _min, vec3& _max) const override;
			void computeLocalInertiaTensor(etk::Matrix3x3& _tensor, float _mass) const override;
	};
}
