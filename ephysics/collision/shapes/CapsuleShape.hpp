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
			CapsuleShape(const CapsuleShape& shape);
			/// Private assignment operator
			CapsuleShape& operator=(const CapsuleShape& shape);
			/// Return a local support point in a given direction without the object margin
			virtual vec3 getLocalSupportPointWithoutMargin(const vec3& direction,
															  void** cachedCollisionData) const;
			/// Return true if a point is inside the collision shape
			virtual bool testPointInside(const vec3& localPoint, ProxyShape* proxyShape) const;
			/// Raycast method with feedback information
			virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const;
			/// Raycasting method between a ray one of the two spheres end cap of the capsule
			bool raycastWithSphereEndCap(const vec3& point1, const vec3& point2,
										 const vec3& sphereCenter, float maxFraction,
										 vec3& hitLocalPoint, float& hitFraction) const;
			/// Return the number of bytes used by the collision shape
			virtual size_t getSizeInBytes() const;
		public :
			/// Constructor
			CapsuleShape(float _radius, float _height);
			/// Destructor
			virtual ~CapsuleShape();
			/// Return the radius of the capsule
			float getRadius() const;
			/// Return the height of the capsule
			float getHeight() const;
			/// Set the scaling vector of the collision shape
			virtual void setLocalScaling(const vec3& scaling);
			/// Return the local bounds of the shape in x, y and z directions
			virtual void getLocalBounds(vec3& min, vec3& max) const;
			/// Return the local inertia tensor of the collision shape
			virtual void computeLocalInertiaTensor(etk::Matrix3x3& tensor, float mass) const;
	};
}
