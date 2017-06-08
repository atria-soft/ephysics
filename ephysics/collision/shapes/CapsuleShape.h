/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/collision/shapes/ConvexShape.h>
#include <ephysics/body/CollisionBody.h>
#include <ephysics/mathematics/mathematics.h>

// ReactPhysics3D namespace
namespace reactphysics3d {

// Class CapsuleShape
/**
 * This class represents a capsule collision shape that is defined around the Y axis.
 * A capsule shape can be seen as the convex hull of two spheres.
 * The capsule shape is defined by its radius (radius of the two spheres of the capsule)
 * and its height (distance between the centers of the two spheres). This collision shape
 * does not have an explicit object margin distance. The margin is implicitly the radius
 * and height of the shape. Therefore, no need to specify an object margin for a
 * capsule shape.
 */
class CapsuleShape : public ConvexShape {

	protected :

		// -------------------- Attributes -------------------- //

		/// Half height of the capsule (height = distance between the centers of the two spheres)
		float mHalfHeight;

		// -------------------- Methods -------------------- //

		/// Private copy-constructor
		CapsuleShape(const CapsuleShape& shape);

		/// Private assignment operator
		CapsuleShape& operator=(const CapsuleShape& shape);

		/// Return a local support point in a given direction without the object margin
		virtual Vector3 getLocalSupportPointWithoutMargin(const Vector3& direction,
														  void** cachedCollisionData) const;

		/// Return true if a point is inside the collision shape
		virtual bool testPointInside(const Vector3& localPoint, ProxyShape* proxyShape) const;

		/// Raycast method with feedback information
		virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const;

		/// Raycasting method between a ray one of the two spheres end cap of the capsule
		bool raycastWithSphereEndCap(const Vector3& point1, const Vector3& point2,
									 const Vector3& sphereCenter, float maxFraction,
									 Vector3& hitLocalPoint, float& hitFraction) const;

		/// Return the number of bytes used by the collision shape
		virtual size_t getSizeInBytes() const;

	public :

		// -------------------- Methods -------------------- //

		/// Constructor
		CapsuleShape(float radius, float height);

		/// Destructor
		virtual ~CapsuleShape();

		/// Return the radius of the capsule
		float getRadius() const;

		/// Return the height of the capsule
		float getHeight() const;

		/// Set the scaling vector of the collision shape
		virtual void setLocalScaling(const Vector3& scaling);

		/// Return the local bounds of the shape in x, y and z directions
		virtual void getLocalBounds(Vector3& min, Vector3& max) const;

		/// Return the local inertia tensor of the collision shape
		virtual void computeLocalInertiaTensor(Matrix3x3& tensor, float mass) const;
};

// Get the radius of the capsule
/**
 * @return The radius of the capsule shape (in meters)
 */
inline float CapsuleShape::getRadius() const {
	return mMargin;
}

// Return the height of the capsule
/**
 * @return The height of the capsule shape (in meters)
 */
inline float CapsuleShape::getHeight() const {
	return mHalfHeight + mHalfHeight;
}

// Set the scaling vector of the collision shape
inline void CapsuleShape::setLocalScaling(const Vector3& scaling) {

	mHalfHeight = (mHalfHeight / mScaling.y) * scaling.y;
	mMargin = (mMargin / mScaling.x) * scaling.x;

	CollisionShape::setLocalScaling(scaling);
}

// Return the number of bytes used by the collision shape
inline size_t CapsuleShape::getSizeInBytes() const {
	return sizeof(CapsuleShape);
}

// Return the local bounds of the shape in x, y and z directions
// This method is used to compute the AABB of the box
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
inline void CapsuleShape::getLocalBounds(Vector3& min, Vector3& max) const {

	// Maximum bounds
	max.x = mMargin;
	max.y = mHalfHeight + mMargin;
	max.z = mMargin;

	// Minimum bounds
	min.x = -mMargin;
	min.y = -max.y;
	min.z = min.x;
}

// Return a local support point in a given direction without the object margin.
/// A capsule is the convex hull of two spheres S1 and S2. The support point in the direction "d"
/// of the convex hull of a set of convex objects is the support point "p" in the set of all
/// support points from all the convex objects with the maximum dot product with the direction "d".
/// Therefore, in this method, we compute the support points of both top and bottom spheres of
/// the capsule and return the point with the maximum dot product with the direction vector. Note
/// that the object margin is implicitly the radius and height of the capsule.
inline Vector3 CapsuleShape::getLocalSupportPointWithoutMargin(const Vector3& direction,
														void** cachedCollisionData) const {

	// Support point top sphere
	float dotProductTop = mHalfHeight * direction.y;

	// Support point bottom sphere
	float dotProductBottom = -mHalfHeight * direction.y;

	// Return the point with the maximum dot product
	if (dotProductTop > dotProductBottom) {
		return Vector3(0, mHalfHeight, 0);
	}
	else {
		return Vector3(0, -mHalfHeight, 0);
	}
}

}
