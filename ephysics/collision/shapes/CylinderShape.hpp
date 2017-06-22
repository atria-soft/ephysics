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
 * @brief It represents a cylinder collision shape around the Y axis
 * and centered at the origin. The cylinder is defined by its height
 * and the radius of its base. The "transform" of the corresponding
 * rigid body gives an orientation and a position to the cylinder.
 * This collision shape uses an extra margin distance around it for collision
 * detection purpose. The default margin is 4cm (if your units are meters,
 * which is recommended). In case, you want to simulate small objects
 * (smaller than the margin distance), you might want to reduce the margin by
 * specifying your own margin distance using the "margin" parameter in the
 * constructor of the cylinder shape. Otherwise, it is recommended to use the
 * default margin distance by not using the "margin" parameter in the constructor.
 */
class CylinderShape : public ConvexShape {
	protected :
		float mRadius; //!< Radius of the base
		float m_halfHeight; //!< Half height of the cylinder
		/// Private copy-constructor
		CylinderShape(const CylinderShape& shape);
		/// Private assignment operator
		CylinderShape& operator=(const CylinderShape& shape);
		/// Return a local support point in a given direction without the object margin
		virtual vec3 getLocalSupportPointWithoutMargin(const vec3& direction,
														  void** cachedCollisionData) const;
		/// Return true if a point is inside the collision shape
		virtual bool testPointInside(const vec3& localPoint, ProxyShape* proxyShape) const;
		/// Raycast method with feedback information
		virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const;
		/// Return the number of bytes used by the collision shape
		virtual size_t getSizeInBytes() const;
	public :
		/// Constructor
		CylinderShape(float radius, float height, float margin = OBJECT_MARGIN);
		/// Destructor
		virtual ~CylinderShape();
		/// Return the radius
		float getRadius() const;
		/// Return the height
		float getHeight() const;
		/// Set the scaling vector of the collision shape
		virtual void setLocalScaling(const vec3& scaling);
		/// Return the local bounds of the shape in x, y and z directions
		virtual void getLocalBounds(vec3& min, vec3& max) const;
		/// Return the local inertia tensor of the collision shape
		virtual void computeLocalInertiaTensor(etk::Matrix3x3& tensor, float mass) const;
};

}


