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
	 * @brief This class represents a cone collision shape centered at the
	 * origin and alligned with the Y axis. The cone is defined
	 * by its height and by the radius of its base. The center of the
	 * cone is at the half of the height. The "transform" of the
	 * corresponding rigid body gives an orientation and a position
	 * to the cone. This collision shape uses an extra margin distance around
	 * it for collision detection purpose. The default margin is 4cm (if your
	 * units are meters, which is recommended). In case, you want to simulate small
	 * objects (smaller than the margin distance), you might want to reduce the margin
	 * by specifying your own margin distance using the "margin" parameter in the
	 * constructor of the cone shape. Otherwise, it is recommended to use the
	 * default margin distance by not using the "margin" parameter in the constructor.
	 */
	class ConeShape : public ConvexShape {
		protected :
			float m_radius; //!< Radius of the base
			float m_halfHeight; //!< Half height of the cone
			float m_sinTheta; //!< sine of the semi angle at the apex point
			/// Private copy-constructor
			ConeShape(const ConeShape& _shape) = delete;
			/// Private assignment operator
			ConeShape& operator=(const ConeShape& _shape) = delete;
			/// Return a local support point in a given direction without the object margin
			virtual vec3 getLocalSupportPointWithoutMargin(const vec3& _direction,
			                                               void** _cachedCollisionData) const;
			/// Return true if a point is inside the collision shape
			virtual bool testPointInside(const vec3& _localPoint, ProxyShape* _proxyShape) const;
			/// Raycast method with feedback information
			virtual bool raycast(const Ray& _ray, RaycastInfo& _raycastInfo, ProxyShape* _proxyShape) const;
			/// Return the number of bytes used by the collision shape
			virtual size_t getSizeInBytes() const;
		public :
			/// Constructor
			ConeShape(float _radius, float _height, float _margin = OBJECT_MARGIN);
			/// Return the radius
			float getRadius() const;
			/// Return the height
			float getHeight() const;
			/// Set the scaling vector of the collision shape
			virtual void setLocalScaling(const vec3& _scaling);
			/// Return the local bounds of the shape in x, y and z directions
			virtual void getLocalBounds(vec3& _min, vec3& _max) const;
			/// Return the local inertia tensor of the collision shape
			virtual void computeLocalInertiaTensor(etk::Matrix3x3& _tensor, float _mass) const;
	};
}
