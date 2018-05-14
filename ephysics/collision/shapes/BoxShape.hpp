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
 * @brief It represents a 3D box shape. Those axis are unit length.
 * The three extents are half-widths of the box along the three
 * axis x, y, z local axis. The "transform" of the corresponding
 * rigid body will give an orientation and a position to the box. This
 * collision shape uses an extra margin distance around it for collision
 * detection purpose. The default margin is 4cm (if your units are meters,
 * which is recommended). In case, you want to simulate small objects
 * (smaller than the margin distance), you might want to reduce the margin by
 * specifying your own margin distance using the "margin" parameter in the
 * constructor of the box shape. Otherwise, it is recommended to use the
 * default margin distance by not using the "margin" parameter in the constructor.
 */
class BoxShape : public ConvexShape {
	public:
		/**
		 * @brief Constructor
		 * @param extent The vector with the three extents of the box (in meters)
		 * @param margin The collision margin (in meters) around the collision shape
		 */
		BoxShape(const vec3& _extent, float _margin = OBJECT_MARGIN);
		/// DELETE copy-constructor
		BoxShape(const BoxShape& _shape) = delete;
		/// DELETE assignment operator
		BoxShape& operator=(const BoxShape& _shape) = delete;
		/**
		 * @brief Return the extents of the box
		 * @return The vector with the three extents of the box shape (in meters)
		 */
		vec3 getExtent() const;
		void setLocalScaling(const vec3& _scaling) override;
		void getLocalBounds(vec3& _min, vec3& _max) const override;
		void computeLocalInertiaTensor(etk::Matrix3x3& _tensor, float _mass) const override;
	protected:
		vec3 m_extent; //!< Extent sizes of the box in the x, y and z direction
		vec3 getLocalSupportPointWithoutMargin(const vec3& _direction, void** _cachedCollisionData) const override;
		bool testPointInside(const vec3& _localPoint, ProxyShape* _proxyShape) const override;
		bool raycast(const Ray& _ray, RaycastInfo& _raycastInfo, ProxyShape* _proxyShape) const override;
		size_t getSizeInBytes() const override;
};

}
