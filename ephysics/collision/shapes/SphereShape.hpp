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
	 * @brief Represents a sphere collision shape that is centered
	 * at the origin and defined by its radius. This collision shape does not
	 * have an explicit object margin distance. The margin is implicitly the
	 * radius of the sphere. Therefore, no need to specify an object margin
	 * for a sphere shape.
	 */
	class SphereShape : public ConvexShape {
		protected :
			SphereShape(const SphereShape& _shape);
			SphereShape& operator=(const SphereShape& _shape) = delete;
			vec3 getLocalSupportPointWithoutMargin(const vec3& _direction, void** _cachedCollisionData) const override {
				return vec3(0.0, 0.0, 0.0);
			}
			bool testPointInside(const vec3& _localPoint, ProxyShape* _proxyShape) const override {
				return (_localPoint.length2() < m_margin * m_margin);
			}
			bool raycast(const Ray& _ray, RaycastInfo& _raycastInfo, ProxyShape* _proxyShape) const override;
			size_t getSizeInBytes() const override {
				return sizeof(SphereShape);
			}
		public :
			/**
			 * @brief Constructor
			 * @param[in] radius Radius of the sphere (in meters)
			 */
			SphereShape(float _radius);
			/**
			 * @brief Get the radius of the sphere
			 * @return Radius of the sphere (in meters)
			 */
			float getRadius() const {
				return m_margin;
			}
			void setLocalScaling(const vec3& _scaling) override;
			void getLocalBounds(vec3& _min, vec3& _max) const override;
			void computeLocalInertiaTensor(etk::Matrix3x3& _tensor, float _mass) const override;
			void computeAABB(AABB& _aabb, const etk::Transform3D& _transform) const override;
	};

}
