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
		public :
			/**
			 * @brief Constructor
			 * @param _radius Radius of the cone (in meters)
			 * @param _height Height of the cone (in meters)
			 * @param _margin Collision margin (in meters) around the collision shape
			 */
			ConeShape(float _radius, float _height, float _margin = OBJECT_MARGIN);
			/// DELETE copy-constructor
			ConeShape(const ConeShape& _shape) = delete;
			/// DELETE assignment operator
			ConeShape& operator=(const ConeShape& _shape) = delete;
		protected :
			float m_radius; //!< Radius of the base
			float m_halfHeight; //!< Half height of the cone
			float m_sinTheta; //!< sine of the semi angle at the apex point
			virtual vec3 getLocalSupportPointWithoutMargin(const vec3& _direction, void** _cachedCollisionData) const override;
			bool testPointInside(const vec3& _localPoint, ProxyShape* _proxyShape) const override;
			bool raycast(const Ray& _ray, RaycastInfo& _raycastInfo, ProxyShape* _proxyShape) const override;
			size_t getSizeInBytes() const override;
		public:
			/**
			 * @brief Return the radius
			 * @return Radius of the cone (in meters)
			 */
			float getRadius() const;
			/**
			 * @brief Return the height
			 * @return Height of the cone (in meters)
			 */
			float getHeight() const;
			
			void setLocalScaling(const vec3& _scaling) override;
			void getLocalBounds(vec3& _min, vec3& _max) const override;
			void computeLocalInertiaTensor(etk::Matrix3x3& _tensor, float _mass) const override;
	};
}
