/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#pragma once

#include <ephysics/mathematics/mathematics.hpp>
#include <ephysics/collision/shapes/ConvexShape.hpp>

namespace ephysics {
	/**
	 * @brief Raycast test side for the triangle
	 */
	enum TriangleRaycastSide {
		FRONT, //!< Raycast against front triangle
		BACK, //!< Raycast against back triangle
		FRONT_AND_BACK //!< Raycast against front and back triangle
	};
	
	/**
	 * This class represents a triangle collision shape that is centered
	 * at the origin and defined three points.
	 */
	class TriangleShape: public ConvexShape {
		protected:
			vec3 m_points[3]; //!< Three points of the triangle
			TriangleRaycastSide m_raycastTestType; //!< Raycast test type for the triangle (front, back, front-back)
			/// Private copy-constructor
			TriangleShape(const TriangleShape& _shape);
			/// Private assignment operator
			TriangleShape& operator=(const TriangleShape& _shape);
			vec3 getLocalSupportPointWithoutMargin(const vec3& _direction, void** _cachedCollisionData) const override;
			bool testPointInside(const vec3& _localPoint, ProxyShape* _proxyShape) const override;
			bool raycast(const Ray& _ray, RaycastInfo& _raycastInfo, ProxyShape* _proxyShape) const override;
			size_t getSizeInBytes() const override;
		public:
			/**
			 * @brief Constructor
			 * @param _point1 First point of the triangle
			 * @param _point2 Second point of the triangle
			 * @param _point3 Third point of the triangle
			 * @param _margin The collision margin (in meters) around the collision shape
			 */
			TriangleShape(const vec3& _point1,
			              const vec3& _point2,
			              const vec3& _point3,
			              float _margin = OBJECT_MARGIN);
			void getLocalBounds(vec3& _min, vec3& _max) const override;
			void setLocalScaling(const vec3& _scaling) override;
			void computeLocalInertiaTensor(etk::Matrix3x3& _tensor, float _mass) const override;
			void computeAABB(AABB& aabb, const etk::Transform3D& _transform) const override;
			/// Return the raycast test type (front, back, front-back)
			TriangleRaycastSide getRaycastTestType() const;
			/**
			 * @brief Set the raycast test type (front, back, front-back)
			 * @param[in] _testType Raycast test type for the triangle (front, back, front-back)
			 */
			void setRaycastTestType(TriangleRaycastSide _testType);
			/**
			 * @brief Return the coordinates of a given vertex of the triangle
			 * @param[in] _index Index (0 to 2) of a vertex of the triangle
			 */
			vec3 getVertex(int32_t _index) const;
			friend class ConcaveMeshRaycastCallback;
			friend class TriangleOverlapCallback;
	};
}

