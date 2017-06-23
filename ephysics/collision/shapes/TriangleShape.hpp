/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
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
	class TriangleShape : public ConvexShape {
		protected:
			vec3 m_points[3]; //!< Three points of the triangle
			TriangleRaycastSide m_raycastTestType; //!< Raycast test type for the triangle (front, back, front-back)
			/// Private copy-constructor
			TriangleShape(const TriangleShape& shape);
			/// Private assignment operator
			TriangleShape& operator=(const TriangleShape& shape);
			vec3 getLocalSupportPointWithoutMargin(const vec3& _direction, void** _cachedCollisionData) const override;
			bool testPointInside(const vec3& _localPoint, ProxyShape* _proxyShape) const override;
			bool raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const override;
			size_t getSizeInBytes() const override;
		public:
			/// Constructor
			TriangleShape(const vec3& point1, const vec3& point2, const vec3& point3,
						  float margin = OBJECT_MARGIN);
			/// Destructor
			virtual ~TriangleShape();
			void getLocalBounds(vec3& min, vec3& max) const override;
			void setLocalScaling(const vec3& scaling) override;
			void computeLocalInertiaTensor(etk::Matrix3x3& tensor, float mass) const override;
			void computeAABB(AABB& aabb, const etk::Transform3D& transform) const override;
			/// Return the raycast test type (front, back, front-back)
			TriangleRaycastSide getRaycastTestType() const;
			// Set the raycast test type (front, back, front-back)
			void setRaycastTestType(TriangleRaycastSide testType);
			/// Return the coordinates of a given vertex of the triangle
			vec3 getVertex(int32_t index) const;
			friend class ConcaveMeshRaycastCallback;
			friend class TriangleOverlapCallback;
	};
}

