/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#pragma once

#include <ephysics/collision/shapes/CollisionShape.hpp>
#include <ephysics/collision/shapes/TriangleShape.hpp>

namespace ephysics {
	/**
	 * @brief It is used to encapsulate a callback method for
	 * a single triangle of a ConcaveMesh.
	 */
	class TriangleCallback {
		public:
			virtual ~TriangleCallback() = default;
			/// Report a triangle
			virtual void testTriangle(const vec3* _trianglePoints)=0;
	};
	
	/**
	 * @brief This abstract class represents a concave collision shape associated with a
	 * body that is used during the narrow-phase collision detection.
	 */
	class ConcaveShape : public CollisionShape {
		public :
			/// Constructor
			ConcaveShape(CollisionShapeType _type);
			/// Destructor
			virtual ~ConcaveShape();
			/// DELETE copy-constructor
			ConcaveShape(const ConcaveShape& _shape) = delete;
			/// DELETE assignment operator
			ConcaveShape& operator=(const ConcaveShape& _shape) = delete;
		protected :
			bool m_isSmoothMeshCollisionEnabled; //!< True if the smooth mesh collision algorithm is enabled
			float m_triangleMargin; //!< Margin use for collision detection for each triangle
			TriangleRaycastSide m_raycastTestType; //!< Raycast test type for the triangle (front, back, front-back)
			bool testPointInside(const vec3& _localPoint, ProxyShape* _proxyShape) const override;
		public:
			/// Return the triangle margin
			float getTriangleMargin() const;
			/// Return the raycast test type (front, back, front-back)
			TriangleRaycastSide getRaycastTestType() const;
			/**
			 * @brief Set the raycast test type (front, back, front-back)
			 * @param testType Raycast test type for the triangle (front, back, front-back)
			 */
			void setRaycastTestType(TriangleRaycastSide _testType);
			/// Return true if the collision shape is convex, false if it is concave
			virtual bool isConvex() const override;
			/// Use a callback method on all triangles of the concave shape inside a given AABB
			virtual void testAllTriangles(TriangleCallback& _callback, const AABB& _localAABB) const=0;
			/// Return true if the smooth mesh collision is enabled
			bool getIsSmoothMeshCollisionEnabled() const;
			/**
			 * @brief Enable/disable the smooth mesh collision algorithm
			 *
			 * Smooth mesh collision is used to avoid collisions against some int32_ternal edges of the triangle mesh.
			 * If it is enabled, collsions with the mesh will be smoother but collisions computation is a bit more expensive.
			 */
			void setIsSmoothMeshCollisionEnabled(bool _isEnabled);
	};

}


