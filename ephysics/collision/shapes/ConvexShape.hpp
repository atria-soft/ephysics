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
namespace ephysics {
/**
 * @brief It represents a convex collision shape associated with a
 * body that is used during the narrow-phase collision detection.
 */
class ConvexShape : public CollisionShape {
	protected :
		float m_margin; //!< Margin used for the GJK collision detection algorithm
		/// Private copy-constructor
		ConvexShape(const ConvexShape& shape) = delete;
		/// Private assignment operator
		ConvexShape& operator=(const ConvexShape& shape) = delete;
		// Return a local support point in a given direction with the object margin
		virtual vec3 getLocalSupportPointWithMargin(const vec3& _direction, void** _cachedCollisionData) const;
		/// Return a local support point in a given direction without the object margin
		virtual vec3 getLocalSupportPointWithoutMargin(const vec3& _direction, void** _cachedCollisionData) const=0;
		bool testPointInside(const vec3& _worldPoint, ProxyShape* _proxyShape) const override = 0;
	public :
		/// Constructor
		ConvexShape(CollisionShapeType type, float margin);
		/// Destructor
		virtual ~ConvexShape();
		/**
		 * @brief Get the current object margin
		 * @return The margin (in meters) around the collision shape
		 */
		float getMargin() const {
			return m_margin;
		}
		virtual bool isConvex() const override {
			return true;
		}
		friend class GJKAlgorithm;
		friend class EPAAlgorithm;
};

}


