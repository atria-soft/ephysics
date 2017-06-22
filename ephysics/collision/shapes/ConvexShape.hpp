/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
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
		ConvexShape(const ConvexShape& shape);
		/// Private assignment operator
		ConvexShape& operator=(const ConvexShape& shape);
		// Return a local support point in a given direction with the object margin
		vec3 getLocalSupportPointWithMargin(const vec3& direction,
											   void** cachedCollisionData) const;
		/// Return a local support point in a given direction without the object margin
		virtual vec3 getLocalSupportPointWithoutMargin(const vec3& direction,
														  void** cachedCollisionData) const=0;
		/// Return true if a point is inside the collision shape
		virtual bool testPointInside(const vec3& worldPoint, ProxyShape* proxyShape) const=0;
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


