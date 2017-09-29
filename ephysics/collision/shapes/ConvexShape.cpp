/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

#include <ephysics/collision/shapes/ConvexShape.hpp>

ephysics::ConvexShape::ConvexShape(ephysics::CollisionShapeType _type, float _margin):
  CollisionShape(_type),
  m_margin(_margin) {
	
}

ephysics::ConvexShape::~ConvexShape() {
	
}

vec3 ephysics::ConvexShape::getLocalSupportPointWithMargin(const vec3& _direction, void** _cachedCollisionData) const {
	// Get the support point without margin
	vec3 supportPoint = getLocalSupportPointWithoutMargin(_direction, _cachedCollisionData);
	if (m_margin != 0.0f) {
		// Add the margin to the support point
		vec3 unitVec(0.0, -1.0, 0.0);
		if (_direction.length2() > FLT_EPSILON * FLT_EPSILON) {
			unitVec = _direction.safeNormalized();
		}
		supportPoint += unitVec * m_margin;
	}
	return supportPoint;
}
