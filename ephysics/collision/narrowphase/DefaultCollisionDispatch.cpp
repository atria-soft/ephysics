/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/narrowphase/DefaultCollisionDispatch.hpp>
#include <ephysics/collision/shapes/CollisionShape.hpp>

using namespace ephysics;


DefaultCollisionDispatch::DefaultCollisionDispatch() {
	
}


void DefaultCollisionDispatch::init(CollisionDetection* _collisionDetection) {
	// Initialize the collision algorithms
	m_sphereVsSphereAlgorithm.init(_collisionDetection);
	m_GJKAlgorithm.init(_collisionDetection);
	m_concaveVsConvexAlgorithm.init(_collisionDetection);
}


NarrowPhaseAlgorithm* DefaultCollisionDispatch::selectAlgorithm(int32_t _type1, int32_t _type2) {
	CollisionShapeType shape1Type = static_cast<CollisionShapeType>(_type1);
	CollisionShapeType shape2Type = static_cast<CollisionShapeType>(_type2);
	// Sphere vs Sphere algorithm
	if (shape1Type == SPHERE && shape2Type == SPHERE) {
		return &m_sphereVsSphereAlgorithm;
	} else if (    (    !CollisionShape::isConvex(shape1Type)
	                 && CollisionShape::isConvex(shape2Type) )
	            || (    !CollisionShape::isConvex(shape2Type)
	                 && CollisionShape::isConvex(shape1Type) ) ) {
		// Concave vs Convex algorithm
		return &m_concaveVsConvexAlgorithm;
	} else if (CollisionShape::isConvex(shape1Type) && CollisionShape::isConvex(shape2Type)) {
		// Convex vs Convex algorithm (GJK algorithm)
		return &m_GJKAlgorithm;
	} else {
		return nullptr;
	}
}
