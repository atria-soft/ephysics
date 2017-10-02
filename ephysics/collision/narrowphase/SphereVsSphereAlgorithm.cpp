/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#include <ephysics/collision/narrowphase/SphereVsSphereAlgorithm.hpp>
#include <ephysics/collision/shapes/SphereShape.hpp>

ephysics::SphereVsSphereAlgorithm::SphereVsSphereAlgorithm() :
  NarrowPhaseAlgorithm() {
	
}

void ephysics::SphereVsSphereAlgorithm::testCollision(const ephysics::CollisionShapeInfo& _shape1Info,
                                                      const ephysics::CollisionShapeInfo& _shape2Info,
                                                      ephysics::NarrowPhaseCallback* _narrowPhaseCallback) {
	// Get the sphere collision shapes
	const ephysics::SphereShape* sphereShape1 = static_cast<const ephysics::SphereShape*>(_shape1Info.collisionShape);
	const ephysics::SphereShape* sphereShape2 = static_cast<const ephysics::SphereShape*>(_shape2Info.collisionShape);
	// Get the local-space to world-space transforms
	const etk::Transform3D& transform1 = _shape1Info.shapeToWorldTransform;
	const etk::Transform3D& transform2 = _shape2Info.shapeToWorldTransform;
	// Compute the distance between the centers
	vec3 vectorBetweenCenters = transform2.getPosition() - transform1.getPosition();
	float squaredDistanceBetweenCenters = vectorBetweenCenters.length2();
	// Compute the sum of the radius
	float sumRadius = sphereShape1->getRadius() + sphereShape2->getRadius();
	// If the sphere collision shapes intersect
	if (squaredDistanceBetweenCenters <= sumRadius * sumRadius) {
		vec3 centerSphere2InBody1LocalSpace = transform1.getInverse() * transform2.getPosition();
		vec3 centerSphere1InBody2LocalSpace = transform2.getInverse() * transform1.getPosition();
		vec3 intersectionOnBody1 = sphereShape1->getRadius() * centerSphere2InBody1LocalSpace.safeNormalized();
		vec3 intersectionOnBody2 = sphereShape2->getRadius() * centerSphere1InBody2LocalSpace.safeNormalized();
		float penetrationDepth = sumRadius - etk::sqrt(squaredDistanceBetweenCenters);
		
		// Create the contact info object
		ephysics::ContactPointInfo contactInfo(_shape1Info.proxyShape,
		                                       _shape2Info.proxyShape,
		                                       _shape1Info.collisionShape,
		                                       _shape2Info.collisionShape,
		                                       vectorBetweenCenters.safeNormalized(),
		                                       penetrationDepth,
		                                       intersectionOnBody1,
		                                       intersectionOnBody2);
		// Notify about the new contact
		_narrowPhaseCallback->notifyContact(_shape1Info.overlappingPair, contactInfo);
	}
}
