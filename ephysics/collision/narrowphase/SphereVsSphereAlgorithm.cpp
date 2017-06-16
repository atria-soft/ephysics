/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/narrowphase/SphereVsSphereAlgorithm.hpp>
#include <ephysics/collision/shapes/SphereShape.hpp>

// We want to use the ReactPhysics3D namespace
using namespace ephysics;

// Constructor
SphereVsSphereAlgorithm::SphereVsSphereAlgorithm() :
  NarrowPhaseAlgorithm() {
	
}

// Destructor
SphereVsSphereAlgorithm::~SphereVsSphereAlgorithm() {
	
}

void SphereVsSphereAlgorithm::testCollision(const CollisionShapeInfo& shape1Info,
                                            const CollisionShapeInfo& shape2Info,
                                            NarrowPhaseCallback* narrowPhaseCallback) {
	// Get the sphere collision shapes
	const SphereShape* sphereShape1 = static_cast<const SphereShape*>(shape1Info.collisionShape);
	const SphereShape* sphereShape2 = static_cast<const SphereShape*>(shape2Info.collisionShape);
	// Get the local-space to world-space transforms
	const etk::Transform3D& transform1 = shape1Info.shapeToWorldTransform;
	const etk::Transform3D& transform2 = shape2Info.shapeToWorldTransform;
	// Compute the distance between the centers
	vec3 vectorBetweenCenters = transform2.getPosition() - transform1.getPosition();
	float squaredDistanceBetweenCenters = vectorBetweenCenters.length2();
	// Compute the sum of the radius
	float sumRadius = sphereShape1->getRadius() + sphereShape2->getRadius();
	// If the sphere collision shapes int32_tersect
	if (squaredDistanceBetweenCenters <= sumRadius * sumRadius) {
		vec3 centerSphere2InBody1LocalSpace = transform1.getInverse() * transform2.getPosition();
		vec3 centerSphere1InBody2LocalSpace = transform2.getInverse() * transform1.getPosition();
		vec3 int32_tersectionOnBody1 = sphereShape1->getRadius() *
									  centerSphere2InBody1LocalSpace.safeNormalized();
		vec3 int32_tersectionOnBody2 = sphereShape2->getRadius() *
									  centerSphere1InBody2LocalSpace.safeNormalized();
		float penetrationDepth = sumRadius - std::sqrt(squaredDistanceBetweenCenters);
		
		// Create the contact info object
		ContactPointInfo contactInfo(shape1Info.proxyShape, shape2Info.proxyShape, shape1Info.collisionShape,
									 shape2Info.collisionShape, vectorBetweenCenters.safeNormalized(), penetrationDepth,
									 int32_tersectionOnBody1, int32_tersectionOnBody2);
		// Notify about the new contact
		narrowPhaseCallback->notifyContact(shape1Info.overlappingPair, contactInfo);
	}
}
