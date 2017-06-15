/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/narrowphase/GJK/GJKAlgorithm.h>
#include <ephysics/collision/narrowphase/GJK/Simplex.h>
#include <ephysics/constraint/ContactPoint.h>
#include <ephysics/configuration.h>
#include <ephysics/engine/Profiler.h>
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <cassert>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
GJKAlgorithm::GJKAlgorithm() : NarrowPhaseAlgorithm() {

}

// Destructor
GJKAlgorithm::~GJKAlgorithm() {

}

// Compute a contact info if the two collision shapes collide.
/// This method implements the Hybrid Technique for computing the penetration depth by
/// running the GJK algorithm on original objects (without margin). If the shapes int32_tersect
/// only in the margins, the method compute the penetration depth and contact points
/// (of enlarged objects). If the original objects (without margin) int32_tersect, we
/// call the computePenetrationDepthForEnlargedObjects() method that run the GJK
/// algorithm on the enlarged object to obtain a simplex polytope that contains the
/// origin, they we give that simplex polytope to the EPA algorithm which will compute
/// the correct penetration depth and contact points between the enlarged objects.
void GJKAlgorithm::testCollision(const CollisionShapeInfo& shape1Info,
								 const CollisionShapeInfo& shape2Info,
								 NarrowPhaseCallback* narrowPhaseCallback) {

	PROFILE("GJKAlgorithm::testCollision()");
	
	vec3 suppA;			 // Support point of object A
	vec3 suppB;			 // Support point of object B
	vec3 w;				 // Support point of Minkowski difference A-B
	vec3 pA;				// Closest point of object A
	vec3 pB;				// Closest point of object B
	float vDotw;
	float prevDistSquare;

	assert(shape1Info.collisionShape->isConvex());
	assert(shape2Info.collisionShape->isConvex());

	const ConvexShape* shape1 = static_cast<const ConvexShape*>(shape1Info.collisionShape);
	const ConvexShape* shape2 = static_cast<const ConvexShape*>(shape2Info.collisionShape);

	void** shape1CachedCollisionData = shape1Info.cachedCollisionData;
	void** shape2CachedCollisionData = shape2Info.cachedCollisionData;

	// Get the local-space to world-space transforms
	const etk::Transform3D transform1 = shape1Info.shapeToWorldTransform;
	const etk::Transform3D transform2 = shape2Info.shapeToWorldTransform;

	// etk::Transform3D a point from local space of body 2 to local
	// space of body 1 (the GJK algorithm is done in local space of body 1)
	etk::Transform3D body2Tobody1 = transform1.getInverse() * transform2;

	// Matrix that transform a direction from local
	// space of body 1 int32_to local space of body 2
	etk::Matrix3x3 rotateToBody2 = transform2.getOrientation().getMatrix().getTranspose() *
							  transform1.getOrientation().getMatrix();

	// Initialize the margin (sum of margins of both objects)
	float margin = shape1->getMargin() + shape2->getMargin();
	float marginSquare = margin * margin;
	assert(margin > 0.0);

	// Create a simplex set
	Simplex simplex;

	// Get the previous point V (last cached separating axis)
	vec3 v = mCurrentOverlappingPair->getCachedSeparatingAxis();

	// Initialize the upper bound for the square distance
	float distSquare = DECIMAL_LARGEST;
	
	do {
			  
		// Compute the support points for original objects (without margins) A and B
		suppA = shape1->getLocalSupportPointWithoutMargin(-v, shape1CachedCollisionData);
		suppB = body2Tobody1 *
					 shape2->getLocalSupportPointWithoutMargin(rotateToBody2 * v, shape2CachedCollisionData);

		// Compute the support point for the Minkowski difference A-B
		w = suppA - suppB;
		
		vDotw = v.dot(w);
		
		// If the enlarge objects (with margins) do not int32_tersect
		if (vDotw > 0.0 && vDotw * vDotw > distSquare * marginSquare) {
						
			// Cache the current separating axis for frame coherence
			mCurrentOverlappingPair->setCachedSeparatingAxis(v);
			
			// No int32_tersection, we return
			return;
		}

		// If the objects int32_tersect only in the margins
		if (simplex.isPointInSimplex(w) || distSquare - vDotw <= distSquare * REL_ERROR_SQUARE) {

			// Compute the closet points of both objects (without the margins)
			simplex.computeClosestPointsOfAandB(pA, pB);

			// Project those two points on the margins to have the closest points of both
			// object with the margins
			float dist = sqrt(distSquare);
			assert(dist > 0.0);
			pA = (pA - (shape1->getMargin() / dist) * v);
			pB = body2Tobody1.getInverse() * (pB + (shape2->getMargin() / dist) * v);

			// Compute the contact info
			vec3 normal = transform1.getOrientation() * (-v.safeNormalized());
			float penetrationDepth = margin - dist;
			
			// Reject the contact if the penetration depth is negative (due too numerical errors)
			if (penetrationDepth <= 0.0) return;
			
			// Create the contact info object
			ContactPointInfo contactInfo(shape1Info.proxyShape, shape2Info.proxyShape, shape1Info.collisionShape,
										 shape2Info.collisionShape, normal, penetrationDepth, pA, pB);

			narrowPhaseCallback->notifyContact(shape1Info.overlappingPair, contactInfo);

			// There is an int32_tersection, therefore we return
			return;
		}

		// Add the new support point to the simplex
		simplex.addPoint(w, suppA, suppB);

		// If the simplex is affinely dependent
		if (simplex.isAffinelyDependent()) {

			// Compute the closet points of both objects (without the margins)
			simplex.computeClosestPointsOfAandB(pA, pB);

			// Project those two points on the margins to have the closest points of both
			// object with the margins
			float dist = sqrt(distSquare);
			assert(dist > 0.0);
			pA = (pA - (shape1->getMargin() / dist) * v);
			pB = body2Tobody1.getInverse() * (pB + (shape2->getMargin() / dist) * v);

			// Compute the contact info
			vec3 normal = transform1.getOrientation() * (-v.safeNormalized());
			float penetrationDepth = margin - dist;
			
			// Reject the contact if the penetration depth is negative (due too numerical errors)
			if (penetrationDepth <= 0.0) return;
			
			// Create the contact info object
			ContactPointInfo contactInfo(shape1Info.proxyShape, shape2Info.proxyShape, shape1Info.collisionShape,
										 shape2Info.collisionShape, normal, penetrationDepth, pA, pB);

			narrowPhaseCallback->notifyContact(shape1Info.overlappingPair, contactInfo);

			// There is an int32_tersection, therefore we return
			return;
		}

		// Compute the point of the simplex closest to the origin
		// If the computation of the closest point fail
		if (!simplex.computeClosestPoint(v)) {

			// Compute the closet points of both objects (without the margins)
			simplex.computeClosestPointsOfAandB(pA, pB);

			// Project those two points on the margins to have the closest points of both
			// object with the margins
			float dist = sqrt(distSquare);
			assert(dist > 0.0);
			pA = (pA - (shape1->getMargin() / dist) * v);
			pB = body2Tobody1.getInverse() * (pB + (shape2->getMargin() / dist) * v);

			// Compute the contact info
			vec3 normal = transform1.getOrientation() * (-v.safeNormalized());
			float penetrationDepth = margin - dist;
			
			// Reject the contact if the penetration depth is negative (due too numerical errors)
			if (penetrationDepth <= 0.0) return;
			
			// Create the contact info object
			ContactPointInfo contactInfo(shape1Info.proxyShape, shape2Info.proxyShape, shape1Info.collisionShape,
										 shape2Info.collisionShape, normal, penetrationDepth, pA, pB);

			narrowPhaseCallback->notifyContact(shape1Info.overlappingPair, contactInfo);

			// There is an int32_tersection, therefore we return
			return;
		}

		// Store and update the squared distance of the closest point
		prevDistSquare = distSquare;
		distSquare = v.length2();

		// If the distance to the closest point doesn't improve a lot
		if (prevDistSquare - distSquare <= MACHINE_EPSILON * prevDistSquare) {
			simplex.backupClosestPointInSimplex(v);
			
			// Get the new squared distance
			distSquare = v.length2();

			// Compute the closet points of both objects (without the margins)
			simplex.computeClosestPointsOfAandB(pA, pB);

			// Project those two points on the margins to have the closest points of both
			// object with the margins
			float dist = sqrt(distSquare);
			assert(dist > 0.0);
			pA = (pA - (shape1->getMargin() / dist) * v);
			pB = body2Tobody1.getInverse() * (pB + (shape2->getMargin() / dist) * v);

			// Compute the contact info
			vec3 normal = transform1.getOrientation() * (-v.safeNormalized());
			float penetrationDepth = margin - dist;
			
			// Reject the contact if the penetration depth is negative (due too numerical errors)
			if (penetrationDepth <= 0.0) return;
			
			// Create the contact info object
			ContactPointInfo contactInfo(shape1Info.proxyShape, shape2Info.proxyShape, shape1Info.collisionShape,
										 shape2Info.collisionShape, normal, penetrationDepth, pA, pB);

			narrowPhaseCallback->notifyContact(shape1Info.overlappingPair, contactInfo);

			// There is an int32_tersection, therefore we return
			return;
		}
	} while(!simplex.isFull() && distSquare > MACHINE_EPSILON *
								 simplex.getMaxLengthSquareOfAPoint());

	// The objects (without margins) int32_tersect. Therefore, we run the GJK algorithm
	// again but on the enlarged objects to compute a simplex polytope that contains
	// the origin. Then, we give that simplex polytope to the EPA algorithm to compute
	// the correct penetration depth and contact points between the enlarged objects.
	return computePenetrationDepthForEnlargedObjects(shape1Info, transform1, shape2Info,
													 transform2, narrowPhaseCallback, v);
}

/// This method runs the GJK algorithm on the two enlarged objects (with margin)
/// to compute a simplex polytope that contains the origin. The two objects are
/// assumed to int32_tersect in the original objects (without margin). Therefore such
/// a polytope must exist. Then, we give that polytope to the EPA algorithm to
/// compute the correct penetration depth and contact points of the enlarged objects.
void GJKAlgorithm::computePenetrationDepthForEnlargedObjects(const CollisionShapeInfo& shape1Info,
															 const etk::Transform3D& transform1,
															 const CollisionShapeInfo& shape2Info,
															 const etk::Transform3D& transform2,
															 NarrowPhaseCallback* narrowPhaseCallback,
															 vec3& v) {
	PROFILE("GJKAlgorithm::computePenetrationDepthForEnlargedObjects()");

	Simplex simplex;
	vec3 suppA;
	vec3 suppB;
	vec3 w;
	float vDotw;
	float distSquare = DECIMAL_LARGEST;
	float prevDistSquare;

	assert(shape1Info.collisionShape->isConvex());
	assert(shape2Info.collisionShape->isConvex());

	const ConvexShape* shape1 = static_cast<const ConvexShape*>(shape1Info.collisionShape);
	const ConvexShape* shape2 = static_cast<const ConvexShape*>(shape2Info.collisionShape);

	void** shape1CachedCollisionData = shape1Info.cachedCollisionData;
	void** shape2CachedCollisionData = shape2Info.cachedCollisionData;

	// etk::Transform3D a point from local space of body 2 to local space
	// of body 1 (the GJK algorithm is done in local space of body 1)
	etk::Transform3D body2ToBody1 = transform1.getInverse() * transform2;

	// Matrix that transform a direction from local space of body 1 int32_to local space of body 2
	etk::Matrix3x3 rotateToBody2 = transform2.getOrientation().getMatrix().getTranspose() *
							  transform1.getOrientation().getMatrix();
	
	do {
		// Compute the support points for the enlarged object A and B
		suppA = shape1->getLocalSupportPointWithMargin(-v, shape1CachedCollisionData);
		suppB = body2ToBody1 * shape2->getLocalSupportPointWithMargin(rotateToBody2 * v, shape2CachedCollisionData);

		// Compute the support point for the Minkowski difference A-B
		w = suppA - suppB;

		vDotw = v.dot(w);

		// If the enlarge objects do not int32_tersect
		if (vDotw > 0.0) {

			// No int32_tersection, we return
			return;
		}

		// Add the new support point to the simplex
		simplex.addPoint(w, suppA, suppB);

		if (simplex.isAffinelyDependent()) {
			return;
		}

		if (!simplex.computeClosestPoint(v)) {
			return;
		}

		// Store and update the square distance
		prevDistSquare = distSquare;
		distSquare = v.length2();

		if (prevDistSquare - distSquare <= MACHINE_EPSILON * prevDistSquare) {
			return;
		}

	} while(!simplex.isFull() && distSquare > MACHINE_EPSILON *
								 simplex.getMaxLengthSquareOfAPoint());

	// Give the simplex computed with GJK algorithm to the EPA algorithm
	// which will compute the correct penetration depth and contact points
	// between the two enlarged objects
	return mAlgoEPA.computePenetrationDepthAndContactPoints(simplex, shape1Info,
															transform1, shape2Info, transform2,
															v, narrowPhaseCallback);
}

// Use the GJK Algorithm to find if a point is inside a convex collision shape
bool GJKAlgorithm::testPointInside(const vec3& localPoint, ProxyShape* proxyShape) {

	vec3 suppA;			 // Support point of object A
	vec3 w;				 // Support point of Minkowski difference A-B
	float prevDistSquare;

	assert(proxyShape->getCollisionShape()->isConvex());

	const ConvexShape* shape = static_cast<const ConvexShape*>(proxyShape->getCollisionShape());

	void** shapeCachedCollisionData = proxyShape->getCachedCollisionData();

	// Support point of object B (object B is a single point)
	const vec3 suppB(localPoint);

	// Create a simplex set
	Simplex simplex;

	// Initial supporting direction
	vec3 v(1, 1, 1);

	// Initialize the upper bound for the square distance
	float distSquare = DECIMAL_LARGEST;

	do {

		// Compute the support points for original objects (without margins) A and B
		suppA = shape->getLocalSupportPointWithoutMargin(-v, shapeCachedCollisionData);

		// Compute the support point for the Minkowski difference A-B
		w = suppA - suppB;

		// Add the new support point to the simplex
		simplex.addPoint(w, suppA, suppB);

		// If the simplex is affinely dependent
		if (simplex.isAffinelyDependent()) {

			return false;
		}

		// Compute the point of the simplex closest to the origin
		// If the computation of the closest point fail
		if (!simplex.computeClosestPoint(v)) {

			return false;
		}

		// Store and update the squared distance of the closest point
		prevDistSquare = distSquare;
		distSquare = v.length2();

		// If the distance to the closest point doesn't improve a lot
		if (prevDistSquare - distSquare <= MACHINE_EPSILON * prevDistSquare) {

			return false;
		}
	} while(!simplex.isFull() && distSquare > MACHINE_EPSILON *
								 simplex.getMaxLengthSquareOfAPoint());

	// The point is inside the collision shape
	return true;
}


// Ray casting algorithm agains a convex collision shape using the GJK Algorithm
/// This method implements the GJK ray casting algorithm described by Gino Van Den Bergen in
/// "Ray Casting against General Convex Objects with Application to Continuous Collision Detection".
bool GJKAlgorithm::raycast(const Ray& ray, ProxyShape* proxyShape, RaycastInfo& raycastInfo) {

	assert(proxyShape->getCollisionShape()->isConvex());

	const ConvexShape* shape = static_cast<const ConvexShape*>(proxyShape->getCollisionShape());

	void** shapeCachedCollisionData = proxyShape->getCachedCollisionData();

	vec3 suppA;	  // Current lower bound point on the ray (starting at ray's origin)
	vec3 suppB;	  // Support point on the collision shape
	const float machineEpsilonSquare = MACHINE_EPSILON * MACHINE_EPSILON;
	const float epsilon = float(0.0001);

	// Convert the ray origin and direction int32_to the local-space of the collision shape
	vec3 rayDirection = ray.point2 - ray.point1;

	// If the points of the segment are two close, return no hit
	if (rayDirection.length2() < machineEpsilonSquare) return false;

	vec3 w;

	// Create a simplex set
	Simplex simplex;

	vec3 n(0.0f, float(0.0), float(0.0));
	float lambda = 0.0f;
	suppA = ray.point1;	// Current lower bound point on the ray (starting at ray's origin)
	suppB = shape->getLocalSupportPointWithoutMargin(rayDirection, shapeCachedCollisionData);
	vec3 v = suppA - suppB;
	float vDotW, vDotR;
	float distSquare = v.length2();
	int32_t nbIterations = 0;

	// GJK Algorithm loop
	while (distSquare > epsilon && nbIterations < MAX_ITERATIONS_GJK_RAYCAST) {

		// Compute the support points
		suppB = shape->getLocalSupportPointWithoutMargin(v, shapeCachedCollisionData);
		w = suppA - suppB;

		vDotW = v.dot(w);

		if (vDotW > float(0)) {

			vDotR = v.dot(rayDirection);

			if (vDotR >= -machineEpsilonSquare) {
				return false;
			}
			else {

				// We have found a better lower bound for the hit point along the ray
				lambda = lambda - vDotW / vDotR;
				suppA = ray.point1 + lambda * rayDirection;
				w = suppA - suppB;
				n = v;
			}
		}

		// Add the new support point to the simplex
		if (!simplex.isPointInSimplex(w)) {
			simplex.addPoint(w, suppA, suppB);
		}

		// Compute the closest point
		if (simplex.computeClosestPoint(v)) {

			distSquare = v.length2();
		}
		else {
			distSquare = 0.0f;
		}

		// If the current lower bound distance is larger than the maximum raycasting distance
		if (lambda > ray.maxFraction) return false;

		nbIterations++;
	}

	// If the origin was inside the shape, we return no hit
	if (lambda < MACHINE_EPSILON) return false;

	// Compute the closet points of both objects (without the margins)
	vec3 pointA;
	vec3 pointB;
	simplex.computeClosestPointsOfAandB(pointA, pointB);

	// A raycast hit has been found, we fill in the raycast info
	raycastInfo.hitFraction = lambda;
	raycastInfo.worldPoint = pointB;
	raycastInfo.body = proxyShape->getBody();
	raycastInfo.proxyShape = proxyShape;

	if (n.length2() >= machineEpsilonSquare) { // The normal vector is valid
		raycastInfo.worldNormal = n;
	}
	else {  // Degenerated normal vector, we return a zero normal vector
		raycastInfo.worldNormal = vec3(float(0), float(0), float(0));
	}

	return true;
}
