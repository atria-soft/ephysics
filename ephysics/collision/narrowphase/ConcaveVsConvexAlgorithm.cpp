/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

#include <ephysics/collision/shapes/ConcaveShape.hpp>
#include <ephysics/collision/shapes/TriangleShape.hpp>
#include <ephysics/collision/narrowphase/ConcaveVsConvexAlgorithm.hpp>
#include <ephysics/collision/CollisionDetection.hpp>
#include <ephysics/engine/CollisionWorld.hpp>

using namespace ephysics;

ConcaveVsConvexAlgorithm::ConcaveVsConvexAlgorithm() {
	
}

ConcaveVsConvexAlgorithm::~ConcaveVsConvexAlgorithm() {
	
}

void ConcaveVsConvexAlgorithm::testCollision(const CollisionShapeInfo& shape1Info,
											 const CollisionShapeInfo& shape2Info,
											 NarrowPhaseCallback* narrowPhaseCallback) {
	ProxyShape* convexProxyShape;
	ProxyShape* concaveProxyShape;
	const ConvexShape* convexShape;
	const ConcaveShape* concaveShape;
	// Collision shape 1 is convex, collision shape 2 is concave
	if (shape1Info.collisionShape->isConvex()) {
		convexProxyShape = shape1Info.proxyShape;
		convexShape = static_cast<const ConvexShape*>(shape1Info.collisionShape);
		concaveProxyShape = shape2Info.proxyShape;
		concaveShape = static_cast<const ConcaveShape*>(shape2Info.collisionShape);
	} else {
		// Collision shape 2 is convex, collision shape 1 is concave
		convexProxyShape = shape2Info.proxyShape;
		convexShape = static_cast<const ConvexShape*>(shape2Info.collisionShape);
		concaveProxyShape = shape1Info.proxyShape;
		concaveShape = static_cast<const ConcaveShape*>(shape1Info.collisionShape);
	}
	// Set the parameters of the callback object
	ConvexVsTriangleCallback convexVsTriangleCallback;
	convexVsTriangleCallback.setCollisionDetection(m_collisionDetection);
	convexVsTriangleCallback.setConvexShape(convexShape);
	convexVsTriangleCallback.setConcaveShape(concaveShape);
	convexVsTriangleCallback.setProxyShapes(convexProxyShape, concaveProxyShape);
	convexVsTriangleCallback.setOverlappingPair(shape1Info.overlappingPair);
	// Compute the convex shape AABB in the local-space of the convex shape
	AABB aabb;
	convexShape->computeAABB(aabb, convexProxyShape->getLocalToWorldTransform());
	// If smooth mesh collision is enabled for the concave mesh
	if (concaveShape->getIsSmoothMeshCollisionEnabled()) {
		etk::Vector<SmoothMeshContactInfo> contactPoints;
		SmoothCollisionNarrowPhaseCallback smoothNarrowPhaseCallback(contactPoints);
		convexVsTriangleCallback.setNarrowPhaseCallback(&smoothNarrowPhaseCallback);
		// Call the convex vs triangle callback for each triangle of the concave shape
		concaveShape->testAllTriangles(convexVsTriangleCallback, aabb);
		// Run the smooth mesh collision algorithm
		processSmoothMeshCollision(shape1Info.overlappingPair, contactPoints, narrowPhaseCallback);
	} else {
		convexVsTriangleCallback.setNarrowPhaseCallback(narrowPhaseCallback);
		// Call the convex vs triangle callback for each triangle of the concave shape
		concaveShape->testAllTriangles(convexVsTriangleCallback, aabb);
	}
}

void ConvexVsTriangleCallback::testTriangle(const vec3* trianglePoints) {
	// Create a triangle collision shape
	float margin = m_concaveShape->getTriangleMargin();
	TriangleShape triangleShape(trianglePoints[0], trianglePoints[1], trianglePoints[2], margin);
	// Select the collision algorithm to use between the triangle and the convex shape
	NarrowPhaseAlgorithm* algo = m_collisionDetection->getCollisionAlgorithm(triangleShape.getType(),
																			m_convexShape->getType());
	// If there is no collision algorithm between those two kinds of shapes
	if (algo == nullptr) {
		return;
	}
	// Notify the narrow-phase algorithm about the overlapping pair we are going to test
	algo->setCurrentOverlappingPair(m_overlappingPair);
	// Create the CollisionShapeInfo objects
	CollisionShapeInfo shapeConvexInfo(m_convexProxyShape, m_convexShape, m_convexProxyShape->getLocalToWorldTransform(),
									   m_overlappingPair, m_convexProxyShape->getCachedCollisionData());
	CollisionShapeInfo shapeConcaveInfo(m_concaveProxyShape, &triangleShape,
										m_concaveProxyShape->getLocalToWorldTransform(),
										m_overlappingPair, m_concaveProxyShape->getCachedCollisionData());
	// Use the collision algorithm to test collision between the triangle and the other convex shape
	algo->testCollision(shapeConvexInfo, shapeConcaveInfo, m_narrowPhaseCallback);
}

void ConcaveVsConvexAlgorithm::processSmoothMeshCollision(OverlappingPair* overlappingPair,
														  etk::Vector<SmoothMeshContactInfo> contactPoints,
														  NarrowPhaseCallback* narrowPhaseCallback) {
	// Set with the triangle vertices already processed to void further contacts with same triangle
	etk::Vector<etk::Pair<int32_t, vec3>> processTriangleVertices;
	// Sort the list of narrow-phase contacts according to their penetration depth
	contactPoints.sort(0,
	                   contactPoints.size()-1,
	                   [](const SmoothMeshContactInfo& _contact1, const SmoothMeshContactInfo& _contact2) {
	                    	return _contact1.contactInfo.penetrationDepth < _contact2.contactInfo.penetrationDepth;
	                    });
	// For each contact point (from smaller penetration depth to larger)
	etk::Vector<SmoothMeshContactInfo>::Iterator it;
	for (it = contactPoints.begin(); it != contactPoints.end(); ++it) {
		const SmoothMeshContactInfo info = *it;
		const vec3& contactPoint = info.isFirstShapeTriangle ? info.contactInfo.localPoint1 : info.contactInfo.localPoint2;
		// Compute the barycentric coordinates of the point in the triangle
		float u, v, w;
		computeBarycentricCoordinatesInTriangle(info.triangleVertices[0],
												info.triangleVertices[1],
												info.triangleVertices[2],
												contactPoint, u, v, w);
		int32_t nbZeros = 0;
		bool isUZero = approxEqual(u, 0, 0.0001);
		bool isVZero = approxEqual(v, 0, 0.0001);
		bool isWZero = approxEqual(w, 0, 0.0001);
		if (isUZero) {
			nbZeros++;
		}
		if (isVZero) {
			nbZeros++;
		}
		if (isWZero) {
			nbZeros++;
		}
		// If it is a vertex contact
		if (nbZeros == 2) {
			vec3 contactVertex = !isUZero ? info.triangleVertices[0] : (!isVZero ? info.triangleVertices[1] : info.triangleVertices[2]);
			// Check that this triangle vertex has not been processed yet
			if (!hasVertexBeenProcessed(processTriangleVertices, contactVertex)) {
				// Keep the contact as it is and report it
				narrowPhaseCallback->notifyContact(overlappingPair, info.contactInfo);
			}
		} else if (nbZeros == 1) {
			// If it is an edge contact
			vec3 contactVertex1 = isUZero ? info.triangleVertices[1] : (isVZero ? info.triangleVertices[0] : info.triangleVertices[0]);
			vec3 contactVertex2 = isUZero ? info.triangleVertices[2] : (isVZero ? info.triangleVertices[2] : info.triangleVertices[1]);
			// Check that this triangle edge has not been processed yet
			if (!hasVertexBeenProcessed(processTriangleVertices, contactVertex1) &&
				!hasVertexBeenProcessed(processTriangleVertices, contactVertex2)) {
				// Keep the contact as it is and report it
				narrowPhaseCallback->notifyContact(overlappingPair, info.contactInfo);
			}
		} else {
			// If it is a face contact
			ContactPointInfo newContactInfo(info.contactInfo);
			ProxyShape* firstShape;
			ProxyShape* secondShape;
			if (info.isFirstShapeTriangle) {
				firstShape = overlappingPair->getShape1();
				secondShape = overlappingPair->getShape2();
			} else {
				firstShape = overlappingPair->getShape2();
				secondShape = overlappingPair->getShape1();
			}
			// We use the triangle normal as the contact normal
			vec3 a = info.triangleVertices[1] - info.triangleVertices[0];
			vec3 b = info.triangleVertices[2] - info.triangleVertices[0];
			vec3 localNormal = a.cross(b);
			newContactInfo.normal = firstShape->getLocalToWorldTransform().getOrientation() * localNormal;
			vec3 firstLocalPoint = info.isFirstShapeTriangle ? info.contactInfo.localPoint1 : info.contactInfo.localPoint2;
			vec3 firstWorldPoint = firstShape->getLocalToWorldTransform() * firstLocalPoint;
			newContactInfo.normal.normalize();
			if (newContactInfo.normal.dot(info.contactInfo.normal) < 0) {
				newContactInfo.normal = -newContactInfo.normal;
			}
			// We recompute the contact point on the second body with the new normal as described in
			// the Smooth Mesh Contacts with GJK of the Game Physics Pearls book (from Gino van Den Bergen and
			// Dirk Gregorius) to avoid adding torque
			etk::Transform3D worldToLocalSecondPoint = secondShape->getLocalToWorldTransform().getInverse();
			if (info.isFirstShapeTriangle) {
				vec3 newSecondWorldPoint = firstWorldPoint + newContactInfo.normal;
				newContactInfo.localPoint2 = worldToLocalSecondPoint * newSecondWorldPoint;
			} else {
				vec3 newSecondWorldPoint = firstWorldPoint - newContactInfo.normal;
				newContactInfo.localPoint1 = worldToLocalSecondPoint * newSecondWorldPoint;
			}
			// Report the contact
			narrowPhaseCallback->notifyContact(overlappingPair, newContactInfo);
		}
		// Add the three vertices of the triangle to the set of processed
		// triangle vertices
		addProcessedVertex(processTriangleVertices, info.triangleVertices[0]);
		addProcessedVertex(processTriangleVertices, info.triangleVertices[1]);
		addProcessedVertex(processTriangleVertices, info.triangleVertices[2]);
	}
}

bool ConcaveVsConvexAlgorithm::hasVertexBeenProcessed(const etk::Vector<etk::Pair<int32_t, vec3>>& _processTriangleVertices, const vec3& _vertex) const {
	/* TODO : etk::Vector<etk::Pair<int32_t, vec3>> was an unordered map ... ==> stupid idee... I replace code because I do not have enouth time to do something good...
	int32_t key = int32_t(_vertex.x() * _vertex.y() * _vertex.z());
	auto range = _processTriangleVertices.equal_range(key);
	for (auto it = range.first; it != range.second; ++it) {
		if (    _vertex.x() == it->second.x()
		     && _vertex.y() == it->second.y()
		     && _vertex.z() == it->second.z()) {
			return true;
		}
	}
	return false;
	*/
	// TODO : This is not really the same ...
	for (auto &it: _processTriangleVertices) {
		if (    _vertex.x() == it.second.x()
		     && _vertex.y() == it.second.y()
		     && _vertex.z() == it.second.z()) {
			return true;
		}
	}
	return false;
}

void SmoothCollisionNarrowPhaseCallback::notifyContact(OverlappingPair* _overlappingPair,
													   const ContactPointInfo& _contactInfo) {
	vec3 triangleVertices[3];
	bool isFirstShapeTriangle;
	// If the collision shape 1 is the triangle
	if (_contactInfo.collisionShape1->getType() == TRIANGLE) {
		assert(_contactInfo.collisionShape2->getType() != TRIANGLE);
		const TriangleShape* triangleShape = static_cast<const TriangleShape*>(_contactInfo.collisionShape1);
		triangleVertices[0] = triangleShape->getVertex(0);
		triangleVertices[1] = triangleShape->getVertex(1);
		triangleVertices[2] = triangleShape->getVertex(2);
		isFirstShapeTriangle = true;
	} else {  // If the collision shape 2 is the triangle
		assert(_contactInfo.collisionShape2->getType() == TRIANGLE);
		const TriangleShape* triangleShape = static_cast<const TriangleShape*>(_contactInfo.collisionShape2);
		triangleVertices[0] = triangleShape->getVertex(0);
		triangleVertices[1] = triangleShape->getVertex(1);
		triangleVertices[2] = triangleShape->getVertex(2);
		isFirstShapeTriangle = false;
	}
	SmoothMeshContactInfo smoothContactInfo(_contactInfo, isFirstShapeTriangle, triangleVertices[0], triangleVertices[1], triangleVertices[2]);
	// Add the narrow-phase contact int32_to the list of contact to process for
	// smooth mesh collision
	m_contactPoints.pushBack(smoothContactInfo);
}
