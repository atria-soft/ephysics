/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#include <ephysics/collision/narrowphase/EPA/EPAAlgorithm.hpp>
#include <ephysics/engine/Profiler.hpp>
#include <ephysics/collision/narrowphase/GJK/GJKAlgorithm.hpp>
#include <ephysics/collision/narrowphase/EPA/TrianglesStore.hpp>

using namespace ephysics;

EPAAlgorithm::EPAAlgorithm() {
	
}

EPAAlgorithm::~EPAAlgorithm() {
	
}

int32_t EPAAlgorithm::isOriginInTetrahedron(const vec3& _p1, const vec3& _p2, const vec3& _p3, const vec3& _p4) const {
	// Check vertex 1
	vec3 normal1 = (_p2-_p1).cross(_p3-_p1);
	if ((normal1.dot(_p1) > 0.0) == (normal1.dot(_p4) > 0.0)) {
		return 4;
	}
	// Check vertex 2
	vec3 normal2 = (_p4-_p2).cross(_p3-_p2);
	if ((normal2.dot(_p2) > 0.0) == (normal2.dot(_p1) > 0.0)) {
		return 1;
	}
	// Check vertex 3
	vec3 normal3 = (_p4-_p3).cross(_p1-_p3);
	if ((normal3.dot(_p3) > 0.0) == (normal3.dot(_p2) > 0.0)) {
		return 2;
	}
	// Check vertex 4
	vec3 normal4 = (_p2-_p4).cross(_p1-_p4);
	if ((normal4.dot(_p4) > 0.0) == (normal4.dot(_p3) > 0.0)) {
		return 3;
	}
	// The origin is in the tetrahedron, we return 0
	return 0;
}

void EPAAlgorithm::computePenetrationDepthAndContactPoints(const Simplex& _simplex,
                                                           CollisionShapeInfo _shape1Info,
                                                           const etk::Transform3D& _transform1,
                                                           CollisionShapeInfo _shape2Info,
                                                           const etk::Transform3D& _transform2,
                                                           vec3& _vector,
                                                           NarrowPhaseCallback* narrowPhaseCallback) {
	PROFILE("EPAAlgorithm::computePenetrationDepthAndContactPoints()");
	assert(_shape1Info.collisionShape->isConvex());
	assert(_shape2Info.collisionShape->isConvex());
	const ConvexShape* shape1 = static_cast<const ConvexShape*>(_shape1Info.collisionShape);
	const ConvexShape* shape2 = static_cast<const ConvexShape*>(_shape2Info.collisionShape);
	void** shape1CachedCollisionData = _shape1Info.cachedCollisionData;
	void** shape2CachedCollisionData = _shape2Info.cachedCollisionData;
	vec3 suppPointsA[MAX_SUPPORT_POINTS];  // Support points of object A in local coordinates
	vec3 suppPointsB[MAX_SUPPORT_POINTS];  // Support points of object B in local coordinates
	vec3 points[MAX_SUPPORT_POINTS];	   // Current points
	TrianglesStore triangleStore;			 // Store the triangles
	etk::Set<TriangleEPA*> triangleHeap; // list of face candidate of the EPA algorithm sorted lower square dist to upper square dist
	triangleHeap.setComparator([](TriangleEPA * const & _face1, TriangleEPA * const & _face2) {
	                           		return (_face1->getDistSquare() < _face2->getDistSquare());
	                           });
	// etk::Transform3D a point from local space of body 2 to local
	// space of body 1 (the GJK algorithm is done in local space of body 1)
	etk::Transform3D body2Tobody1 = _transform1.getInverse() * _transform2;
	// Matrix that transform a direction from local
	// space of body 1 int32_to local space of body 2
	etk::Quaternion rotateToBody2 = _transform2.getOrientation().getInverse() * _transform1.getOrientation();
	// Get the simplex computed previously by the GJK algorithm
	uint32_t nbVertices = _simplex.getSimplex(suppPointsA, suppPointsB, points);
	// Compute the tolerance
	float tolerance = FLT_EPSILON * _simplex.getMaxLengthSquareOfAPoint();
	// Clear the storing of triangles
	triangleStore.clear();
	// Select an action according to the number of points in the simplex
	// computed with GJK algorithm in order to obtain an initial polytope for
	// The EPA algorithm.
	switch(nbVertices) {
		case 1:
			// Only one point in the simplex (which should be the origin).
			// We have a touching contact with zero penetration depth.
			// We drop that kind of contact. Therefore, we return false
			return;
		case 2: {
			// The simplex returned by GJK is a line segment d containing the origin.
			// We add two additional support points to construct a hexahedron (two tetrahedron
			// glued together with triangle faces. The idea is to compute three different vectors
			// v1, v2 and v3 that are orthogonal to the segment d. The three vectors are relatively
			// rotated of 120 degree around the d segment. The the three new points to
			// construct the polytope are the three support points in those three directions
			// v1, v2 and v3.
			// Direction of the segment
			vec3 d = (points[1] - points[0]).safeNormalized();
			// Choose the coordinate axis from the minimal absolute component of the vector d
			int32_t minAxis = d.absolute().getMinAxis();
			// Compute sin(60)
			const float sin60 = float(sqrt(3.0)) * 0.5f;
			// Create a rotation quaternion to rotate the vector v1 to get the vectors
			// v2 and v3
			etk::Quaternion rotationQuat(d.x() * sin60, d.y() * sin60, d.z() * sin60, 0.5);
			// Compute the vector v1, v2, v3
			vec3 v1 = d.cross(vec3(minAxis == 0, minAxis == 1, minAxis == 2));
			vec3 v2 = rotationQuat * v1;
			vec3 v3 = rotationQuat * v2;
			// Compute the support point in the direction of v1
			suppPointsA[2] = shape1->getLocalSupportPointWithMargin(v1, shape1CachedCollisionData);
			suppPointsB[2] = body2Tobody1 *
					   shape2->getLocalSupportPointWithMargin(rotateToBody2 * (-v1), shape2CachedCollisionData);
			points[2] = suppPointsA[2] - suppPointsB[2];
			// Compute the support point in the direction of v2
			suppPointsA[3] = shape1->getLocalSupportPointWithMargin(v2, shape1CachedCollisionData);
			suppPointsB[3] = body2Tobody1 *
					 shape2->getLocalSupportPointWithMargin(rotateToBody2 * (-v2), shape2CachedCollisionData);
			points[3] = suppPointsA[3] - suppPointsB[3];
			// Compute the support point in the direction of v3
			suppPointsA[4] = shape1->getLocalSupportPointWithMargin(v3, shape1CachedCollisionData);
			suppPointsB[4] = body2Tobody1 *
							shape2->getLocalSupportPointWithMargin(rotateToBody2 * (-v3), shape2CachedCollisionData);
			points[4] = suppPointsA[4] - suppPointsB[4];
			// Now we have an hexahedron (two tetrahedron glued together). We can simply keep the
			// tetrahedron that contains the origin in order that the initial polytope of the
			// EPA algorithm is a tetrahedron, which is simpler to deal with.
			// If the origin is in the tetrahedron of points 0, 2, 3, 4
			if (isOriginInTetrahedron(points[0], points[2], points[3], points[4]) == 0) {
				// We use the point 4 instead of point 1 for the initial tetrahedron
				suppPointsA[1] = suppPointsA[4];
				suppPointsB[1] = suppPointsB[4];
				points[1] = points[4];
			}
			// If the origin is in the tetrahedron of points 1, 2, 3, 4
			else if (isOriginInTetrahedron(points[1], points[2], points[3], points[4]) == 0) {
				// We use the point 4 instead of point 0 for the initial tetrahedron
				suppPointsA[0] = suppPointsA[4];
				suppPointsB[0] = suppPointsB[4];
				points[0] = points[4];
			}
			else {
				// The origin is not in the initial polytope
				return;
			}
			// The polytope contains now 4 vertices
			nbVertices = 4;
		}
		case 4: {
			// The simplex computed by the GJK algorithm is a tetrahedron. Here we check
			// if this tetrahedron contains the origin. If it is the case, we keep it and
			// otherwise we remove the wrong vertex of the tetrahedron and go in the case
			// where the GJK algorithm compute a simplex of three vertices.
			// Check if the tetrahedron contains the origin (or wich is the wrong vertex otherwise)
			int32_t badVertex = isOriginInTetrahedron(points[0], points[1], points[2], points[3]);
			// If the origin is in the tetrahedron
			if (badVertex == 0) {
				// The tetrahedron is a correct initial polytope for the EPA algorithm.
				// Therefore, we construct the tetrahedron.
				// Comstruct the 4 triangle faces of the tetrahedron
				TriangleEPA* face0 = triangleStore.newTriangle(points, 0, 1, 2);
				TriangleEPA* face1 = triangleStore.newTriangle(points, 0, 3, 1);
				TriangleEPA* face2 = triangleStore.newTriangle(points, 0, 2, 3);
				TriangleEPA* face3 = triangleStore.newTriangle(points, 1, 3, 2);
				// If the constructed tetrahedron is not correct
				if (!((face0 != NULL) && (face1 != NULL) && (face2 != NULL) && (face3 != NULL)
				   && face0->getDistSquare() > 0.0 && face1->getDistSquare() > 0.0
				   && face2->getDistSquare() > 0.0 && face3->getDistSquare() > 0.0)) {
					return;
				}
				// Associate the edges of neighbouring triangle faces
				link(EdgeEPA(face0, 0), EdgeEPA(face1, 2));
				link(EdgeEPA(face0, 1), EdgeEPA(face3, 2));
				link(EdgeEPA(face0, 2), EdgeEPA(face2, 0));
				link(EdgeEPA(face1, 0), EdgeEPA(face2, 2));
				link(EdgeEPA(face1, 1), EdgeEPA(face3, 0));
				link(EdgeEPA(face2, 1), EdgeEPA(face3, 1));
				// Add the triangle faces in the candidate heap
				addFaceCandidate(face0, triangleHeap, FLT_MAX);
				addFaceCandidate(face1, triangleHeap, FLT_MAX);
				addFaceCandidate(face2, triangleHeap, FLT_MAX);
				addFaceCandidate(face3, triangleHeap, FLT_MAX);
				break;
			}
			// The tetrahedron contains a wrong vertex (the origin is not inside the tetrahedron)
			// Remove the wrong vertex and continue to the next case with the
			// three remaining vertices
			if (badVertex < 4) {
				suppPointsA[badVertex-1] = suppPointsA[3];
				suppPointsB[badVertex-1] = suppPointsB[3];
				points[badVertex-1] = points[3];
			}
			// We have removed the wrong vertex
			nbVertices = 3;
		}
		case 3: {
			// The GJK algorithm returned a triangle that contains the origin.
			// We need two new vertices to create two tetrahedron. The two new
			// vertices are the support points in the "n" and "-n" direction
			// where "n" is the normal of the triangle. Then, we use only the
			// tetrahedron that contains the origin.
			// Compute the normal of the triangle
			vec3 v1 = points[1] - points[0];
			vec3 v2 = points[2] - points[0];
			vec3 n = v1.cross(v2);
			// Compute the two new vertices to obtain a hexahedron
			suppPointsA[3] = shape1->getLocalSupportPointWithMargin(n, shape1CachedCollisionData);
			suppPointsB[3] = body2Tobody1 *
					 shape2->getLocalSupportPointWithMargin(rotateToBody2 * (-n), shape2CachedCollisionData);
			points[3] = suppPointsA[3] - suppPointsB[3];
			suppPointsA[4] = shape1->getLocalSupportPointWithMargin(-n, shape1CachedCollisionData);
			suppPointsB[4] = body2Tobody1 *
					 shape2->getLocalSupportPointWithMargin(rotateToBody2 * n, shape2CachedCollisionData);
			points[4] = suppPointsA[4] - suppPointsB[4];
			TriangleEPA* face0 = nullptr;
			TriangleEPA* face1 = nullptr;
			TriangleEPA* face2 = nullptr;
			TriangleEPA* face3 = nullptr;
			// If the origin is in the first tetrahedron
			if (isOriginInTetrahedron(points[0], points[1],
									  points[2], points[3]) == 0) {
				// The tetrahedron is a correct initial polytope for the EPA algorithm.
				// Therefore, we construct the tetrahedron.
				// Comstruct the 4 triangle faces of the tetrahedron
				face0 = triangleStore.newTriangle(points, 0, 1, 2);
				face1 = triangleStore.newTriangle(points, 0, 3, 1);
				face2 = triangleStore.newTriangle(points, 0, 2, 3);
				face3 = triangleStore.newTriangle(points, 1, 3, 2);
			}
			else if (isOriginInTetrahedron(points[0], points[1],
										   points[2], points[4]) == 0) {
				// The tetrahedron is a correct initial polytope for the EPA algorithm.
				// Therefore, we construct the tetrahedron.
				// Comstruct the 4 triangle faces of the tetrahedron
				face0 = triangleStore.newTriangle(points, 0, 1, 2);
				face1 = triangleStore.newTriangle(points, 0, 4, 1);
				face2 = triangleStore.newTriangle(points, 0, 2, 4);
				face3 = triangleStore.newTriangle(points, 1, 4, 2);
			}
			else {
				return;
			}
			// If the constructed tetrahedron is not correct
			if (!(    face0 != nullptr
			       && face1 != nullptr
			       && face2 != nullptr
			       && face3 != nullptr
			       && face0->getDistSquare() > 0.0
			       && face1->getDistSquare() > 0.0
			       && face2->getDistSquare() > 0.0
			       && face3->getDistSquare() > 0.0) ) {
				return;
			}
			// Associate the edges of neighbouring triangle faces
			link(EdgeEPA(face0, 0), EdgeEPA(face1, 2));
			link(EdgeEPA(face0, 1), EdgeEPA(face3, 2));
			link(EdgeEPA(face0, 2), EdgeEPA(face2, 0));
			link(EdgeEPA(face1, 0), EdgeEPA(face2, 2));
			link(EdgeEPA(face1, 1), EdgeEPA(face3, 0));
			link(EdgeEPA(face2, 1), EdgeEPA(face3, 1));
			// Add the triangle faces in the candidate heap
			addFaceCandidate(face0, triangleHeap, FLT_MAX);
			addFaceCandidate(face1, triangleHeap, FLT_MAX);
			addFaceCandidate(face2, triangleHeap, FLT_MAX);
			addFaceCandidate(face3, triangleHeap, FLT_MAX);
			nbVertices = 4;
		}
		break;
	}
	// At this point, we have a polytope that contains the origin. Therefore, we
	// can run the EPA algorithm.
	if (triangleHeap.size() == 0) {
		return;
	}
	TriangleEPA* triangle = 0;
	float upperBoundSquarePenDepth = FLT_MAX;
	do {
		triangle = triangleHeap[0];
		triangleHeap.popFront();
		EPHY_INFO("rm from heap:");
		for (size_t iii=0; iii<triangleHeap.size(); ++iii) {
			EPHY_INFO("    [" << iii << "] " << triangleHeap[iii]->getDistSquare());
		}
		// If the candidate face in the heap is not obsolete
		if (!triangle->getIsObsolete()) {
			// If we have reached the maximum number of support points
			if (nbVertices == MAX_SUPPORT_POINTS) {
				assert(false);
				break;
			}
			// Compute the support point of the Minkowski
			// difference (A-B) in the closest point direction
			suppPointsA[nbVertices] = shape1->getLocalSupportPointWithMargin(triangle->getClosestPoint(), shape1CachedCollisionData);
			suppPointsB[nbVertices] = body2Tobody1 * shape2->getLocalSupportPointWithMargin(rotateToBody2 * (-triangle->getClosestPoint()), shape2CachedCollisionData);
			points[nbVertices] = suppPointsA[nbVertices] - suppPointsB[nbVertices];
			int32_t indexNewVertex = nbVertices;
			nbVertices++;
			// Update the upper bound of the penetration depth
			float wDotv = points[indexNewVertex].dot(triangle->getClosestPoint());
			EPHY_INFO("      point=" << points[indexNewVertex]);
			EPHY_INFO("close point=" << triangle->getClosestPoint());
			EPHY_INFO("         ==>" << wDotv);
			EPHY_ASSERT(wDotv >= 0.0, "depth penetration error");
			float wDotVSquare = wDotv * wDotv / triangle->getDistSquare();
			if (wDotVSquare < upperBoundSquarePenDepth) {
				upperBoundSquarePenDepth = wDotVSquare;
			}
			// Compute the error
			float error = wDotv - triangle->getDistSquare();
			if (error <= etk::max(tolerance, REL_ERROR_SQUARE * wDotv) ||
				points[indexNewVertex] == points[(*triangle)[0]] ||
				points[indexNewVertex] == points[(*triangle)[1]] ||
				points[indexNewVertex] == points[(*triangle)[2]]) {
				break;
			}
			// Now, we compute the silhouette cast by the new vertex. The current triangle
			// face will not be in the convex hull. We start the local recursive silhouette
			// algorithm from the current triangle face.
			size_t i = triangleStore.getNbTriangles();
			if (!triangle->computeSilhouette(points, indexNewVertex, triangleStore)) {
				break;
			}
			// Add all the new triangle faces computed with the silhouette algorithm
			// to the candidates list of faces of the current polytope
			while(i != triangleStore.getNbTriangles()) {
				TriangleEPA* newTriangle = &triangleStore[i];
				addFaceCandidate(newTriangle, triangleHeap, upperBoundSquarePenDepth);
				i++;
			}
		}
	} while(    triangleHeap.size() > 0
	         && triangleHeap[0]->getDistSquare() <= upperBoundSquarePenDepth);
	// Compute the contact info
	_vector = _transform1.getOrientation() * triangle->getClosestPoint();
	vec3 pALocal = triangle->computeClosestPointOfObject(suppPointsA);
	vec3 pBLocal = body2Tobody1.getInverse() * triangle->computeClosestPointOfObject(suppPointsB);
	vec3 normal = _vector.safeNormalized();
	float penetrationDepth = _vector.length();
	EPHY_ASSERT(penetrationDepth >= 0.0, "penetration depth <0");
	if (normal.length2() < FLT_EPSILON) {
		return;
	}
	// Create the contact info object
	ContactPointInfo contactInfo(_shape1Info.proxyShape, _shape2Info.proxyShape, _shape1Info.collisionShape, _shape2Info.collisionShape, normal, penetrationDepth, pALocal, pBLocal);
	narrowPhaseCallback->notifyContact(_shape1Info.overlappingPair, contactInfo);
}
