/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/narrowphase/EPA/EdgeEPA.h>
#include <ephysics/collision/narrowphase/EPA/TriangleEPA.h>
#include <ephysics/collision/narrowphase/EPA/TrianglesStore.h>
#include <cassert>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;


// Constructor
EdgeEPA::EdgeEPA() {
	
}

// Constructor
EdgeEPA::EdgeEPA(TriangleEPA* ownerTriangle, int32_t index)
		: mOwnerTriangle(ownerTriangle), mIndex(index) {
	assert(index >= 0 && index < 3);
}

// Copy-constructor
EdgeEPA::EdgeEPA(const EdgeEPA& edge) {
	mOwnerTriangle = edge.mOwnerTriangle;
	mIndex = edge.mIndex;
}

// Destructor
EdgeEPA::~EdgeEPA() {

}

// Return the index of the source vertex of the edge (vertex starting the edge)
uint32_t EdgeEPA::getSourceVertexIndex() const {
	return (*mOwnerTriangle)[mIndex];
}

// Return the index of the target vertex of the edge (vertex ending the edge)
uint32_t EdgeEPA::getTargetVertexIndex() const {
	return (*mOwnerTriangle)[indexOfNextCounterClockwiseEdge(mIndex)];
}

// Execute the recursive silhouette algorithm from this edge
bool EdgeEPA::computeSilhouette(const vec3* vertices, uint32_t indexNewVertex,
								TrianglesStore& triangleStore) {
	// If the edge has not already been visited
	if (!mOwnerTriangle->getIsObsolete()) {

		// If the triangle of this edge is not visible from the given point
		if (!mOwnerTriangle->isVisibleFromVertex(vertices, indexNewVertex)) {
			TriangleEPA* triangle = triangleStore.newTriangle(vertices, indexNewVertex,
															  getTargetVertexIndex(),
															  getSourceVertexIndex());

			// If the triangle has been created
			if (triangle != NULL) {
				halfLink(EdgeEPA(triangle, 1), *this);
				return true;
			}

			return false;
		}
		else {

			// The current triangle is visible and therefore obsolete
			mOwnerTriangle->setIsObsolete(true);

			int32_t backup = triangleStore.getNbTriangles();

			if(!mOwnerTriangle->getAdjacentEdge(indexOfNextCounterClockwiseEdge(
												this->mIndex)).computeSilhouette(vertices,
																				 indexNewVertex,
																				 triangleStore)) {
				mOwnerTriangle->setIsObsolete(false);

				TriangleEPA* triangle = triangleStore.newTriangle(vertices, indexNewVertex,
																  getTargetVertexIndex(),
																  getSourceVertexIndex());

				// If the triangle has been created
				if (triangle != NULL) {
					halfLink(EdgeEPA(triangle, 1), *this);
					return true;
				}

				return false;
			}
			else if (!mOwnerTriangle->getAdjacentEdge(indexOfPreviousCounterClockwiseEdge(
												  this->mIndex)).computeSilhouette(vertices,
																				   indexNewVertex,
																				   triangleStore)) {
				mOwnerTriangle->setIsObsolete(false);

				triangleStore.setNbTriangles(backup);

				TriangleEPA* triangle = triangleStore.newTriangle(vertices, indexNewVertex,
																  getTargetVertexIndex(),
																  getSourceVertexIndex());

				if (triangle != NULL) {
					halfLink(EdgeEPA(triangle, 1), *this);
					return true;
				}

				return false;
			}
		}
	}

	return true;
}
