/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/shapes/ConcaveMeshShape.h>
#include <iostream>

using namespace reactphysics3d;

// Constructor
ConcaveMeshShape::ConcaveMeshShape(TriangleMesh* triangleMesh):
  ConcaveShape(CONCAVE_MESH) {
	m_triangleMesh = triangleMesh;
	m_raycastTestType = FRONT;
	// Insert all the triangles int32_to the dynamic AABB tree
	initBVHTree();
}

// Destructor
ConcaveMeshShape::~ConcaveMeshShape() {
	
}

// Insert all the triangles int32_to the dynamic AABB tree
void ConcaveMeshShape::initBVHTree() {
	// TODO : Try to randomly add the triangles int32_to the tree to obtain a better tree
	// For each sub-part of the mesh
	for (uint32_t subPart=0; subPart<m_triangleMesh->getNbSubparts(); subPart++) {
		// Get the triangle vertex array of the current sub-part
		TriangleVertexArray* triangleVertexArray = m_triangleMesh->getSubpart(subPart);
		TriangleVertexArray::VertexDataType vertexType = triangleVertexArray->getVertexDataType();
		TriangleVertexArray::IndexDataType indexType = triangleVertexArray->getIndexDataType();
		unsigned char* verticesStart = triangleVertexArray->getVerticesStart();
		unsigned char* indicesStart = triangleVertexArray->getIndicesStart();
		int32_t vertexStride = triangleVertexArray->getVerticesStride();
		int32_t indexStride = triangleVertexArray->getIndicesStride();
		// For each triangle of the concave mesh
		for (uint32_t triangleIndex=0; triangleIndex<triangleVertexArray->getNbTriangles(); triangleIndex++) {
			void* vertexIndexPointer = (indicesStart + triangleIndex * 3 * indexStride);
			Vector3 trianglePoints[3];
			// For each vertex of the triangle
			for (int32_t k=0; k < 3; k++) {
				// Get the index of the current vertex in the triangle
				int32_t vertexIndex = 0;
				if (indexType == TriangleVertexArray::INDEX_INTEGER_TYPE) {
					vertexIndex = ((uint32_t*)vertexIndexPointer)[k];
				} else if (indexType == TriangleVertexArray::INDEX_SHORT_TYPE) {
					vertexIndex = ((unsigned short*)vertexIndexPointer)[k];
				} else {
					assert(false);
				}
				// Get the vertices components of the triangle
				if (vertexType == TriangleVertexArray::VERTEX_FLOAT_TYPE) {
					const float* vertices = (float*)(verticesStart + vertexIndex * vertexStride);
					trianglePoints[k][0] = float(vertices[0]) * mScaling.x;
					trianglePoints[k][1] = float(vertices[1]) * mScaling.y;
					trianglePoints[k][2] = float(vertices[2]) * mScaling.z;
				} else if (vertexType == TriangleVertexArray::VERTEX_DOUBLE_TYPE) {
					const double* vertices = (double*)(verticesStart + vertexIndex * vertexStride);
					trianglePoints[k][0] = float(vertices[0]) * mScaling.x;
					trianglePoints[k][1] = float(vertices[1]) * mScaling.y;
					trianglePoints[k][2] = float(vertices[2]) * mScaling.z;
				} else {
					assert(false);
				}
			}
			// Create the AABB for the triangle
			AABB aabb = AABB::createAABBForTriangle(trianglePoints);
			aabb.inflate(m_triangleMargin, m_triangleMargin, m_triangleMargin);
			// Add the AABB with the index of the triangle int32_to the dynamic AABB tree
			m_dynamicAABBTree.addObject(aabb, subPart, triangleIndex);
		}
	}
}

// Return the three vertices coordinates (in the array outTriangleVertices) of a triangle
// given the start vertex index pointer of the triangle
void ConcaveMeshShape::getTriangleVerticesWithIndexPointer(int32_t subPart, int32_t triangleIndex, Vector3* outTriangleVertices) const {
	// Get the triangle vertex array of the current sub-part
	TriangleVertexArray* triangleVertexArray = m_triangleMesh->getSubpart(subPart);
	if (triangleVertexArray == nullptr) {
		std::cout << "get nullptr ..." << std::endl;
	}
	TriangleVertexArray::VertexDataType vertexType = triangleVertexArray->getVertexDataType();
	TriangleVertexArray::IndexDataType indexType = triangleVertexArray->getIndexDataType();
	unsigned char* verticesStart = triangleVertexArray->getVerticesStart();
	unsigned char* indicesStart = triangleVertexArray->getIndicesStart();
	int32_t vertexStride = triangleVertexArray->getVerticesStride();
	int32_t indexStride = triangleVertexArray->getIndicesStride();
	void* vertexIndexPointer = (indicesStart + triangleIndex * 3 * indexStride);
	// For each vertex of the triangle
	for (int32_t k=0; k < 3; k++) {
		// Get the index of the current vertex in the triangle
		int32_t vertexIndex = 0;
		if (indexType == TriangleVertexArray::INDEX_INTEGER_TYPE) {
			vertexIndex = ((uint32_t*)vertexIndexPointer)[k];
		} else if (indexType == TriangleVertexArray::INDEX_SHORT_TYPE) {
			vertexIndex = ((unsigned short*)vertexIndexPointer)[k];
		} else {
			std::cout << "wrong type of array : " << int32_t(indexType) << std::endl;
			assert(false);
		}
		// Get the vertices components of the triangle
		if (vertexType == TriangleVertexArray::VERTEX_FLOAT_TYPE) {
			const float* vertices = (float*)(verticesStart + vertexIndex * vertexStride);
			outTriangleVertices[k][0] = float(vertices[0]) * mScaling.x;
			outTriangleVertices[k][1] = float(vertices[1]) * mScaling.y;
			outTriangleVertices[k][2] = float(vertices[2]) * mScaling.z;
		} else if (vertexType == TriangleVertexArray::VERTEX_DOUBLE_TYPE) {
			const double* vertices = (double*)(verticesStart + vertexIndex * vertexStride);
			outTriangleVertices[k][0] = float(vertices[0]) * mScaling.x;
			outTriangleVertices[k][1] = float(vertices[1]) * mScaling.y;
			outTriangleVertices[k][2] = float(vertices[2]) * mScaling.z;
		} else {
			assert(false);
		}
	}
}

// Use a callback method on all triangles of the concave shape inside a given AABB
void ConcaveMeshShape::testAllTriangles(TriangleCallback& callback, const AABB& localAABB) const {
	ConvexTriangleAABBOverlapCallback overlapCallback(callback, *this, m_dynamicAABBTree);
	// Ask the Dynamic AABB Tree to report all the triangles that are overlapping
	// with the AABB of the convex shape.
	m_dynamicAABBTree.reportAllShapesOverlappingWithAABB(localAABB, overlapCallback);
}

// Raycast method with feedback information
/// Note that only the first triangle hit by the ray in the mesh will be returned, even if
/// the ray hits many triangles.
bool ConcaveMeshShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const {
	PROFILE("ConcaveMeshShape::raycast()");
	// Create the callback object that will compute ray casting against triangles
	ConcaveMeshRaycastCallback raycastCallback(m_dynamicAABBTree, *this, proxyShape, raycastInfo, ray);
	// Ask the Dynamic AABB Tree to report all AABB nodes that are hit by the ray.
	// The raycastCallback object will then compute ray casting against the triangles
	// in the hit AABBs.
	m_dynamicAABBTree.raycast(ray, raycastCallback);
	raycastCallback.raycastTriangles();
	return raycastCallback.getIsHit();
}

// Collect all the AABB nodes that are hit by the ray in the Dynamic AABB Tree
float ConcaveMeshRaycastCallback::raycastBroadPhaseShape(int32_t nodeId, const Ray& ray) {
	// Add the id of the hit AABB node int32_to
	m_hitAABBNodes.push_back(nodeId);
	return ray.maxFraction;
}

// Raycast all collision shapes that have been collected
void ConcaveMeshRaycastCallback::raycastTriangles() {
	std::vector<int32_t>::const_iterator it;
	float smallestHitFraction = m_ray.maxFraction;
	for (it = m_hitAABBNodes.begin(); it != m_hitAABBNodes.end(); ++it) {
		// Get the node data (triangle index and mesh subpart index)
		int32_t* data = m_dynamicAABBTree.getNodeDataInt(*it);
		// Get the triangle vertices for this node from the concave mesh shape
		Vector3 trianglePoints[3];
		m_concaveMeshShape.getTriangleVerticesWithIndexPointer(data[0], data[1], trianglePoints);
		// Create a triangle collision shape
		float margin = m_concaveMeshShape.getTriangleMargin();
		TriangleShape triangleShape(trianglePoints[0], trianglePoints[1], trianglePoints[2], margin);
		triangleShape.setRaycastTestType(m_concaveMeshShape.getRaycastTestType());
		// Ray casting test against the collision shape
		RaycastInfo raycastInfo;
		bool isTriangleHit = triangleShape.raycast(m_ray, raycastInfo, m_proxyShape);
		// If the ray hit the collision shape
		if (isTriangleHit && raycastInfo.hitFraction <= smallestHitFraction) {
			assert(raycastInfo.hitFraction >= float(0.0));
			m_raycastInfo.body = raycastInfo.body;
			m_raycastInfo.proxyShape = raycastInfo.proxyShape;
			m_raycastInfo.hitFraction = raycastInfo.hitFraction;
			m_raycastInfo.worldPoint = raycastInfo.worldPoint;
			m_raycastInfo.worldNormal = raycastInfo.worldNormal;
			m_raycastInfo.meshSubpart = data[0];
			m_raycastInfo.triangleIndex = data[1];
			smallestHitFraction = raycastInfo.hitFraction;
			mIsHit = true;
		}
	}
}
