/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#include <ephysics/collision/shapes/ConcaveMeshShape.hpp>
#include <ephysics/debug.hpp>

using namespace ephysics;

ConcaveMeshShape::ConcaveMeshShape(TriangleMesh* _triangleMesh):
  ConcaveShape(CONCAVE_MESH) {
	m_triangleMesh = _triangleMesh;
	m_raycastTestType = FRONT;
	initBVHTree();
}

void ConcaveMeshShape::initBVHTree() {
	// TODO : Try to randomly add the triangles into the tree to obtain a better tree
	// For each sub-part of the mesh
	for (uint32_t subPart=0; subPart<m_triangleMesh->getNbSubparts(); subPart++) {
		// Get the triangle vertex array of the current sub-part
		TriangleVertexArray* triangleVertexArray = m_triangleMesh->getSubpart(subPart);
		// For each triangle of the concave mesh
		for (size_t iii=0; iii<triangleVertexArray->getNbTriangles(); ++iii) {
			ephysics::Triangle trianglePoints = triangleVertexArray->getTriangle(iii);
			vec3 trianglePoints2[3];
			trianglePoints2[0] = trianglePoints[0];
			trianglePoints2[1] = trianglePoints[1];
			trianglePoints2[2] = trianglePoints[2];
			// Create the AABB for the triangle
			AABB aabb = AABB::createAABBForTriangle(trianglePoints2);
			aabb.inflate(m_triangleMargin, m_triangleMargin, m_triangleMargin);
			// Add the AABB with the index of the triangle int32_to the dynamic AABB tree
			m_dynamicAABBTree.addObject(aabb, subPart, iii);
		}
	}
}

void ConcaveMeshShape::getTriangleVerticesWithIndexPointer(int32_t _subPart, int32_t _triangleIndex, vec3* _outTriangleVertices) const {
	EPHY_ASSERT(_outTriangleVertices != nullptr, "Input check error");
	// Get the triangle vertex array of the current sub-part
	TriangleVertexArray* triangleVertexArray = m_triangleMesh->getSubpart(_subPart);
	if (triangleVertexArray == nullptr) {
		EPHY_ERROR("get nullptr ...");
	}
	ephysics::Triangle trianglePoints = triangleVertexArray->getTriangle(_triangleIndex);
	_outTriangleVertices[0] = trianglePoints[0] * m_scaling;
	_outTriangleVertices[1] = trianglePoints[1] * m_scaling;
	_outTriangleVertices[2] = trianglePoints[2] * m_scaling;
}

void ConcaveMeshShape::testAllTriangles(TriangleCallback& _callback, const AABB& _localAABB) const {
	// Ask the Dynamic AABB Tree to report all the triangles that are overlapping
	// with the AABB of the convex shape.
	m_dynamicAABBTree.reportAllShapesOverlappingWithAABB(_localAABB, [&](int32_t _nodeId) {
	                                                     	// Get the node data (triangle index and mesh subpart index)
	                                                     	int32_t* data = m_dynamicAABBTree.getNodeDataInt(_nodeId);
	                                                     	// Get the triangle vertices for this node from the concave mesh shape
	                                                     	vec3 trianglePoints[3];
	                                                     	getTriangleVerticesWithIndexPointer(data[0], data[1], trianglePoints);
	                                                     	// Call the callback to test narrow-phase collision with this triangle
	                                                     	_callback.testTriangle(trianglePoints);
	                                                     });
}

bool ConcaveMeshShape::raycast(const Ray& _ray, RaycastInfo& _raycastInfo, ProxyShape* _proxyShape) const {
	PROFILE("ConcaveMeshShape::raycast()");
	// Create the callback object that will compute ray casting against triangles
	ConcaveMeshRaycastCallback raycastCallback(m_dynamicAABBTree, *this, _proxyShape, _raycastInfo, _ray);
	// Ask the Dynamic AABB Tree to report all AABB nodes that are hit by the ray.
	// The raycastCallback object will then compute ray casting against the triangles
	// in the hit AABBs.
	m_dynamicAABBTree.raycast(_ray, [&](int32_t _nodeId, const ephysics::Ray& _ray) mutable { return raycastCallback(_nodeId, _ray);});
	raycastCallback.raycastTriangles();
	return raycastCallback.getIsHit();
}

float ConcaveMeshRaycastCallback::operator()(int32_t _nodeId, const Ray& _ray) {
	// Add the id of the hit AABB node int32_to
	m_hitAABBNodes.pushBack(_nodeId);
	return _ray.maxFraction;
}

void ConcaveMeshRaycastCallback::raycastTriangles() {
	etk::Vector<int32_t>::Iterator it;
	float smallestHitFraction = m_ray.maxFraction;
	for (it = m_hitAABBNodes.begin(); it != m_hitAABBNodes.end(); ++it) {
		// Get the node data (triangle index and mesh subpart index)
		int32_t* data = m_dynamicAABBTree.getNodeDataInt(*it);
		// Get the triangle vertices for this node from the concave mesh shape
		vec3 trianglePoints[3];
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
			assert(raycastInfo.hitFraction >= 0.0f);
			m_raycastInfo.body = raycastInfo.body;
			m_raycastInfo.proxyShape = raycastInfo.proxyShape;
			m_raycastInfo.hitFraction = raycastInfo.hitFraction;
			m_raycastInfo.worldPoint = raycastInfo.worldPoint;
			m_raycastInfo.worldNormal = raycastInfo.worldNormal;
			m_raycastInfo.meshSubpart = data[0];
			m_raycastInfo.triangleIndex = data[1];
			smallestHitFraction = raycastInfo.hitFraction;
			m_isHit = true;
		}
	}
}

size_t ConcaveMeshShape::getSizeInBytes() const {
	return sizeof(ConcaveMeshShape);
}

void ConcaveMeshShape::getLocalBounds(vec3& _min, vec3& _max) const {
	// Get the AABB of the whole tree
	AABB treeAABB = m_dynamicAABBTree.getRootAABB();
	_min = treeAABB.getMin();
	_max = treeAABB.getMax();
}

void ConcaveMeshShape::setLocalScaling(const vec3& _scaling) {
	CollisionShape::setLocalScaling(_scaling);
	m_dynamicAABBTree.reset();
	initBVHTree();
}

void ConcaveMeshShape::computeLocalInertiaTensor(etk::Matrix3x3& _tensor, float _mass) const {
	// Default inertia tensor
	// Note that this is not very realistic for a concave triangle mesh.
	// However, in most cases, it will only be used static bodies and therefore,
	// the inertia tensor is not used.
	_tensor.setValue(_mass, 0,     0,
	                 0,     _mass, 0,
	                 0,     0,     _mass);
}


