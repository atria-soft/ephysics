/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/collision/shapes/ConvexShape.h>
#include <ephysics/engine/CollisionWorld.h>
#include <ephysics/mathematics/mathematics.h>
#include <ephysics/collision/TriangleMesh.h>
#include <ephysics/collision/narrowphase/GJK/GJKAlgorithm.h>
#include <vector>
#include <set>
#include <map>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Declaration
class CollisionWorld;

// Class ConvexMeshShape
/**
 * This class represents a convex mesh shape. In order to create a convex mesh shape, you
 * need to indicate the local-space position of the mesh vertices. You do it either by
 * passing a vertices array to the constructor or using the addVertex() method. Make sure
 * that the set of vertices that you use to create the shape are indeed part of a convex
 * mesh. The center of mass of the shape will be at the origin of the local-space geometry
 * that you use to create the mesh. The method used for collision detection with a convex
 * mesh shape has an O(n) running time with "n" beeing the number of vertices in the mesh.
 * Therefore, you should try not to use too many vertices. However, it is possible to speed
 * up the collision detection by using the edges information of your mesh. The running time
 * of the collision detection that uses the edges is almost O(1) constant time at the cost
 * of additional memory used to store the vertices. You can indicate edges information
 * with the addEdge() method. Then, you must use the setIsEdgesInformationUsed(true) method
 * in order to use the edges information for collision detection.
 */
class ConvexMeshShape : public ConvexShape {

	protected :

		// -------------------- Attributes -------------------- //

		/// Array with the vertices of the mesh
		std::vector<Vector3> m_vertices;

		/// Number of vertices in the mesh
		uint32_t m_numberVertices;

		/// Mesh minimum bounds in the three local x, y and z directions
		Vector3 m_minBounds;

		/// Mesh maximum bounds in the three local x, y and z directions
		Vector3 m_maxBounds;

		/// True if the shape contains the edges of the convex mesh in order to
		/// make the collision detection faster
		bool m_isEdgesInformationUsed;

		/// Adjacency list representing the edges of the mesh
		std::map<uint32_t, std::set<uint32_t> > m_edgesAdjacencyList;

		// -------------------- Methods -------------------- //

		/// Private copy-constructor
		ConvexMeshShape(const ConvexMeshShape& shape);

		/// Private assignment operator
		ConvexMeshShape& operator=(const ConvexMeshShape& shape);

		/// Recompute the bounds of the mesh
		void recalculateBounds();

		/// Set the scaling vector of the collision shape
		virtual void setLocalScaling(const Vector3& scaling);

		/// Return a local support point in a given direction without the object margin.
		virtual Vector3 getLocalSupportPointWithoutMargin(const Vector3& direction,
														  void** cachedCollisionData) const;

		/// Return true if a point is inside the collision shape
		virtual bool testPointInside(const Vector3& localPoint, ProxyShape* proxyShape) const;

		/// Raycast method with feedback information
		virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const;

		/// Return the number of bytes used by the collision shape
		virtual size_t getSizeInBytes() const;

	public :

		// -------------------- Methods -------------------- //

		/// Constructor to initialize with an array of 3D vertices.
		ConvexMeshShape(const float* arrayVertices, uint32_t nbVertices, int32_t stride,
						float margin = OBJECT_MARGIN);

		/// Constructor to initialize with a triangle vertex array
		ConvexMeshShape(TriangleVertexArray* triangleVertexArray, bool isEdgesInformationUsed = true,
						float margin = OBJECT_MARGIN);

		/// Constructor.
		ConvexMeshShape(float margin = OBJECT_MARGIN);

		/// Destructor
		virtual ~ConvexMeshShape();

		/// Return the local bounds of the shape in x, y and z directions
		virtual void getLocalBounds(Vector3& min, Vector3& max) const;

		/// Return the local inertia tensor of the collision shape.
		virtual void computeLocalInertiaTensor(Matrix3x3& tensor, float mass) const;

		/// Add a vertex int32_to the convex mesh
		void addVertex(const Vector3& vertex);

		/// Add an edge int32_to the convex mesh by specifying the two vertex indices of the edge.
		void addEdge(uint32_t v1, uint32_t v2);

		/// Return true if the edges information is used to speed up the collision detection
		bool isEdgesInformationUsed() const;

		/// Set the variable to know if the edges information is used to speed up the
		/// collision detection
		void setIsEdgesInformationUsed(bool isEdgesUsed);
};

/// Set the scaling vector of the collision shape
inline void ConvexMeshShape::setLocalScaling(const Vector3& scaling) {
	ConvexShape::setLocalScaling(scaling);
	recalculateBounds();
}

// Return the number of bytes used by the collision shape
inline size_t ConvexMeshShape::getSizeInBytes() const {
	return sizeof(ConvexMeshShape);
}

// Return the local bounds of the shape in x, y and z directions
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
inline void ConvexMeshShape::getLocalBounds(Vector3& min, Vector3& max) const {
	min = m_minBounds;
	max = m_maxBounds;
}

// Return the local inertia tensor of the collision shape.
/// The local inertia tensor of the convex mesh is approximated using the inertia tensor
/// of its bounding box.
/**
* @param[out] tensor The 3x3 inertia tensor matrix of the shape in local-space
*					coordinates
* @param mass Mass to use to compute the inertia tensor of the collision shape
*/
inline void ConvexMeshShape::computeLocalInertiaTensor(Matrix3x3& tensor, float mass) const {
	float factor = (float(1.0) / float(3.0)) * mass;
	Vector3 realExtent = float(0.5) * (m_maxBounds - m_minBounds);
	assert(realExtent.x > 0 && realExtent.y > 0 && realExtent.z > 0);
	float xSquare = realExtent.x * realExtent.x;
	float ySquare = realExtent.y * realExtent.y;
	float zSquare = realExtent.z * realExtent.z;
	tensor.setAllValues(factor * (ySquare + zSquare), 0.0, 0.0,
						0.0, factor * (xSquare + zSquare), 0.0,
						0.0, 0.0, factor * (xSquare + ySquare));
}

// Add a vertex int32_to the convex mesh
/**
 * @param vertex Vertex to be added
 */
inline void ConvexMeshShape::addVertex(const Vector3& vertex) {

	// Add the vertex in to vertices array
	m_vertices.push_back(vertex);
	m_numberVertices++;

	// Update the bounds of the mesh
	if (vertex.x * mScaling.x > m_maxBounds.x) m_maxBounds.x = vertex.x * mScaling.x;
	if (vertex.x * mScaling.x < m_minBounds.x) m_minBounds.x = vertex.x * mScaling.x;
	if (vertex.y * mScaling.y > m_maxBounds.y) m_maxBounds.y = vertex.y * mScaling.y;
	if (vertex.y * mScaling.y < m_minBounds.y) m_minBounds.y = vertex.y * mScaling.y;
	if (vertex.z * mScaling.z > m_maxBounds.z) m_maxBounds.z = vertex.z * mScaling.z;
	if (vertex.z * mScaling.z < m_minBounds.z) m_minBounds.z = vertex.z * mScaling.z;
}

// Add an edge int32_to the convex mesh by specifying the two vertex indices of the edge.
/// Note that the vertex indices start at zero and need to correspond to the order of
/// the vertices in the vertices array in the constructor or the order of the calls
/// of the addVertex() methods that you use to add vertices int32_to the convex mesh.
/**
* @param v1 Index of the first vertex of the edge to add
* @param v2 Index of the second vertex of the edge to add
*/
inline void ConvexMeshShape::addEdge(uint32_t v1, uint32_t v2) {

	// If the entry for vertex v1 does not exist in the adjacency list
	if (m_edgesAdjacencyList.count(v1) == 0) {
		m_edgesAdjacencyList.insert(std::make_pair(v1, std::set<uint32_t>()));
	}

	// If the entry for vertex v2 does not exist in the adjacency list
	if (m_edgesAdjacencyList.count(v2) == 0) {
		m_edgesAdjacencyList.insert(std::make_pair(v2, std::set<uint32_t>()));
	}

	// Add the edge in the adjacency list
	m_edgesAdjacencyList[v1].insert(v2);
	m_edgesAdjacencyList[v2].insert(v1);
}

// Return true if the edges information is used to speed up the collision detection
/**
 * @return True if the edges information is used and false otherwise
 */
inline bool ConvexMeshShape::isEdgesInformationUsed() const {
	return m_isEdgesInformationUsed;
}

// Set the variable to know if the edges information is used to speed up the
// collision detection
/**
 * @param isEdgesUsed True if you want to use the edges information to speed up
 *					the collision detection with the convex mesh shape
 */
inline void ConvexMeshShape::setIsEdgesInformationUsed(bool isEdgesUsed) {
	m_isEdgesInformationUsed = isEdgesUsed;
}

// Return true if a point is inside the collision shape
inline bool ConvexMeshShape::testPointInside(const Vector3& localPoint,
											 ProxyShape* proxyShape) const {

	// Use the GJK algorithm to test if the point is inside the convex mesh
	return proxyShape->mBody->mWorld.m_collisionDetection.
		   mNarrowPhaseGJKAlgorithm.testPointInside(localPoint, proxyShape);
}

}
