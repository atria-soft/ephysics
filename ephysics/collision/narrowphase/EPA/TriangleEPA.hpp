/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once
#include <ephysics/mathematics/mathematics.hpp>
#include <ephysics/configuration.hpp>
#include <ephysics/collision/narrowphase/EPA/EdgeEPA.hpp>
namespace ephysics {
	bool link(const EdgeEPA& edge0, const EdgeEPA& edge1);
	void halfLink(const EdgeEPA& edge0, const EdgeEPA& edge1);
	/**
	 * @brief Class TriangleEPA
	 * This class represents a triangle face of the current polytope in the EPA algorithm.
	 */
	class TriangleEPA {
		private:
			uint32_t m_indicesVertices[3]; //!< Indices of the vertices y_i of the triangle
			EdgeEPA m_adjacentEdges[3]; //!< Three adjacent edges of the triangle (edges of other triangles)
			bool m_isObsolete; //!< True if the triangle face is visible from the new support point
			float m_determinant; //!< Determinant
			vec3 m_closestPoint; //!< Point v closest to the origin on the affine hull of the triangle
			float m_lambda1; //!< Lambda1 value such that v = lambda0 * y_0 + lambda1 * y_1 + lambda2 * y_2
			float m_lambda2; //!< Lambda1 value such that v = lambda0 * y_0 + lambda1 * y_1 + lambda2 * y_2
			float m_distSquare; //!< Square distance of the point closest point v to the origin
		public:
			/// Private copy-constructor
			TriangleEPA(const TriangleEPA& _triangle) {
				m_indicesVertices[0] = _triangle.m_indicesVertices[0];
				m_indicesVertices[1] = _triangle.m_indicesVertices[1];
				m_indicesVertices[2] = _triangle.m_indicesVertices[2];
				m_adjacentEdges[0] = _triangle.m_adjacentEdges[0];
				m_adjacentEdges[1] = _triangle.m_adjacentEdges[1];
				m_adjacentEdges[2] = _triangle.m_adjacentEdges[2];
				m_isObsolete = _triangle.m_isObsolete;
				m_determinant = _triangle.m_determinant;
				m_closestPoint = _triangle.m_closestPoint;
				m_lambda1 = _triangle.m_lambda1;
				m_lambda2 = _triangle.m_lambda2;
				m_distSquare = _triangle.m_distSquare;
			}
			/// Private assignment operator
			TriangleEPA& operator=(const TriangleEPA& _triangle) {
				m_indicesVertices[0] = _triangle.m_indicesVertices[0];
				m_indicesVertices[1] = _triangle.m_indicesVertices[1];
				m_indicesVertices[2] = _triangle.m_indicesVertices[2];
				m_adjacentEdges[0] = _triangle.m_adjacentEdges[0];
				m_adjacentEdges[1] = _triangle.m_adjacentEdges[1];
				m_adjacentEdges[2] = _triangle.m_adjacentEdges[2];
				m_isObsolete = _triangle.m_isObsolete;
				m_determinant = _triangle.m_determinant;
				m_closestPoint = _triangle.m_closestPoint;
				m_lambda1 = _triangle.m_lambda1;
				m_lambda2 = _triangle.m_lambda2;
				m_distSquare = _triangle.m_distSquare;
				return *this;
			}
			/// Constructor
			TriangleEPA();
			/// Constructor
			TriangleEPA(uint32_t _v1, uint32_t _v2, uint32_t _v3);
			/// Constructor
			void set(uint32_t _v1, uint32_t _v2, uint32_t _v3);
			/// Destructor
			~TriangleEPA();
			/// Return an adjacent edge of the triangle
			EdgeEPA& getAdjacentEdge(int32_t _index) {
				assert(_index >= 0 && _index < 3);
				return m_adjacentEdges[_index];
			}
			/// Set an adjacent edge of the triangle
			void setAdjacentEdge(int32_t _index, EdgeEPA& _edge) {
				assert(_index >=0 && _index < 3);
				m_adjacentEdges[_index] = _edge;
			}
			/// Return the square distance of the closest point to origin
			float getDistSquare() const {
				return m_distSquare;
			}
			/// Set the isObsolete value
			void setIsObsolete(bool _isObsolete) {
				m_isObsolete = _isObsolete;
			}
			/// Return true if the triangle face is obsolete
			bool getIsObsolete() const {
				return m_isObsolete;
			}
			/// Return the point closest to the origin
			const vec3& getClosestPoint() const {
				return m_closestPoint;
			}
			// Return true if the closest point on affine hull is inside the triangle
			bool isClosestPointInternalToTriangle() const {
				return (m_lambda1 >= 0.0 && m_lambda2 >= 0.0 && (m_lambda1 + m_lambda2) <= m_determinant);
			}
			/// Return true if the triangle is visible from a given vertex
			bool isVisibleFromVertex(const vec3* _vertices, uint32_t _index) const {
				vec3 closestToVert = _vertices[_index] - m_closestPoint;
				return (m_closestPoint.dot(closestToVert) > 0.0);
			}
			/// Compute the point v closest to the origin of this triangle
			bool computeClosestPoint(const vec3* _vertices);
			/// Compute the point of an object closest to the origin
			vec3 computeClosestPointOfObject(const vec3* _supportPointsOfObject) const{
				const vec3& p0 = _supportPointsOfObject[m_indicesVertices[0]];
				return p0 + 1.0f/m_determinant * (m_lambda1 * (_supportPointsOfObject[m_indicesVertices[1]] - p0) +
									   m_lambda2 * (_supportPointsOfObject[m_indicesVertices[2]] - p0));
			}
			// Execute the recursive silhouette algorithm from this triangle face.
			/// The parameter "vertices" is an array that contains the vertices of the current polytope and the
			/// parameter "indexNewVertex" is the index of the new vertex in this array. The goal of the
			/// silhouette algorithm is to add the new vertex in the polytope by keeping it convex. Therefore,
			/// the triangle faces that are visible from the new vertex must be removed from the polytope and we
			/// need to add triangle faces where each face contains the new vertex and an edge of the silhouette.
			/// The silhouette is the connected set of edges that are part of the border between faces that
			/// are seen and faces that are not seen from the new vertex. This method starts from the nearest
			/// face from the new vertex, computes the silhouette and create the new faces from the new vertex in
			/// order that we always have a convex polytope. The faces visible from the new vertex are set
			/// obselete and will not be considered as being a candidate face in the future.
			bool computeSilhouette(const vec3* _vertices, uint32_t _index, TrianglesStore& _triangleStore);
			/// Access operator
			uint32_t operator[](int32_t _pos) const {
				assert(_pos >= 0 && _pos <3);
				return m_indicesVertices[_pos];
			}
			/// Link an edge with another one. It means that the current edge of a triangle will
			/// be associated with the edge of another triangle in order that both triangles
			/// are neighbour along both edges).
			friend bool link(const EdgeEPA& _edge0, const EdgeEPA& _edge1);
			/// Make an half link of an edge with another one from another triangle. An half-link
			/// between an edge "edge0" and an edge "edge1" represents the fact that "edge1" is an
			/// adjacent edge of "edge0" but not the opposite. The opposite edge connection will
			/// be made later.
			friend void halfLink(const EdgeEPA& _edge0, const EdgeEPA& _edge1);
	};

}
