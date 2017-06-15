/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/collision/shapes/ConcaveShape.h>
#include <ephysics/collision/shapes/TriangleShape.h>
#include <ephysics/engine/Profiler.h>

namespace reactphysics3d {
	class HeightFieldShape;
	/**
	 * @brief This class is used for testing AABB and triangle overlap for raycasting
	 */
	class TriangleOverlapCallback : public TriangleCallback {
		protected:
			const Ray& m_ray;
			ProxyShape* m_proxyShape;
			RaycastInfo& m_raycastInfo;
			bool m_isHit;
			float m_smallestHitFraction;
			const HeightFieldShape& m_heightFieldShape;
		public:
			TriangleOverlapCallback(const Ray& _ray,
			                        ProxyShape* _proxyShape,
			                        RaycastInfo& _raycastInfo,
			                        const HeightFieldShape& _heightFieldShape):
			  m_ray(_ray),
			  m_proxyShape(_proxyShape),
			  m_raycastInfo(_raycastInfo),
			  m_heightFieldShape(_heightFieldShape) {
				m_isHit = false;
				m_smallestHitFraction = m_ray.maxFraction;
			}
			bool getIsHit() const {
				return m_isHit;
			}
			/// Raycast test between a ray and a triangle of the heightfield
			virtual void testTriangle(const vec3* _trianglePoints);
	};


/**
 * @brief This class represents a static height field that can be used to represent
 * a terrain. The height field is made of a grid with rows and columns with a
 * height value at each grid point. Note that the height values are not copied int32_to the shape
 * but are shared instead. The height values can be of type int32_teger, float or double.
 * When creating a HeightFieldShape, you need to specify the minimum and maximum height value of
 * your height field. Note that the HeightFieldShape will be re-centered based on its AABB. It means
 * that for instance, if the minimum height value is -200 and the maximum value is 400, the final
 * minimum height of the field in the simulation will be -300 and the maximum height will be 300.
 */
class HeightFieldShape : public ConcaveShape {

	public:

		/// Data type for the height data of the height field
		enum HeightDataType {HEIGHT_FLOAT_TYPE, HEIGHT_DOUBLE_TYPE, HEIGHT_INT_TYPE};

	protected:

		// -------------------- Attributes -------------------- //

		/// Number of columns in the grid of the height field
		int32_t m_numberColumns;

		/// Number of rows in the grid of the height field
		int32_t m_numberRows;

		/// Height field width
		float m_width;

		/// Height field length
		float m_length;

		/// Minimum height of the height field
		float m_minHeight;

		/// Maximum height of the height field
		float m_maxHeight;

		/// Up axis direction (0 => x, 1 => y, 2 => z)
		int32_t m_upAxis;

		/// Height values scale for height field with int32_teger height values
		float m_integerHeightScale;

		/// Data type of the height values
		HeightDataType m_heightDataType;

		/// Array of data with all the height values of the height field
		const void*	m_heightFieldData;

		/// Local AABB of the height field (without scaling)
		AABB m_AABB;

		// -------------------- Methods -------------------- //

		/// Private copy-constructor
		HeightFieldShape(const HeightFieldShape& shape);

		/// Private assignment operator
		HeightFieldShape& operator=(const HeightFieldShape& shape);

		/// Raycast method with feedback information
		virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const;

		/// Return the number of bytes used by the collision shape
		virtual size_t getSizeInBytes() const;

		/// Insert all the triangles int32_to the dynamic AABB tree
		void initBVHTree();

		/// Return the three vertices coordinates (in the array outTriangleVertices) of a triangle
		/// given the start vertex index pointer of the triangle.
		void getTriangleVerticesWithIndexPointer(int32_t subPart, int32_t triangleIndex,
												 vec3* outTriangleVertices) const;

		/// Return the vertex (local-coordinates) of the height field at a given (x,y) position
		vec3 getVertexAt(int32_t x, int32_t y) const;

		/// Return the height of a given (x,y) point in the height field
		float getHeightAt(int32_t x, int32_t y) const;

		/// Return the closest inside int32_teger grid value of a given floating grid value
		int32_t computeIntegerGridValue(float value) const;

		/// Compute the min/max grid coords corresponding to the int32_tersection of the AABB of the height field and the AABB to collide
		void computeMinMaxGridCoordinates(int32_t* minCoords, int32_t* maxCoords, const AABB& aabbToCollide) const;

	public:

		/// Constructor
		HeightFieldShape(int32_t nbGridColumns, int32_t nbGridRows, float minHeight, float maxHeight,
						 const void* heightFieldData, HeightDataType dataType,
						 int32_t upAxis = 1, float int32_tegerHeightScale = 1.0f);

		/// Destructor
		~HeightFieldShape();

		/// Return the number of rows in the height field
		int32_t getNbRows() const;

		/// Return the number of columns in the height field
		int32_t getNbColumns() const;

		/// Return the type of height value in the height field
		HeightDataType getHeightDataType() const;

		/// Return the local bounds of the shape in x, y and z directions.
		virtual void getLocalBounds(vec3& min, vec3& max) const;

		/// Set the local scaling vector of the collision shape
		virtual void setLocalScaling(const vec3& scaling);

		/// Return the local inertia tensor of the collision shape
		virtual void computeLocalInertiaTensor(etk::Matrix3x3& tensor, float mass) const;

		/// Use a callback method on all triangles of the concave shape inside a given AABB
		virtual void testAllTriangles(TriangleCallback& callback, const AABB& localAABB) const;

		// ---------- Friendship ----------- //

		friend class ConvexTriangleAABBOverlapCallback;
		friend class ConcaveMeshRaycastCallback;
};

// Return the number of rows in the height field
inline int32_t HeightFieldShape::getNbRows() const {
	return m_numberRows;
}

// Return the number of columns in the height field
inline int32_t HeightFieldShape::getNbColumns() const {
	return m_numberColumns;
}

// Return the type of height value in the height field
inline HeightFieldShape::HeightDataType HeightFieldShape::getHeightDataType() const {
	return m_heightDataType;
}

// Return the number of bytes used by the collision shape
inline size_t HeightFieldShape::getSizeInBytes() const {
	return sizeof(HeightFieldShape);
}

// Set the local scaling vector of the collision shape
inline void HeightFieldShape::setLocalScaling(const vec3& scaling) {
	CollisionShape::setLocalScaling(scaling);
}

// Return the height of a given (x,y) point in the height field
inline float HeightFieldShape::getHeightAt(int32_t x, int32_t y) const {

	switch(m_heightDataType) {
		case HEIGHT_FLOAT_TYPE : return ((float*)m_heightFieldData)[y * m_numberColumns + x];
		case HEIGHT_DOUBLE_TYPE : return ((double*)m_heightFieldData)[y * m_numberColumns + x];
		case HEIGHT_INT_TYPE : return ((int32_t*)m_heightFieldData)[y * m_numberColumns + x] * m_integerHeightScale;
		default: assert(false); return 0;
	}
}

// Return the closest inside int32_teger grid value of a given floating grid value
inline int32_t HeightFieldShape::computeIntegerGridValue(float value) const {
	return (value < 0.0f) ? value - 0.5f : value + float(0.5);
}

// Return the local inertia tensor
/**
 * @param[out] tensor The 3x3 inertia tensor matrix of the shape in local-space
 *					coordinates
 * @param mass Mass to use to compute the inertia tensor of the collision shape
 */
inline void HeightFieldShape::computeLocalInertiaTensor(etk::Matrix3x3& tensor, float mass) const {

	// Default inertia tensor
	// Note that this is not very realistic for a concave triangle mesh.
	// However, in most cases, it will only be used static bodies and therefore,
	// the inertia tensor is not used.
	tensor.setValue(mass, 0, 0,
						0, mass, 0,
						0, 0, mass);
}

}

