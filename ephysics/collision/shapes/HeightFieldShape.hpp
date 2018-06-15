/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#pragma once

#include <ephysics/collision/shapes/ConcaveShape.hpp>
#include <ephysics/collision/shapes/TriangleShape.hpp>
#include <ephysics/engine/Profiler.hpp>

namespace ephysics {
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
			/**
			 * @brief Data type for the height data of the height field
			 */
			enum HeightDataType {
				HEIGHT_FLOAT_TYPE,
				HEIGHT_DOUBLE_TYPE,
				HEIGHT_INT_TYPE
			};
		protected:
			int32_t m_numberColumns; //!< Number of columns in the grid of the height field
			int32_t m_numberRows; //!< Number of rows in the grid of the height field
			float m_width; //!< Height field width
			float m_length; //!< Height field length
			float m_minHeight; //!< Minimum height of the height field
			float m_maxHeight; //!< Maximum height of the height field
			int32_t m_upAxis; //!< Up axis direction (0 => x, 1 => y, 2 => z)
			float m_integerHeightScale; //!< Height values scale for height field with int32_teger height values
			HeightDataType m_heightDataType; //!< Data type of the height values
			const void*	m_heightFieldData; //!< Array of data with all the height values of the height field
			AABB m_AABB; //!< Local AABB of the height field (without scaling)
			/// DELETED copy-constructor
			HeightFieldShape(const HeightFieldShape&) = delete;
			/// DELETED assignment operator
			HeightFieldShape& operator=(const HeightFieldShape&) = delete;
			bool raycast(const Ray& _ray, RaycastInfo& _raycastInfo, ProxyShape* _proxyShape) const override;
			size_t getSizeInBytes() const override;
			/// Insert all the triangles int32_to the dynamic AABB tree
			void initBVHTree();
			/// Return the three vertices coordinates (in the array outTriangleVertices) of a triangle
			/// given the start vertex index pointer of the triangle.
			void getTriangleVerticesWithIndexPointer(int32_t _subPart,
			                                         int32_t _triangleIndex,
			                                         vec3* _outTriangleVertices) const;
			/// Return the vertex (local-coordinates) of the height field at a given (x,y) position
			vec3 getVertexAt(int32_t _x, int32_t _y) const;
			/// Return the height of a given (x,y) point in the height field
			float getHeightAt(int32_t _x, int32_t _y) const;
			/// Return the closest inside int32_teger grid value of a given floating grid value
			int32_t computeIntegerGridValue(float _value) const;
			/// Compute the min/max grid coords corresponding to the int32_tersection of the AABB of the height field and the AABB to collide
			void computeMinMaxGridCoordinates(int32_t* _minCoords, int32_t* _maxCoords, const AABB& _aabbToCollide) const;
		public:
			/**
			 * @brief Contructor
			 * @param nbGridColumns Number of columns in the grid of the height field
			 * @param nbGridRows Number of rows in the grid of the height field
			 * @param minHeight Minimum height value of the height field
			 * @param maxHeight Maximum height value of the height field
			 * @param heightFieldData Pointer to the first height value data (note that values are shared and not copied)
			 * @param dataType Data type for the height values (int32_t, float, double)
			 * @param upAxis Integer representing the up axis direction (0 for x, 1 for y and 2 for z)
			 * @param int32_tegerHeightScale Scaling factor used to scale the height values (only when height values type is int32_teger)
			 */
			HeightFieldShape(int32_t _nbGridColumns,
			                 int32_t _nbGridRows,
			                 float _minHeight,
			                 float _maxHeight,
			                 const void* _heightFieldData,
			                 HeightDataType _dataType,
			                 int32_t _upAxis = 1, float _integerHeightScale = 1.0f);
			/// Return the number of rows in the height field
			int32_t getNbRows() const;
			/// Return the number of columns in the height field
			int32_t getNbColumns() const;
			/// Return the type of height value in the height field
			HeightDataType getHeightDataType() const;
			void getLocalBounds(vec3& _min, vec3& _max) const override;
			void setLocalScaling(const vec3& _scaling) override;
			void computeLocalInertiaTensor(etk::Matrix3x3& _tensor, float _mass) const override;
			virtual void testAllTriangles(TriangleCallback& _callback, const AABB& _localAABB) const override;
			friend class ConvexTriangleAABBOverlapCallback;
			friend class ConcaveMeshRaycastCallback;
	};

}

