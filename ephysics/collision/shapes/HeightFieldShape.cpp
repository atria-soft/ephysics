/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/shapes/HeightFieldShape.hpp>

using namespace ephysics;

// Constructor
/**
 * @param nbGridColumns Number of columns in the grid of the height field
 * @param nbGridRows Number of rows in the grid of the height field
 * @param minHeight Minimum height value of the height field
 * @param maxHeight Maximum height value of the height field
 * @param heightFieldData Pointer to the first height value data (note that values are shared and not copied)
 * @param dataType Data type for the height values (int32_t, float, double)
 * @param upAxis Integer representing the up axis direction (0 for x, 1 for y and 2 for z)
 * @param int32_tegerHeightScale Scaling factor used to scale the height values (only when height values type is int32_teger)
 */
HeightFieldShape::HeightFieldShape(int32_t nbGridColumns, int32_t nbGridRows, float minHeight, float maxHeight,
								   const void* heightFieldData, HeightDataType dataType, int32_t upAxis,
								   float int32_tegerHeightScale)
				 : ConcaveShape(HEIGHTFIELD), m_numberColumns(nbGridColumns), m_numberRows(nbGridRows),
				   m_width(nbGridColumns - 1), m_length(nbGridRows - 1), m_minHeight(minHeight),
				   m_maxHeight(maxHeight), m_upAxis(upAxis), m_integerHeightScale(int32_tegerHeightScale),
				   m_heightDataType(dataType) {

	assert(nbGridColumns >= 2);
	assert(nbGridRows >= 2);
	assert(m_width >= 1);
	assert(m_length >= 1);
	assert(minHeight <= maxHeight);
	assert(upAxis == 0 || upAxis == 1 || upAxis == 2);

	m_heightFieldData = heightFieldData;

	float halfHeight = (m_maxHeight - m_minHeight) * 0.5f;
	assert(halfHeight >= 0);

	// Compute the local AABB of the height field
	if (m_upAxis == 0) {
		m_AABB.setMin(vec3(-halfHeight, -m_width * 0.5f, -m_length * float(0.5)));
		m_AABB.setMax(vec3(halfHeight, m_width * 0.5f, m_length* float(0.5)));
	}
	else if (m_upAxis == 1) {
		m_AABB.setMin(vec3(-m_width * 0.5f, -halfHeight, -m_length * float(0.5)));
		m_AABB.setMax(vec3(m_width * 0.5f, halfHeight, m_length * float(0.5)));
	}
	else if (m_upAxis == 2) {
		m_AABB.setMin(vec3(-m_width * 0.5f, -m_length * float(0.5), -halfHeight));
		m_AABB.setMax(vec3(m_width * 0.5f, m_length * float(0.5), halfHeight));
	}
}

// Destructor
HeightFieldShape::~HeightFieldShape() {

}

// Return the local bounds of the shape in x, y and z directions.
// This method is used to compute the AABB of the box
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
void HeightFieldShape::getLocalBounds(vec3& min, vec3& max) const {
	min = m_AABB.getMin() * m_scaling;
	max = m_AABB.getMax() * m_scaling;
}

// Test collision with the triangles of the height field shape. The idea is to use the AABB
// of the body when need to test and see against which triangles of the height-field we need
// to test for collision. We compute the sub-grid points that are inside the other body's AABB
// and then for each rectangle in the sub-grid we generate two triangles that we use to test collision.
void HeightFieldShape::testAllTriangles(TriangleCallback& callback, const AABB& localAABB) const {

   // Compute the non-scaled AABB
   vec3 inverseScaling(1.0f / m_scaling.x(), 1.0f / m_scaling.y(), float(1.0) / m_scaling.z());
   AABB aabb(localAABB.getMin() * inverseScaling, localAABB.getMax() * inverseScaling);

   // Compute the int32_teger grid coordinates inside the area we need to test for collision
   int32_t minGridCoords[3];
   int32_t maxGridCoords[3];
   computeMinMaxGridCoordinates(minGridCoords, maxGridCoords, aabb);

   // Compute the starting and ending coords of the sub-grid according to the up axis
   int32_t iMin = 0;
   int32_t iMax = 0;
   int32_t jMin = 0;
   int32_t jMax = 0;
   switch(m_upAxis) {
		case 0 : iMin = clamp(minGridCoords[1], 0, m_numberColumns - 1);
				 iMax = clamp(maxGridCoords[1], 0, m_numberColumns - 1);
				 jMin = clamp(minGridCoords[2], 0, m_numberRows - 1);
				 jMax = clamp(maxGridCoords[2], 0, m_numberRows - 1);
				 break;
		case 1 : iMin = clamp(minGridCoords[0], 0, m_numberColumns - 1);
				 iMax = clamp(maxGridCoords[0], 0, m_numberColumns - 1);
				 jMin = clamp(minGridCoords[2], 0, m_numberRows - 1);
				 jMax = clamp(maxGridCoords[2], 0, m_numberRows - 1);
				 break;
		case 2 : iMin = clamp(minGridCoords[0], 0, m_numberColumns - 1);
				 iMax = clamp(maxGridCoords[0], 0, m_numberColumns - 1);
				 jMin = clamp(minGridCoords[1], 0, m_numberRows - 1);
				 jMax = clamp(maxGridCoords[1], 0, m_numberRows - 1);
				 break;
   }

   assert(iMin >= 0 && iMin < m_numberColumns);
   assert(iMax >= 0 && iMax < m_numberColumns);
   assert(jMin >= 0 && jMin < m_numberRows);
   assert(jMax >= 0 && jMax < m_numberRows);

   // For each sub-grid points (except the last ones one each dimension)
   for (int32_t i = iMin; i < iMax; i++) {
	   for (int32_t j = jMin; j < jMax; j++) {

		   // Compute the four point of the current quad
		   vec3 p1 = getVertexAt(i, j);
		   vec3 p2 = getVertexAt(i, j + 1);
		   vec3 p3 = getVertexAt(i + 1, j);
		   vec3 p4 = getVertexAt(i + 1, j + 1);

		   // Generate the first triangle for the current grid rectangle
		   vec3 trianglePoints[3] = {p1, p2, p3};

		   // Test collision against the first triangle
		   callback.testTriangle(trianglePoints);

		   // Generate the second triangle for the current grid rectangle
		   trianglePoints[0] = p3;
		   trianglePoints[1] = p2;
		   trianglePoints[2] = p4;

		   // Test collision against the second triangle
		   callback.testTriangle(trianglePoints);
	   }
   }
}

// Compute the min/max grid coords corresponding to the int32_tersection of the AABB of the height field and
// the AABB to collide
void HeightFieldShape::computeMinMaxGridCoordinates(int32_t* minCoords, int32_t* maxCoords, const AABB& aabbToCollide) const {

	// Clamp the min/max coords of the AABB to collide inside the height field AABB
	vec3 minPoint = etk::max(aabbToCollide.getMin(), m_AABB.getMin());
	minPoint = etk::min(minPoint, m_AABB.getMax());

	vec3 maxPoint = etk::min(aabbToCollide.getMax(), m_AABB.getMax());
	maxPoint = etk::max(maxPoint, m_AABB.getMin());

	// Translate the min/max points such that the we compute grid points from [0 ... mNbWidthGridPoints]
	// and from [0 ... mNbLengthGridPoints] because the AABB coordinates range are [-mWdith/2 ... m_width/2]
	// and [-m_length/2 ... m_length/2]
	const vec3 translateVec = m_AABB.getExtent() * 0.5f;
	minPoint += translateVec;
	maxPoint += translateVec;

	// Convert the floating min/max coords of the AABB int32_to closest int32_teger
	// grid values (note that we use the closest grid coordinate that is out
	// of the AABB)
	minCoords[0] = computeIntegerGridValue(minPoint.x()) - 1;
	minCoords[1] = computeIntegerGridValue(minPoint.y()) - 1;
	minCoords[2] = computeIntegerGridValue(minPoint.z()) - 1;

	maxCoords[0] = computeIntegerGridValue(maxPoint.x()) + 1;
	maxCoords[1] = computeIntegerGridValue(maxPoint.y()) + 1;
	maxCoords[2] = computeIntegerGridValue(maxPoint.z()) + 1;
}

// Raycast method with feedback information
/// Note that only the first triangle hit by the ray in the mesh will be returned, even if
/// the ray hits many triangles.
bool HeightFieldShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const {

	// TODO : Implement raycasting without using an AABB for the ray
	//		but using a dynamic AABB tree or octree instead

	PROFILE("HeightFieldShape::raycast()");

	TriangleOverlapCallback triangleCallback(ray, proxyShape, raycastInfo, *this);

	// Compute the AABB for the ray
	const vec3 rayEnd = ray.point1 + ray.maxFraction * (ray.point2 - ray.point1);
	const AABB rayAABB(etk::min(ray.point1, rayEnd), etk::max(ray.point1, rayEnd));

	testAllTriangles(triangleCallback, rayAABB);

	return triangleCallback.getIsHit();
}

// Return the vertex (local-coordinates) of the height field at a given (x,y) position
vec3 HeightFieldShape::getVertexAt(int32_t x, int32_t y) const {

	// Get the height value
	const float height = getHeightAt(x, y);

	// Height values origin
	const float heightOrigin = -(m_maxHeight - m_minHeight) * 0.5f - m_minHeight;

	vec3 vertex;
	switch (m_upAxis) {
		case 0: vertex = vec3(heightOrigin + height, -m_width * 0.5f + x, -m_length * float(0.5) + y);
				break;
		case 1: vertex = vec3(-m_width * 0.5f + x, heightOrigin + height, -m_length * float(0.5) + y);
				break;
		case 2: vertex = vec3(-m_width * 0.5f + x, -m_length * float(0.5) + y, heightOrigin + height);
				break;
		default: assert(false);
	}

	assert(m_AABB.contains(vertex));

	return vertex * m_scaling;
}

// Raycast test between a ray and a triangle of the heightfield
void TriangleOverlapCallback::testTriangle(const vec3* trianglePoints) {

	// Create a triangle collision shape
	float margin = m_heightFieldShape.getTriangleMargin();
	TriangleShape triangleShape(trianglePoints[0], trianglePoints[1], trianglePoints[2], margin);
	triangleShape.setRaycastTestType(m_heightFieldShape.getRaycastTestType());

	// Ray casting test against the collision shape
	RaycastInfo raycastInfo;
	bool isTriangleHit = triangleShape.raycast(m_ray, raycastInfo, m_proxyShape);

	// If the ray hit the collision shape
	if (isTriangleHit && raycastInfo.hitFraction <= m_smallestHitFraction) {

		assert(raycastInfo.hitFraction >= 0.0f);

		m_raycastInfo.body = raycastInfo.body;
		m_raycastInfo.proxyShape = raycastInfo.proxyShape;
		m_raycastInfo.hitFraction = raycastInfo.hitFraction;
		m_raycastInfo.worldPoint = raycastInfo.worldPoint;
		m_raycastInfo.worldNormal = raycastInfo.worldNormal;
		m_raycastInfo.meshSubpart = -1;
		m_raycastInfo.triangleIndex = -1;

		m_smallestHitFraction = raycastInfo.hitFraction;
		m_isHit = true;
	}
}

// Return the number of rows in the height field
int32_t HeightFieldShape::getNbRows() const {
	return m_numberRows;
}

// Return the number of columns in the height field
int32_t HeightFieldShape::getNbColumns() const {
	return m_numberColumns;
}

// Return the type of height value in the height field
HeightFieldShape::HeightDataType HeightFieldShape::getHeightDataType() const {
	return m_heightDataType;
}

// Return the number of bytes used by the collision shape
size_t HeightFieldShape::getSizeInBytes() const {
	return sizeof(HeightFieldShape);
}

// Set the local scaling vector of the collision shape
void HeightFieldShape::setLocalScaling(const vec3& scaling) {
	CollisionShape::setLocalScaling(scaling);
}

// Return the height of a given (x,y) point in the height field
float HeightFieldShape::getHeightAt(int32_t x, int32_t y) const {

	switch(m_heightDataType) {
		case HEIGHT_FLOAT_TYPE : return ((float*)m_heightFieldData)[y * m_numberColumns + x];
		case HEIGHT_DOUBLE_TYPE : return ((double*)m_heightFieldData)[y * m_numberColumns + x];
		case HEIGHT_INT_TYPE : return ((int32_t*)m_heightFieldData)[y * m_numberColumns + x] * m_integerHeightScale;
		default: assert(false); return 0;
	}
}

// Return the closest inside int32_teger grid value of a given floating grid value
int32_t HeightFieldShape::computeIntegerGridValue(float value) const {
	return (value < 0.0f) ? value - 0.5f : value + float(0.5);
}

// Return the local inertia tensor
/**
 * @param[out] tensor The 3x3 inertia tensor matrix of the shape in local-space
 *					coordinates
 * @param mass Mass to use to compute the inertia tensor of the collision shape
 */
void HeightFieldShape::computeLocalInertiaTensor(etk::Matrix3x3& tensor, float mass) const {

	// Default inertia tensor
	// Note that this is not very realistic for a concave triangle mesh.
	// However, in most cases, it will only be used static bodies and therefore,
	// the inertia tensor is not used.
	tensor.setValue(mass, 0, 0,
						0, mass, 0,
						0, 0, mass);
}
