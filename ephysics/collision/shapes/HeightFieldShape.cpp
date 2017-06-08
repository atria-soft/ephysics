/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/shapes/HeightFieldShape.h>

using namespace reactphysics3d;

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
				 : ConcaveShape(HEIGHTFIELD), mNbColumns(nbGridColumns), mNbRows(nbGridRows),
				   mWidth(nbGridColumns - 1), mLength(nbGridRows - 1), mMinHeight(minHeight),
				   mMaxHeight(maxHeight), mUpAxis(upAxis), mIntegerHeightScale(int32_tegerHeightScale),
				   mHeightDataType(dataType) {

	assert(nbGridColumns >= 2);
	assert(nbGridRows >= 2);
	assert(mWidth >= 1);
	assert(mLength >= 1);
	assert(minHeight <= maxHeight);
	assert(upAxis == 0 || upAxis == 1 || upAxis == 2);

	mHeightFieldData = heightFieldData;

	float halfHeight = (mMaxHeight - mMinHeight) * float(0.5);
	assert(halfHeight >= 0);

	// Compute the local AABB of the height field
	if (mUpAxis == 0) {
		mAABB.setMin(Vector3(-halfHeight, -mWidth * float(0.5), -mLength * float(0.5)));
		mAABB.setMax(Vector3(halfHeight, mWidth * float(0.5), mLength* float(0.5)));
	}
	else if (mUpAxis == 1) {
		mAABB.setMin(Vector3(-mWidth * float(0.5), -halfHeight, -mLength * float(0.5)));
		mAABB.setMax(Vector3(mWidth * float(0.5), halfHeight, mLength * float(0.5)));
	}
	else if (mUpAxis == 2) {
		mAABB.setMin(Vector3(-mWidth * float(0.5), -mLength * float(0.5), -halfHeight));
		mAABB.setMax(Vector3(mWidth * float(0.5), mLength * float(0.5), halfHeight));
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
void HeightFieldShape::getLocalBounds(Vector3& min, Vector3& max) const {
	min = mAABB.getMin() * mScaling;
	max = mAABB.getMax() * mScaling;
}

// Test collision with the triangles of the height field shape. The idea is to use the AABB
// of the body when need to test and see against which triangles of the height-field we need
// to test for collision. We compute the sub-grid points that are inside the other body's AABB
// and then for each rectangle in the sub-grid we generate two triangles that we use to test collision.
void HeightFieldShape::testAllTriangles(TriangleCallback& callback, const AABB& localAABB) const {

   // Compute the non-scaled AABB
   Vector3 inverseScaling(float(1.0) / mScaling.x, float(1.0) / mScaling.y, float(1.0) / mScaling.z);
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
   switch(mUpAxis) {
		case 0 : iMin = clamp(minGridCoords[1], 0, mNbColumns - 1);
				 iMax = clamp(maxGridCoords[1], 0, mNbColumns - 1);
				 jMin = clamp(minGridCoords[2], 0, mNbRows - 1);
				 jMax = clamp(maxGridCoords[2], 0, mNbRows - 1);
				 break;
		case 1 : iMin = clamp(minGridCoords[0], 0, mNbColumns - 1);
				 iMax = clamp(maxGridCoords[0], 0, mNbColumns - 1);
				 jMin = clamp(minGridCoords[2], 0, mNbRows - 1);
				 jMax = clamp(maxGridCoords[2], 0, mNbRows - 1);
				 break;
		case 2 : iMin = clamp(minGridCoords[0], 0, mNbColumns - 1);
				 iMax = clamp(maxGridCoords[0], 0, mNbColumns - 1);
				 jMin = clamp(minGridCoords[1], 0, mNbRows - 1);
				 jMax = clamp(maxGridCoords[1], 0, mNbRows - 1);
				 break;
   }

   assert(iMin >= 0 && iMin < mNbColumns);
   assert(iMax >= 0 && iMax < mNbColumns);
   assert(jMin >= 0 && jMin < mNbRows);
   assert(jMax >= 0 && jMax < mNbRows);

   // For each sub-grid points (except the last ones one each dimension)
   for (int32_t i = iMin; i < iMax; i++) {
	   for (int32_t j = jMin; j < jMax; j++) {

		   // Compute the four point of the current quad
		   Vector3 p1 = getVertexAt(i, j);
		   Vector3 p2 = getVertexAt(i, j + 1);
		   Vector3 p3 = getVertexAt(i + 1, j);
		   Vector3 p4 = getVertexAt(i + 1, j + 1);

		   // Generate the first triangle for the current grid rectangle
		   Vector3 trianglePoints[3] = {p1, p2, p3};

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
	Vector3 minPoint = Vector3::max(aabbToCollide.getMin(), mAABB.getMin());
	minPoint = Vector3::min(minPoint, mAABB.getMax());

	Vector3 maxPoint = Vector3::min(aabbToCollide.getMax(), mAABB.getMax());
	maxPoint = Vector3::max(maxPoint, mAABB.getMin());

	// Translate the min/max points such that the we compute grid points from [0 ... mNbWidthGridPoints]
	// and from [0 ... mNbLengthGridPoints] because the AABB coordinates range are [-mWdith/2 ... mWidth/2]
	// and [-mLength/2 ... mLength/2]
	const Vector3 translateVec = mAABB.getExtent() * float(0.5);
	minPoint += translateVec;
	maxPoint += translateVec;

	// Convert the floating min/max coords of the AABB int32_to closest int32_teger
	// grid values (note that we use the closest grid coordinate that is out
	// of the AABB)
	minCoords[0] = computeIntegerGridValue(minPoint.x) - 1;
	minCoords[1] = computeIntegerGridValue(minPoint.y) - 1;
	minCoords[2] = computeIntegerGridValue(minPoint.z) - 1;

	maxCoords[0] = computeIntegerGridValue(maxPoint.x) + 1;
	maxCoords[1] = computeIntegerGridValue(maxPoint.y) + 1;
	maxCoords[2] = computeIntegerGridValue(maxPoint.z) + 1;
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
	const Vector3 rayEnd = ray.point1 + ray.maxFraction * (ray.point2 - ray.point1);
	const AABB rayAABB(Vector3::min(ray.point1, rayEnd), Vector3::max(ray.point1, rayEnd));

	testAllTriangles(triangleCallback, rayAABB);

	return triangleCallback.getIsHit();
}

// Return the vertex (local-coordinates) of the height field at a given (x,y) position
Vector3 HeightFieldShape::getVertexAt(int32_t x, int32_t y) const {

	// Get the height value
	const float height = getHeightAt(x, y);

	// Height values origin
	const float heightOrigin = -(mMaxHeight - mMinHeight) * float(0.5) - mMinHeight;

	Vector3 vertex;
	switch (mUpAxis) {
		case 0: vertex = Vector3(heightOrigin + height, -mWidth * float(0.5) + x, -mLength * float(0.5) + y);
				break;
		case 1: vertex = Vector3(-mWidth * float(0.5) + x, heightOrigin + height, -mLength * float(0.5) + y);
				break;
		case 2: vertex = Vector3(-mWidth * float(0.5) + x, -mLength * float(0.5) + y, heightOrigin + height);
				break;
		default: assert(false);
	}

	assert(mAABB.contains(vertex));

	return vertex * mScaling;
}

// Raycast test between a ray and a triangle of the heightfield
void TriangleOverlapCallback::testTriangle(const Vector3* trianglePoints) {

	// Create a triangle collision shape
	float margin = mHeightFieldShape.getTriangleMargin();
	TriangleShape triangleShape(trianglePoints[0], trianglePoints[1], trianglePoints[2], margin);
	triangleShape.setRaycastTestType(mHeightFieldShape.getRaycastTestType());

	// Ray casting test against the collision shape
	RaycastInfo raycastInfo;
	bool isTriangleHit = triangleShape.raycast(m_ray, raycastInfo, m_proxyShape);

	// If the ray hit the collision shape
	if (isTriangleHit && raycastInfo.hitFraction <= mSmallestHitFraction) {

		assert(raycastInfo.hitFraction >= float(0.0));

		m_raycastInfo.body = raycastInfo.body;
		m_raycastInfo.proxyShape = raycastInfo.proxyShape;
		m_raycastInfo.hitFraction = raycastInfo.hitFraction;
		m_raycastInfo.worldPoint = raycastInfo.worldPoint;
		m_raycastInfo.worldNormal = raycastInfo.worldNormal;
		m_raycastInfo.meshSubpart = -1;
		m_raycastInfo.triangleIndex = -1;

		mSmallestHitFraction = raycastInfo.hitFraction;
		mIsHit = true;
	}
}
