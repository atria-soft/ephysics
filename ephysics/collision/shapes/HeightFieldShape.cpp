/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#include <ephysics/collision/shapes/HeightFieldShape.hpp>

// TODO: REMOVE this...
using namespace ephysics;

HeightFieldShape::HeightFieldShape(int32_t _nbGridColumns,
                                   int32_t _nbGridRows,
                                   float _minHeight,
                                   float _maxHeight,
                                   const void* _heightFieldData,
                                   HeightDataType _dataType,
                                   int32_t _upAxis,
                                   float _integerHeightScale):
  ConcaveShape(HEIGHTFIELD),
  m_numberColumns(_nbGridColumns),
  m_numberRows(_nbGridRows),
  m_width(_nbGridColumns - 1),
  m_length(_nbGridRows - 1),
  m_minHeight(_minHeight),
  m_maxHeight(_maxHeight),
  m_upAxis(_upAxis),
  m_integerHeightScale(_integerHeightScale),
  m_heightDataType(_dataType) {
	assert(_nbGridColumns >= 2);
	assert(_nbGridRows >= 2);
	assert(m_width >= 1);
	assert(m_length >= 1);
	assert(_minHeight <= _maxHeight);
	assert(_upAxis == 0 || _upAxis == 1 || _upAxis == 2);
	m_heightFieldData = _heightFieldData;
	float halfHeight = (m_maxHeight - m_minHeight) * 0.5f;
	assert(halfHeight >= 0);
	// Compute the local AABB of the height field
	if (m_upAxis == 0) {
		m_AABB.setMin(vec3(-halfHeight, -m_width * 0.5f, -m_length * float(0.5)));
		m_AABB.setMax(vec3(halfHeight, m_width * 0.5f, m_length* float(0.5)));
	} else if (m_upAxis == 1) {
		m_AABB.setMin(vec3(-m_width * 0.5f, -halfHeight, -m_length * float(0.5)));
		m_AABB.setMax(vec3(m_width * 0.5f, halfHeight, m_length * float(0.5)));
	} else if (m_upAxis == 2) {
		m_AABB.setMin(vec3(-m_width * 0.5f, -m_length * float(0.5), -halfHeight));
		m_AABB.setMax(vec3(m_width * 0.5f, m_length * float(0.5), halfHeight));
	}
}

void HeightFieldShape::getLocalBounds(vec3& _min, vec3& _max) const {
	_min = m_AABB.getMin() * m_scaling;
	_max = m_AABB.getMax() * m_scaling;
}

void HeightFieldShape::testAllTriangles(TriangleCallback& _callback, const AABB& _localAABB) const {
	// Compute the non-scaled AABB
	vec3 inverseScaling(1.0f / m_scaling.x(), 1.0f / m_scaling.y(), float(1.0) / m_scaling.z());
	AABB aabb(_localAABB.getMin() * inverseScaling, _localAABB.getMax() * inverseScaling);
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
		case 0 :
			iMin = clamp(minGridCoords[1], 0, m_numberColumns - 1);
			iMax = clamp(maxGridCoords[1], 0, m_numberColumns - 1);
			jMin = clamp(minGridCoords[2], 0, m_numberRows - 1);
			jMax = clamp(maxGridCoords[2], 0, m_numberRows - 1);
			break;
		case 1 :
			iMin = clamp(minGridCoords[0], 0, m_numberColumns - 1);
			iMax = clamp(maxGridCoords[0], 0, m_numberColumns - 1);
			jMin = clamp(minGridCoords[2], 0, m_numberRows - 1);
			jMax = clamp(maxGridCoords[2], 0, m_numberRows - 1);
			break;
		case 2 :
			iMin = clamp(minGridCoords[0], 0, m_numberColumns - 1);
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
			_callback.testTriangle(trianglePoints);
			// Generate the second triangle for the current grid rectangle
			trianglePoints[0] = p3;
			trianglePoints[1] = p2;
			trianglePoints[2] = p4;
			// Test collision against the second triangle
			_callback.testTriangle(trianglePoints);
		}
	}
}

void HeightFieldShape::computeMinMaxGridCoordinates(int32_t* _minCoords, int32_t* _maxCoords, const AABB& _aabbToCollide) const {
	// Clamp the min/max coords of the AABB to collide inside the height field AABB
	vec3 minPoint = etk::max(_aabbToCollide.getMin(), m_AABB.getMin());
	minPoint = etk::min(minPoint, m_AABB.getMax());
	vec3 maxPoint = etk::min(_aabbToCollide.getMax(), m_AABB.getMax());
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
	_minCoords[0] = computeIntegerGridValue(minPoint.x()) - 1;
	_minCoords[1] = computeIntegerGridValue(minPoint.y()) - 1;
	_minCoords[2] = computeIntegerGridValue(minPoint.z()) - 1;
	_maxCoords[0] = computeIntegerGridValue(maxPoint.x()) + 1;
	_maxCoords[1] = computeIntegerGridValue(maxPoint.y()) + 1;
	_maxCoords[2] = computeIntegerGridValue(maxPoint.z()) + 1;
}

bool HeightFieldShape::raycast(const Ray& _ray, RaycastInfo& _raycastInfo, ProxyShape* _proxyShape) const {
	// TODO : Implement raycasting without using an AABB for the ray
	//		but using a dynamic AABB tree or octree instead
	PROFILE("HeightFieldShape::raycast()");
	TriangleOverlapCallback triangleCallback(_ray, _proxyShape, _raycastInfo, *this);
	// Compute the AABB for the ray
	const vec3 rayEnd = _ray.point1 + _ray.maxFraction * (_ray.point2 - _ray.point1);
	const AABB rayAABB(etk::min(_ray.point1, rayEnd), etk::max(_ray.point1, rayEnd));
	testAllTriangles(triangleCallback, rayAABB);
	return triangleCallback.getIsHit();
}

vec3 HeightFieldShape::getVertexAt(int32_t _xxx, int32_t _yyy) const {
	// Get the height value
	const float height = getHeightAt(_xxx, _yyy);
	// Height values origin
	const float heightOrigin = -(m_maxHeight - m_minHeight) * 0.5f - m_minHeight;
	vec3 vertex;
	switch (m_upAxis) {
		case 0:
			vertex = vec3(heightOrigin + height, -m_width * 0.5f + _xxx, -m_length * float(0.5) + _yyy);
			break;
		case 1:
			vertex = vec3(-m_width * 0.5f + _xxx, heightOrigin + height, -m_length * float(0.5) + _yyy);
			break;
		case 2:
			vertex = vec3(-m_width * 0.5f + _xxx, -m_length * float(0.5) + _yyy, heightOrigin + height);
			break;
		default:
			assert(false);
	}
	assert(m_AABB.contains(vertex));
	return vertex * m_scaling;
}

void TriangleOverlapCallback::testTriangle(const vec3* _trianglePoints) {
	// Create a triangle collision shape
	float margin = m_heightFieldShape.getTriangleMargin();
	TriangleShape triangleShape(_trianglePoints[0], _trianglePoints[1], _trianglePoints[2], margin);
	triangleShape.setRaycastTestType(m_heightFieldShape.getRaycastTestType());
	// Ray casting test against the collision shape
	RaycastInfo raycastInfo;
	bool isTriangleHit = triangleShape.raycast(m_ray, raycastInfo, m_proxyShape);
	// If the ray hit the collision shape
	if (    isTriangleHit
	     && raycastInfo.hitFraction <= m_smallestHitFraction) {
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

int32_t HeightFieldShape::getNbRows() const {
	return m_numberRows;
}

int32_t HeightFieldShape::getNbColumns() const {
	return m_numberColumns;
}

HeightFieldShape::HeightDataType HeightFieldShape::getHeightDataType() const {
	return m_heightDataType;
}

size_t HeightFieldShape::getSizeInBytes() const {
	return sizeof(HeightFieldShape);
}

void HeightFieldShape::setLocalScaling(const vec3& _scaling) {
	CollisionShape::setLocalScaling(_scaling);
}

float HeightFieldShape::getHeightAt(int32_t _xxx, int32_t _yyy) const {
	switch(m_heightDataType) {
		case HEIGHT_FLOAT_TYPE:
			return ((float*)m_heightFieldData)[_yyy * m_numberColumns + _xxx];
		case HEIGHT_DOUBLE_TYPE:
			return ((double*)m_heightFieldData)[_yyy * m_numberColumns + _xxx];
		case HEIGHT_INT_TYPE:
			return ((int32_t*)m_heightFieldData)[_yyy * m_numberColumns + _xxx] * m_integerHeightScale;
		default:
			assert(false);
			return 0;
	}
}

int32_t HeightFieldShape::computeIntegerGridValue(float _value) const {
	return (_value < 0.0f) ? _value - 0.5f : _value + 0.5f;
}

void HeightFieldShape::computeLocalInertiaTensor(etk::Matrix3x3& _tensor, float _mass) const {
	// Default inertia tensor
	// Note that this is not very realistic for a concave triangle mesh.
	// However, in most cases, it will only be used static bodies and therefore,
	// the inertia tensor is not used.
	_tensor.setValue(_mass, 0, 0,
	                 0, _mass, 0,
	                 0, 0, _mass);
}
