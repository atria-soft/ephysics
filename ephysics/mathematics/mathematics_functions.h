/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/configuration.h>

#include <algorithm>
#include <cassert>
#include <cmath>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// ---------- Mathematics functions ---------- //

/// Function to test if two real numbers are (almost) equal
/// We test if two numbers a and b are such that (a-b) are in [-EPSILON; EPSILON]
inline bool approxEqual(float a, float b, float epsilon = MACHINE_EPSILON) {
	return (std::fabs(a - b) < epsilon);
}

/// Function that returns the result of the "value" clamped by
/// two others values "lowerLimit" and "upperLimit"
inline int32_t clamp(int32_t value, int32_t lowerLimit, int32_t upperLimit) {
	assert(lowerLimit <= upperLimit);
	return std::min(std::max(value, lowerLimit), upperLimit);
}

/// Function that returns the result of the "value" clamped by
/// two others values "lowerLimit" and "upperLimit"
inline float clamp(float value, float lowerLimit, float upperLimit) {
	assert(lowerLimit <= upperLimit);
	return std::min(std::max(value, lowerLimit), upperLimit);
}

/// Return the minimum value among three values
inline float min3(float a, float b, float c) {
	return std::min(std::min(a, b), c);
}

/// Return the maximum value among three values
inline float max3(float a, float b, float c) {
	return std::max(std::max(a, b), c);
}

/// Return true if two values have the same sign
inline bool sameSign(float a, float b) {
	return a * b >= 0.0f;
}

/// Clamp a vector such that it is no longer than a given maximum length
vec3 clamp(const vec3& vector, float maxLength);

/// Compute the barycentric coordinates u, v, w of a point p inside the triangle (a, b, c)
void computeBarycentricCoordinatesInTriangle(const vec3& a, const vec3& b, const vec3& c,
											 const vec3& p, float& u, float& v, float& w);

}
