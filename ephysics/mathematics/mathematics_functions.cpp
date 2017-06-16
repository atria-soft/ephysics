/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/mathematics/mathematics_functions.hpp>
#include <etk/math/Vector3D.hpp>

using namespace ephysics;

/// Compute the barycentric coordinates u, v, w of a point p inside the triangle (a, b, c)
/// This method uses the technique described in the book Real-Time collision detection by
/// Christer Ericson.
void ephysics::computeBarycentricCoordinatesInTriangle(const vec3& a, const vec3& b, const vec3& c,
											 const vec3& p, float& u, float& v, float& w) {
	const vec3 v0 = b - a;
	const vec3 v1 = c - a;
	const vec3 v2 = p - a;

	float d00 = v0.dot(v0);
	float d01 = v0.dot(v1);
	float d11 = v1.dot(v1);
	float d20 = v2.dot(v0);
	float d21 = v2.dot(v1);

	float denom = d00 * d11 - d01 * d01;
	v = (d11 * d20 - d01 * d21) / denom;
	w = (d00 * d21 - d01 * d20) / denom;
	u = 1.0f - v - w;
}

// Clamp a vector such that it is no longer than a given maximum length
vec3 ephysics::clamp(const vec3& vector, float maxLength) {
	if (vector.length2() > maxLength * maxLength) {
		return vector.safeNormalized() * maxLength;
	}
	return vector;
}
