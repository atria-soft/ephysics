/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/mathematics/mathematics_functions.h>
#include <ephysics/mathematics/Vector3.h>

using namespace reactphysics3d;

/// Compute the barycentric coordinates u, v, w of a point p inside the triangle (a, b, c)
/// This method uses the technique described in the book Real-Time collision detection by
/// Christer Ericson.
void reactphysics3d::computeBarycentricCoordinatesInTriangle(const Vector3& a, const Vector3& b, const Vector3& c,
											 const Vector3& p, float& u, float& v, float& w) {
	const Vector3 v0 = b - a;
	const Vector3 v1 = c - a;
	const Vector3 v2 = p - a;

	float d00 = v0.dot(v0);
	float d01 = v0.dot(v1);
	float d11 = v1.dot(v1);
	float d20 = v2.dot(v0);
	float d21 = v2.dot(v1);

	float denom = d00 * d11 - d01 * d01;
	v = (d11 * d20 - d01 * d21) / denom;
	w = (d00 * d21 - d01 * d20) / denom;
	u = float(1.0) - v - w;
}

// Clamp a vector such that it is no longer than a given maximum length
Vector3 reactphysics3d::clamp(const Vector3& vector, float maxLength) {
	if (vector.lengthSquare() > maxLength * maxLength) {
		return vector.getUnit() * maxLength;
	}
	return vector;
}
