/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/mathematics/Vector3.h>
#include <iostream>
#include <vector>

// Namespaces
using namespace reactphysics3d;

// Constructor of the class Vector3D
Vector3::Vector3() : x(0.0), y(0.0), z(0.0) {

}

// Constructor with arguments
Vector3::Vector3(float newX, float newY, float newZ) : x(newX), y(newY), z(newZ) {

}

// Copy-constructor
Vector3::Vector3(const Vector3& vector) : x(vector.x), y(vector.y), z(vector.z) {

}

// Destructor
Vector3::~Vector3() {

}

// Return the corresponding unit vector
Vector3 Vector3::getUnit() const {
	float lengthVector = length();

	if (lengthVector < MACHINE_EPSILON) {
		return *this;
	}

	// Compute and return the unit vector
	float lengthInv = float(1.0) / lengthVector;
	return Vector3(x * lengthInv, y * lengthInv, z * lengthInv);
}

// Return one unit orthogonal vector of the current vector
Vector3 Vector3::getOneUnitOrthogonalVector() const {

	assert(length() > MACHINE_EPSILON);

	// Get the minimum element of the vector
	Vector3 vectorAbs(fabs(x), fabs(y), fabs(z));
	int32_t minElement = vectorAbs.getMinAxis();

	if (minElement == 0) {
		return Vector3(0.0, -z, y) / sqrt(y*y + z*z);
	}
	else if (minElement == 1) {
		return Vector3(-z, 0.0, x) / sqrt(x*x + z*z);
	}
	else {
		return Vector3(-y, x, 0.0) / sqrt(x*x + y*y);
	}

}
