/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/mathematics/Vector2.h>
#include <vector>

// Namespaces
using namespace reactphysics3d;

// Constructor
Vector2::Vector2() : x(0.0), y(0.0) {

}

// Constructor with arguments
Vector2::Vector2(float newX, float newY) : x(newX), y(newY) {

}

// Copy-constructor
Vector2::Vector2(const Vector2& vector) : x(vector.x), y(vector.y) {

}

// Destructor
Vector2::~Vector2() {

}

// Return the corresponding unit vector
Vector2 Vector2::getUnit() const {
	float lengthVector = length();

	if (lengthVector < MACHINE_EPSILON) {
		return *this;
	}

	// Compute and return the unit vector
	float lengthInv = float(1.0) / lengthVector;
	return Vector2(x * lengthInv, y * lengthInv);
}

// Return one unit orthogonal vector of the current vector
Vector2 Vector2::getOneUnitOrthogonalVector() const {

	float l = length();
	assert(l > MACHINE_EPSILON);

	return Vector2(-y / l, x / l);
}
