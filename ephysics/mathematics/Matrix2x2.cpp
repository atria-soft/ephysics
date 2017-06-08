/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/mathematics/Matrix2x2.h>

using namespace reactphysics3d;

// Constructor of the class Matrix2x2
Matrix2x2::Matrix2x2() {

	// Initialize all values in the matrix to zero
	setAllValues(0.0, 0.0, 0.0, 0.0);
}

// Constructor
Matrix2x2::Matrix2x2(float value) {
	setAllValues(value, value, value, value);
}

// Constructor with arguments
Matrix2x2::Matrix2x2(float a1, float a2, float b1, float b2) {

	// Initialize the matrix with the values
	setAllValues(a1, a2, b1, b2);
}

// Destructor
Matrix2x2::~Matrix2x2() {

}

// Copy-constructor
Matrix2x2::Matrix2x2(const Matrix2x2& matrix) {
	setAllValues(matrix.mRows[0][0], matrix.mRows[0][1],
				 matrix.mRows[1][0], matrix.mRows[1][1]);
}

// Assignment operator
Matrix2x2& Matrix2x2::operator=(const Matrix2x2& matrix) {

	// Check for self-assignment
	if (&matrix != this) {
		setAllValues(matrix.mRows[0][0], matrix.mRows[0][1],
					 matrix.mRows[1][0], matrix.mRows[1][1]);
	}
	return *this;
}

// Return the inverse matrix
Matrix2x2 Matrix2x2::getInverse() const {

	// Compute the determinant of the matrix
	float determinant = getDeterminant();

	// Check if the determinant is equal to zero
	assert(std::abs(determinant) > MACHINE_EPSILON);

	float invDeterminant = float(1.0) / determinant;

	Matrix2x2 tempMatrix(mRows[1][1], -mRows[0][1], -mRows[1][0], mRows[0][0]);

	// Return the inverse matrix
	return (invDeterminant * tempMatrix);
}
