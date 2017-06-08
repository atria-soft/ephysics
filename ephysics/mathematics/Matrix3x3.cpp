/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <iostream>
#include <ephysics/mathematics/Matrix3x3.h>

// Namespaces
using namespace reactphysics3d;

// Constructor of the class Matrix3x3
Matrix3x3::Matrix3x3() {
	// Initialize all values in the matrix to zero
	setAllValues(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

// Constructor
Matrix3x3::Matrix3x3(float value) {
	setAllValues(value, value, value, value, value, value, value, value, value);
}

// Constructor with arguments
Matrix3x3::Matrix3x3(float a1, float a2, float a3,
					 float b1, float b2, float b3,
					 float c1, float c2, float c3) {
	// Initialize the matrix with the values
	setAllValues(a1, a2, a3, b1, b2, b3, c1, c2, c3);
}

// Destructor
Matrix3x3::~Matrix3x3() {

}

// Copy-constructor
Matrix3x3::Matrix3x3(const Matrix3x3& matrix) {
	setAllValues(matrix.mRows[0][0], matrix.mRows[0][1], matrix.mRows[0][2],
				 matrix.mRows[1][0], matrix.mRows[1][1], matrix.mRows[1][2],
				 matrix.mRows[2][0], matrix.mRows[2][1], matrix.mRows[2][2]);
}

// Assignment operator
Matrix3x3& Matrix3x3::operator=(const Matrix3x3& matrix) {

	// Check for self-assignment
	if (&matrix != this) {
		setAllValues(matrix.mRows[0][0], matrix.mRows[0][1], matrix.mRows[0][2],
					 matrix.mRows[1][0], matrix.mRows[1][1], matrix.mRows[1][2],
					 matrix.mRows[2][0], matrix.mRows[2][1], matrix.mRows[2][2]);
	}
	return *this;
}

// Return the inverse matrix
Matrix3x3 Matrix3x3::getInverse() const {

	// Compute the determinant of the matrix
	float determinant = getDeterminant();

	// Check if the determinant is equal to zero
	assert(std::abs(determinant) > MACHINE_EPSILON);

	float invDeterminant = float(1.0) / determinant;

	Matrix3x3 tempMatrix((mRows[1][1]*mRows[2][2]-mRows[2][1]*mRows[1][2]),
						 -(mRows[0][1]*mRows[2][2]-mRows[2][1]*mRows[0][2]),
						 (mRows[0][1]*mRows[1][2]-mRows[0][2]*mRows[1][1]),
							-(mRows[1][0]*mRows[2][2]-mRows[2][0]*mRows[1][2]),
						 (mRows[0][0]*mRows[2][2]-mRows[2][0]*mRows[0][2]),
						 -(mRows[0][0]*mRows[1][2]-mRows[1][0]*mRows[0][2]),
							(mRows[1][0]*mRows[2][1]-mRows[2][0]*mRows[1][1]),
						 -(mRows[0][0]*mRows[2][1]-mRows[2][0]*mRows[0][1]),
						 (mRows[0][0]*mRows[1][1]-mRows[0][1]*mRows[1][0]));

	// Return the inverse matrix
	return (invDeterminant * tempMatrix);
}



