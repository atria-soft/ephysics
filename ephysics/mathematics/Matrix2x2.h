/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <cassert>
#include <ephysics/mathematics/Vector2.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class Matrix2x2
/**
 * This class represents a 2x2 matrix.
 */
class Matrix2x2 {

	private :

		// -------------------- Attributes -------------------- //

		/// Rows of the matrix;
		Vector2 mRows[2];

	public :

		// -------------------- Methods -------------------- //

		/// Constructor
		Matrix2x2();

		/// Constructor
		Matrix2x2(float value);

		/// Constructor
		Matrix2x2(float a1, float a2, float b1, float b2);

		/// Destructor
		~Matrix2x2();

		/// Copy-constructor
		Matrix2x2(const Matrix2x2& matrix);

		/// Assignment operator
		Matrix2x2& operator=(const Matrix2x2& matrix);

		/// Set all the values in the matrix
		void setAllValues(float a1, float a2, float b1, float b2);

		/// Set the matrix to zero
		void setToZero();

		/// Return a column
		Vector2 getColumn(int32_t i) const;

		/// Return a row
		Vector2 getRow(int32_t i) const;

		/// Return the transpose matrix
		Matrix2x2 getTranspose() const;

		/// Return the determinant of the matrix
		float getDeterminant() const;

		/// Return the trace of the matrix
		float getTrace() const;

		/// Return the inverse matrix
		Matrix2x2 getInverse() const;

		/// Return the matrix with absolute values
		Matrix2x2 getAbsoluteMatrix() const;

		/// Set the matrix to the identity matrix
		void setToIdentity();

		/// Return the 2x2 identity matrix
		static Matrix2x2 identity();

		/// Return the 2x2 zero matrix
		static Matrix2x2 zero();

		/// Overloaded operator for addition
		friend Matrix2x2 operator+(const Matrix2x2& matrix1, const Matrix2x2& matrix2);

		/// Overloaded operator for substraction
		friend Matrix2x2 operator-(const Matrix2x2& matrix1, const Matrix2x2& matrix2);

		/// Overloaded operator for the negative of the matrix
		friend Matrix2x2 operator-(const Matrix2x2& matrix);

		/// Overloaded operator for multiplication with a number
		friend Matrix2x2 operator*(float nb, const Matrix2x2& matrix);

		/// Overloaded operator for multiplication with a matrix
		friend Matrix2x2 operator*(const Matrix2x2& matrix, float nb);

		/// Overloaded operator for matrix multiplication
		friend Matrix2x2 operator*(const Matrix2x2& matrix1, const Matrix2x2& matrix2);

		/// Overloaded operator for multiplication with a vector
		friend Vector2 operator*(const Matrix2x2& matrix, const Vector2& vector);

		/// Overloaded operator for equality condition
		bool operator==(const Matrix2x2& matrix) const;

		/// Overloaded operator for the is different condition
		bool operator!= (const Matrix2x2& matrix) const;

		/// Overloaded operator for addition with assignment
		Matrix2x2& operator+=(const Matrix2x2& matrix);

		/// Overloaded operator for substraction with assignment
		Matrix2x2& operator-=(const Matrix2x2& matrix);

		/// Overloaded operator for multiplication with a number with assignment
		Matrix2x2& operator*=(float nb);

		/// Overloaded operator to read element of the matrix.
		const Vector2& operator[](int32_t row) const;

		/// Overloaded operator to read/write element of the matrix.
		Vector2& operator[](int32_t row);
};

// Method to set all the values in the matrix
inline void Matrix2x2::setAllValues(float a1, float a2,
									float b1, float b2) {
	mRows[0][0] = a1; mRows[0][1] = a2;
	mRows[1][0] = b1; mRows[1][1] = b2;
}

// Set the matrix to zero
inline void Matrix2x2::setToZero() {
	mRows[0].setToZero();
	mRows[1].setToZero();
}

// Return a column
inline Vector2 Matrix2x2::getColumn(int32_t i) const {
	assert(i>= 0 && i<2);
	return Vector2(mRows[0][i], mRows[1][i]);
}

// Return a row
inline Vector2 Matrix2x2::getRow(int32_t i) const {
	assert(i>= 0 && i<2);
	return mRows[i];
}

// Return the transpose matrix
inline Matrix2x2 Matrix2x2::getTranspose() const {

	// Return the transpose matrix
	return Matrix2x2(mRows[0][0], mRows[1][0],
					 mRows[0][1], mRows[1][1]);
}

// Return the determinant of the matrix
inline float Matrix2x2::getDeterminant() const {

	// Compute and return the determinant of the matrix
	return mRows[0][0] * mRows[1][1] - mRows[1][0] * mRows[0][1];
}

// Return the trace of the matrix
inline float Matrix2x2::getTrace() const {

	// Compute and return the trace
	return (mRows[0][0] + mRows[1][1]);
}

// Set the matrix to the identity matrix
inline void Matrix2x2::setToIdentity() {
	mRows[0][0] = 1.0; mRows[0][1] = 0.0;
	mRows[1][0] = 0.0; mRows[1][1] = 1.0;
}

// Return the 2x2 identity matrix
inline Matrix2x2 Matrix2x2::identity() {

	// Return the isdentity matrix
	return Matrix2x2(1.0, 0.0, 0.0, 1.0);
}

// Return the 2x2 zero matrix
inline Matrix2x2 Matrix2x2::zero() {
	return Matrix2x2(0.0, 0.0, 0.0, 0.0);
}

// Return the matrix with absolute values
inline Matrix2x2 Matrix2x2::getAbsoluteMatrix() const {
	return Matrix2x2(fabs(mRows[0][0]), fabs(mRows[0][1]),
					 fabs(mRows[1][0]), fabs(mRows[1][1]));
}

// Overloaded operator for addition
inline Matrix2x2 operator+(const Matrix2x2& matrix1, const Matrix2x2& matrix2) {
	return Matrix2x2(matrix1.mRows[0][0] + matrix2.mRows[0][0],
					 matrix1.mRows[0][1] + matrix2.mRows[0][1],
					 matrix1.mRows[1][0] + matrix2.mRows[1][0],
					 matrix1.mRows[1][1] + matrix2.mRows[1][1]);
}

// Overloaded operator for substraction
inline Matrix2x2 operator-(const Matrix2x2& matrix1, const Matrix2x2& matrix2) {
	return Matrix2x2(matrix1.mRows[0][0] - matrix2.mRows[0][0],
					 matrix1.mRows[0][1] - matrix2.mRows[0][1],
					 matrix1.mRows[1][0] - matrix2.mRows[1][0],
					 matrix1.mRows[1][1] - matrix2.mRows[1][1]);
}

// Overloaded operator for the negative of the matrix
inline Matrix2x2 operator-(const Matrix2x2& matrix) {
	return Matrix2x2(-matrix.mRows[0][0], -matrix.mRows[0][1],
					 -matrix.mRows[1][0], -matrix.mRows[1][1]);
}

// Overloaded operator for multiplication with a number
inline Matrix2x2 operator*(float nb, const Matrix2x2& matrix) {
	return Matrix2x2(matrix.mRows[0][0] * nb, matrix.mRows[0][1] * nb,
					 matrix.mRows[1][0] * nb, matrix.mRows[1][1] * nb);
}

// Overloaded operator for multiplication with a matrix
inline Matrix2x2 operator*(const Matrix2x2& matrix, float nb) {
	return nb * matrix;
}

// Overloaded operator for matrix multiplication
inline Matrix2x2 operator*(const Matrix2x2& matrix1, const Matrix2x2& matrix2) {
	return Matrix2x2(matrix1.mRows[0][0] * matrix2.mRows[0][0] + matrix1.mRows[0][1] *
					 matrix2.mRows[1][0],
					 matrix1.mRows[0][0] * matrix2.mRows[0][1] + matrix1.mRows[0][1] *
					 matrix2.mRows[1][1],
					 matrix1.mRows[1][0] * matrix2.mRows[0][0] + matrix1.mRows[1][1] *
					 matrix2.mRows[1][0],
					 matrix1.mRows[1][0] * matrix2.mRows[0][1] + matrix1.mRows[1][1] *
					 matrix2.mRows[1][1]);
}

// Overloaded operator for multiplication with a vector
inline Vector2 operator*(const Matrix2x2& matrix, const Vector2& vector) {
	return Vector2(matrix.mRows[0][0]*vector.x + matrix.mRows[0][1]*vector.y,
				   matrix.mRows[1][0]*vector.x + matrix.mRows[1][1]*vector.y);
}

// Overloaded operator for equality condition
inline bool Matrix2x2::operator==(const Matrix2x2& matrix) const {
	return (mRows[0][0] == matrix.mRows[0][0] && mRows[0][1] == matrix.mRows[0][1] &&
			mRows[1][0] == matrix.mRows[1][0] && mRows[1][1] == matrix.mRows[1][1]);
}

// Overloaded operator for the is different condition
inline bool Matrix2x2::operator!= (const Matrix2x2& matrix) const {
	return !(*this == matrix);
}

// Overloaded operator for addition with assignment
inline Matrix2x2& Matrix2x2::operator+=(const Matrix2x2& matrix) {
   mRows[0][0] += matrix.mRows[0][0]; mRows[0][1] += matrix.mRows[0][1];
   mRows[1][0] += matrix.mRows[1][0]; mRows[1][1] += matrix.mRows[1][1];
   return *this;
}

// Overloaded operator for substraction with assignment
inline Matrix2x2& Matrix2x2::operator-=(const Matrix2x2& matrix) {
   mRows[0][0] -= matrix.mRows[0][0]; mRows[0][1] -= matrix.mRows[0][1];
   mRows[1][0] -= matrix.mRows[1][0]; mRows[1][1] -= matrix.mRows[1][1];
   return *this;
}

// Overloaded operator for multiplication with a number with assignment
inline Matrix2x2& Matrix2x2::operator*=(float nb) {
   mRows[0][0] *= nb; mRows[0][1] *= nb;
   mRows[1][0] *= nb; mRows[1][1] *= nb;
   return *this;
}

// Overloaded operator to return a row of the matrix.
/// This operator is also used to access a matrix value using the syntax
/// matrix[row][col].
inline const Vector2& Matrix2x2::operator[](int32_t row) const {
	return mRows[row];
}

// Overloaded operator to return a row of the matrix.
/// This operator is also used to access a matrix value using the syntax
/// matrix[row][col].
inline Vector2& Matrix2x2::operator[](int32_t row) {
	return mRows[row];
}

}
