/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <cmath>
#include <cassert>
#include <ephysics/mathematics/mathematics_functions.h>



/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class Vector3
/**
 * This class represents a 3D vector.
 */
struct Vector3 {

	public:

		// -------------------- Attributes -------------------- //

		/// Component x
		float x;

		/// Component y
		float y;

		/// Component z
		float z;

		// -------------------- Methods -------------------- //

		/// Constructor of the class Vector3D
		Vector3();

		/// Constructor with arguments
		Vector3(float newX, float newY, float newZ);

		/// Copy-constructor
		Vector3(const Vector3& vector);

		/// Destructor
		~Vector3();

		/// Set all the values of the vector
		void setAllValues(float newX, float newY, float newZ);

		/// Set the vector to zero
		void setToZero();

		/// Return the length of the vector
		float length() const;

		/// Return the square of the length of the vector
		float lengthSquare() const;

		/// Return the corresponding unit vector
		Vector3 getUnit() const;

		/// Return one unit orthogonal vector of the current vector
		Vector3 getOneUnitOrthogonalVector() const;

		/// Return true if the vector is unit and false otherwise
		bool isUnit() const;

		/// Return true if the current vector is the zero vector
		bool isZero() const;

		/// Dot product of two vectors
		float dot(const Vector3& vector) const;

		/// Cross product of two vectors
		Vector3 cross(const Vector3& vector) const;

		/// Normalize the vector
		void normalize();

		/// Return the corresponding absolute value vector
		Vector3 getAbsoluteVector() const;

		/// Return the axis with the minimal value
		int32_t getMinAxis() const;

		/// Return the axis with the maximal value
		int32_t getMaxAxis() const;

		/// Return the minimum value among the three components of a vector
		float getMinValue() const;

		/// Return the maximum value among the three components of a vector
		float getMaxValue() const;

		/// Overloaded operator for the equality condition
		bool operator== (const Vector3& vector) const;

		/// Overloaded operator for the is different condition
		bool operator!= (const Vector3& vector) const;

		/// Overloaded operator for addition with assignment
		Vector3& operator+=(const Vector3& vector);

		/// Overloaded operator for substraction with assignment
		Vector3& operator-=(const Vector3& vector);

		/// Overloaded operator for multiplication with a number with assignment
		Vector3& operator*=(float number);

		/// Overloaded operator for division by a number with assignment
		Vector3& operator/=(float number);

		/// Overloaded operator for value access
		float& operator[] (int32_t index);

		/// Overloaded operator for value access
		const float& operator[] (int32_t index) const;

		/// Overloaded operator
		Vector3& operator=(const Vector3& vector);

		/// Overloaded less than operator for ordering to be used inside std::set for instance
		bool operator<(const Vector3& vector) const;

		/// Return a vector taking the minimum components of two vectors
		static Vector3 min(const Vector3& vector1, const Vector3& vector2);

		/// Return a vector taking the maximum components of two vectors
		static Vector3 max(const Vector3& vector1, const Vector3& vector2);

		/// Return the zero vector
		static Vector3 zero();

		// -------------------- Friends -------------------- //

		friend Vector3 operator+(const Vector3& vector1, const Vector3& vector2);
		friend Vector3 operator-(const Vector3& vector1, const Vector3& vector2);
		friend Vector3 operator-(const Vector3& vector);
		friend Vector3 operator*(const Vector3& vector, float number);
		friend Vector3 operator*(float number, const Vector3& vector);
		friend Vector3 operator*(const Vector3& vector1, const Vector3& vector2);
		friend Vector3 operator/(const Vector3& vector, float number);
		friend Vector3 operator/(const Vector3& vector1, const Vector3& vector2);
};

// Set the vector to zero
inline void Vector3::setToZero() {
	x = 0;
	y = 0;
	z = 0;
}

// Set all the values of the vector
inline void Vector3::setAllValues(float newX, float newY, float newZ) {
	x = newX;
	y = newY;
	z = newZ;
}

// Return the length of the vector
inline float Vector3::length() const {
	return sqrt(x*x + y*y + z*z);
}

// Return the square of the length of the vector
inline float Vector3::lengthSquare() const {
	return x*x + y*y + z*z;
}

// Scalar product of two vectors (inline)
inline float Vector3::dot(const Vector3& vector) const {
	return (x*vector.x + y*vector.y + z*vector.z);
}

// Cross product of two vectors (inline)
inline Vector3 Vector3::cross(const Vector3& vector) const {
	return Vector3(y * vector.z - z * vector.y,
				   z * vector.x - x * vector.z,
				   x * vector.y - y * vector.x);
}

// Normalize the vector
inline void Vector3::normalize() {
	float l = length();
	if (l < MACHINE_EPSILON) {
		return;
	}
	x /= l;
	y /= l;
	z /= l;
}

// Return the corresponding absolute value vector
inline Vector3 Vector3::getAbsoluteVector() const {
	return Vector3(std::abs(x), std::abs(y), std::abs(z));
}

// Return the axis with the minimal value
inline int32_t Vector3::getMinAxis() const {
	return (x < y ? (x < z ? 0 : 2) : (y < z ? 1 : 2));
}

// Return the axis with the maximal value
inline int32_t Vector3::getMaxAxis() const {
	return (x < y ? (y < z ? 2 : 1) : (x < z ? 2 : 0));
}

// Return true if the vector is unit and false otherwise
inline bool Vector3::isUnit() const {
	return approxEqual(lengthSquare(), 1.0);
}

// Return true if the vector is the zero vector
inline bool Vector3::isZero() const {
	return approxEqual(lengthSquare(), 0.0);
}

// Overloaded operator for the equality condition
inline bool Vector3::operator== (const Vector3& vector) const {
	return (x == vector.x && y == vector.y && z == vector.z);
}

// Overloaded operator for the is different condition
inline bool Vector3::operator!= (const Vector3& vector) const {
	return !(*this == vector);
}

// Overloaded operator for addition with assignment
inline Vector3& Vector3::operator+=(const Vector3& vector) {
	x += vector.x;
	y += vector.y;
	z += vector.z;
	return *this;
}

// Overloaded operator for substraction with assignment
inline Vector3& Vector3::operator-=(const Vector3& vector) {
	x -= vector.x;
	y -= vector.y;
	z -= vector.z;
	return *this;
}

// Overloaded operator for multiplication with a number with assignment
inline Vector3& Vector3::operator*=(float number) {
	x *= number;
	y *= number;
	z *= number;
	return *this;
}

// Overloaded operator for division by a number with assignment
inline Vector3& Vector3::operator/=(float number) {
	assert(number > std::numeric_limits<float>::epsilon());
	x /= number;
	y /= number;
	z /= number;
	return *this;
}

// Overloaded operator for value access
inline float& Vector3::operator[] (int32_t index) {
	return (&x)[index];
}

// Overloaded operator for value access
inline const float& Vector3::operator[] (int32_t index) const {
	return (&x)[index];
}

// Overloaded operator for addition
inline Vector3 operator+(const Vector3& vector1, const Vector3& vector2) {
	return Vector3(vector1.x + vector2.x, vector1.y + vector2.y, vector1.z + vector2.z);
}

// Overloaded operator for substraction
inline Vector3 operator-(const Vector3& vector1, const Vector3& vector2) {
	return Vector3(vector1.x - vector2.x, vector1.y - vector2.y, vector1.z - vector2.z);
}

// Overloaded operator for the negative of a vector
inline Vector3 operator-(const Vector3& vector) {
	return Vector3(-vector.x, -vector.y, -vector.z);
}

// Overloaded operator for multiplication with a number
inline Vector3 operator*(const Vector3& vector, float number) {
	return Vector3(number * vector.x, number * vector.y, number * vector.z);
}

// Overloaded operator for division by a number
inline Vector3 operator/(const Vector3& vector, float number) {
	assert(number > MACHINE_EPSILON);
	return Vector3(vector.x / number, vector.y / number, vector.z / number);
}

// Overload operator for division between two vectors
inline Vector3 operator/(const Vector3& vector1, const Vector3& vector2) {
	assert(vector2.x > MACHINE_EPSILON);
	assert(vector2.y > MACHINE_EPSILON);
	assert(vector2.z > MACHINE_EPSILON);
	return Vector3(vector1.x / vector2.x, vector1.y / vector2.y, vector1.z / vector2.z);
}

// Overloaded operator for multiplication with a number
inline Vector3 operator*(float number, const Vector3& vector) {
	return vector * number;
}

// Overload operator for multiplication between two vectors
inline Vector3 operator*(const Vector3& vector1, const Vector3& vector2) {
	return Vector3(vector1.x * vector2.x, vector1.y * vector2.y, vector1.z * vector2.z);
}

// Assignment operator
inline Vector3& Vector3::operator=(const Vector3& vector) {
	if (&vector != this) {
		x = vector.x;
		y = vector.y;
		z = vector.z;
	}
	return *this;
}

// Overloaded less than operator for ordering to be used inside std::set for instance
inline bool Vector3::operator<(const Vector3& vector) const {
	return (x == vector.x ? (y == vector.y ? z < vector.z : y < vector.y) : x < vector.x);
}

// Return a vector taking the minimum components of two vectors
inline Vector3 Vector3::min(const Vector3& vector1, const Vector3& vector2) {
	return Vector3(std::min(vector1.x, vector2.x),
				   std::min(vector1.y, vector2.y),
				   std::min(vector1.z, vector2.z));
}

// Return a vector taking the maximum components of two vectors
inline Vector3 Vector3::max(const Vector3& vector1, const Vector3& vector2) {
	return Vector3(std::max(vector1.x, vector2.x),
				   std::max(vector1.y, vector2.y),
				   std::max(vector1.z, vector2.z));
}

// Return the minimum value among the three components of a vector
inline float Vector3::getMinValue() const {
	return std::min(std::min(x, y), z);
}

// Return the maximum value among the three components of a vector
inline float Vector3::getMaxValue() const {
	return std::max(std::max(x, y), z);
}

// Return the zero vector
inline Vector3 Vector3::zero() {
	return Vector3(0, 0, 0);
}

}
