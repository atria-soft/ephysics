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

// Class Vector2
/**
 * This class represents a 2D vector.
 */
struct Vector2 {

	public:

		// -------------------- Attributes -------------------- //

		/// Component x
		float x;

		/// Component y
		float y;

		// -------------------- Methods -------------------- //

		/// Constructor of the class Vector3D
		Vector2();

		/// Constructor with arguments
		Vector2(float newX, float newY);

		/// Copy-constructor
		Vector2(const Vector2& vector);

		/// Destructor
		~Vector2();

		/// Set all the values of the vector
		void setAllValues(float newX, float newY);

		/// Set the vector to zero
		void setToZero();

		/// Return the length of the vector
		float length() const;

		/// Return the square of the length of the vector
		float lengthSquare() const;

		/// Return the corresponding unit vector
		Vector2 getUnit() const;

		/// Return one unit orthogonal vector of the current vector
		Vector2 getOneUnitOrthogonalVector() const;

		/// Return true if the vector is unit and false otherwise
		bool isUnit() const;

		/// Return true if the current vector is the zero vector
		bool isZero() const;

		/// Dot product of two vectors
		float dot(const Vector2& vector) const;

		/// Normalize the vector
		void normalize();

		/// Return the corresponding absolute value vector
		Vector2 getAbsoluteVector() const;

		/// Return the axis with the minimal value
		int32_t getMinAxis() const;

		/// Return the axis with the maximal value
		int32_t getMaxAxis() const;

		/// Overloaded operator for the equality condition
		bool operator== (const Vector2& vector) const;

		/// Overloaded operator for the is different condition
		bool operator!= (const Vector2& vector) const;

		/// Overloaded operator for addition with assignment
		Vector2& operator+=(const Vector2& vector);

		/// Overloaded operator for substraction with assignment
		Vector2& operator-=(const Vector2& vector);

		/// Overloaded operator for multiplication with a number with assignment
		Vector2& operator*=(float number);

		/// Overloaded operator for division by a number with assignment
		Vector2& operator/=(float number);

		/// Overloaded operator for value access
		float& operator[] (int32_t index);

		/// Overloaded operator for value access
		const float& operator[] (int32_t index) const;

		/// Overloaded operator
		Vector2& operator=(const Vector2& vector);

		/// Overloaded less than operator for ordering to be used inside std::set for instance
		bool operator<(const Vector2& vector) const;

		/// Return a vector taking the minimum components of two vectors
		static Vector2 min(const Vector2& vector1, const Vector2& vector2);

		/// Return a vector taking the maximum components of two vectors
		static Vector2 max(const Vector2& vector1, const Vector2& vector2);

		/// Return the zero vector
		static Vector2 zero();

		// -------------------- Friends -------------------- //

		friend Vector2 operator+(const Vector2& vector1, const Vector2& vector2);
		friend Vector2 operator-(const Vector2& vector1, const Vector2& vector2);
		friend Vector2 operator-(const Vector2& vector);
		friend Vector2 operator*(const Vector2& vector, float number);
		friend Vector2 operator*(float number, const Vector2& vector);
		friend Vector2 operator*(const Vector2& vector1, const Vector2& vector2);
		friend Vector2 operator/(const Vector2& vector, float number);
		friend Vector2 operator/(const Vector2& vector1, const Vector2& vector2);
};

// Set the vector to zero
inline void Vector2::setToZero() {
	x = 0;
	y = 0;
}

// Set all the values of the vector
inline void Vector2::setAllValues(float newX, float newY) {
	x = newX;
	y = newY;
}

// Return the length of the vector
inline float Vector2::length() const {
	return sqrt(x*x + y*y);
}

// Return the square of the length of the vector
inline float Vector2::lengthSquare() const {
	return x*x + y*y;
}

// Scalar product of two vectors (inline)
inline float Vector2::dot(const Vector2& vector) const {
	return (x*vector.x + y*vector.y);
}

// Normalize the vector
inline void Vector2::normalize() {
	float l = length();
	if (l < MACHINE_EPSILON) {
		return;
	}
	x /= l;
	y /= l;
}

// Return the corresponding absolute value vector
inline Vector2 Vector2::getAbsoluteVector() const {
	return Vector2(std::abs(x), std::abs(y));
}

// Return the axis with the minimal value
inline int32_t Vector2::getMinAxis() const {
	return (x < y ? 0 : 1);
}

// Return the axis with the maximal value
inline int32_t Vector2::getMaxAxis() const {
	return (x < y ? 1 : 0);
}

// Return true if the vector is unit and false otherwise
inline bool Vector2::isUnit() const {
	return approxEqual(lengthSquare(), 1.0);
}

// Return true if the vector is the zero vector
inline bool Vector2::isZero() const {
	return approxEqual(lengthSquare(), 0.0);
}

// Overloaded operator for the equality condition
inline bool Vector2::operator== (const Vector2& vector) const {
	return (x == vector.x && y == vector.y);
}

// Overloaded operator for the is different condition
inline bool Vector2::operator!= (const Vector2& vector) const {
	return !(*this == vector);
}

// Overloaded operator for addition with assignment
inline Vector2& Vector2::operator+=(const Vector2& vector) {
	x += vector.x;
	y += vector.y;
	return *this;
}

// Overloaded operator for substraction with assignment
inline Vector2& Vector2::operator-=(const Vector2& vector) {
	x -= vector.x;
	y -= vector.y;
	return *this;
}

// Overloaded operator for multiplication with a number with assignment
inline Vector2& Vector2::operator*=(float number) {
	x *= number;
	y *= number;
	return *this;
}

// Overloaded operator for division by a number with assignment
inline Vector2& Vector2::operator/=(float number) {
	assert(number > std::numeric_limits<float>::epsilon());
	x /= number;
	y /= number;
	return *this;
}

// Overloaded operator for value access
inline float& Vector2::operator[] (int32_t index) {
	return (&x)[index];
}

// Overloaded operator for value access
inline const float& Vector2::operator[] (int32_t index) const {
	return (&x)[index];
}

// Overloaded operator for addition
inline Vector2 operator+(const Vector2& vector1, const Vector2& vector2) {
	return Vector2(vector1.x + vector2.x, vector1.y + vector2.y);
}

// Overloaded operator for substraction
inline Vector2 operator-(const Vector2& vector1, const Vector2& vector2) {
	return Vector2(vector1.x - vector2.x, vector1.y - vector2.y);
}

// Overloaded operator for the negative of a vector
inline Vector2 operator-(const Vector2& vector) {
	return Vector2(-vector.x, -vector.y);
}

// Overloaded operator for multiplication with a number
inline Vector2 operator*(const Vector2& vector, float number) {
	return Vector2(number * vector.x, number * vector.y);
}

// Overloaded operator for multiplication of two vectors
inline Vector2 operator*(const Vector2& vector1, const Vector2& vector2) {
	return Vector2(vector1.x * vector2.x, vector1.y * vector2.y);
}

// Overloaded operator for division by a number
inline Vector2 operator/(const Vector2& vector, float number) {
	assert(number > MACHINE_EPSILON);
	return Vector2(vector.x / number, vector.y / number);
}

// Overload operator for division between two vectors
inline Vector2 operator/(const Vector2& vector1, const Vector2& vector2) {
	assert(vector2.x > MACHINE_EPSILON);
	assert(vector2.y > MACHINE_EPSILON);
	return Vector2(vector1.x / vector2.x, vector1.y / vector2.y);
}

// Overloaded operator for multiplication with a number
inline Vector2 operator*(float number, const Vector2& vector) {
	return vector * number;
}

// Assignment operator
inline Vector2& Vector2::operator=(const Vector2& vector) {
	if (&vector != this) {
		x = vector.x;
		y = vector.y;
	}
	return *this;
}

// Overloaded less than operator for ordering to be used inside std::set for instance
inline bool Vector2::operator<(const Vector2& vector) const {
	return (x == vector.x ? y < vector.y : x < vector.x);
}

// Return a vector taking the minimum components of two vectors
inline Vector2 Vector2::min(const Vector2& vector1, const Vector2& vector2) {
	return Vector2(std::min(vector1.x, vector2.x),
				   std::min(vector1.y, vector2.y));
}

// Return a vector taking the maximum components of two vectors
inline Vector2 Vector2::max(const Vector2& vector1, const Vector2& vector2) {
	return Vector2(std::max(vector1.x, vector2.x),
				   std::max(vector1.y, vector2.y));
}

// Return the zero vector
inline Vector2 Vector2::zero() {
	return Vector2(0, 0);
}

}
