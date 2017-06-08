/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/mathematics/Matrix3x3.h>
#include <ephysics/mathematics/Vector3.h>
#include <ephysics/mathematics/Quaternion.h>

// ReactPhysiscs3D namespace
namespace reactphysics3d {

// Class Transform
/**
 * This class represents a position and an orientation in 3D. It can
 * also be seen as representing a translation and a rotation.
 */
class Transform {

	private :

		// -------------------- Attributes -------------------- //

		/// Position
		Vector3 mPosition;

		/// Orientation
		Quaternion mOrientation;

	public :

		// -------------------- Methods -------------------- //

		/// Constructor
		Transform();

		/// Constructor
		Transform(const Vector3& position, const Matrix3x3& orientation);

		/// Constructor
		Transform(const Vector3& position, const Quaternion& orientation);

		/// Destructor
		~Transform();

		/// Copy-constructor
		Transform(const Transform& transform);

		/// Return the origin of the transform
		const Vector3& getPosition() const;

		/// Set the origin of the transform
		void setPosition(const Vector3& position);

		/// Return the orientation quaternion
		const Quaternion& getOrientation() const;

		/// Set the rotation quaternion
		void setOrientation(const Quaternion& orientation);

		/// Set the transform to the identity transform
		void setToIdentity();

		/// Set the transform from an OpenGL transform matrix
		void setFromOpenGL(float* openglMatrix);

		/// Get the OpenGL matrix of the transform
		void getOpenGLMatrix(float* openglMatrix) const;

		/// Return the inverse of the transform
		Transform getInverse() const;

		/// Return an int32_terpolated transform
		static Transform int32_terpolateTransforms(const Transform& oldTransform,
											   const Transform& newTransform,
											   float int32_terpolationFactor);

		/// Return the identity transform
		static Transform identity();

		/// Return the transformed vector
		Vector3 operator*(const Vector3& vector) const;

		/// Operator of multiplication of a transform with another one
		Transform operator*(const Transform& transform2) const;

		/// Return true if the two transforms are equal
		bool operator==(const Transform& transform2) const;

		/// Return true if the two transforms are different
		bool operator!=(const Transform& transform2) const;

		/// Assignment operator
		Transform& operator=(const Transform& transform);
};

// Return the position of the transform
inline const Vector3& Transform::getPosition() const {
	return mPosition;
}

// Set the origin of the transform
inline void Transform::setPosition(const Vector3& position) {
	mPosition = position;
}

// Return the rotation matrix
inline const Quaternion& Transform::getOrientation() const {
	return mOrientation;
}

// Set the rotation matrix of the transform
inline void Transform::setOrientation(const Quaternion& orientation) {
	mOrientation = orientation;
}

// Set the transform to the identity transform
inline void Transform::setToIdentity() {
	mPosition = Vector3(0.0, 0.0, 0.0);
	mOrientation = Quaternion::identity();
}										   

// Set the transform from an OpenGL transform matrix
inline void Transform::setFromOpenGL(float* openglMatrix) {
	Matrix3x3 matrix(openglMatrix[0], openglMatrix[4], openglMatrix[8],
					 openglMatrix[1], openglMatrix[5], openglMatrix[9],
					 openglMatrix[2], openglMatrix[6], openglMatrix[10]);
	mOrientation = Quaternion(matrix);
	mPosition.setAllValues(openglMatrix[12], openglMatrix[13], openglMatrix[14]);
}

// Get the OpenGL matrix of the transform
inline void Transform::getOpenGLMatrix(float* openglMatrix) const {
	const Matrix3x3& matrix = mOrientation.getMatrix();
	openglMatrix[0] = matrix[0][0]; openglMatrix[1] = matrix[1][0];
	openglMatrix[2] = matrix[2][0]; openglMatrix[3] = 0.0;
	openglMatrix[4] = matrix[0][1]; openglMatrix[5] = matrix[1][1];
	openglMatrix[6] = matrix[2][1]; openglMatrix[7] = 0.0;
	openglMatrix[8] = matrix[0][2]; openglMatrix[9] = matrix[1][2];
	openglMatrix[10] = matrix[2][2]; openglMatrix[11] = 0.0;
	openglMatrix[12] = mPosition.x; openglMatrix[13] = mPosition.y;
	openglMatrix[14] = mPosition.z; openglMatrix[15] = 1.0;
}

// Return the inverse of the transform
inline Transform Transform::getInverse() const {
	const Quaternion& invQuaternion = mOrientation.getInverse();
	Matrix3x3 invMatrix = invQuaternion.getMatrix();
	return Transform(invMatrix * (-mPosition), invQuaternion);
}

// Return an int32_terpolated transform
inline Transform Transform::int32_terpolateTransforms(const Transform& oldTransform,
												  const Transform& newTransform,
												  float int32_terpolationFactor) {

	Vector3 int32_terPosition = oldTransform.mPosition * (float(1.0) - int32_terpolationFactor) +
							newTransform.mPosition * int32_terpolationFactor;

	Quaternion int32_terOrientation = Quaternion::slerp(oldTransform.mOrientation,
													newTransform.mOrientation,
													int32_terpolationFactor);

	return Transform(int32_terPosition, int32_terOrientation);
}

// Return the identity transform
inline Transform Transform::identity() {
	return Transform(Vector3(0, 0, 0), Quaternion::identity());
}

// Return the transformed vector
inline Vector3 Transform::operator*(const Vector3& vector) const {
	return (mOrientation.getMatrix() * vector) + mPosition;
}

// Operator of multiplication of a transform with another one
inline Transform Transform::operator*(const Transform& transform2) const {
	return Transform(mPosition + mOrientation.getMatrix() * transform2.mPosition,
					 mOrientation * transform2.mOrientation);
}

// Return true if the two transforms are equal
inline bool Transform::operator==(const Transform& transform2) const {
	return (mPosition == transform2.mPosition) && (mOrientation == transform2.mOrientation);
}	

// Return true if the two transforms are different
inline bool Transform::operator!=(const Transform& transform2) const {
	return !(*this == transform2);
}

// Assignment operator
inline Transform& Transform::operator=(const Transform& transform) {
	if (&transform != this) {
		mPosition = transform.mPosition;
		mOrientation = transform.mOrientation;
	}
	return *this;
}

}
