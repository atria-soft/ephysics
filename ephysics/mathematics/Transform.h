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
		Vector3 m_position;

		/// Orientation
		Quaternion m_orientation;

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
	return m_position;
}

// Set the origin of the transform
inline void Transform::setPosition(const Vector3& position) {
	m_position = position;
}

// Return the rotation matrix
inline const Quaternion& Transform::getOrientation() const {
	return m_orientation;
}

// Set the rotation matrix of the transform
inline void Transform::setOrientation(const Quaternion& orientation) {
	m_orientation = orientation;
}

// Set the transform to the identity transform
inline void Transform::setToIdentity() {
	m_position = Vector3(0.0, 0.0, 0.0);
	m_orientation = Quaternion::identity();
}										   

// Set the transform from an OpenGL transform matrix
inline void Transform::setFromOpenGL(float* openglMatrix) {
	Matrix3x3 matrix(openglMatrix[0], openglMatrix[4], openglMatrix[8],
					 openglMatrix[1], openglMatrix[5], openglMatrix[9],
					 openglMatrix[2], openglMatrix[6], openglMatrix[10]);
	m_orientation = Quaternion(matrix);
	m_position.setAllValues(openglMatrix[12], openglMatrix[13], openglMatrix[14]);
}

// Get the OpenGL matrix of the transform
inline void Transform::getOpenGLMatrix(float* openglMatrix) const {
	const Matrix3x3& matrix = m_orientation.getMatrix();
	openglMatrix[0] = matrix[0][0]; openglMatrix[1] = matrix[1][0];
	openglMatrix[2] = matrix[2][0]; openglMatrix[3] = 0.0;
	openglMatrix[4] = matrix[0][1]; openglMatrix[5] = matrix[1][1];
	openglMatrix[6] = matrix[2][1]; openglMatrix[7] = 0.0;
	openglMatrix[8] = matrix[0][2]; openglMatrix[9] = matrix[1][2];
	openglMatrix[10] = matrix[2][2]; openglMatrix[11] = 0.0;
	openglMatrix[12] = m_position.x; openglMatrix[13] = m_position.y;
	openglMatrix[14] = m_position.z; openglMatrix[15] = 1.0;
}

// Return the inverse of the transform
inline Transform Transform::getInverse() const {
	const Quaternion& invQuaternion = m_orientation.getInverse();
	Matrix3x3 invMatrix = invQuaternion.getMatrix();
	return Transform(invMatrix * (-m_position), invQuaternion);
}

// Return an int32_terpolated transform
inline Transform Transform::int32_terpolateTransforms(const Transform& oldTransform,
												  const Transform& newTransform,
												  float int32_terpolationFactor) {

	Vector3 int32_terPosition = oldTransform.m_position * (float(1.0) - int32_terpolationFactor) +
							newTransform.m_position * int32_terpolationFactor;

	Quaternion int32_terOrientation = Quaternion::slerp(oldTransform.m_orientation,
													newTransform.m_orientation,
													int32_terpolationFactor);

	return Transform(int32_terPosition, int32_terOrientation);
}

// Return the identity transform
inline Transform Transform::identity() {
	return Transform(Vector3(0, 0, 0), Quaternion::identity());
}

// Return the transformed vector
inline Vector3 Transform::operator*(const Vector3& vector) const {
	return (m_orientation.getMatrix() * vector) + m_position;
}

// Operator of multiplication of a transform with another one
inline Transform Transform::operator*(const Transform& transform2) const {
	return Transform(m_position + m_orientation.getMatrix() * transform2.m_position,
					 m_orientation * transform2.m_orientation);
}

// Return true if the two transforms are equal
inline bool Transform::operator==(const Transform& transform2) const {
	return (m_position == transform2.m_position) && (m_orientation == transform2.m_orientation);
}	

// Return true if the two transforms are different
inline bool Transform::operator!=(const Transform& transform2) const {
	return !(*this == transform2);
}

// Assignment operator
inline Transform& Transform::operator=(const Transform& transform) {
	if (&transform != this) {
		m_position = transform.m_position;
		m_orientation = transform.m_orientation;
	}
	return *this;
}

}
