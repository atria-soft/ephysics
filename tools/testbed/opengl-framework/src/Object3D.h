/********************************************************************************
* OpenGL-Framework															  *
* Copyright (c) 2013 Daniel Chappuis											*
*********************************************************************************
*																			   *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.														 *
*																			   *
* Permission is granted to anyone to use this software for any purpose,		 *
* including commercial applications, and to alter it and redistribute it		*
* freely, subject to the following restrictions:								*
*																			   *
* 1. The origin of this software must not be misrepresented; you must not claim *
*	that you wrote the original software. If you use this software in a		*
*	product, an acknowledgment in the product documentation would be		   *
*	appreciated but is not required.										   *
*																			   *
* 2. Altered source versions must be plainly marked as such, and must not be	*
*	misrepresented as being the original software.							 *
*																			   *
* 3. This notice may not be removed or altered from any source distribution.	*
*																			   *
********************************************************************************/

#ifndef OBJECT3D_H
#define OBJECT3D_H

// Libraries
#include <ephysics/maths/vec3.h>
#include <ephysics/maths/Matrix4.h>

namespace openglframework {

// Class Object3D
// This class represent a generic 3D object on the scene.
class Object3D {

	protected:

		// -------------------- Attributes -------------------- //

		// etk::Transform3Dation matrix that convert local-space
		// coordinates to world-space coordinates
		Matrix4 m_transformMatrix;

	public:

		// -------------------- Methods -------------------- //

		// Constructor
		Object3D();

		// Destructor
		virtual ~Object3D();

		// Return the transform matrix
		const Matrix4& getTransformMatrix() const;

		// Set the transform matrix
		void setTransformMatrix(const Matrix4& matrix);

		// Set to the identity transform
		void setToIdentity();

		// Return the origin of object in world-space
		vec3 getOrigin() const;

		// Translate the object in world-space
		void translateWorld(const vec3& v);

		// Translate the object in local-space
		void translateLocal(const vec3& v);

		// Rotate the object in world-space
		void rotateWorld(const vec3& axis, float angle);

		// Rotate the object in local-space
		void rotateLocal(const vec3& axis, float angle);

		// Rotate around a world-space point
		void rotateAroundWorldPoint(const vec3& axis, float angle, const vec3& point);

		// Rotate around a local-space point
		void rotateAroundLocalPoint(const vec3& axis, float angle, const vec3& worldPoint);
};

// Return the transform matrix
inline const Matrix4& Object3D::getTransformMatrix() const {
	return m_transformMatrix;
}

// Set the transform matrix
inline void Object3D::setTransformMatrix(const Matrix4& matrix) {
	m_transformMatrix = matrix;
}

// Set to the identity transform
inline void Object3D::setToIdentity() {
	m_transformMatrix.setToIdentity();
}

 // Return the origin of object in world-space
inline vec3 Object3D::getOrigin() const {
	return m_transformMatrix * vec3(0.0, 0.0, 0.0);
}

// Translate the object in world-space
inline void Object3D::translateWorld(const vec3& v) {
	m_transformMatrix = Matrix4::translationMatrix(v) * m_transformMatrix;
}

// Translate the object in local-space
inline void Object3D::translateLocal(const vec3& v) {
	m_transformMatrix = m_transformMatrix * Matrix4::translationMatrix(v);
}

// Rotate the object in world-space
inline void Object3D::rotateWorld(const vec3& axis, float angle) {
	m_transformMatrix = Matrix4::rotationMatrix(axis, angle) * m_transformMatrix;
}

// Rotate the object in local-space
inline void Object3D::rotateLocal(const vec3& axis, float angle) {
	m_transformMatrix = m_transformMatrix * Matrix4::rotationMatrix(axis, angle);
}

// Rotate the object around a world-space point
inline void Object3D::rotateAroundWorldPoint(const vec3& axis, float angle,
											 const vec3& worldPoint) {
	m_transformMatrix = Matrix4::translationMatrix(worldPoint) * Matrix4::rotationMatrix(axis, angle)
					   * Matrix4::translationMatrix(-worldPoint) * m_transformMatrix;
}

// Rotate the object around a local-space point
inline void Object3D::rotateAroundLocalPoint(const vec3& axis, float angle,
											 const vec3& worldPoint) {

	// Convert the world point int32_to the local coordinate system
	vec3 localPoint = m_transformMatrix.getInverse() * worldPoint;

	m_transformMatrix = m_transformMatrix * Matrix4::translationMatrix(localPoint)
					   * Matrix4::rotationMatrix(axis, angle)
					   * Matrix4::translationMatrix(-localPoint);
}

}

#endif
