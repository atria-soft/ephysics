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

#ifndef CAMERA_H
#define CAMERA_H

// Libraries
#include <ephysics/Object3D.hpp>
#include <ephysics/definitions.hpp>

namespace openglframework {

// Class Camera
class Camera : public Object3D {

	protected :

		// ------------------- Attributes ------------------- //

		// Field of view
		float mFieldOfView;

		// Radius of the scene
		float mSceneRadius;

		// Near plane
		float mNearPlane;

		// Far plane
		float mFarPlane;

		// Width of the camera
		uint32_t m_width;

		// Height of the camera
		uint32_t mHeight;

		// Projection matrix
		Matrix4 mProjectionMatrix;

		// ------------------- Methods ------------------- //

		// Update the projection matrix
		void updateProjectionMatrix();

	public:

		// ------------------- Methods ------------------- //

		// Constructor
		Camera();

		// Destructor
		~Camera();

		// Get the projection matrix
		const Matrix4& getProjectionMatrix() const;

		// Set the dimensions of the camera
		void setDimensions(uint32_t width, uint32_t height);

		// Get the radius of the scene the camera should capture
		float getSceneRadius() const;

		// Set the radius of the scene the camera should capture
		// This will update the clipping planes accordingly
		void setSceneRadius(float radius);

		// Set the clipping planes
		void setClippingPlanes(float near, float far);

		// Set the field of view
		void setFieldOfView(float fov);

		// Set the zoom of the camera (a fraction between 0 and 1)
		void setZoom(float fraction);

		// Translate the camera go a given point using the dx, dy fraction
		void translateCamera(float dx, float dy, const vec3& worldPoint);

		// Get the near clipping plane
		float getNearClippingPlane() const;

		// Get the far clipping plane
		float getFarClippingPlane() const;

		// Get the width
		uint32_t getWidth() const;

		// Get the height
		uint32_t getHeight() const;
};

// Get the projection matrix
inline const Matrix4& Camera::getProjectionMatrix() const {
	return mProjectionMatrix;
}

// Set the dimensions of the camera
inline void Camera::setDimensions(uint32_t width, uint32_t height) {
	m_width = width;
	mHeight = height;
	updateProjectionMatrix();
}

// Get the radius of the scene the camera should capture
inline float Camera::getSceneRadius() const {
	return mSceneRadius;
}

// Set the radius of the scene the camera should capture
// This will update the clipping planes accordingly
inline void Camera::setSceneRadius(float radius) {
	mSceneRadius = radius;
	setClippingPlanes(0.01f * radius, 10.0f * radius);
}

// Set the clipping planes
inline void Camera::setClippingPlanes(float near, float far) {
	mNearPlane = near;
	mFarPlane = far;
	updateProjectionMatrix();
}

// Set the field of view
inline void Camera::setFieldOfView(float fov) {
	mFieldOfView = fov;
	updateProjectionMatrix();
}

// Set the zoom of the camera (a fraction between 0 and 1)
inline void Camera::setZoom(float fraction) {
	vec3 zoomVector(0, 0, mSceneRadius * fraction);
	translateLocal(zoomVector);
}

// Get the near clipping plane
inline float Camera::getNearClippingPlane() const {
	return mNearPlane;
}

// Get the far clipping plane
inline float Camera::getFarClippingPlane() const {
	return mFarPlane;
}

// Get the width
inline uint32_t Camera::getWidth() const {
	return m_width;
}

// Get the height
inline uint32_t Camera::getHeight() const {
	return mHeight;
}

}

#endif
