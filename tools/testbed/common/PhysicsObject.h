/********************************************************************************
* ReactPhysics3D physics library, http://www.ephysics.com				 *
* Copyright (c) 2010-2016 Daniel Chappuis									   *
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

#ifndef PHYSICSOBJECT_H
#define PHYSICSOBJECT_H

// Libraries
#include <ephysics/openglframework.hpp>
#include <ephysics/ephysics.hpp>

// Class PhysicsObject
class PhysicsObject {

	protected:

		/// Body used to simulate the dynamics of the box
		ephysics::CollisionBody* m_body;

		/// Previous transform of the body (for int32_terpolation)
		ephysics::etk::Transform3D mPreviousTransform;

		/// Main color of the box
		openglframework::Color mColor;

		/// Sleeping color
		openglframework::Color mSleepingColor;

		// Compute the new transform matrix
		openglframework::Matrix4 computeetk::Transform3D(float int32_terpolationFactor,
												 const openglframework::Matrix4 &scalingMatrix);

	public:

		/// Constructor
		PhysicsObject();

		/// Update the transform matrix of the object
		virtual void updateetk::Transform3D(float int32_terpolationFactor)=0;

		/// Set the color of the box
		void setColor(const openglframework::Color& color);

		/// Set the sleeping color of the box
		void setSleepingColor(const openglframework::Color& color);

		/// Return a pointer to the collision body of the box
		ephysics::CollisionBody* getCollisionBody();

		/// Return a pointer to the rigid body of the box
		ephysics::RigidBody* getRigidBody();

		/// Set the scaling of the object
		virtual void setScaling(const openglframework::vec3& scaling)=0;
};

// Set the color of the box
inline void PhysicsObject::setColor(const openglframework::Color& color) {
	mColor = color;
}

// Set the sleeping color of the box
inline void PhysicsObject::setSleepingColor(const openglframework::Color& color) {
	mSleepingColor = color;
}

// Return a pointer to the collision body of the box
inline ephysics::CollisionBody* PhysicsObject::getCollisionBody() {
	return m_body;
}

// Return a pointer to the rigid body of the box (NULL if it's not a rigid body)
inline ephysics::RigidBody* PhysicsObject::getRigidBody() {
	return dynamic_cast<ephysics::RigidBody*>(m_body);
}

#endif

