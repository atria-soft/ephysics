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

#ifndef TEST_TRANSFORM_H
#define TEST_TRANSFORM_H

// Libraries
#include <test/Test.hpp>
#include <etk/math/Transform3D.hpp>

/// Reactphysics3D namespace
namespace ephysics {

// Class Testetk::Transform3D
/**
 * Unit test for the etk::Transform3D class
 */
class Testetk::Transform3D : public Test {

	private :

		// ---------- Atributes ---------- //

		/// Identity transform
		etk::Transform3D mIdentityTransform;

		/// First example transform
		etk::Transform3D m_transform1;

		/// Second example transform
		etk::Transform3D m_transform2;

	public :

		// ---------- Methods ---------- //

		/// Constructor
		Testetk::Transform3D(const std::string& name) : Test(name) {

			mIdentityetk::Transform3D.setToIdentity();

			float sinA = sin(PI/8.0f);
			float cosA = cos(PI/8.0f);
			m_transform1 = etk::Transform3D(vec3(4, 5, 6), etk::Quaternion(sinA, sinA, sinA, cosA));

			float sinB = sin(PI/3.0f);
			float cosB = cos(PI/3.0f);
			m_transform2 = etk::Transform3D(vec3(8, 45, -6), etk::Quaternion(sinB, sinB, sinB, cosB));
		}

		/// Run the tests
		void run() {
			testConstructors();
			testGetSet();
			testInverse();
			testGetSetOpenGLMatrix();
			testInterpolateetk::Transform3D();
			testIdentity();
			testOperators();
		}

		/// Test the constructors
		void testConstructors() {
			etk::Transform3D transform1(vec3(1, 2, 3), etk::Quaternion(6, 7, 8, 9));
			etk::Transform3D transform2(vec3(4, 5, 6), etk::Matrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1));
			etk::Transform3D transform3(transform1);
			test(transform1.getPosition() == vec3(1, 2, 3));
			test(transform1.getOrientation() == etk::Quaternion(6, 7, 8, 9));
			test(transform2.getPosition() == vec3(4, 5, 6));
			test(transform2.getOrientation() == etk::Quaternion::identity());
			test(transform3 == transform1);
		}

		/// Test getter and setter
		void testGetSet() {
			test(mIdentityetk::Transform3D.getPosition() == vec3(0, 0, 0));
			test(mIdentityetk::Transform3D.getOrientation() == etk::Quaternion::identity());
			etk::Transform3D transform;
			transform.setPosition(vec3(5, 7, 8));
			transform.setOrientation(etk::Quaternion(1, 2, 3, 1));
			test(transform.getPosition() == vec3(5, 7, 8));
			test(transform.getOrientation() == etk::Quaternion(1, 2, 3, 1));
			transform.setToIdentity();
			test(transform.getPosition() == vec3(0, 0, 0));
			test(transform.getOrientation() == etk::Quaternion::identity());
		}

		/// Test the inverse
		void testInverse() {
			etk::Transform3D inverseTransform = m_transform1.getInverse();
			vec3 vector(2, 3, 4);
			vec3 tempVector = m_transform1 * vector;
			vec3 tempvec2 = inverseetk::Transform3D * tempVector;
			test(approxEqual(tempvec2.x(), vector.x, float(10e-6)));
			test(approxEqual(tempvec2.y(), vector.y, float(10e-6)));
			test(approxEqual(tempvec2.z(), vector.z, float(10e-6)));
		}

		/// Test methods to set and get transform matrix from and to OpenGL
		void testGetSetOpenGLMatrix() {
			etk::Transform3D transform;
			vec3 position = m_transform1.getPosition();
			etk::Matrix3x3 orientation = m_transform1.getOrientation().getMatrix();
			float openglMatrix[16] = {orientation[0][0], orientation[1][0],
										orientation[2][0], 0,
										orientation[0][1], orientation[1][1],
										orientation[2][1], 0,
										orientation[0][2], orientation[1][2],
										orientation[2][2], 0,
										position.x(), position.y(), position.z(), 1};
			transform.setFromOpenGL(openglMatrix);
			float openglMatrix2[16];
			transform.getOpenGLMatrix(openglMatrix2);
			test(approxEqual(openglMatrix2[0], orientation[0][0]));
			test(approxEqual(openglMatrix2[1], orientation[1][0]));
			test(approxEqual(openglMatrix2[2], orientation[2][0]));
			test(approxEqual(openglMatrix2[3], 0));
			test(approxEqual(openglMatrix2[4], orientation[0][1]));
			test(approxEqual(openglMatrix2[5], orientation[1][1]));
			test(approxEqual(openglMatrix2[6], orientation[2][1]));
			test(approxEqual(openglMatrix2[7], 0));
			test(approxEqual(openglMatrix2[8], orientation[0][2]));
			test(approxEqual(openglMatrix2[9], orientation[1][2]));
			test(approxEqual(openglMatrix2[10], orientation[2][2]));
			test(approxEqual(openglMatrix2[11], 0));
			test(approxEqual(openglMatrix2[12], position.x()));
			test(approxEqual(openglMatrix2[13], position.y()));
			test(approxEqual(openglMatrix2[14], position.z()));
			test(approxEqual(openglMatrix2[15], 1));
		}

		/// Test the method to int32_terpolate transforms
		void testInterpolateetk::Transform3D() {
			etk::Transform3D transformStart = Transform::int32_terpolateTransforms(m_transform1, m_transform2,0);
			etk::Transform3D transformEnd = Transform::int32_terpolateTransforms(m_transform1, m_transform2,1);
			test(transformStart == m_transform1);
			test(transformEnd == m_transform2);

			float sinA = sin(PI/3.0f);
			float cosA = cos(PI/3.0f);
			float sinB = sin(PI/6.0f);
			float cosB = cos(PI/6.0f);
			etk::Transform3D transform1(vec3(4, 5, 6), etk::Quaternion::identity());
			etk::Transform3D transform2(vec3(8, 11, 16), etk::Quaternion(sinA, sinA, sinA, cosA));
			etk::Transform3D transform = Transform::int32_terpolateTransforms(transform1, transform2, 0.5);
			vec3 position = transform.getPosition();
			etk::Quaternion orientation = transform.getOrientation();
			test(approxEqual(position.x(), 6));
			test(approxEqual(position.y(), 8));
			test(approxEqual(position.z(), 11));
			test(approxEqual(orientation.x(), sinB));
			test(approxEqual(orientation.y(), sinB));
			test(approxEqual(orientation.z(), sinB));
			test(approxEqual(orientation.w, cosB));
		}

		/// Test the identity methods
		void testIdentity() {
			etk::Transform3D transform = Transform::identity();
			test(transform.getPosition() == vec3(0, 0, 0));
			test(transform.getOrientation() == etk::Quaternion::identity());

			etk::Transform3D transform2(vec3(5, 6, 2), etk::Quaternion(3, 5, 1, 6));
			transform2.setToIdentity();
			test(transform2.getPosition() == vec3(0, 0, 0));
			test(transform2.getOrientation() == etk::Quaternion::identity());
		}

		/// Test the overloaded operators
		void testOperators() {

			// Equality, inequality operator
			test(m_transform1 == m_transform1);
			test(m_transform1 != m_transform2);

			// Assignment operator
			etk::Transform3D transform;
			transform = m_transform1;
			test(transform == m_transform1);

			// Multiplication
			vec3 vector(7, 53, 5);
			vec3 vector2 = m_transform2 * (m_transform1 * vector);
			vec3 vector3 = (m_transform2 * m_transform1) * vector;
			test(approxEqual(vector2.x(), vector3.x, float(10e-6)));
			test(approxEqual(vector2.y(), vector3.y, float(10e-6)));
			test(approxEqual(vector2.z(), vector3.z, float(10e-6)));
		}
 };

}

#endif
