/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com				 *
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

#ifndef TEST_QUATERNION_H
#define TEST_QUATERNION_H

// Libraries
#include <test/Test.h>
#include <ephysics/mathematics/etk::Quaternion.h>

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class Testetk::Quaternion
/**
 * Unit test for the etk::Quaternion class
 */
class Testetk::Quaternion : public Test {

	private :

		// ---------- Atributes ---------- //

		/// Identity etk::Quaternion
		etk::Quaternion mIdentity;

		/// First test quaternion
		etk::Quaternion mQuaternion1;

	public :

		// ---------- Methods ---------- //

		/// Constructor
		Testetk::Quaternion(const std::string& name) : Test(name), mIdentity(Quaternion::identity()) {

			float sinA = sin(float(PI/8.0));
			float cosA = cos(float(PI/8.0));
			vec3 vector(2, 3, 4);
			vector.normalize();
			metk::Quaternion1 = etk::Quaternion(vector.x() * sinA, vector.y() * sinA, vector.z() * sinA, cosA);
			metk::Quaternion1.normalize();
		}

		/// Run the tests
		void run() {
			testConstructors();
			testUnitLengthNormalize();
			testOthersMethods();
			testOperators();
		}

		/// Test the constructors
		void testConstructors() {

			etk::Quaternion quaternion1(mQuaternion1);
			test(metk::Quaternion1 == quaternion1);

			etk::Quaternion quaternion2(4, 5, 6, 7);
			test(quaternion2 == etk::Quaternion(4, 5, 6, 7));

			etk::Quaternion quaternion3(8, vec3(3, 5, 2));
			test(quaternion3 == etk::Quaternion(3, 5, 2, 8));

			etk::Quaternion quaternion4(mQuaternion1.getMatrix());
			test(approxEqual(quaternion4.x(), metk::Quaternion1.x));
			test(approxEqual(quaternion4.y(), metk::Quaternion1.y));
			test(approxEqual(quaternion4.z(), metk::Quaternion1.z));
			test(approxEqual(quaternion4.w, metk::Quaternion1.w));

			// Test conversion from Euler angles to quaternion

			const float PI_OVER_2 = PI * 0.5f;
			const float PI_OVER_4 = PI_OVER_2 * 0.5f;
			etk::Quaternion quaternion5(PI_OVER_2, 0, 0);
			etk::Quaternion quaternionTest5(std::sin(PI_OVER_4), 0, 0, std::cos(PI_OVER_4));
			quaternionTest5.normalize();
			test(approxEqual(quaternion5.x(), quaternionTest5.x));
			test(approxEqual(quaternion5.y(), quaternionTest5.y));
			test(approxEqual(quaternion5.z(), quaternionTest5.z));
			test(approxEqual(quaternion5.w, quaternionTest5.w));

			etk::Quaternion quaternion6(0, PI_OVER_2, 0);
			etk::Quaternion quaternionTest6(0, std::sin(PI_OVER_4), 0, std::cos(PI_OVER_4));
			quaternionTest6.normalize();
			test(approxEqual(quaternion6.x(), quaternionTest6.x));
			test(approxEqual(quaternion6.y(), quaternionTest6.y));
			test(approxEqual(quaternion6.z(), quaternionTest6.z));
			test(approxEqual(quaternion6.w, quaternionTest6.w));

			etk::Quaternion quaternion7(vec3(0, 0, PI_OVER_2));
			etk::Quaternion quaternionTest7(0, 0, std::sin(PI_OVER_4), std::cos(PI_OVER_4));
			quaternionTest7.normalize();
			test(approxEqual(quaternion7.x(), quaternionTest7.x));
			test(approxEqual(quaternion7.y(), quaternionTest7.y));
			test(approxEqual(quaternion7.z(), quaternionTest7.z));
			test(approxEqual(quaternion7.w, quaternionTest7.w));
		}

		/// Test unit, length, normalize methods
		void testUnitLengthNormalize() {

			// Test method that returns the length
			etk::Quaternion quaternion(2, 3, -4, 5);
			test(approxEqual(quaternion.length(), sqrt(float(54.0))));

			// Test method that returns a unit quaternion
			test(approxEqual(quaternion.safeNormalized().length(), 1.0));

			// Test the normalization method
			etk::Quaternion quaternion2(4, 5, 6, 7);
			quaternion2.normalize();
			test(approxEqual(quaternion2.length(), 1.0));
		}

		/// Test others methods
		void testOthersMethods() {

			// Test the method to set the values
			etk::Quaternion quaternion;
			quaternion.setValue(1, 2, 3, 4);
			test(quaternion == etk::Quaternion(1, 2, 3, 4));

			// Test the method to set the quaternion to zero
			quaternion.setZero();
			test(quaternion == etk::Quaternion(0, 0, 0, 0));

			// Tes the methods to get or set to identity
			etk::Quaternion identity1(1, 2, 3, 4);
			identity1.setToIdentity();
			test(identity1 == etk::Quaternion(0, 0, 0, 1));
			test(etk::Quaternion::identity() == etk::Quaternion(0, 0, 0, 1));

			// Test the method to get the vector (x, y, z)
			vec3 v = metk::Quaternion1.getVectorV();
			test(v.x() == metk::Quaternion1.x);
			test(v.y() == metk::Quaternion1.y);
			test(v.z() == metk::Quaternion1.z);

			// Test the conjugate method
			etk::Quaternion conjugate = mQuaternion1.getConjugate();
			test(conjugate.x() == -metk::Quaternion1.x);
			test(conjugate.y() == -metk::Quaternion1.y);
			test(conjugate.z() == -metk::Quaternion1.z);
			test(conjugate.w == metk::Quaternion1.w);

			// Test the inverse methods
			etk::Quaternion inverse1 = mQuaternion1.getInverse();
			etk::Quaternion inverse2(mQuaternion1);
			inverse2.inverse();
			test(inverse2 == inverse1);
			etk::Quaternion product = mQuaternion1 * inverse1;
			test(approxEqual(product.x(), mIdentity.x, float(10e-6)));
			test(approxEqual(product.y(), mIdentity.y, float(10e-6)));
			test(approxEqual(product.z(), mIdentity.z, float(10e-6)));
			test(approxEqual(product.w, mIdentity.w, float(10e-6)));

			// Test the dot product
			etk::Quaternion quaternion1(2, 3, 4, 5);
			etk::Quaternion quaternion2(6, 7, 8, 9);
			float dotProduct = quaternion1.dot(quaternion2);
			test(dotProduct == 110);

			// Test the method that returns the rotation angle and axis
			vec3 axis;
			float angle;
			vec3 originalAxis = vec3(2, 3, 4).safeNormalized();
			metk::Quaternion1.getRotationAngleAxis(angle, axis);
			test(approxEqual(axis.x(), originalAxis.x));
			test(approxEqual(angle, float(PI/4.0), float(10e-6)));

			// Test the method that returns the corresponding matrix
			etk::Matrix3x3 matrix = metk::Quaternion1.getMatrix();
			vec3 vector(56, -2, 82);
			vec3 vector1 = matrix * vector;
			vec3 vector2 = metk::Quaternion1 * vector;
			test(approxEqual(vector1.x(), vector2.x, float(10e-6)));
			test(approxEqual(vector1.y(), vector2.y, float(10e-6)));
			test(approxEqual(vector1.z(), vector2.z, float(10e-6)));

			// Test slerp method
			etk::Quaternion quatStart = quaternion1.safeNormalized();
			etk::Quaternion quatEnd = quaternion2.safeNormalized();
			etk::Quaternion test1 = etk::Quaternion::slerp(quatStart, quatEnd, 0.0);
			etk::Quaternion test2 = etk::Quaternion::slerp(quatStart, quatEnd, 1.0);
			test(test1 == quatStart);
			test(test2 == quatEnd);
			float sinA = sin(float(PI/4.0));
			float cosA = cos(float(PI/4.0));
			etk::Quaternion quat(sinA, 0, 0, cosA);
			etk::Quaternion test3 = etk::Quaternion::slerp(mIdentity, quat, 0.5f);
			test(approxEqual(test3.x(), sin(float(PI/8.0))));
			test(approxEqual(test3.y(), 0.0));
			test(approxEqual(test3.z(), 0.0));
			test(approxEqual(test3.w, cos(float(PI/8.0)), float(10e-6)));
		}

		/// Test overloaded operators
		void testOperators() {

			// Test addition
			etk::Quaternion quat1(4, 5, 2, 10);
			etk::Quaternion quat2(-2, 7, 8, 3);
			etk::Quaternion test1 = quat1 + quat2;
			etk::Quaternion test11(-6, 52, 2, 8);
			test11 += quat1;
			test(test1 == etk::Quaternion(2, 12, 10, 13));
			test(test11 == etk::Quaternion(-2, 57, 4, 18));

			// Test substraction
			etk::Quaternion test2 = quat1 - quat2;
			etk::Quaternion test22(-73, 62, 25, 9);
			test22 -= quat1;
			test(test2 == etk::Quaternion(6, -2, -6, 7));
			test(test22 == etk::Quaternion(-77, 57, 23, -1));

			// Test multiplication with a number
			etk::Quaternion test3 = quat1 * 3.0;
			test(test3 == etk::Quaternion(12, 15, 6, 30));

			// Test multiplication between two quaternions
			etk::Quaternion test4 = quat1 * quat2;
			etk::Quaternion test5 = mQuaternion1 * mIdentity;
			test(test4 == etk::Quaternion(18, 49, 124, -13));
			test(test5 == metk::Quaternion1);

			// Test multiplication between a quaternion and a point
			vec3 point(5, -24, 563);
			vec3 vector1 = mIdentity * point;
			vec3 vector2 = metk::Quaternion1 * point;
			vec3 testvec2 = metk::Quaternion1.getMatrix() * point;
			test(vector1 == point);
			test(approxEqual(vector2.x(), testvec2.x, float(10e-5)));
			test(approxEqual(vector2.y(), testvec2.y, float(10e-5)));
			test(approxEqual(vector2.z(), testvec2.z, float(10e-5)));

			// Test assignment operator
			etk::Quaternion quaternion;
			quaternion = metk::Quaternion1;
			test(quaternion == metk::Quaternion1);

			// Test equality operator
			test(metk::Quaternion1 == mQuaternion1);
		}
 };

}

#endif
