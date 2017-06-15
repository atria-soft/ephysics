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

#ifndef TEST_VECTOR2_H
#define TEST_VECTOR2_H

// Libraries
#include <test/Test.h>
#include <etk/math/Vector2D.hpp>

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class Testvec2
/**
 * Unit test for the vec2 class
 */
class Testvec2 : public Test {

	private :

		// ---------- Atributes ---------- //

		/// Zero vector
		vec2 mVectorZero;

		// Vector (3, 4)
		vec2 mvec34;

	public :

		// ---------- Methods ---------- //

		/// Constructor
		Testvec2(const std::string& name) : Test(name), mVectorZero(0, 0), mvec34(3, 4) {}

		/// Run the tests
		void run() {
			testConstructors();
			testLengthMethods();
			testDotProduct();
			testOthersMethods();
			testOperators();
		}

		/// Test the constructors, getter and setter
		void testConstructors() {

			// Test constructor
			test(mVectorZero.x() == 0.0);
			test(mVectorZero.y() == 0.0);
			test(mvec34.x() == 3.0);
			test(mvec34.y() == 4.0);

			// Test copy-constructor
			vec2 newVector(mvec34);
			test(newVector.x() == 3.0);
			test(newVector.y() == 4.0);

			// Test method to set values
			vec2 newvec2;
			newvec2.setValue(float(6.1), float(7.2));
			test(approxEqual(newvec2.x(), float(6.1)));
			test(approxEqual(newvec2.y(), float(7.2)));

			// Test method to set to zero
			newvec2.setZero();
			test(newvec2 == vec2(0, 0));
		}

		/// Test the length, unit vector and normalize methods
		void testLengthMethods() {

			// Test length methods
			test(mVectorZero.length() == 0.0);
			test(mVectorZero.length2() == 0.0);
			test(vec2(1, 0).length() == 1.0);
			test(vec2(0, 1).length() == 1.0);
			test(mvec34.length2() == 25.0);

			// Test unit vector methods
			test(vec2(1, 0).isUnit());
			test(vec2(0, 1).isUnit());
			test(!mvec34.isUnit());
			test(vec2(5, 0).safeNormalized() == vec2(1, 0));
			test(vec2(0, 5).safeNormalized() == vec2(0, 1));

			test(!mvec34.isZero());
			test(mVectorZero.isZero());

			// Test normalization method
			vec2 mVector10(1, 0);
			vec2 mVector01(0, 1);
			vec2 mVector50(5, 0);
			vec2 mVector05(0, 5);
			mVector10.normalize();
			mVector01.normalize();
			mVector50.normalize();
			mVector05.normalize();
			test(mVector10 == vec2(1, 0));
			test(mVector01 == vec2(0, 1));
			test(mVector50 == vec2(1, 0));
			test(mVector05 == vec2(0, 1));
		}

		/// Test the dot product
		void testDotProduct() {

			// Test the dot product
			test(vec2(5, 0).dot(vec2(0, 8)) == 0);
			test(vec2(5, 8).dot(vec2(0, 0)) == 0);
			test(vec2(12, 45).dot(vec2(0, 0)) == 0);
			test(vec2(5, 7).dot(vec2(5, 7)) == 74);
			test(vec2(3, 6).dot(vec2(-3, -6)) == -45);
			test(vec2(2, 3).dot(vec2(-7, 4)) == -2);
			test(vec2(4, 3).dot(vec2(8, 9)) == 59);
		}

		/// Test others methods
		void testOthersMethods() {

			// Test the method that returns the absolute vector
			test(vec2(4, 5).absolute() == vec2(4, 5));
			test(vec2(-7, -24).absolute() == vec2(7, 24));

			// Test the method that returns the minimal element
			test(vec2(6, 35).getMinAxis() == 0);
			test(vec2(564, 45).getMinAxis() == 1);
			test(vec2(98, 23).getMinAxis() == 1);
			test(vec2(-53, -25).getMinAxis() == 0);

			// Test the method that returns the maximal element
			test(vec2(6, 35).getMaxAxis() == 1);
			test(vec2(7, 537).getMaxAxis() == 1);
			test(vec2(98, 23).getMaxAxis() == 0);
			test(vec2(-53, -25).getMaxAxis() == 1);

			// Test the methot that return a max/min vector
			vec2 vec1(-5, 4);
			vec2 vec2(-8, 6);
			test(vec2::min(vec1, vec2) == vec2(-8, 4));
			test(vec2::max(vec1, vec2) == vec2(-5, 6));
		}

		/// Test the operators
		void testOperators() {

			// Test the [] operator
			test(mvec34[0] == 3);
			test(mvec34[1] == 4);

			// Assignment operator
			vec2 newVector(6, 4);
			newVector = vec2(7, 8);
			test(newVector == vec2(7, 8));

			// Equality, inequality operators
			test(vec2(5, 7) == vec2(5, 7));
			test(vec2(63, 64) != vec2(63, 84));
			test(vec2(63, 64) != vec2(12, 64));

			// Addition, substraction
			vec2 vector1(6, 33);
			vec2 vector2(7, 68);
			test(vec2(63, 24) + vec2(3, 4) == vec2(66, 28));
			test(vec2(63, 24) - vec2(3, 4) == vec2(60, 20));
			vector1 += vec2(5, 10);
			vector2 -= vec2(10, 21);
			test(vector1 == vec2(11, 43));
			test(vector2 == vec2(-3, 47));

			// Multiplication, division
			vec2 vector3(6, 33);
			vec2 vector4(15, 60);
			test(vec2(63, 24) * 3 == vec2(189, 72));
			test(3 * vec2(63, 24) == vec2(189, 72));
			test(vec2(14, 8) / 2 == vec2(7, 4));
			vector3 *= 10;
			vector4 /= 3;
			test(vector3 == vec2(60, 330));
			test(vector4 == vec2(5, 20));
			vec2 vector5(21, 80);
			vec2 vector6(7, 10);
			vec2 vector7 = vector5 * vector6;
			test(vector7 == vec2(147, 800));
			vec2 vector8 = vector5 / vector6;
			test(approxEqual(vector8.x(), 3));
			test(approxEqual(vector8.y(), 8));

			// Negative operator
			vec2 vector9(-34, 5);
			vec2 negative = -vector9;
			test(negative == vec2(34, -5));
		}
 };

}

#endif
