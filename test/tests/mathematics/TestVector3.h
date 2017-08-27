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

#ifndef TEST_VECTOR3_H
#define TEST_VECTOR3_H

// Libraries
#include <test/Test.hpp>
#include <etk/math/Vector3D.hpp>

/// Reactphysics3D namespace
namespace ephysics {

// Class Testvec3
/**
 * Unit test for the vec3 class
 */
class Testvec3 : public Test {

	private :

		// ---------- Atributes ---------- //

		/// Zero vector
		vec3 mVectorZero;

		// Vector (3, 4, 5)
		vec3 mvec345;

	public :

		// ---------- Methods ---------- //

		/// Constructor
		Testvec3(const etk::String& name): Test(name),mVectorZero(0, 0, 0),mvec345(3, 4, 5) {}

		/// Run the tests
		void run() {
			testConstructors();
			testLengthMethods();
			testDotCrossProducts();
			testOthersMethods();
			testOperators();
		}

		/// Test the constructors, getter and setter
		void testConstructors() {

			// Test constructor
			test(mVectorZero.x() == 0.0);
			test(mVectorZero.y() == 0.0);
			test(mVectorZero.z() == 0.0);
			test(mvec345.x() == 3.0);
			test(mvec345.y() == 4.0);
			test(mvec345.z() == 5.0);

			// Test copy-constructor
			vec3 newVector(mvec345);
			test(newVector.x() == 3.0);
			test(newVector.y() == 4.0);
			test(newVector.z() == 5.0);

			// Test method to set values
			vec3 newvec2;
			newvec2.setValue(float(6.1), float(7.2), float(8.6));
			test(approxEqual(newvec2.x(), float(6.1)));
			test(approxEqual(newvec2.y(), float(7.2)));
			test(approxEqual(newvec2.z(), float(8.6)));

			// Test method to set to zero
			newvec2.setZero();
			test(newvec2 == vec3(0, 0, 0));
		}

		/// Test the length, unit vector and normalize methods
		void testLengthMethods() {

			// Test length methods
			test(mVectorZero.length() == 0.0);
			test(mVectorZero.length2() == 0.0);
			test(vec3(1, 0, 0).length() == 1.0);
			test(vec3(0, 1, 0).length() == 1.0);
			test(vec3(0, 0, 1).length() == 1.0);
			test(mvec345.length2() == 50.0);

			// Test unit vector methods
			test(vec3(1, 0, 0).isUnit());
			test(vec3(0, 1, 0).isUnit());
			test(vec3(0, 0, 1).isUnit());
			test(!mvec345.isUnit());
			test(vec3(5, 0, 0).safeNormalized() == vec3(1, 0, 0));
			test(vec3(0, 5, 0).safeNormalized() == vec3(0, 1, 0));
			test(vec3(0, 0, 5).safeNormalized() == vec3(0, 0, 1));

			test(!mvec345.isZero());
			test(mVectorZero.isZero());

			// Test normalization method
			vec3 mVector100(1, 0, 0);
			vec3 mVector010(0, 1, 0);
			vec3 mVector001(0, 0, 1);
			vec3 mVector500(5, 0, 0);
			vec3 mVector050(0, 5, 0);
			vec3 mVector005(0, 0, 5);
			mVector100.normalize();
			mVector010.normalize();
			mVector001.normalize();
			mVector500.normalize();
			mVector050.normalize();
			mVector005.normalize();
			test(mVector100 == vec3(1, 0, 0));
			test(mVector010 == vec3(0, 1, 0));
			test(mVector001 == vec3(0, 0, 1));
			test(mVector500 == vec3(1, 0, 0));
			test(mVector050 == vec3(0, 1, 0));
			test(mVector005 == vec3(0, 0, 1));
		}

		/// Test the dot and cross products
		void testDotCrossProducts() {

			// Test the dot product
			test(vec3(5, 0, 0).dot(vec3(0, 8, 0)) == 0);
			test(vec3(5, 8, 0).dot(vec3(0, 0, 6)) == 0);
			test(vec3(12, 45, 83).dot(vec3(0, 0, 0)) == 0);
			test(vec3(5, 7, 8).dot(vec3(5, 7, 8)) == 138);
			test(vec3(3, 6, 78).dot(vec3(-3, -6, -78)) == -6129);
			test(vec3(2, 3, 5).dot(vec3(2, 3, 5)) == 38);
			test(vec3(4, 3, 2).dot(vec3(8, 9, 10)) == 79);

			// Test the cross product
			test(vec3(0, 0, 0).cross(vec3(0, 0, 0)) == vec3(0, 0, 0));
			test(vec3(6, 7, 2).cross(vec3(6, 7, 2)) == vec3(0, 0, 0));
			test(vec3(1, 0, 0).cross(vec3(0, 1, 0)) == vec3(0, 0, 1));
			test(vec3(0, 1, 0).cross(vec3(0, 0, 1)) == vec3(1, 0, 0));
			test(vec3(0, 0, 1).cross(vec3(0, 1, 0)) == vec3(-1, 0, 0));
			test(vec3(4, 7, 24).cross(vec3(8, 13, 11)) == vec3(-235, 148, -4));
			test(vec3(-4, 42, -2).cross(vec3(35, 7, -21)) == vec3(-868, -154, -1498));
		}

		/// Test others methods
		void testOthersMethods() {

			// Test the method that returns the absolute vector
			test(vec3(4, 5, 6).absolute() == vec3(4, 5, 6));
			test(vec3(-7, -24, -12).absolute() == vec3(7, 24, 12));

			// Test the method that returns the minimal element
			test(vec3(6, 35, 82).getMinAxis() == 0);
			test(vec3(564, 45, 532).getMinAxis() == 1);
			test(vec3(98, 23, 3).getMinAxis() == 2);
			test(vec3(-53, -25, -63).getMinAxis() == 2);

			// Test the method that returns the maximal element
			test(vec3(6, 35, 82).getMaxAxis() == 2);
			test(vec3(7, 533, 36).getMaxAxis() == 1);
			test(vec3(98, 23, 3).getMaxAxis() == 0);
			test(vec3(-53, -25, -63).getMaxAxis() == 1);

			// Test the methot that return a max/min vector
			vec3 vec1(-5, 4, 2);
			vec3 vec2(-8, 6, -1);
			test(vec3::min(vec1, vec2) == vec3(-8, 4, -1));
			test(vec3::max(vec1, vec2) == vec3(-5, 6, 2));
		}

		/// Test the operators
		void testOperators() {

			// Test the [] operator
			test(mvec345[0] == 3);
			test(mvec345[1] == 4);
			test(mvec345[2] == 5);

			// Assignment operator
			vec3 newVector(6, 4, 2);
			newVector = vec3(7, 8, 9);
			test(newVector == vec3(7, 8, 9));

			// Equality, inequality operators
			test(vec3(5, 7, 3) == vec3(5, 7, 3));
			test(vec3(63, 64, 24) != vec3(63, 64, 5));
			test(vec3(63, 64, 24) != vec3(12, 64, 24));
			test(vec3(63, 64, 24) != vec3(63, 8, 24));

			// Addition, substraction
			vec3 vector1(6, 33, 62);
			vec3 vector2(7, 68, 35);
			test(vec3(63, 24, 5) + vec3(3, 4, 2) == vec3(66, 28, 7));
			test(vec3(63, 24, 5) - vec3(3, 4, 2) == vec3(60, 20, 3));
			vector1 += vec3(5, 10, 12);
			vector2 -= vec3(10, 21, 5);
			test(vector1 == vec3(11, 43, 74));
			test(vector2 == vec3(-3, 47, 30));

			// Multiplication, division
			vec3 vector3(6, 33, 62);
			vec3 vector4(15, 60, 33);
			test(vec3(63, 24, 5) * 3 == vec3(189, 72, 15));
			test(3 * vec3(63, 24, 5) == vec3(189, 72, 15));
			test(vec3(14, 8, 50) / 2 == vec3(7, 4, 25));
			vector3 *= 10;
			vector4 /= 3;
			test(vector3 == vec3(60, 330, 620));
			test(vector4 == vec3(5, 20, 11));
			vec3 vector5(21, 80, 45);
			vec3 vector6(7, 10, 3);
			vec3 vector7 = vector5 * vector6;
			test(vector7 == vec3(147, 800, 135));
			vec3 vector8 = vector5 / vector6;
			test(approxEqual(vector8.x(), 3));
			test(approxEqual(vector8.y(), 8));
			test(approxEqual(vector8.z(), 15));

			// Negative operator
			vec3 vector9(-34, 5, 422);
			vec3 negative = -vector9;
			test(negative == vec3(34, -5, -422));
		}
 };

}

#endif
