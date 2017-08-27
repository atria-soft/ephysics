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

#ifndef TEST_MATRIX2X2_H
#define TEST_MATRIX2X2_H

// Libraries
#include <test/Test.hpp>
#include <etk/math/Matrix2x2.hpp>

/// Reactphysics3D namespace
namespace ephysics {

// Class Testetk::Matrix2x2
/**
 * Unit test for the etk::Matrix2x2 class
 */
class Testetk::Matrix2x2 : public Test {

	private :

		// ---------- Atributes ---------- //

		/// Identity transform
		etk::Matrix2x2 mIdentity;

		/// First example matrix
		etk::Matrix2x2 mMatrix1;

	public :

		// ---------- Methods ---------- //

		/// Constructor
		Testetk::Matrix2x2(const etk::String& name)
			   : Test(name), mIdentity(etk::Matrix2x2::identity()), mMatrix1(2, 24, -4, 5) {

		}

		/// Run the tests
		void run() {
			testConstructors();
			testGetSet();
			testIdentity();
			testZero();
			testOthersMethods();
			testOperators();
		}

		/// Test the constructors
		void testConstructors() {

			etk::Matrix2x2 test1(5.0);
			etk::Matrix2x2 test2(2, 3, 4, 5);
			etk::Matrix2x2 test3(mMatrix1);

			test(test1[0][0] == 5);
			test(test1[0][1] == 5);
			test(test1[1][0] == 5);
			test(test1[1][1] == 5);

			test(test2[0][0] == 2);
			test(test2[0][1] == 3);
			test(test2[1][0] == 4);
			test(test2[1][1] == 5);

			test(test3 == mMatrix1);
		}

		/// Test the getter and setter methods
		void testGetSet() {

			// Test method to set all the values
			etk::Matrix2x2 test2;
			test2.setValue(2, 24, -4, 5);
			test(test2 == mMatrix1);

			// Test method to set to zero
			test2.setZero();
			test(test2 == etk::Matrix2x2(0, 0, 0, 0));

			// Test method that returns a column
			vec2 column1 = mMatrix1.getColumn(0);
			vec2 column2 = mMatrix1.getColumn(1);
			test(column1 == vec2(2, -4));
			test(column2 == vec2(24, 5));

			// Test method that returns a row
			vec2 row1 = mMatrix1.getRow(0);
			vec2 row2 = mMatrix1.getRow(1);
			test(row1 == vec2(2, 24));
			test(row2 == vec2(-4, 5));
		}

		/// Test the identity methods
		void testIdentity() {

			etk::Matrix2x2 identity = Matrix2x2::identity();
			etk::Matrix2x2 test1;
			test1.setToIdentity();

			test(identity[0][0] == 1);
			test(identity[0][1] == 0);
			test(identity[1][0] == 0);
			test(identity[1][1] == 1);

			test(test1 == etk::Matrix2x2::identity());
		}

		/// Test the zero method
		void testZero() {

			etk::Matrix2x2 zero = Matrix2x2::zero();

			test(zero[0][0] == 0);
			test(zero[0][1] == 0);
			test(zero[1][0] == 0);
			test(zero[1][1] == 0);
		}

		/// Test others methods
		void testOthersMethods() {

			// Test transpose
			etk::Matrix2x2 transpose = mMatrix1.getTranspose();
			test(transpose == etk::Matrix2x2(2, -4, 24, 5));

			// Test trace
			test(mMatrix1.getTrace() ==7);
			test(etk::Matrix2x2::identity().getTrace() == 2);

			// Test determinant
			etk::Matrix2x2 matrix(-24, 64, 253, -35);
			test(mMatrix1.getDeterminant() == 106);
			test(matrix.getDeterminant() == -15352);
			test(mIdentity.getDeterminant() == 1);

			// Test inverse
			etk::Matrix2x2 matrix2(1, 2, 3, 4);
			etk::Matrix2x2 inverseMatrix = matrix2.getInverse();
			test(approxEqual(inverseMatrix[0][0], float(-2), float(10e-6)));
			test(approxEqual(inverseMatrix[0][1], float(1), float(10e-6)));
			test(approxEqual(inverseMatrix[1][0], float(1.5), float(10e-6)));
			test(approxEqual(inverseMatrix[1][1], float(-0.5), float(10e-6)));
			etk::Matrix2x2 inverseMatrix1 = mMatrix1.getInverse();
			test(approxEqual(inverseMatrix1[0][0], float(0.047169811), float(10e-6)));
			test(approxEqual(inverseMatrix1[0][1], float(-0.226415094), float(10e-6)));
			test(approxEqual(inverseMatrix1[1][0], float(0.037735849), float(10e-6)));
			test(approxEqual(inverseMatrix1[1][1], float(0.018867925), float(10e-6)));

			// Test absolute matrix
			etk::Matrix2x2 matrix3(-2, -3, -4, -5);
			test(matrix.getAbsolute() == etk::Matrix2x2(24, 64, 253, 35));
			etk::Matrix2x2 absoluteMatrix = matrix3.getAbsolute();
			test(absoluteMatrix == etk::Matrix2x2(2, 3, 4, 5));
		}

		/// Test the operators
		void testOperators() {

			// Test addition
			etk::Matrix2x2 matrix1(2, 3, 4, 5);
			etk::Matrix2x2 matrix2(-2, 3, -5, 10);
			etk::Matrix2x2 addition1 = matrix1 + matrix2;
			etk::Matrix2x2 addition2(matrix1);
			addition2 += matrix2;
			test(addition1 == etk::Matrix2x2(0, 6, -1, 15));
			test(addition2 == etk::Matrix2x2(0, 6, -1, 15));

			// Test substraction
			etk::Matrix2x2 substraction1 = matrix1 - matrix2;
			etk::Matrix2x2 substraction2(matrix1);
			substraction2 -= matrix2;
			test(substraction1 == etk::Matrix2x2(4, 0, 9, -5));
			test(substraction2 == etk::Matrix2x2(4, 0, 9, -5));

			// Test negative operator
			etk::Matrix2x2 negative = -matrix1;
			test(negative == etk::Matrix2x2(-2, -3, -4, -5));

			// Test multiplication with a number
			etk::Matrix2x2 multiplication1 = 3 * matrix1;
			etk::Matrix2x2 multiplication2 = matrix1 * 3;
			etk::Matrix2x2 multiplication3(matrix1);
			multiplication3 *= 3;
			test(multiplication1 == etk::Matrix2x2(6, 9, 12, 15));
			test(multiplication2 == etk::Matrix2x2(6, 9, 12, 15));
			test(multiplication3 == etk::Matrix2x2(6, 9, 12, 15));

			// Test multiplication with a matrix
			etk::Matrix2x2 multiplication4 = matrix1 * matrix2;
			etk::Matrix2x2 multiplication5 = matrix2 * matrix1;
			test(multiplication4 == etk::Matrix2x2(-19, 36, -33, 62));
			test(multiplication5 == etk::Matrix2x2(8, 9, 30, 35));

			// Test multiplication with a vector
			vec2 vector1(3, -32);
			vec2 vector2(-31, -422);
			vec2 test1 = matrix1 * vector1;
			vec2 test2 = matrix2 * vector2;
			test(test1 == vec2(-90, -148));
			test(test2 == vec2(-1204, -4065));

			// Test equality operators
			test(etk::Matrix2x2(34, 38, 43, 64) ==
				 etk::Matrix2x2(34, 38, 43, 64));
			test(etk::Matrix2x2(34, 64, 43, 7) !=
				 etk::Matrix2x2(34, 38, 43, 64));

			// Test operator to read a value
			test(mMatrix1[0][0] == 2);
			test(mMatrix1[0][1] == 24);
			test(mMatrix1[1][0] == -4);
			test(mMatrix1[1][1] == 5);

			// Test operator to set a value
			etk::Matrix2x2 test3;
			test3[0][0] = 2;
			test3[0][1] = 24;
			test3[1][0] = -4;
			test3[1][1] = 5;
			test(test3 == mMatrix1);
		}

 };

}

#endif
