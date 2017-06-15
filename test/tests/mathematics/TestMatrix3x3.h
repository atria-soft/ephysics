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

#ifndef TEST_MATRIX3X3_H
#define TEST_MATRIX3X3_H

// Libraries
#include <test/Test.h>
#include <etk/math/Matrix3x3.hpp>

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class Testetk::Matrix3x3
/**
 * Unit test for the etk::Matrix3x3 class
 */
class Testetk::Matrix3x3 : public Test {

	private :

		// ---------- Atributes ---------- //

		/// Identity transform
		etk::Matrix3x3 mIdentity;

		/// First example matrix
		etk::Matrix3x3 mMatrix1;

	public :

		// ---------- Methods ---------- //

		/// Constructor
		Testetk::Matrix3x3(const std::string& name)
			: Test(name), mIdentity(etk::Matrix3x3::identity()),
			  mMatrix1(2, 24, 4, 5, -6, 234, -15, 11, 66) {


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

			etk::Matrix3x3 test1(5.0);
			etk::Matrix3x3 test2(2, 3, 4, 5, 6, 7, 8, 9, 10);
			etk::Matrix3x3 test3(mMatrix1);
			test(test1[0][0] == 5);
			test(test1[0][1] == 5);
			test(test1[0][2] == 5);
			test(test1[1][0] == 5);
			test(test1[1][1] == 5);
			test(test1[1][2] == 5);
			test(test1[2][0] == 5);
			test(test1[2][1] == 5);
			test(test1[2][2] == 5);

			test(test2[0][0] == 2);
			test(test2[0][1] == 3);
			test(test2[0][2] == 4);
			test(test2[1][0] == 5);
			test(test2[1][1] == 6);
			test(test2[1][2] == 7);
			test(test2[2][0] == 8);
			test(test2[2][1] == 9);
			test(test2[2][2] == 10);

			test(test3 == mMatrix1);
		}

		/// Test the getter and setter methods
		void testGetSet() {

			// Test method to set all the values
			etk::Matrix3x3 test2;
			test2.setValue(2, 24, 4, 5, -6, 234, -15, 11, 66);
			test(test2 == mMatrix1);

			// Test method to set to zero
			test2.setZero();
			test(test2 == etk::Matrix3x3(0, 0, 0, 0, 0, 0, 0, 0, 0));

			// Test method that returns a column
			vec3 column1 = mMatrix1.getColumn(0);
			vec3 column2 = mMatrix1.getColumn(1);
			vec3 column3 = mMatrix1.getColumn(2);
			test(column1 == vec3(2, 5, -15));
			test(column2 == vec3(24, -6, 11));
			test(column3 == vec3(4, 234, 66));

			// Test method that returns a row
			vec3 row1 = mMatrix1.getRow(0);
			vec3 row2 = mMatrix1.getRow(1);
			vec3 row3 = mMatrix1.getRow(2);
			test(row1 == vec3(2, 24, 4));
			test(row2 == vec3(5, -6, 234));
			test(row3 == vec3(-15, 11, 66));
		}

		/// Test the identity methods
		void testIdentity() {

			etk::Matrix3x3 identity = Matrix3x3::identity();
			etk::Matrix3x3 test1;
			test1.setToIdentity();

			test(identity[0][0] == 1);
			test(identity[0][1] == 0);
			test(identity[0][2] == 0);
			test(identity[1][0] == 0);
			test(identity[1][1] == 1);
			test(identity[1][2] == 0);
			test(identity[2][0] == 0);
			test(identity[2][1] == 0);
			test(identity[2][2] == 1);

			test(test1 == etk::Matrix3x3::identity());
		}

		/// Test the zero method
		void testZero() {

			etk::Matrix3x3 zero = Matrix3x3::zero();

			test(zero[0][0] == 0);
			test(zero[0][1] == 0);
			test(zero[0][2] == 0);
			test(zero[1][0] == 0);
			test(zero[1][1] == 0);
			test(zero[1][2] == 0);
			test(zero[2][0] == 0);
			test(zero[2][1] == 0);
			test(zero[2][2] == 0);
		}

		/// Test others methods
		void testOthersMethods() {

			// Test transpose
			etk::Matrix3x3 transpose = mMatrix1.getTranspose();
			test(transpose == etk::Matrix3x3(2, 5, -15, 24, -6, 11, 4, 234, 66));

			// Test trace
			test(mMatrix1.getTrace() == 62);
			test(etk::Matrix3x3::identity().getTrace() == 3);

			// Test determinant
			etk::Matrix3x3 matrix(-24, 64, 253, -35, 52, 72, 21, -35, -363);
			test(mMatrix1.getDeterminant() == -98240);
			test(matrix.getDeterminant() == -290159);
			test(mIdentity.getDeterminant() == 1);

			// Test inverse
			etk::Matrix3x3 inverseMatrix = matrix.getInverse();
			test(approxEqual(inverseMatrix[0][0], float(0.056369), float(10e-6)));
			test(approxEqual(inverseMatrix[0][1], float(-0.049549), float(10e-6)));
			test(approxEqual(inverseMatrix[0][2], float(0.029460), float(10e-6)));
			test(approxEqual(inverseMatrix[1][0], float(0.038575), float(10e-6)));
			test(approxEqual(inverseMatrix[1][1], float(-0.011714), float(10e-6)));
			test(approxEqual(inverseMatrix[1][2], float(0.024562), float(10e-6)));
			test(approxEqual(inverseMatrix[2][0], float(-0.000458), float(10e-6)));
			test(approxEqual(inverseMatrix[2][1], float(-0.001737), float(10e-6)));
			test(approxEqual(inverseMatrix[2][2], float(-0.003419), float(10e-6)));
			etk::Matrix3x3 inverseMatrix1 = mMatrix1.getInverse();
			test(approxEqual(inverseMatrix1[0][0], float(0.030232), float(10e-6)));
			test(approxEqual(inverseMatrix1[0][1], float(0.015676), float(10e-6)));
			test(approxEqual(inverseMatrix1[0][2], float(-0.057410), float(10e-6)));
			test(approxEqual(inverseMatrix1[1][0], float(0.039088), float(10e-6)));
			test(approxEqual(inverseMatrix1[1][1], float(-0.001954), float(10e-6)));
			test(approxEqual(inverseMatrix1[1][2], float(0.004560), float(10e-6)));
			test(approxEqual(inverseMatrix1[2][0], float(0.000356), float(10e-6)));
			test(approxEqual(inverseMatrix1[2][1], float(0.003888), float(10e-6)));
			test(approxEqual(inverseMatrix1[2][2], float(0.001344), float(10e-6)));

			// Test absolute matrix
			etk::Matrix3x3 matrix2(-2, -3, -4, -5, -6, -7, -8, -9, -10);
			test(matrix.getAbsolute() == etk::Matrix3x3(24, 64, 253, 35, 52, 72, 21, 35, 363));
			etk::Matrix3x3 absoluteMatrix = matrix2.getAbsolute();
			test(absoluteMatrix == etk::Matrix3x3(2, 3, 4, 5, 6, 7, 8, 9, 10));

			// Test method that computes skew-symmetric matrix for cross product
			vec3 vector1(3, -5, 6);
			vec3 vector2(73, 42, 26);
			etk::Matrix3x3 skewMatrix = etk::Matrix3x3::computeSkewSymmetricMatrixForCrossProduct(vector1);
			test(skewMatrix == etk::Matrix3x3(0, -6, -5, 6, 0, -3, 5, 3, 0));
			vec3 crossProduct1 = vector1.cross(vector2);
			vec3 crossProduct2 = skewMatrix * vector2;
			test(crossProduct1 == crossProduct2);
		}

		/// Test the operators
		void testOperators() {

			// Test addition
			etk::Matrix3x3 matrix1(2, 3, 4, 5, 6, 7, 8, 9, 10);
			etk::Matrix3x3 matrix2(-2, 3, -5, 10, 4, 7, 2, 5, 8);
			etk::Matrix3x3 addition1 = matrix1 + matrix2;
			etk::Matrix3x3 addition2(matrix1);
			addition2 += matrix2;
			test(addition1 == etk::Matrix3x3(0, 6, -1, 15, 10, 14, 10, 14, 18));
			test(addition2 == etk::Matrix3x3(0, 6, -1, 15, 10, 14, 10, 14, 18));

			// Test substraction
			etk::Matrix3x3 substraction1 = matrix1 - matrix2;
			etk::Matrix3x3 substraction2(matrix1);
			substraction2 -= matrix2;
			test(substraction1 == etk::Matrix3x3(4, 0, 9, -5, 2, 0, 6, 4, 2));
			test(substraction2 == etk::Matrix3x3(4, 0, 9, -5, 2, 0, 6, 4, 2));

			// Test negative operator
			etk::Matrix3x3 negative = -matrix1;
			test(negative == etk::Matrix3x3(-2, -3, -4, -5, -6, -7, -8, -9, -10));

			// Test multiplication with a number
			etk::Matrix3x3 multiplication1 = 3 * matrix1;
			etk::Matrix3x3 multiplication2 = matrix1 * 3;
			etk::Matrix3x3 multiplication3(matrix1);
			multiplication3 *= 3;
			test(multiplication1 == etk::Matrix3x3(6, 9, 12, 15, 18, 21, 24, 27, 30));
			test(multiplication2 == etk::Matrix3x3(6, 9, 12, 15, 18, 21, 24, 27, 30));
			test(multiplication3 == etk::Matrix3x3(6, 9, 12, 15, 18, 21, 24, 27, 30));

			// Test multiplication with a matrix
			etk::Matrix3x3 multiplication4 = matrix1 * matrix2;
			etk::Matrix3x3 multiplication5 = matrix2 * matrix1;
			test(multiplication4 == etk::Matrix3x3(34, 38, 43, 64, 74, 73, 94, 110, 103));
			test(multiplication5 == etk::Matrix3x3(-29, -33, -37, 96, 117, 138, 93, 108, 123));

			// Test multiplication with a vector
			vec3 vector1(3, -32, 59);
			vec3 vector2(-31, -422, 34);
			vec3 test1 = matrix1 * vector1;
			vec3 test2 = matrix2 * vector2;
			test(test1 == vec3(146, 236, 326));
			test(test2 == vec3(-1374, -1760, -1900));

			// Test equality operators
			test(etk::Matrix3x3(34, 38, 43, 64, 74, 73, 94, 110, 103) ==
				 etk::Matrix3x3(34, 38, 43, 64, 74, 73, 94, 110, 103));
			test(etk::Matrix3x3(34, 64, 43, 7, -1, 73, 94, 110, 103) !=
				 etk::Matrix3x3(34, 38, 43, 64, 74, 73, 94, 110, 103));

			// Test operator to read a value
			test(mMatrix1[0][0] == 2);
			test(mMatrix1[0][1] == 24);
			test(mMatrix1[0][2] == 4);
			test(mMatrix1[1][0] == 5);
			test(mMatrix1[1][1] == -6);
			test(mMatrix1[1][2] == 234);
			test(mMatrix1[2][0] == -15);
			test(mMatrix1[2][1] == 11);
			test(mMatrix1[2][2] == 66);

			// Test operator to set a value
			etk::Matrix3x3 test3;
			test3[0][0] = 2;
			test3[0][1] = 24;
			test3[0][2] = 4;
			test3[1][0] = 5;
			test3[1][1] = -6;
			test3[1][2] = 234;
			test3[2][0] = -15;
			test3[2][1] = 11;
			test3[2][2] = 66;
			test(test3 == mMatrix1);
		}

 };

}

#endif
