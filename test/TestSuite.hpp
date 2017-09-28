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

#ifndef TEST_SUITE_H
#define TEST_SUITE_H

// Libraries
#include <test/Test.hpp>
#include <etk/Vector.hpp>

/// Reactphysics3D namespace
namespace ephysics {

// Class TestSuite
/**
 * This class represents a test suite that can
 * contains multiple unit tests. You can also add a test suite inside
 * another test suite (all the tests of the first test suite will be added
 * to the second one).
 */
class TestSuite {

	private :

		// ---------- Attributes ---------- //

		/// Name of the test suite
		etk::String m_name;

		/// Output stream
		etk::Stream* mOutputStream;

		/// All the tests of the test suite
		etk::Vector<Test*> mTests;

		// ---------- Methods ---------- //

		/// Reset the test suite
		void reset();

		/// Private copy-constructor
		TestSuite(const TestSuite& testSuite);

		/// Private assigmnent operator
		TestSuite& operator=(const TestSuite testSuite);

  public :

		// ---------- Methods ---------- //

		/// Constructor
		TestSuite(const etk::String& name, etk::Stream* outputStream = &std::cout);

		/// Return the name of the test suite
		etk::String getName() const;

		/// Return the number of passed tests
		long getNbPassedTests() const;

		/// Return the number of failed tests
		long getNbFailedTests() const;

		/// Return the output stream
		const etk::Stream* getOutputStream() const;

		/// Set the output stream
		void setOutputStream(etk::Stream* outputStream);

		/// Add a unit test in the test suite
		void addTest(Test* test);

		/// Add a test suite to the current test suite
		void addTestSuite(const TestSuite& testSuite);

		/// Launch the tests of the test suite
		void run();

		/// Display the tests report and return the number of failed tests
		long report() const;

		// Delete all the tests
		void clear();

};

// Return the name of the test suite
inline etk::String TestSuite::getName() const {
	return m_name;
}

// Return the output stream
inline const etk::Stream* TestSuite::getOutputStream() const {
	return mOutputStream;
}

// Set the output stream
inline void TestSuite::setOutputStream(etk::Stream* outputStream) {
	mOutputStream = outputStream;
}

}

#endif
