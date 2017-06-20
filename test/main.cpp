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

// Libraries
#include <test/TestSuite.hpp>
#include <test/tests/mathematics/TestMathematicsFunctions.hpp>
#include <test/tests/collision/TestPointInside.hpp>
#include <test/tests/collision/TestRaycast.hpp>
#include <test/tests/collision/TestCollisionWorld.hpp>
#include <test/tests/collision/TestAABB.hpp>
#include <test/tests/collision/TestDynamicAABBTree.hpp>

using namespace ephysics;

int32_t main() {

	TestSuite testSuite("ReactPhysics3D Tests");

	// ---------- Mathematics tests ---------- //

	testSuite.addTest(new Testvec2("vec2"));
	testSuite.addTest(new Testvec3("vec3"));
	testSuite.addTest(new Testetk::Transform3D("Transform"));
	testSuite.addTest(new Testetk::Quaternion("Quaternion"));
	testSuite.addTest(new Testetk::Matrix3x3("Matrix3x3"));
	testSuite.addTest(new Testetk::Matrix2x2("Matrix2x2"));
	testSuite.addTest(new TestMathematicsFunctions("Maths Functions"));

	// ---------- Collision Detection tests ---------- //

	testSuite.addTest(new TestAABB("AABB"));
	testSuite.addTest(new TestPointInside("IsPointInside"));
	testSuite.addTest(new TestRaycast("Raycasting"));
	testSuite.addTest(new TestCollisionWorld("CollisionWorld"));
	testSuite.addTest(new TestDynamicAABBTree("DynamicAABBTree"));

	// Run the tests
	testSuite.run();

	// Display the report
	long nbFailedTests = testSuite.report();

	// Clear the tests from the test suite
	testSuite.clear();

	return nbFailedTests;
}
