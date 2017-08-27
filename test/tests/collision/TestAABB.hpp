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

#ifndef TEST_AABB_H
#define TEST_AABB_H

// Libraries
#include <ephysics/ephysics.hpp>

/// Reactphysics3D namespace
namespace ephysics {


// Class TestAABB
/**
 * Unit test for the AABB class.
 */
class TestAABB : public Test {

	private :

		// ---------- Atributes ---------- //

		AABB m_AABB1;
		AABB m_AABB2;
		AABB m_AABB3;
		AABB m_AABB4;

	public :

		// ---------- Methods ---------- //

		/// Constructor
		TestAABB(const etk::String& name) : Test(name) {

			m_AABB1.setMin(vec3(-10, -10, -10));
			m_AABB1.setMax(vec3(10, 10, 10));

			// AABB2 int32_tersect with AABB1
			m_AABB2.setMin(vec3(-5, 4, -30));
			m_AABB2.setMax(vec3(-2, 20, 30));

			// AABB3 contains AABB1
			m_AABB3.setMin(vec3(-25, -25, -25));
			m_AABB3.setMax(vec3(25, 25, 25));

			// AABB4 does not collide with AABB1
			m_AABB4.setMin(vec3(-40, -40, -40));
			m_AABB4.setMax(vec3(-15, -25, -12));
		}

		/// Destructor
		~TestAABB() {

		}

		/// Run the tests
		void run() {
			testBasicMethods();
			testMergeMethods();
			testIntersection();
		}

		void testBasicMethods() {

			// -------- Test constructors -------- //
			AABB aabb1;
			AABB aabb2(vec3(-3, -5, -8), vec3(65, -1, 56));
			vec3 trianglePoints[] = {
				vec3(-5, 7, 23), vec3(45, -34, -73), vec3(-12, 98, 76)
			};
			AABB aabb3 = AABB::createAABBForTriangle(trianglePoints);

			test(aabb1.getMin().x() == 0);
			test(aabb1.getMin().y() == 0);
			test(aabb1.getMin().z() == 0);
			test(aabb1.getMax().x() == 0);
			test(aabb1.getMax().y() == 0);
			test(aabb1.getMax().z() == 0);

			test(aabb2.getMin().x() == -3);
			test(aabb2.getMin().y() == -5);
			test(aabb2.getMin().z() == -8);
			test(aabb2.getMax().x() == 65);
			test(aabb2.getMax().y() == -1);
			test(aabb2.getMax().z() == 56);

			test(aabb3.getMin().x() == -12);
			test(aabb3.getMin().y() == -34);
			test(aabb3.getMin().z() == -73);
			test(aabb3.getMax().x() == 45);
			test(aabb3.getMax().y() == 98);
			test(aabb3.getMax().z() == 76);

			// -------- Test inflate() -------- //
			AABB aabbInflate(vec3(-3, 4, 8), vec3(-1, 6, 32));
			aabbInflate.inflate(1, 2, 3);
			test(approxEqual(aabbInflate.getMin().x(), -4, 0.00001));
			test(approxEqual(aabbInflate.getMin().y(), 2, 0.00001));
			test(approxEqual(aabbInflate.getMin().z(), 5, 0.00001));
			test(approxEqual(aabbInflate.getMax().x(), 0, 0.00001));
			test(approxEqual(aabbInflate.getMax().y(), 8, 0.00001));
			test(approxEqual(aabbInflate.getMax().z(), 35, 0.00001));

			// -------- Test getExtent() --------- //

			test(approxEqual(m_AABB1.getExtent().x(), 20));
			test(approxEqual(m_AABB1.getExtent().y(), 20));
			test(approxEqual(m_AABB1.getExtent().z(), 20));

			test(approxEqual(m_AABB2.getExtent().x(), 3));
			test(approxEqual(m_AABB2.getExtent().y(), 16));
			test(approxEqual(m_AABB2.getExtent().z(), 60));

			test(approxEqual(m_AABB3.getExtent().x(), 50));
			test(approxEqual(m_AABB3.getExtent().y(), 50));
			test(approxEqual(m_AABB3.getExtent().z(), 50));

			// -------- Test getCenter() -------- //

			test(m_AABB1.getCenter().x() == 0);
			test(m_AABB1.getCenter().y() == 0);
			test(m_AABB1.getCenter().z() == 0);

			test(approxEqual(m_AABB2.getCenter().x(), -3.5));
			test(approxEqual(m_AABB2.getCenter().y(), 12));
			test(approxEqual(m_AABB2.getCenter().z(), 0));

			// -------- Test setMin(), setMax(), getMin(), getMax() -------- //

			AABB aabb5;
			aabb5.setMin(vec3(-12, 34, 6));
			aabb5.setMax(vec3(-3, 56, 20));

			test(aabb5.getMin().x() == -12);
			test(aabb5.getMin().y() == 34);
			test(aabb5.getMin().z() == 6);
			test(aabb5.getMax().x() == -3);
			test(aabb5.getMax().y() == 56);
			test(aabb5.getMax().z() == 20);

			// -------- Test assignment operator -------- //

			AABB aabb6;
			aabb6 = aabb2;

			test(aabb6.getMin().x() == -3);
			test(aabb6.getMin().y() == -5);
			test(aabb6.getMin().z() == -8);
			test(aabb6.getMax().x() == 65);
			test(aabb6.getMax().y() == -1);
			test(aabb6.getMax().z() == 56);

			// -------- Test getVolume() -------- //

			test(approxEqual(m_AABB1.getVolume(), 8000));
			test(approxEqual(m_AABB2.getVolume(), 2880));
		}

		void testMergeMethods() {

			AABB aabb1(vec3(-45, 7, -2), vec3(23, 8, 1));
			AABB aabb2(vec3(-15, 6, 23), vec3(-5, 9, 45));

			// -------- Test mergeTwoAABBs() -------- //

			AABB aabb3;
			aabb3.mergeTwoAABBs(aabb1, m_AABB1);

			test(aabb3.getMin().x() == -45);
			test(aabb3.getMin().y() == -10);
			test(aabb3.getMin().z() == -10);
			test(aabb3.getMax().x() == 23);
			test(aabb3.getMax().y() == 10);
			test(aabb3.getMax().z() == 10);

			AABB aabb4;
			aabb4.mergeTwoAABBs(aabb1, aabb2);

			test(aabb4.getMin().x() == -45);
			test(aabb4.getMin().y() == 6);
			test(aabb4.getMin().z() == -2);
			test(aabb4.getMax().x() == 23);
			test(aabb4.getMax().y() == 9);
			test(aabb4.getMax().z() == 45);

			// -------- Test mergeWithAABB() -------- //

			aabb1.mergeWithAABB(m_AABB1);

			test(aabb1.getMin().x() == -45);
			test(aabb1.getMin().y() == -10);
			test(aabb1.getMin().z() == -10);
			test(aabb1.getMax().x() == 23);
			test(aabb1.getMax().y() == 10);
			test(aabb1.getMax().z() == 10);

			aabb2.mergeWithAABB(m_AABB1);

			test(aabb2.getMin().x() == -15);
			test(aabb2.getMin().y() == -10);
			test(aabb2.getMin().z() == -10);
			test(aabb2.getMax().x() == 10);
			test(aabb2.getMax().y() == 10);
			test(aabb2.getMax().z() == 45);
		}

		void testIntersection() {

			// -------- Test contains(AABB) -------- //
			test(!m_AABB1.contains(m_AABB2));
			test(m_AABB3.contains(m_AABB1));
			test(!m_AABB1.contains(m_AABB3));
			test(!m_AABB1.contains(m_AABB4));
			test(!m_AABB4.contains(m_AABB1));

			// -------- Test contains(vec3) -------- //
			test(m_AABB1.contains(vec3(0, 0, 0)));
			test(m_AABB1.contains(vec3(-5, 6, 9)));
			test(m_AABB1.contains(vec3(-9, -4, -9)));
			test(m_AABB1.contains(vec3(9, 4, 7)));
			test(!m_AABB1.contains(vec3(-11, -4, -9)));
			test(!m_AABB1.contains(vec3(1, 12, -9)));
			test(!m_AABB1.contains(vec3(1, 8, -13)));
			test(!m_AABB1.contains(vec3(-14, 82, -13)));

			// -------- Test testCollision() -------- //
			test(m_AABB1.testCollision(m_AABB2));
			test(m_AABB2.testCollision(m_AABB1));
			test(m_AABB1.testCollision(m_AABB3));
			test(m_AABB3.testCollision(m_AABB1));
			test(!m_AABB1.testCollision(m_AABB4));
			test(!m_AABB4.testCollision(m_AABB1));

			// -------- Test testCollisionTriangleAABB() -------- //

			AABB aabb(vec3(100, 100, 100), vec3(200, 200, 200));
			vec3 trianglePoints[] = {
				vec3(-2, 4, 6), vec3(20, -34, -73), vec3(-12, 98, 76)
			};
			test(m_AABB1.testCollisionTriangleAABB(trianglePoints));
			test(!aabb.testCollisionTriangleAABB(trianglePoints));

			// -------- Test testRayIntersect() -------- //

			Ray ray1(vec3(-20, 4, -7), vec3(20, 4, -7));
			Ray ray2(vec3(-20, 11, -7), vec3(20, 11, -7));
			Ray ray3(vec3(0, 15, 0), vec3(0, -15, 0));
			Ray ray4(vec3(0, -15, 0), vec3(0, 15, 0));
			Ray ray5(vec3(-3, 4, 8), vec3(-7, 9, 4));
			Ray ray6(vec3(-4, 6, -100), vec3(-4, 6, -9));
			Ray ray7(vec3(-4, 6, -100), vec3(-4, 6, -11), 0.6);
			Ray ray8(vec3(-403, -432, -100), vec3(134, 643, 23));

			test(m_AABB1.testRayIntersect(ray1));
			test(!m_AABB1.testRayIntersect(ray2));
			test(m_AABB1.testRayIntersect(ray3));
			test(m_AABB1.testRayIntersect(ray4));
			test(m_AABB1.testRayIntersect(ray5));
			test(m_AABB1.testRayIntersect(ray6));
			test(!m_AABB1.testRayIntersect(ray7));
			test(!m_AABB1.testRayIntersect(ray8));
		}
 };

}

#endif
