/** @file
 * @author Edouard DUPIN
 * @copyright 2017, Edouard DUPIN, all right reserved
 * @license MPL v2.0 (see license file)
 */

#include <etest/etest.hpp>
#include <ephysics/ephysics.hpp>
/*
	ephysics::AABB m_AABB1(vec3(-10, -10, -10), vec3(10, 10, 10));
	ephysics::AABB m_AABB2(vec3(-5, 4, -30), vec3(-2, 20, 30));
	ephysics::AABB m_AABB3(vec3(-25, -25, -25), vec3(25, 25, 25));
	ephysics::AABB m_AABB4(vec3(-40, -40, -40), vec3(-15, -25, -12));
*/

TEST(TestAABB, methode_constructors) {
	ephysics::AABB aabb1;
	ephysics::AABB aabb2(vec3(-3, -5, -8), vec3(65, -1, 56));
	vec3 trianglePoints[] = {
		vec3(-5, 7, 23), vec3(45, -34, -73), vec3(-12, 98, 76)
	};
	ephysics::AABB aabb3 = ephysics::AABB::createAABBForTriangle(trianglePoints);
	
	EXPECT_EQ(aabb1.getMin().x(), 0);
	EXPECT_EQ(aabb1.getMin().y(), 0);
	EXPECT_EQ(aabb1.getMin().z(), 0);
	EXPECT_EQ(aabb1.getMax().x(), 0);
	EXPECT_EQ(aabb1.getMax().y(), 0);
	EXPECT_EQ(aabb1.getMax().z(), 0);
	
	EXPECT_EQ(aabb2.getMin().x(), -3);
	EXPECT_EQ(aabb2.getMin().y(), -5);
	EXPECT_EQ(aabb2.getMin().z(), -8);
	EXPECT_EQ(aabb2.getMax().x(), 65);
	EXPECT_EQ(aabb2.getMax().y(), -1);
	EXPECT_EQ(aabb2.getMax().z(), 56);
	
	EXPECT_EQ(aabb3.getMin().x(), -12);
	EXPECT_EQ(aabb3.getMin().y(), -34);
	EXPECT_EQ(aabb3.getMin().z(), -73);
	EXPECT_EQ(aabb3.getMax().x(), 45);
	EXPECT_EQ(aabb3.getMax().y(), 98);
	EXPECT_EQ(aabb3.getMax().z(), 76);
}

TEST(TestAABB, methode_inflate) {
	ephysics::AABB aabbInflate(vec3(-3, 4, 8), vec3(-1, 6, 32));
	aabbInflate.inflate(1, 2, 3);
	EXPECT_FLOAT_EQ(aabbInflate.getMin().x(), -4);
	EXPECT_FLOAT_EQ(aabbInflate.getMin().y(), 2);
	EXPECT_FLOAT_EQ(aabbInflate.getMin().z(), 5);
	EXPECT_FLOAT_EQ(aabbInflate.getMax().x(), 0);
	EXPECT_FLOAT_EQ(aabbInflate.getMax().y(), 8);
	EXPECT_FLOAT_EQ(aabbInflate.getMax().z(), 35);
}

TEST(TestAABB, methode_getExtent) {
	ephysics::AABB m_AABB1(vec3(-10, -10, -10), vec3(10, 10, 10));
	ephysics::AABB m_AABB2(vec3(-5, 4, -30), vec3(-2, 20, 30));
	ephysics::AABB m_AABB3(vec3(-25, -25, -25), vec3(25, 25, 25));
	EXPECT_FLOAT_EQ(m_AABB1.getExtent().x(), 20);
	EXPECT_FLOAT_EQ(m_AABB1.getExtent().y(), 20);
	EXPECT_FLOAT_EQ(m_AABB1.getExtent().z(), 20);

	EXPECT_FLOAT_EQ(m_AABB2.getExtent().x(), 3);
	EXPECT_FLOAT_EQ(m_AABB2.getExtent().y(), 16);
	EXPECT_FLOAT_EQ(m_AABB2.getExtent().z(), 60);

	EXPECT_FLOAT_EQ(m_AABB3.getExtent().x(), 50);
	EXPECT_FLOAT_EQ(m_AABB3.getExtent().y(), 50);
	EXPECT_FLOAT_EQ(m_AABB3.getExtent().z(), 50);
}

TEST(TestAABB, methode_getCenter) {
	ephysics::AABB m_AABB1(vec3(-10, -10, -10), vec3(10, 10, 10));
	ephysics::AABB m_AABB2(vec3(-5, 4, -30), vec3(-2, 20, 30));
	EXPECT_EQ(m_AABB1.getCenter().x(), 0);
	EXPECT_EQ(m_AABB1.getCenter().y(), 0);
	EXPECT_EQ(m_AABB1.getCenter().z(), 0);
	EXPECT_FLOAT_EQ(m_AABB2.getCenter().x(), -3.5);
	EXPECT_FLOAT_EQ(m_AABB2.getCenter().y(), 12);
	EXPECT_FLOAT_EQ(m_AABB2.getCenter().z(), 0);
}

TEST(TestAABB, methode_minMax) {
	ephysics::AABB aabb5;
	aabb5.setMin(vec3(-12, 34, 6));
	aabb5.setMax(vec3(-3, 56, 20));
	EXPECT_EQ(aabb5.getMin().x(), -12);
	EXPECT_EQ(aabb5.getMin().y(), 34);
	EXPECT_EQ(aabb5.getMin().z(), 6);
	EXPECT_EQ(aabb5.getMax().x(), -3);
	EXPECT_EQ(aabb5.getMax().y(), 56);
	EXPECT_EQ(aabb5.getMax().z(), 20);
}

TEST(TestAABB, methode_asignement) {
	ephysics::AABB aabb2(vec3(-3, -5, -8), vec3(65, -1, 56));
	ephysics::AABB aabb6;
	aabb6 = aabb2;
	EXPECT_EQ(aabb6.getMin().x(), -3);
	EXPECT_EQ(aabb6.getMin().y(), -5);
	EXPECT_EQ(aabb6.getMin().z(), -8);
	EXPECT_EQ(aabb6.getMax().x(), 65);
	EXPECT_EQ(aabb6.getMax().y(), -1);
	EXPECT_EQ(aabb6.getMax().z(), 56);

}

TEST(TestAABB, methode_getVolume) {
	ephysics::AABB m_AABB1(vec3(-10, -10, -10), vec3(10, 10, 10));
	ephysics::AABB m_AABB2(vec3(-5, 4, -30), vec3(-2, 20, 30));
	EXPECT_FLOAT_EQ(m_AABB1.getVolume(), 8000);
	EXPECT_FLOAT_EQ(m_AABB2.getVolume(), 2880);
}

TEST(TestAABB, merge_mergeTwoAABBs) {
	ephysics::AABB m_AABB1(vec3(-10, -10, -10), vec3(10, 10, 10));
	ephysics::AABB m_AABB4(vec3(-40, -40, -40), vec3(-15, -25, -12));
	ephysics::AABB aabb1(vec3(-45, 7, -2), vec3(23, 8, 1));
	ephysics::AABB aabb2(vec3(-15, 6, 23), vec3(-5, 9, 45));
	
	ephysics::AABB aabb3;
	aabb3.mergeTwoAABBs(aabb1, m_AABB1);
	EXPECT_EQ(aabb3.getMin().x(), -45);
	EXPECT_EQ(aabb3.getMin().y(), -10);
	EXPECT_EQ(aabb3.getMin().z(), -10);
	EXPECT_EQ(aabb3.getMax().x(), 23);
	EXPECT_EQ(aabb3.getMax().y(), 10);
	EXPECT_EQ(aabb3.getMax().z(), 10);
	ephysics::AABB aabb4;
	aabb4.mergeTwoAABBs(aabb1, aabb2);
	EXPECT_EQ(aabb4.getMin().x(), -45);
	EXPECT_EQ(aabb4.getMin().y(), 6);
	EXPECT_EQ(aabb4.getMin().z(), -2);
	EXPECT_EQ(aabb4.getMax().x(), 23);
	EXPECT_EQ(aabb4.getMax().y(), 9);
	EXPECT_EQ(aabb4.getMax().z(), 45);
}

TEST(TestAABB, merge_mergeWithAABB) {
	ephysics::AABB m_AABB1(vec3(-10, -10, -10), vec3(10, 10, 10));
	ephysics::AABB aabb1(vec3(-45, 7, -2), vec3(23, 8, 1));
	ephysics::AABB aabb2(vec3(-15, 6, 23), vec3(-5, 9, 45));
	aabb1.mergeWithAABB(m_AABB1);
	EXPECT_EQ(aabb1.getMin().x(), -45);
	EXPECT_EQ(aabb1.getMin().y(), -10);
	EXPECT_EQ(aabb1.getMin().z(), -10);
	EXPECT_EQ(aabb1.getMax().x(), 23);
	EXPECT_EQ(aabb1.getMax().y(), 10);
	EXPECT_EQ(aabb1.getMax().z(), 10);
	aabb2.mergeWithAABB(m_AABB1);
	EXPECT_EQ(aabb2.getMin().x(), -15);
	EXPECT_EQ(aabb2.getMin().y(), -10);
	EXPECT_EQ(aabb2.getMin().z(), -10);
	EXPECT_EQ(aabb2.getMax().x(), 10);
	EXPECT_EQ(aabb2.getMax().y(), 10);
	EXPECT_EQ(aabb2.getMax().z(), 45);
}

TEST(TestAABB, intersection_containsAABB) {
	ephysics::AABB m_AABB1(vec3(-10, -10, -10), vec3(10, 10, 10));
	ephysics::AABB m_AABB2(vec3(-5, 4, -30), vec3(-2, 20, 30));
	ephysics::AABB m_AABB3(vec3(-25, -25, -25), vec3(25, 25, 25));
	ephysics::AABB m_AABB4(vec3(-40, -40, -40), vec3(-15, -25, -12));
	EXPECT_EQ(m_AABB1.contains(m_AABB2), false);
	EXPECT_EQ(m_AABB3.contains(m_AABB1), true);
	EXPECT_EQ(m_AABB1.contains(m_AABB3), false);
	EXPECT_EQ(m_AABB1.contains(m_AABB4), false);
	EXPECT_EQ(m_AABB4.contains(m_AABB1), false);
}

TEST(TestAABB, intersection_containsVec3) {
	ephysics::AABB m_AABB1(vec3(-10, -10, -10), vec3(10, 10, 10));
	EXPECT_EQ(m_AABB1.contains(vec3(0, 0, 0)), true);
	EXPECT_EQ(m_AABB1.contains(vec3(-5, 6, 9)), true);
	EXPECT_EQ(m_AABB1.contains(vec3(-9, -4, -9)), true);
	EXPECT_EQ(m_AABB1.contains(vec3(9, 4, 7)), true);
	EXPECT_EQ(m_AABB1.contains(vec3(-11, -4, -9)), false);
	EXPECT_EQ(m_AABB1.contains(vec3(1, 12, -9)), false);
	EXPECT_EQ(m_AABB1.contains(vec3(1, 8, -13)), false);
	EXPECT_EQ(m_AABB1.contains(vec3(-14, 82, -13)), false);
}

TEST(TestAABB, intersection_testCollision) {
	ephysics::AABB m_AABB1(vec3(-10, -10, -10), vec3(10, 10, 10));
	ephysics::AABB m_AABB2(vec3(-5, 4, -30), vec3(-2, 20, 30));
	ephysics::AABB m_AABB3(vec3(-25, -25, -25), vec3(25, 25, 25));
	ephysics::AABB m_AABB4(vec3(-40, -40, -40), vec3(-15, -25, -12));
	EXPECT_EQ(m_AABB1.testCollision(m_AABB2), true);
	EXPECT_EQ(m_AABB2.testCollision(m_AABB1), true);
	EXPECT_EQ(m_AABB1.testCollision(m_AABB3), true);
	EXPECT_EQ(m_AABB3.testCollision(m_AABB1), true);
	EXPECT_EQ(m_AABB1.testCollision(m_AABB4), false);
	EXPECT_EQ(m_AABB4.testCollision(m_AABB1), false);
}

TEST(TestAABB, intersection_testCollisionTriangleAABB) {
	ephysics::AABB m_AABB1(vec3(-10, -10, -10), vec3(10, 10, 10));
	ephysics::AABB aabb(vec3(100, 100, 100), vec3(200, 200, 200));
	vec3 trianglePoints[] = {
		vec3(-2, 4, 6), vec3(20, -34, -73), vec3(-12, 98, 76)
	};
	EXPECT_EQ(m_AABB1.testCollisionTriangleAABB(trianglePoints), true);
	EXPECT_EQ(aabb.testCollisionTriangleAABB(trianglePoints), false);
}

TEST(TestAABB, intersection_testRayIntersect) {
	ephysics::AABB m_AABB1(vec3(-10, -10, -10), vec3(10, 10, 10));
	ephysics::Ray ray1(vec3(-20, 4, -7), vec3(20, 4, -7));
	ephysics::Ray ray2(vec3(-20, 11, -7), vec3(20, 11, -7));
	ephysics::Ray ray3(vec3(0, 15, 0), vec3(0, -15, 0));
	ephysics::Ray ray4(vec3(0, -15, 0), vec3(0, 15, 0));
	ephysics::Ray ray5(vec3(-3, 4, 8), vec3(-7, 9, 4));
	ephysics::Ray ray6(vec3(-4, 6, -100), vec3(-4, 6, -9));
	ephysics::Ray ray7(vec3(-4, 6, -100), vec3(-4, 6, -11), 0.6);
	ephysics::Ray ray8(vec3(-403, -432, -100), vec3(134, 643, 23));
	EXPECT_EQ(m_AABB1.testRayIntersect(ray1), true);
	EXPECT_EQ(m_AABB1.testRayIntersect(ray2), false);
	EXPECT_EQ(m_AABB1.testRayIntersect(ray3), true);
	EXPECT_EQ(m_AABB1.testRayIntersect(ray4), true);
	EXPECT_EQ(m_AABB1.testRayIntersect(ray5), true);
	EXPECT_EQ(m_AABB1.testRayIntersect(ray6), true);
	EXPECT_EQ(m_AABB1.testRayIntersect(ray7), false);
	EXPECT_EQ(m_AABB1.testRayIntersect(ray8), false);
}

