/** @file
 * @author Edouard DUPIN
 * @copyright 2017, Edouard DUPIN, all right reserved
 * @license MPL v2.0 (see license file)
 */

#include <etest/etest.hpp>
#include <ephysics/ephysics.hpp>
#include <test-debug/debug.hpp>

// Enumeration for categories
enum CollisionCategory {
	CATEGORY_1 = 0x0001,
	CATEGORY_2 = 0x0002,
	CATEGORY_3 = 0x0004
};
class WorldCollisionCallback : public ephysics::CollisionCallback {
	public:
		bool boxCollideWithSphere1;
		bool boxCollideWithCylinder;
		bool sphere1CollideWithCylinder;
		bool sphere1CollideWithSphere2;
		ephysics::CollisionBody* boxBody;
		ephysics::CollisionBody* sphere1Body;
		ephysics::CollisionBody* sphere2Body;
		ephysics::CollisionBody* cylinderBody;
		WorldCollisionCallback() {
			reset();
		}
		void reset() {
			boxCollideWithSphere1 = false;
			boxCollideWithCylinder = false;
			sphere1CollideWithCylinder = false;
			sphere1CollideWithSphere2 = false;
		}
		// This method will be called for contact
		virtual void notifyContact(const ephysics::ContactPointInfo& _contactPointInfo) {
			if (isContactBetweenBodies(boxBody, sphere1Body, _contactPointInfo)) {
				TEST_WARNING("plop 1 boxCollideWithSphere1");
				boxCollideWithSphere1 = true;
			} else if (isContactBetweenBodies(boxBody, cylinderBody, _contactPointInfo)) {
				TEST_WARNING("plop 2 boxCollideWithCylinder");
				boxCollideWithCylinder = true;
			} else if (isContactBetweenBodies(sphere1Body, cylinderBody, _contactPointInfo)) {
				TEST_WARNING("plop 3 sphere1CollideWithCylinder");
				sphere1CollideWithCylinder = true;
			} else if (isContactBetweenBodies(sphere1Body, sphere2Body, _contactPointInfo)) {
				TEST_WARNING("plop 4 sphere1CollideWithSphere2");
				sphere1CollideWithSphere2 = true;
			}
		}
		bool isContactBetweenBodies(const ephysics::CollisionBody* _body1,
		                            const ephysics::CollisionBody* _body2,
		                            const ephysics::ContactPointInfo& _contactPointInfo) {
			return    (    _contactPointInfo.shape1->getBody()->getID() == _body1->getID()
			            && _contactPointInfo.shape2->getBody()->getID() == _body2->getID() )
			       || (    _contactPointInfo.shape2->getBody()->getID() == _body1->getID()
			            && _contactPointInfo.shape1->getBody()->getID() == _body2->getID() );
		}
};

/**
 * Unit test for the CollisionWorld class.
 */
class TestCollisionWorld {
	public:
		// Physics world
		ephysics::CollisionWorld* m_world;
		// Bodies
		ephysics::CollisionBody* m_boxBody;
		ephysics::CollisionBody* m_sphere1Body;
		ephysics::CollisionBody* m_sphere2Body;
		ephysics::CollisionBody* m_cylinderBody;
		// Collision shapes
		ephysics::BoxShape* m_boxShape;
		ephysics::SphereShape* m_sphereShape;
		ephysics::CylinderShape* m_cylinderShape;
		// Proxy shapes
		ephysics::ProxyShape* m_boxProxyShape;
		ephysics::ProxyShape* m_sphere1ProxyShape;
		ephysics::ProxyShape* m_sphere2ProxyShape;
		ephysics::ProxyShape* m_cylinderProxyShape;
		// Collision callback class
		WorldCollisionCallback m_collisionCallback;
	public :
		TestCollisionWorld() {
			// Create the world
			m_world = ETK_NEW(ephysics::CollisionWorld);
			// Create the bodies
			etk::Transform3D boxTransform(vec3(10, 0, 0), etk::Quaternion::identity());
			m_boxBody = m_world->createCollisionBody(boxTransform);
			m_boxShape = ETK_NEW(ephysics::BoxShape, vec3(3,3,3));
			m_boxProxyShape = m_boxBody->addCollisionShape(m_boxShape, etk::Transform3D::identity());
			m_sphereShape = ETK_NEW(ephysics::SphereShape, 3.0);
			etk::Transform3D sphere1Transform(vec3(10,5, 0), etk::Quaternion::identity());
			m_sphere1Body = m_world->createCollisionBody(sphere1Transform);
			m_sphere1ProxyShape = m_sphere1Body->addCollisionShape(m_sphereShape, etk::Transform3D::identity());
			etk::Transform3D sphere2Transform(vec3(30, 10, 10), etk::Quaternion::identity());
			m_sphere2Body = m_world->createCollisionBody(sphere2Transform);
			m_sphere2ProxyShape = m_sphere2Body->addCollisionShape(m_sphereShape, etk::Transform3D::identity());
			etk::Transform3D cylinderTransform(vec3(10, -5, 0), etk::Quaternion::identity());
			m_cylinderBody = m_world->createCollisionBody(cylinderTransform);
			m_cylinderShape = ETK_NEW(ephysics::CylinderShape, 2, 5);
			m_cylinderProxyShape = m_cylinderBody->addCollisionShape(m_cylinderShape, etk::Transform3D::identity());
			// Assign collision categories to proxy shapes
			m_boxProxyShape->setCollisionCategoryBits(CATEGORY_1);
			m_sphere1ProxyShape->setCollisionCategoryBits(CATEGORY_1);
			m_sphere2ProxyShape->setCollisionCategoryBits(CATEGORY_2);
			m_cylinderProxyShape->setCollisionCategoryBits(CATEGORY_3);
			m_collisionCallback.boxBody = m_boxBody;
			m_collisionCallback.sphere1Body = m_sphere1Body;
			m_collisionCallback.sphere2Body = m_sphere2Body;
			m_collisionCallback.cylinderBody = m_cylinderBody;
		}
		~TestCollisionWorld() {
			ETK_DELETE(ephysics::BoxShape, m_boxShape);
			ETK_DELETE(ephysics::SphereShape, m_sphereShape);
			ETK_DELETE(ephysics::CylinderShape, m_cylinderShape);
			ETK_DELETE(ephysics::CollisionWorld, m_world);
		}
};


TEST(TestCollisionWorld, testCollisions_1) {
	TestCollisionWorld tmp;
	tmp.m_collisionCallback.reset();
	tmp.m_world->testCollision(&tmp.m_collisionCallback);
	EXPECT_EQ(tmp.m_collisionCallback.boxCollideWithSphere1, true);
	EXPECT_EQ(tmp.m_collisionCallback.boxCollideWithCylinder, true);
	EXPECT_EQ(tmp.m_collisionCallback.sphere1CollideWithCylinder, false);
	EXPECT_EQ(tmp.m_collisionCallback.sphere1CollideWithSphere2, false);

	EXPECT_EQ(tmp.m_world->testAABBOverlap(tmp.m_boxBody, tmp.m_sphere1Body), true);
	EXPECT_EQ(tmp.m_world->testAABBOverlap(tmp.m_boxBody, tmp.m_cylinderBody), true);
	EXPECT_EQ(tmp.m_world->testAABBOverlap(tmp.m_sphere1Body, tmp.m_cylinderBody), false);
	EXPECT_EQ(tmp.m_world->testAABBOverlap(tmp.m_sphere1Body, tmp.m_sphere2Body), false);

	EXPECT_EQ(tmp.m_world->testAABBOverlap(tmp.m_boxProxyShape, tmp.m_sphere1ProxyShape), true);
	EXPECT_EQ(tmp.m_world->testAABBOverlap(tmp.m_boxProxyShape, tmp.m_cylinderProxyShape), true);
	EXPECT_EQ(tmp.m_world->testAABBOverlap(tmp.m_sphere1ProxyShape, tmp.m_cylinderProxyShape), false);
	EXPECT_EQ(tmp.m_world->testAABBOverlap(tmp.m_sphere1ProxyShape, tmp.m_sphere2ProxyShape), false);

	tmp.m_collisionCallback.reset();
	tmp.m_world->testCollision(tmp.m_cylinderBody, &tmp.m_collisionCallback);
	EXPECT_EQ(tmp.m_collisionCallback.boxCollideWithSphere1, false);
	EXPECT_EQ(tmp.m_collisionCallback.boxCollideWithCylinder, true);
	EXPECT_EQ(tmp.m_collisionCallback.sphere1CollideWithCylinder, false);
	EXPECT_EQ(tmp.m_collisionCallback.sphere1CollideWithSphere2, false);

	tmp.m_collisionCallback.reset();
	tmp.m_world->testCollision(tmp.m_boxBody, tmp.m_sphere1Body, &tmp.m_collisionCallback);
	EXPECT_EQ(tmp.m_collisionCallback.boxCollideWithSphere1, true);
	EXPECT_EQ(tmp.m_collisionCallback.boxCollideWithCylinder, false);
	EXPECT_EQ(tmp.m_collisionCallback.sphere1CollideWithCylinder, false);
	EXPECT_EQ(tmp.m_collisionCallback.sphere1CollideWithSphere2, false);

	tmp.m_collisionCallback.reset();
	tmp.m_world->testCollision(tmp.m_boxBody, tmp.m_cylinderBody, &tmp.m_collisionCallback);
	EXPECT_EQ(tmp.m_collisionCallback.boxCollideWithSphere1, false);
	EXPECT_EQ(tmp.m_collisionCallback.boxCollideWithCylinder, true);
	EXPECT_EQ(tmp.m_collisionCallback.sphere1CollideWithCylinder, false);
	EXPECT_EQ(tmp.m_collisionCallback.sphere1CollideWithSphere2, false);

	tmp.m_collisionCallback.reset();
	tmp.m_world->testCollision(tmp.m_cylinderProxyShape, &tmp.m_collisionCallback);
	EXPECT_EQ(tmp.m_collisionCallback.boxCollideWithSphere1, false);
	EXPECT_EQ(tmp.m_collisionCallback.boxCollideWithCylinder, true);
	EXPECT_EQ(tmp.m_collisionCallback.sphere1CollideWithCylinder, false);
	EXPECT_EQ(tmp.m_collisionCallback.sphere1CollideWithSphere2, false);

	tmp.m_collisionCallback.reset();
	tmp.m_world->testCollision(tmp.m_boxProxyShape, tmp.m_cylinderProxyShape, &tmp.m_collisionCallback);
	EXPECT_EQ(tmp.m_collisionCallback.boxCollideWithSphere1, false);
	EXPECT_EQ(tmp.m_collisionCallback.boxCollideWithCylinder, true);
	EXPECT_EQ(tmp.m_collisionCallback.sphere1CollideWithCylinder, false);
	EXPECT_EQ(tmp.m_collisionCallback.sphere1CollideWithSphere2, false);

	// Move sphere 1 to collide with sphere 2
	tmp.m_sphere1Body->setTransform(etk::Transform3D(vec3(30, 15, 10), etk::Quaternion::identity()));

	tmp.m_collisionCallback.reset();
	tmp.m_world->testCollision(&tmp.m_collisionCallback);
	EXPECT_EQ(tmp.m_collisionCallback.boxCollideWithSphere1, false);
	EXPECT_EQ(tmp.m_collisionCallback.boxCollideWithCylinder, true);
	EXPECT_EQ(tmp.m_collisionCallback.sphere1CollideWithCylinder, false);
	EXPECT_EQ(tmp.m_collisionCallback.sphere1CollideWithSphere2, true);

	tmp.m_collisionCallback.reset();
	tmp.m_world->testCollision(tmp.m_boxBody, tmp.m_sphere1Body, &tmp.m_collisionCallback);
	EXPECT_EQ(tmp.m_collisionCallback.boxCollideWithSphere1, false);
	EXPECT_EQ(tmp.m_collisionCallback.boxCollideWithCylinder, false);
	EXPECT_EQ(tmp.m_collisionCallback.sphere1CollideWithCylinder, false);
	EXPECT_EQ(tmp.m_collisionCallback.sphere1CollideWithSphere2, false);

	tmp.m_collisionCallback.reset();
	tmp.m_world->testCollision(tmp.m_boxBody, tmp.m_cylinderBody, &tmp.m_collisionCallback);
	EXPECT_EQ(tmp.m_collisionCallback.boxCollideWithSphere1, false);
	EXPECT_EQ(tmp.m_collisionCallback.boxCollideWithCylinder, true);
	EXPECT_EQ(tmp.m_collisionCallback.sphere1CollideWithCylinder, false);
	EXPECT_EQ(tmp.m_collisionCallback.sphere1CollideWithSphere2, false);

	// Move sphere 1 to collide with box
	tmp.m_sphere1Body->setTransform(etk::Transform3D(vec3(10, 5, 0), etk::Quaternion::identity()));

	// --------- Test collision with inactive bodies --------- //

	tmp.m_collisionCallback.reset();
	tmp.m_boxBody->setIsActive(false);
	tmp.m_cylinderBody->setIsActive(false);
	tmp.m_sphere1Body->setIsActive(false);
	tmp.m_sphere2Body->setIsActive(false);
	tmp.m_world->testCollision(&tmp.m_collisionCallback);
	EXPECT_EQ(tmp.m_collisionCallback.boxCollideWithSphere1, false);
	EXPECT_EQ(tmp.m_collisionCallback.boxCollideWithCylinder, false);
	EXPECT_EQ(tmp.m_collisionCallback.sphere1CollideWithCylinder, false);
	EXPECT_EQ(tmp.m_collisionCallback.sphere1CollideWithSphere2, false);

	EXPECT_EQ(tmp.m_world->testAABBOverlap(tmp.m_boxBody, tmp.m_sphere1Body), false);
	EXPECT_EQ(tmp.m_world->testAABBOverlap(tmp.m_boxBody, tmp.m_cylinderBody), false);
	EXPECT_EQ(tmp.m_world->testAABBOverlap(tmp.m_sphere1Body, tmp.m_cylinderBody), false);
	EXPECT_EQ(tmp.m_world->testAABBOverlap(tmp.m_sphere1Body, tmp.m_sphere2Body), false);

	EXPECT_EQ(tmp.m_world->testAABBOverlap(tmp.m_boxProxyShape, tmp.m_sphere1ProxyShape), false);
	EXPECT_EQ(tmp.m_world->testAABBOverlap(tmp.m_boxProxyShape, tmp.m_cylinderProxyShape), false);
	EXPECT_EQ(tmp.m_world->testAABBOverlap(tmp.m_sphere1ProxyShape, tmp.m_cylinderProxyShape), false);
	EXPECT_EQ(tmp.m_world->testAABBOverlap(tmp.m_sphere1ProxyShape, tmp.m_sphere2ProxyShape), false);

	tmp.m_boxBody->setIsActive(true);
	tmp.m_cylinderBody->setIsActive(true);
	tmp.m_sphere1Body->setIsActive(true);
	tmp.m_sphere2Body->setIsActive(true);

	// --------- Test collision with collision filtering -------- //

	tmp.m_boxProxyShape->setCollideWithMaskBits(CATEGORY_1 | CATEGORY_3);
	tmp.m_sphere1ProxyShape->setCollideWithMaskBits(CATEGORY_1 | CATEGORY_2);
	tmp.m_sphere2ProxyShape->setCollideWithMaskBits(CATEGORY_1);
	tmp.m_cylinderProxyShape->setCollideWithMaskBits(CATEGORY_1);

	tmp.m_collisionCallback.reset();
	tmp.m_world->testCollision(&tmp.m_collisionCallback);
	EXPECT_EQ(tmp.m_collisionCallback.boxCollideWithSphere1, true);
	EXPECT_EQ(tmp.m_collisionCallback.boxCollideWithCylinder, true);
	EXPECT_EQ(tmp.m_collisionCallback.sphere1CollideWithCylinder, false);
	EXPECT_EQ(tmp.m_collisionCallback.sphere1CollideWithSphere2, false);

	// Move sphere 1 to collide with sphere 2
	tmp.m_sphere1Body->setTransform(etk::Transform3D(vec3(30, 15, 10), etk::Quaternion::identity()));

	tmp.m_collisionCallback.reset();
	tmp.m_world->testCollision(&tmp.m_collisionCallback);
	EXPECT_EQ(tmp.m_collisionCallback.boxCollideWithSphere1, false);
	EXPECT_EQ(tmp.m_collisionCallback.boxCollideWithCylinder, true);
	EXPECT_EQ(tmp.m_collisionCallback.sphere1CollideWithCylinder, false);
	EXPECT_EQ(tmp.m_collisionCallback.sphere1CollideWithSphere2, true);

	tmp.m_boxProxyShape->setCollideWithMaskBits(CATEGORY_2);
	tmp.m_sphere1ProxyShape->setCollideWithMaskBits(CATEGORY_2);
	tmp.m_sphere2ProxyShape->setCollideWithMaskBits(CATEGORY_3);
	tmp.m_cylinderProxyShape->setCollideWithMaskBits(CATEGORY_1);

	tmp.m_collisionCallback.reset();
	tmp.m_world->testCollision(&tmp.m_collisionCallback);
	EXPECT_EQ(tmp.m_collisionCallback.boxCollideWithSphere1, false);
	EXPECT_EQ(tmp.m_collisionCallback.boxCollideWithCylinder, false);
	EXPECT_EQ(tmp.m_collisionCallback.sphere1CollideWithCylinder, false);
	EXPECT_EQ(tmp.m_collisionCallback.sphere1CollideWithSphere2, false);

	// Move sphere 1 to collide with box
	tmp.m_sphere1Body->setTransform(etk::Transform3D(vec3(10, 5, 0), etk::Quaternion::identity()));

	tmp.m_boxProxyShape->setCollideWithMaskBits(0xFFFF);
	tmp.m_sphere1ProxyShape->setCollideWithMaskBits(0xFFFF);
	tmp.m_sphere2ProxyShape->setCollideWithMaskBits(0xFFFF);
	tmp.m_cylinderProxyShape->setCollideWithMaskBits(0xFFFF);
}
