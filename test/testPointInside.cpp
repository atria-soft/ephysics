/** @file
 * @author Edouard DUPIN
 * @copyright 2017, Edouard DUPIN, all right reserved
 * @license MPL v2.0 (see license file)
 */

#include <etest/etest.hpp>
#include <ephysics/ephysics.hpp>
#include <ephysics/collision/shapes/BoxShape.hpp>
#include <ephysics/collision/shapes/SphereShape.hpp>
#include <ephysics/collision/shapes/CapsuleShape.hpp>
#include <ephysics/collision/shapes/ConeShape.hpp>
#include <ephysics/collision/shapes/ConvexMeshShape.hpp>
#include <ephysics/collision/shapes/CylinderShape.hpp>

class TestPointInside {
	public:
		// Physics world
		ephysics::CollisionWorld* m_world;
		// Bodies
		ephysics::CollisionBody* m_boxBody;
		ephysics::CollisionBody* m_sphereBody;
		ephysics::CollisionBody* m_capsuleBody;
		ephysics::CollisionBody* m_coneBody;
		ephysics::CollisionBody* m_convexMeshBody;
		ephysics::CollisionBody* m_convexMeshBodyEdgesInfo;
		ephysics::CollisionBody* m_cylinderBody;
		ephysics::CollisionBody* m_compoundBody;
		// Collision shapes
		ephysics::BoxShape* m_boxShape;
		ephysics::SphereShape* m_sphereShape;
		ephysics::CapsuleShape* m_capsuleShape;
		ephysics::ConeShape* m_coneShape;
		ephysics::ConvexMeshShape* m_convexMeshShape;
		ephysics::ConvexMeshShape* m_convexMeshShapeBodyEdgesInfo;
		ephysics::CylinderShape* m_cylinderShape;
		// etk::Transform3D
		etk::Transform3D m_bodyTransform;
		etk::Transform3D m_shapeTransform;
		etk::Transform3D m_localShapeToWorld;
		etk::Transform3D m_localShape2ToWorld;
		// Proxy Shapes
		ephysics::ProxyShape* m_boxProxyShape;
		ephysics::ProxyShape* m_sphereProxyShape;
		ephysics::ProxyShape* m_capsuleProxyShape;
		ephysics::ProxyShape* m_coneProxyShape;
		ephysics::ProxyShape* m_convexMeshProxyShape;
		ephysics::ProxyShape* m_convexMeshProxyShapeEdgesInfo;
		ephysics::ProxyShape* m_cylinderProxyShape;
	public :
		TestPointInside() {
			// Create the world
			m_world = ETK_NEW(ephysics::CollisionWorld);
			// Body transform
			vec3 position(-3, 2, 7);
			etk::Quaternion orientation(M_PI / 5, M_PI / 6, M_PI / 7, 1);
			m_bodyTransform = etk::Transform3D(position, orientation);
			// Create the bodies
			m_boxBody = m_world->createCollisionBody(m_bodyTransform);
			m_sphereBody = m_world->createCollisionBody(m_bodyTransform);
			m_capsuleBody = m_world->createCollisionBody(m_bodyTransform);
			m_coneBody = m_world->createCollisionBody(m_bodyTransform);
			m_convexMeshBody = m_world->createCollisionBody(m_bodyTransform);
			m_convexMeshBodyEdgesInfo = m_world->createCollisionBody(m_bodyTransform);
			m_cylinderBody = m_world->createCollisionBody(m_bodyTransform);
			m_compoundBody = m_world->createCollisionBody(m_bodyTransform);
			// Collision shape transform
			vec3 shapePosition(1, -4, -3);
			etk::Quaternion shapeOrientation(3 * M_PI / 6 , -M_PI / 8, M_PI / 3, 1);
			m_shapeTransform = etk::Transform3D(shapePosition, shapeOrientation);
			// Compute the the transform from a local shape point to world-space
			m_localShapeToWorld = m_bodyTransform * m_shapeTransform;
			// Create collision shapes
			m_boxShape = ETK_NEW(ephysics::BoxShape, vec3(2, 3, 4), 0);
			m_boxProxyShape = m_boxBody->addCollisionShape(m_boxShape, m_shapeTransform);
			m_sphereShape = ETK_NEW(ephysics::SphereShape, 3);
			m_sphereProxyShape = m_sphereBody->addCollisionShape(m_sphereShape, m_shapeTransform);
			m_capsuleShape = ETK_NEW(ephysics::CapsuleShape, 2, 10);
			m_capsuleProxyShape = m_capsuleBody->addCollisionShape(m_capsuleShape, m_shapeTransform);
			m_coneShape = ETK_NEW(ephysics::ConeShape, 2, 6, 0);
			m_coneProxyShape = m_coneBody->addCollisionShape(m_coneShape, m_shapeTransform);
			m_convexMeshShape = ETK_NEW(ephysics::ConvexMeshShape, 0.0f);			 // Box of dimension (2, 3, 4)
			m_convexMeshShape->addVertex(vec3(-2, -3, -4));
			m_convexMeshShape->addVertex(vec3(2, -3, -4));
			m_convexMeshShape->addVertex(vec3(2, -3, 4));
			m_convexMeshShape->addVertex(vec3(-2, -3, 4));
			m_convexMeshShape->addVertex(vec3(-2, 3, -4));
			m_convexMeshShape->addVertex(vec3(2, 3, -4));
			m_convexMeshShape->addVertex(vec3(2, 3, 4));
			m_convexMeshShape->addVertex(vec3(-2, 3, 4));
			m_convexMeshProxyShape = m_convexMeshBody->addCollisionShape(m_convexMeshShape, m_shapeTransform);
			m_convexMeshShapeBodyEdgesInfo = ETK_NEW(ephysics::ConvexMeshShape, 0.0f);
			m_convexMeshShapeBodyEdgesInfo->addVertex(vec3(-2, -3, -4));
			m_convexMeshShapeBodyEdgesInfo->addVertex(vec3(2, -3, -4));
			m_convexMeshShapeBodyEdgesInfo->addVertex(vec3(2, -3, 4));
			m_convexMeshShapeBodyEdgesInfo->addVertex(vec3(-2, -3, 4));
			m_convexMeshShapeBodyEdgesInfo->addVertex(vec3(-2, 3, -4));
			m_convexMeshShapeBodyEdgesInfo->addVertex(vec3(2, 3, -4));
			m_convexMeshShapeBodyEdgesInfo->addVertex(vec3(2, 3, 4));
			m_convexMeshShapeBodyEdgesInfo->addVertex(vec3(-2, 3, 4));
			m_convexMeshShapeBodyEdgesInfo->addEdge(0, 1);
			m_convexMeshShapeBodyEdgesInfo->addEdge(1, 2);
			m_convexMeshShapeBodyEdgesInfo->addEdge(2, 3);
			m_convexMeshShapeBodyEdgesInfo->addEdge(0, 3);
			m_convexMeshShapeBodyEdgesInfo->addEdge(4, 5);
			m_convexMeshShapeBodyEdgesInfo->addEdge(5, 6);
			m_convexMeshShapeBodyEdgesInfo->addEdge(6, 7);
			m_convexMeshShapeBodyEdgesInfo->addEdge(4, 7);
			m_convexMeshShapeBodyEdgesInfo->addEdge(0, 4);
			m_convexMeshShapeBodyEdgesInfo->addEdge(1, 5);
			m_convexMeshShapeBodyEdgesInfo->addEdge(2, 6);
			m_convexMeshShapeBodyEdgesInfo->addEdge(3, 7);
			m_convexMeshShapeBodyEdgesInfo->setIsEdgesInformationUsed(true);
			m_convexMeshProxyShapeEdgesInfo = m_convexMeshBodyEdgesInfo->addCollisionShape(m_convexMeshShapeBodyEdgesInfo, m_shapeTransform);
			m_cylinderShape = ETK_NEW(ephysics::CylinderShape, 3, 8, 0);
			m_cylinderProxyShape = m_cylinderBody->addCollisionShape(m_cylinderShape, m_shapeTransform);
			// Compound shape is a cylinder and a sphere
			vec3 positionShape2(vec3(4, 2, -3));
			etk::Quaternion orientationShape2(-3 *M_PI / 8, 1.5 * M_PI/ 3, M_PI / 13, 1.0f);
			etk::Transform3D shapeTransform2(positionShape2, orientationShape2);
			m_localShape2ToWorld = m_bodyTransform * shapeTransform2;
			m_compoundBody->addCollisionShape(m_cylinderShape, m_shapeTransform);
			m_compoundBody->addCollisionShape(m_sphereShape, shapeTransform2);
		}
		/// Destructor
		~TestPointInside() {
			ETK_DELETE(ephysics::BoxShape, m_boxShape);
			ETK_DELETE(ephysics::SphereShape, m_sphereShape);
			ETK_DELETE(ephysics::CapsuleShape, m_capsuleShape);
			ETK_DELETE(ephysics::ConeShape, m_coneShape);
			ETK_DELETE(ephysics::ConvexMeshShape, m_convexMeshShape);
			ETK_DELETE(ephysics::ConvexMeshShape, m_convexMeshShapeBodyEdgesInfo);
			ETK_DELETE(ephysics::CylinderShape, m_cylinderShape);
			ETK_DELETE(ephysics::CollisionWorld, m_world);
		}
};

TEST(TestPointInside, box) {
	TestPointInside tmp;
	// Tests with CollisionBody
	EXPECT_EQ(true, tmp.m_boxBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 0)));
	EXPECT_EQ(true, tmp.m_boxBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_boxBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_boxBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, -2.9, 0)));
	EXPECT_EQ(true, tmp.m_boxBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 2.9, 0)));
	EXPECT_EQ(true, tmp.m_boxBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, -3.9)));
	EXPECT_EQ(true, tmp.m_boxBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 3.9)));
	EXPECT_EQ(true, tmp.m_boxBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1.9, -2.9, -3.9)));
	EXPECT_EQ(true, tmp.m_boxBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.9, 2.9, 3.9)));
	EXPECT_EQ(true, tmp.m_boxBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1, -2, -1.5)));
	EXPECT_EQ(true, tmp.m_boxBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1, 2, -2.5)));
	EXPECT_EQ(true, tmp.m_boxBody->testPointInside(tmp.m_localShapeToWorld * vec3(1, -2, 3.5)));

	EXPECT_EQ(false, tmp.m_boxBody->testPointInside(tmp.m_localShapeToWorld * vec3(-2.1, 0, 0)));
	EXPECT_EQ(false, tmp.m_boxBody->testPointInside(tmp.m_localShapeToWorld * vec3(2.1, 0, 0)));
	EXPECT_EQ(false, tmp.m_boxBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, -3.1, 0)));
	EXPECT_EQ(false, tmp.m_boxBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 3.1, 0)));
	EXPECT_EQ(false, tmp.m_boxBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, -4.1)));
	EXPECT_EQ(false, tmp.m_boxBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 4.1)));
	EXPECT_EQ(false, tmp.m_boxBody->testPointInside(tmp.m_localShapeToWorld * vec3(-2.1, -3.1, -4.1)));
	EXPECT_EQ(false, tmp.m_boxBody->testPointInside(tmp.m_localShapeToWorld * vec3(2.1, 3.1, 4.1)));
	EXPECT_EQ(false, tmp.m_boxBody->testPointInside(tmp.m_localShapeToWorld * vec3(-10, -2, -1.5)));
	EXPECT_EQ(false, tmp.m_boxBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1, 4, -2.5)));
	EXPECT_EQ(false, tmp.m_boxBody->testPointInside(tmp.m_localShapeToWorld * vec3(1, -2, 4.5)));

	// Tests with ProxyBoxShape
	EXPECT_EQ(true, tmp.m_boxProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 0)));
	EXPECT_EQ(true, tmp.m_boxProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-1.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_boxProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(1.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_boxProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, -2.9, 0)));
	EXPECT_EQ(true, tmp.m_boxProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 2.9, 0)));
	EXPECT_EQ(true, tmp.m_boxProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, -3.9)));
	EXPECT_EQ(true, tmp.m_boxProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 3.9)));
	EXPECT_EQ(true, tmp.m_boxProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-1.9, -2.9, -3.9)));
	EXPECT_EQ(true, tmp.m_boxProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(1.9, 2.9, 3.9)));
	EXPECT_EQ(true, tmp.m_boxProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-1, -2, -1.5)));
	EXPECT_EQ(true, tmp.m_boxProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-1, 2, -2.5)));
	EXPECT_EQ(true, tmp.m_boxProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(1, -2, 3.5)));

	EXPECT_EQ(false, tmp.m_boxProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-2.1, 0, 0)));
	EXPECT_EQ(false, tmp.m_boxProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(2.1, 0, 0)));
	EXPECT_EQ(false, tmp.m_boxProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, -3.1, 0)));
	EXPECT_EQ(false, tmp.m_boxProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 3.1, 0)));
	EXPECT_EQ(false, tmp.m_boxProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, -4.1)));
	EXPECT_EQ(false, tmp.m_boxProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 4.1)));
	EXPECT_EQ(false, tmp.m_boxProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-2.1, -3.1, -4.1)));
	EXPECT_EQ(false, tmp.m_boxProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(2.1, 3.1, 4.1)));
	EXPECT_EQ(false, tmp.m_boxProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-10, -2, -1.5)));
	EXPECT_EQ(false, tmp.m_boxProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-1, 4, -2.5)));
	EXPECT_EQ(false, tmp.m_boxProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(1, -2, 4.5)));
}

TEST(TestPointInside, sphere) {
	TestPointInside tmp;
	// Tests with CollisionBody
	EXPECT_EQ(true, tmp.m_sphereBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 0)));
	EXPECT_EQ(true, tmp.m_sphereBody->testPointInside(tmp.m_localShapeToWorld * vec3(2.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_sphereBody->testPointInside(tmp.m_localShapeToWorld * vec3(-2.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_sphereBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 2.9, 0)));
	EXPECT_EQ(true, tmp.m_sphereBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, -2.9, 0)));
	EXPECT_EQ(true, tmp.m_sphereBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 2.9)));
	EXPECT_EQ(true, tmp.m_sphereBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 2.9)));
	EXPECT_EQ(true, tmp.m_sphereBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1, -2, -1.5)));
	EXPECT_EQ(true, tmp.m_sphereBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1, 2, -1.5)));
	EXPECT_EQ(true, tmp.m_sphereBody->testPointInside(tmp.m_localShapeToWorld * vec3(1, -2, 1.5)));

	EXPECT_EQ(false, tmp.m_sphereBody->testPointInside(tmp.m_localShapeToWorld * vec3(3.1, 0, 0)));
	EXPECT_EQ(false, tmp.m_sphereBody->testPointInside(tmp.m_localShapeToWorld * vec3(-3.1, 0, 0)));
	EXPECT_EQ(false, tmp.m_sphereBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 3.1, 0)));
	EXPECT_EQ(false, tmp.m_sphereBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, -3.1, 0)));
	EXPECT_EQ(false, tmp.m_sphereBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 3.1)));
	EXPECT_EQ(false, tmp.m_sphereBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, -3.1)));
	EXPECT_EQ(false, tmp.m_sphereBody->testPointInside(tmp.m_localShapeToWorld * vec3(-2, -2, -2)));
	EXPECT_EQ(false, tmp.m_sphereBody->testPointInside(tmp.m_localShapeToWorld * vec3(-2, 2, -1.5)));
	EXPECT_EQ(false, tmp.m_sphereBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.5, -2, 2.5)));

	// Tests with ProxySphereShape
	EXPECT_EQ(true, tmp.m_sphereProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 0)));
	EXPECT_EQ(true, tmp.m_sphereProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(2.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_sphereProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-2.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_sphereProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 2.9, 0)));
	EXPECT_EQ(true, tmp.m_sphereProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, -2.9, 0)));
	EXPECT_EQ(true, tmp.m_sphereProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 2.9)));
	EXPECT_EQ(true, tmp.m_sphereProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 2.9)));
	EXPECT_EQ(true, tmp.m_sphereProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-1, -2, -1.5)));
	EXPECT_EQ(true, tmp.m_sphereProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-1, 2, -1.5)));
	EXPECT_EQ(true, tmp.m_sphereProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(1, -2, 1.5)));

	EXPECT_EQ(false, tmp.m_sphereProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(3.1, 0, 0)));
	EXPECT_EQ(false, tmp.m_sphereProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-3.1, 0, 0)));
	EXPECT_EQ(false, tmp.m_sphereProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 3.1, 0)));
	EXPECT_EQ(false, tmp.m_sphereProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, -3.1, 0)));
	EXPECT_EQ(false, tmp.m_sphereProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 3.1)));
	EXPECT_EQ(false, tmp.m_sphereProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, -3.1)));
	EXPECT_EQ(false, tmp.m_sphereProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-2, -2, -2)));
	EXPECT_EQ(false, tmp.m_sphereProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-2, 2, -1.5)));
	EXPECT_EQ(false, tmp.m_sphereProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(1.5, -2, 2.5)));
}

TEST(TestPointInside, capsule) {
	TestPointInside tmp;
	// Tests with CollisionBody
	EXPECT_EQ(true, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 0)));
	EXPECT_EQ(true, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 5, 0)));
	EXPECT_EQ(true, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, -5, 0)));
	EXPECT_EQ(true, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, -6.9, 0)));
	EXPECT_EQ(true, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 6.9, 0)));
	EXPECT_EQ(true, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 1.9)));
	EXPECT_EQ(true, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, -1.9)));
	EXPECT_EQ(true, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(0.9, 0, 0.9)));
	EXPECT_EQ(true, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(0.9, 0, -0.9)));
	EXPECT_EQ(true, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 5, 1.9)));
	EXPECT_EQ(true, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 5, -1.9)));
	EXPECT_EQ(true, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.9, 5, 0)));
	EXPECT_EQ(true, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1.9, 5, 0)));
	EXPECT_EQ(true, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(0.9, 5, 0.9)));
	EXPECT_EQ(true, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(0.9, 5, -0.9)));
	EXPECT_EQ(true, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, -5, 1.9)));
	EXPECT_EQ(true, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, -5, -1.9)));
	EXPECT_EQ(true, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.9, -5, 0)));
	EXPECT_EQ(true, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1.9, -5, 0)));
	EXPECT_EQ(true, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(0.9, -5, 0.9)));
	EXPECT_EQ(true, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(0.9, -5, -0.9)));
	EXPECT_EQ(true, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1.7, -4, -0.9)));
	EXPECT_EQ(true, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1, 2, 0.4)));
	EXPECT_EQ(true, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.3, 1, 1.5)));

	EXPECT_EQ(false, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, -7.1, 0)));
	EXPECT_EQ(false, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 7.1, 0)));
	EXPECT_EQ(false, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 2.1)));
	EXPECT_EQ(false, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, -2.1)));
	EXPECT_EQ(false, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(2.1, 0, 0)));
	EXPECT_EQ(false, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(-2.1, 0, 0)));
	EXPECT_EQ(false, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 5, 2.1)));
	EXPECT_EQ(false, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 5, -2.1)));
	EXPECT_EQ(false, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(2.1, 5, 0)));
	EXPECT_EQ(false, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(-2.1, 5, 0)));
	EXPECT_EQ(false, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.5, 5, 1.6)));
	EXPECT_EQ(false, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.5, 5, -1.7)));
	EXPECT_EQ(false, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, -5, 2.1)));
	EXPECT_EQ(false, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, -5, -2.1)));
	EXPECT_EQ(false, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(2.1, -5, 0)));
	EXPECT_EQ(false, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(-2.1, -5, 0)));
	EXPECT_EQ(false, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.5, -5, 1.6)));
	EXPECT_EQ(false, tmp.m_capsuleBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.5, -5, -1.7)));

	// Tests with ProxyCapsuleShape
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 0)));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 5, 0)));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, -5, 0)));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, -6.9, 0)));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 6.9, 0)));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 1.9)));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, -1.9)));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(1.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-1.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0.9, 0, 0.9)));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0.9, 0, -0.9)));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 5, 1.9)));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 5, -1.9)));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(1.9, 5, 0)));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-1.9, 5, 0)));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0.9, 5, 0.9)));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0.9, 5, -0.9)));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, -5, 1.9)));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, -5, -1.9)));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(1.9, -5, 0)));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-1.9, -5, 0)));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0.9, -5, 0.9)));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0.9, -5, -0.9)));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-1.7, -4, -0.9)));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-1, 2, 0.4)));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(1.3, 1, 1.5)));

	EXPECT_EQ(false, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, -7.1, 0)));
	EXPECT_EQ(false, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 7.1, 0)));
	EXPECT_EQ(false, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 2.1)));
	EXPECT_EQ(false, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, -2.1)));
	EXPECT_EQ(false, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(2.1, 0, 0)));
	EXPECT_EQ(false, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-2.1, 0, 0)));
	EXPECT_EQ(false, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 5, 2.1)));
	EXPECT_EQ(false, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 5, -2.1)));
	EXPECT_EQ(false, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(2.1, 5, 0)));
	EXPECT_EQ(false, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-2.1, 5, 0)));
	EXPECT_EQ(false, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(1.5, 5, 1.6)));
	EXPECT_EQ(false, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(1.5, 5, -1.7)));
	EXPECT_EQ(false, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, -5, 2.1)));
	EXPECT_EQ(false, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, -5, -2.1)));
	EXPECT_EQ(false, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(2.1, -5, 0)));
	EXPECT_EQ(false, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-2.1, -5, 0)));
	EXPECT_EQ(false, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(1.5, -5, 1.6)));
	EXPECT_EQ(false, tmp.m_capsuleProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(1.5, -5, -1.7)));
}

TEST(TestPointInside, Cone) {
	TestPointInside tmp;
	// Tests with CollisionBody
	EXPECT_EQ(true, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 0)));
	EXPECT_EQ(true, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(0.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(-0.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 0.9)));
	EXPECT_EQ(true, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, -0.9)));
	EXPECT_EQ(true, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(0.6, 0, -0.7)));
	EXPECT_EQ(true, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(0.6, 0, 0.7)));
	EXPECT_EQ(true, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(-0.6, 0, -0.7)));
	EXPECT_EQ(true, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(-0.6, 0, 0.7)));
	EXPECT_EQ(true, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 2.9, 0)));
	EXPECT_EQ(true, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, -2.9, 0)));
	EXPECT_EQ(true, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.96, -2.9, 0)));
	EXPECT_EQ(true, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1.96, -2.9, 0)));
	EXPECT_EQ(true, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, -2.9, 1.96)));
	EXPECT_EQ(true, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, -2.9, -1.96)));
	EXPECT_EQ(true, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.3, -2.9, -1.4)));
	EXPECT_EQ(true, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1.3, -2.9, 1.4)));

	EXPECT_EQ(false, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.1, 0, 0)));
	EXPECT_EQ(false, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1.1, 0, 0)));
	EXPECT_EQ(false, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 1.1)));
	EXPECT_EQ(false, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, -1.1)));
	EXPECT_EQ(false, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(0.8, 0, -0.8)));
	EXPECT_EQ(false, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(0.8, 0, 0.8)));
	EXPECT_EQ(false, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(-0.8, 0, -0.8)));
	EXPECT_EQ(false, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(-0.8, 0, 0.8)));
	EXPECT_EQ(false, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 3.1, 0)));
	EXPECT_EQ(false, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, -3.1, 0)));
	EXPECT_EQ(false, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.97, -2.9, 0)));
	EXPECT_EQ(false, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1.97, -2.9, 0)));
	EXPECT_EQ(false, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, -2.9, 1.97)));
	EXPECT_EQ(false, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, -2.9, -1.97)));
	EXPECT_EQ(false, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.5, -2.9, -1.5)));
	EXPECT_EQ(false, tmp.m_coneBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1.5, -2.9, 1.5)));

	// Tests with ProxyConeShape
	EXPECT_EQ(true, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 0)));
	EXPECT_EQ(true, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-0.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 0.9)));
	EXPECT_EQ(true, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, -0.9)));
	EXPECT_EQ(true, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0.6, 0, -0.7)));
	EXPECT_EQ(true, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0.6, 0, 0.7)));
	EXPECT_EQ(true, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-0.6, 0, -0.7)));
	EXPECT_EQ(true, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-0.6, 0, 0.7)));
	EXPECT_EQ(true, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 2.9, 0)));
	EXPECT_EQ(true, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, -2.9, 0)));
	EXPECT_EQ(true, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(1.96, -2.9, 0)));
	EXPECT_EQ(true, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-1.96, -2.9, 0)));
	EXPECT_EQ(true, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, -2.9, 1.96)));
	EXPECT_EQ(true, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, -2.9, -1.96)));
	EXPECT_EQ(true, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(1.3, -2.9, -1.4)));
	EXPECT_EQ(true, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-1.3, -2.9, 1.4)));

	EXPECT_EQ(false, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(1.1, 0, 0)));
	EXPECT_EQ(false, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-1.1, 0, 0)));
	EXPECT_EQ(false, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 1.1)));
	EXPECT_EQ(false, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, -1.1)));
	EXPECT_EQ(false, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0.8, 0, -0.8)));
	EXPECT_EQ(false, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0.8, 0, 0.8)));
	EXPECT_EQ(false, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-0.8, 0, -0.8)));
	EXPECT_EQ(false, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-0.8, 0, 0.8)));
	EXPECT_EQ(false, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 3.1, 0)));
	EXPECT_EQ(false, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, -3.1, 0)));
	EXPECT_EQ(false, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(1.97, -2.9, 0)));
	EXPECT_EQ(false, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-1.97, -2.9, 0)));
	EXPECT_EQ(false, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, -2.9, 1.97)));
	EXPECT_EQ(false, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, -2.9, -1.97)));
	EXPECT_EQ(false, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(1.5, -2.9, -1.5)));
	EXPECT_EQ(false, tmp.m_coneProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-1.5, -2.9, 1.5)));
}

TEST(TestPointInside, convecMesh) {
	TestPointInside tmp;
	// Tests with CollisionBody
	EXPECT_EQ(true, tmp.m_convexMeshBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 0)));
	EXPECT_EQ(true, tmp.m_convexMeshBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_convexMeshBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_convexMeshBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, -2.9, 0)));
	EXPECT_EQ(true, tmp.m_convexMeshBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 2.9, 0)));
	EXPECT_EQ(true, tmp.m_convexMeshBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, -3.9)));
	EXPECT_EQ(true, tmp.m_convexMeshBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 3.9)));
	EXPECT_EQ(true, tmp.m_convexMeshBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1.9, -2.9, -3.9)));
	EXPECT_EQ(true, tmp.m_convexMeshBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.9, 2.9, 3.9)));
	EXPECT_EQ(true, tmp.m_convexMeshBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1, -2, -1.5)));
	EXPECT_EQ(true, tmp.m_convexMeshBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1, 2, -2.5)));
	EXPECT_EQ(true, tmp.m_convexMeshBody->testPointInside(tmp.m_localShapeToWorld * vec3(1, -2, 3.5)));

	EXPECT_EQ(false, tmp.m_convexMeshBody->testPointInside(tmp.m_localShapeToWorld * vec3(-2.1, 0, 0)));
	EXPECT_EQ(false, tmp.m_convexMeshBody->testPointInside(tmp.m_localShapeToWorld * vec3(2.1, 0, 0)));
	EXPECT_EQ(false, tmp.m_convexMeshBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, -3.1, 0)));
	EXPECT_EQ(false, tmp.m_convexMeshBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 3.1, 0)));
	EXPECT_EQ(false, tmp.m_convexMeshBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, -4.1)));
	EXPECT_EQ(false, tmp.m_convexMeshBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 4.1)));
	EXPECT_EQ(false, tmp.m_convexMeshBody->testPointInside(tmp.m_localShapeToWorld * vec3(-2.1, -3.1, -4.1)));
	EXPECT_EQ(false, tmp.m_convexMeshBody->testPointInside(tmp.m_localShapeToWorld * vec3(2.1, 3.1, 4.1)));
	EXPECT_EQ(false, tmp.m_convexMeshBody->testPointInside(tmp.m_localShapeToWorld * vec3(-10, -2, -1.5)));
	EXPECT_EQ(false, tmp.m_convexMeshBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1, 4, -2.5)));
	EXPECT_EQ(false, tmp.m_convexMeshBody->testPointInside(tmp.m_localShapeToWorld * vec3(1, -2, 4.5)));

	// Tests with ProxyConvexMeshShape
	EXPECT_EQ(true, tmp.m_convexMeshProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 0)));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-1.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(1.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, -2.9, 0)));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 2.9, 0)));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, -3.9)));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 3.9)));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-1.9, -2.9, -3.9)));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(1.9, 2.9, 3.9)));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-1, -2, -1.5)));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-1, 2, -2.5)));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(1, -2, 3.5)));

	EXPECT_EQ(false, tmp.m_convexMeshProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-2.1, 0, 0)));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(2.1, 0, 0)));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, -3.1, 0)));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 3.1, 0)));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, -4.1)));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 4.1)));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-2.1, -3.1, -4.1)));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(2.1, 3.1, 4.1)));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-10, -2, -1.5)));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-1, 4, -2.5)));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(1, -2, 4.5)));

	// ----- Tests using edges information ----- //

	// Tests with CollisionBody
	EXPECT_EQ(true, tmp.m_convexMeshBodyEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 0)));
	EXPECT_EQ(true, tmp.m_convexMeshBodyEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(-1.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_convexMeshBodyEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(1.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_convexMeshBodyEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(0, -2.9, 0)));
	EXPECT_EQ(true, tmp.m_convexMeshBodyEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(0, 2.9, 0)));
	EXPECT_EQ(true, tmp.m_convexMeshBodyEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, -3.9)));
	EXPECT_EQ(true, tmp.m_convexMeshBodyEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 3.9)));
	EXPECT_EQ(true, tmp.m_convexMeshBodyEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(-1.9, -2.9, -3.9)));
	EXPECT_EQ(true, tmp.m_convexMeshBodyEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(1.9, 2.9, 3.9)));
	EXPECT_EQ(true, tmp.m_convexMeshBodyEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(-1, -2, -1.5)));
	EXPECT_EQ(true, tmp.m_convexMeshBodyEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(-1, 2, -2.5)));
	EXPECT_EQ(true, tmp.m_convexMeshBodyEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(1, -2, 3.5)));

	EXPECT_EQ(false, tmp.m_convexMeshBodyEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(-2.1, 0, 0)));
	EXPECT_EQ(false, tmp.m_convexMeshBodyEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(2.1, 0, 0)));
	EXPECT_EQ(false, tmp.m_convexMeshBodyEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(0, -3.1, 0)));
	EXPECT_EQ(false, tmp.m_convexMeshBodyEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(0, 3.1, 0)));
	EXPECT_EQ(false, tmp.m_convexMeshBodyEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, -4.1)));
	EXPECT_EQ(false, tmp.m_convexMeshBodyEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 4.1)));
	EXPECT_EQ(false, tmp.m_convexMeshBodyEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(-2.1, -3.1, -4.1)));
	EXPECT_EQ(false, tmp.m_convexMeshBodyEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(2.1, 3.1, 4.1)));
	EXPECT_EQ(false, tmp.m_convexMeshBodyEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(-10, -2, -1.5)));
	EXPECT_EQ(false, tmp.m_convexMeshBodyEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(-1, 4, -2.5)));
	EXPECT_EQ(false, tmp.m_convexMeshBodyEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(1, -2, 4.5)));

	// Tests with ProxyConvexMeshShape
	EXPECT_EQ(true, tmp.m_convexMeshProxyShapeEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 0)));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShapeEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(-1.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShapeEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(1.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShapeEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(0, -2.9, 0)));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShapeEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(0, 2.9, 0)));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShapeEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, -3.9)));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShapeEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 3.9)));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShapeEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(-1.9, -2.9, -3.9)));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShapeEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(1.9, 2.9, 3.9)));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShapeEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(-1, -2, -1.5)));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShapeEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(-1, 2, -2.5)));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShapeEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(1, -2, 3.5)));

	EXPECT_EQ(false, tmp.m_convexMeshProxyShapeEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(-2.1, 0, 0)));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShapeEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(2.1, 0, 0)));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShapeEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(0, -3.1, 0)));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShapeEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(0, 3.1, 0)));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShapeEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, -4.1)));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShapeEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 4.1)));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShapeEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(-2.1, -3.1, -4.1)));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShapeEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(2.1, 3.1, 4.1)));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShapeEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(-10, -2, -1.5)));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShapeEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(-1, 4, -2.5)));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShapeEdgesInfo->testPointInside(tmp.m_localShapeToWorld * vec3(1, -2, 4.5)));
}

TEST(TestPointInside, cylinder) {
	TestPointInside tmp;
	// Tests with CollisionBody
	EXPECT_EQ(true, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 0)));
	EXPECT_EQ(true, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 3.9, 0)));
	EXPECT_EQ(true, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, -3.9, 0)));
	EXPECT_EQ(true, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(2.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(-2.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 2.9)));
	EXPECT_EQ(true, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, -2.9)));
	EXPECT_EQ(true, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.7, 0, 1.7)));
	EXPECT_EQ(true, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.7, 0, -1.7)));
	EXPECT_EQ(true, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1.7, 0, -1.7)));
	EXPECT_EQ(true, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1.7, 0, 1.7)));
	EXPECT_EQ(true, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(2.9, 3.9, 0)));
	EXPECT_EQ(true, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(-2.9, 3.9, 0)));
	EXPECT_EQ(true, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 3.9, 2.9)));
	EXPECT_EQ(true, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 3.9, -2.9)));
	EXPECT_EQ(true, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.7, 3.9, 1.7)));
	EXPECT_EQ(true, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.7, 3.9, -1.7)));
	EXPECT_EQ(true, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1.7, 3.9, -1.7)));
	EXPECT_EQ(true, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1.7, 3.9, 1.7)));
	EXPECT_EQ(true, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(2.9, -3.9, 0)));
	EXPECT_EQ(true, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(-2.9, -3.9, 0)));
	EXPECT_EQ(true, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, -3.9, 2.9)));
	EXPECT_EQ(true, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, -3.9, -2.9)));
	EXPECT_EQ(true, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.7, -3.9, 1.7)));
	EXPECT_EQ(true, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.7, -3.9, -1.7)));
	EXPECT_EQ(true, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1.7, -3.9, -1.7)));
	EXPECT_EQ(true, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1.7, -3.9, 1.7)));

	EXPECT_EQ(false, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 4.1, 0)));
	EXPECT_EQ(false, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, -4.1, 0)));
	EXPECT_EQ(false, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(3.1, 0, 0)));
	EXPECT_EQ(false, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(-3.1, 0, 0)));
	EXPECT_EQ(false, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 3.1)));
	EXPECT_EQ(false, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, -3.1)));
	EXPECT_EQ(false, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(2.2, 0, 2.2)));
	EXPECT_EQ(false, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(2.2, 0, -2.2)));
	EXPECT_EQ(false, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(-2.2, 0, -2.2)));
	EXPECT_EQ(false, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1.3, 0, 2.8)));
	EXPECT_EQ(false, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(3.1, 3.9, 0)));
	EXPECT_EQ(false, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(-3.1, 3.9, 0)));
	EXPECT_EQ(false, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 3.9, 3.1)));
	EXPECT_EQ(false, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 3.9, -3.1)));
	EXPECT_EQ(false, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(2.2, 3.9, 2.2)));
	EXPECT_EQ(false, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(2.2, 3.9, -2.2)));
	EXPECT_EQ(false, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(-2.2, 3.9, -2.2)));
	EXPECT_EQ(false, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(-2.2, 3.9, 2.2)));
	EXPECT_EQ(false, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(3.1, -3.9, 0)));
	EXPECT_EQ(false, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(-3.1, -3.9, 0)));
	EXPECT_EQ(false, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, -3.9, 3.1)));
	EXPECT_EQ(false, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, -3.9, -3.1)));
	EXPECT_EQ(false, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(2.2, -3.9, 2.2)));
	EXPECT_EQ(false, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(2.2, -3.9, -2.2)));
	EXPECT_EQ(false, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(-2.2, -3.9, -2.2)));
	EXPECT_EQ(false, tmp.m_cylinderBody->testPointInside(tmp.m_localShapeToWorld * vec3(-2.2, -3.9, 2.2)));

	// Tests with ProxyCylinderShape
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 0)));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 3.9, 0)));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, -3.9, 0)));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(2.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-2.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 2.9)));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, -2.9)));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(1.7, 0, 1.7)));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(1.7, 0, -1.7)));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-1.7, 0, -1.7)));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-1.7, 0, 1.7)));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(2.9, 3.9, 0)));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-2.9, 3.9, 0)));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 3.9, 2.9)));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 3.9, -2.9)));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(1.7, 3.9, 1.7)));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(1.7, 3.9, -1.7)));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-1.7, 3.9, -1.7)));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-1.7, 3.9, 1.7)));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(2.9, -3.9, 0)));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-2.9, -3.9, 0)));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, -3.9, 2.9)));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, -3.9, -2.9)));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(1.7, -3.9, 1.7)));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(1.7, -3.9, -1.7)));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-1.7, -3.9, -1.7)));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-1.7, -3.9, 1.7)));

	EXPECT_EQ(false, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 4.1, 0)));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, -4.1, 0)));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(3.1, 0, 0)));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-3.1, 0, 0)));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 3.1)));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, -3.1)));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(2.2, 0, 2.2)));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(2.2, 0, -2.2)));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-2.2, 0, -2.2)));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-1.3, 0, 2.8)));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(3.1, 3.9, 0)));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-3.1, 3.9, 0)));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 3.9, 3.1)));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, 3.9, -3.1)));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(2.2, 3.9, 2.2)));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(2.2, 3.9, -2.2)));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-2.2, 3.9, -2.2)));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-2.2, 3.9, 2.2)));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(3.1, -3.9, 0)));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-3.1, -3.9, 0)));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, -3.9, 3.1)));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(0, -3.9, -3.1)));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(2.2, -3.9, 2.2)));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(2.2, -3.9, -2.2)));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-2.2, -3.9, -2.2)));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->testPointInside(tmp.m_localShapeToWorld * vec3(-2.2, -3.9, 2.2)));
}

TEST(TestPointInside, compound) {
	TestPointInside tmp;
	// Points on the cylinder
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 0)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 3.9, 0)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, -3.9, 0)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShapeToWorld * vec3(2.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShapeToWorld * vec3(-2.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, 2.9)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 0, -2.9)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.7, 0, 1.7)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.7, 0, -1.7)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1.7, 0, -1.7)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1.7, 0, 1.7)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShapeToWorld * vec3(2.9, 3.9, 0)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShapeToWorld * vec3(-2.9, 3.9, 0)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 3.9, 2.9)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, 3.9, -2.9)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.7, 3.9, 1.7)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.7, 3.9, -1.7)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1.7, 3.9, -1.7)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1.7, 3.9, 1.7)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShapeToWorld * vec3(2.9, -3.9, 0)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShapeToWorld * vec3(-2.9, -3.9, 0)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, -3.9, 2.9)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShapeToWorld * vec3(0, -3.9, -2.9)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.7, -3.9, 1.7)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShapeToWorld * vec3(1.7, -3.9, -1.7)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1.7, -3.9, -1.7)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShapeToWorld * vec3(-1.7, -3.9, 1.7)));

	// Points on the sphere
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShape2ToWorld * vec3(0, 0, 0)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShape2ToWorld * vec3(2.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShape2ToWorld * vec3(-2.9, 0, 0)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShape2ToWorld * vec3(0, 2.9, 0)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShape2ToWorld * vec3(0, -2.9, 0)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShape2ToWorld * vec3(0, 0, 2.9)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShape2ToWorld * vec3(0, 0, 2.9)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShape2ToWorld * vec3(-1, -2, -1.5)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShape2ToWorld * vec3(-1, 2, -1.5)));
	EXPECT_EQ(true, tmp.m_compoundBody->testPointInside(tmp.m_localShape2ToWorld * vec3(1, -2, 1.5)));
}

