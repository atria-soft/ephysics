/** @file
 * @author Edouard DUM_PIN
 * @copyright 2017, Edouard DUM_PIN, all right reserved
 * @license MPL v2.0 (see license file)
 */

#include <etest/etest.hpp>
#include <ephysics/ephysics.hpp>
#include <ephysics/engine/CollisionWorld.hpp>
#include <ephysics/body/CollisionBody.hpp>
#include <ephysics/collision/shapes/BoxShape.hpp>
#include <ephysics/collision/shapes/SphereShape.hpp>
#include <ephysics/collision/shapes/CapsuleShape.hpp>
#include <ephysics/collision/shapes/ConeShape.hpp>
#include <ephysics/collision/shapes/ConvexMeshShape.hpp>
#include <ephysics/collision/shapes/CylinderShape.hpp>
#include <ephysics/collision/shapes/TriangleShape.hpp>
#include <ephysics/collision/shapes/ConcaveMeshShape.hpp>
#include <ephysics/collision/shapes/HeightFieldShape.hpp>
// Enumeration for categories
enum Category {
	CATEGORY1 = 0x0001,
	CATEGORY2 = 0x0002
};
class WorldRaycastCallback : public ephysics::RaycastCallback {
	public:
		ephysics::RaycastInfo raycastInfo;
		ephysics::ProxyShape* shapeToTest;
		bool isHit;
		WorldRaycastCallback() {
			isHit = false;
			shapeToTest = null;
		}
		virtual float notifyRaycastHit(const ephysics::RaycastInfo& info) {
			if (shapeToTest->getBody()->getID() == info.body->getID()) {
				raycastInfo.body = info.body;
				raycastInfo.hitFraction = info.hitFraction;
				raycastInfo.proxyShape = info.proxyShape;
				raycastInfo.worldNormal = info.worldNormal;
				raycastInfo.worldPoint = info.worldPoint;
				isHit = true;
			}
			// Return a fraction of 1.0 because we need to gather all hits
			return 1.0f;
		}
		void reset() {
			raycastInfo.body = null;
			raycastInfo.hitFraction = 0.0f;
			raycastInfo.proxyShape = null;
			raycastInfo.worldNormal.setZero();
			raycastInfo.worldPoint.setZero();
			isHit = false;
		}
};

// Class TestPointInside
/**
 * Unit test for the CollisionBody::testPointInside() method.
 */
class TestRaycast {
	public:
		// Raycast callback class
		WorldRaycastCallback m_callback;
		// Epsilon
		float epsilon;
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
		ephysics::CollisionBody* m_triangleBody;
		ephysics::CollisionBody* m_concaveMeshBody;
		ephysics::CollisionBody* m_heightFieldBody;
		// etk::Transform3D
		etk::Transform3D m_bodyTransform;
		etk::Transform3D m_shapeTransform;
		etk::Transform3D m_localShapeToWorld;
		etk::Transform3D m_localShape2ToWorld;
		// Collision shapes
		ephysics::BoxShape* m_boxShape;
		ephysics::SphereShape* m_sphereShape;
		ephysics::CapsuleShape* m_capsuleShape;
		ephysics::ConeShape* m_coneShape;
		ephysics::ConvexMeshShape* m_convexMeshShape;
		ephysics::ConvexMeshShape* m_convexMeshShapeEdgesInfo;
		ephysics::CylinderShape* m_cylinderShape;
		ephysics::TriangleShape* m_triangleShape;
		ephysics::ConcaveShape* m_concaveMeshShape;
		ephysics::HeightFieldShape* m_heightFieldShape;
		// Proxy Shapes
		ephysics::ProxyShape* m_boxProxyShape;
		ephysics::ProxyShape* m_sphereProxyShape;
		ephysics::ProxyShape* m_capsuleProxyShape;
		ephysics::ProxyShape* m_coneProxyShape;
		ephysics::ProxyShape* m_convexMeshProxyShape;
		ephysics::ProxyShape* m_convexMeshProxyShapeEdgesInfo;
		ephysics::ProxyShape* m_cylinderProxyShape;
		ephysics::ProxyShape* m_compoundSphereProxyShape;
		ephysics::ProxyShape* m_compoundCylinderProxyShape;
		ephysics::ProxyShape* m_triangleProxyShape;
		ephysics::ProxyShape* m_concaveMeshProxyShape;
		ephysics::ProxyShape* m_heightFieldProxyShape;
		// Triangle meshes
		ephysics::TriangleMesh m_concaveTriangleMesh;
		etk::Vector<vec3> m_concaveMeshVertices;
		etk::Vector<uint32_t> m_concaveMeshIndices;
		ephysics::TriangleVertexArray* m_concaveMeshVertexArray;
		float m_heightFieldData[100];
	public :
		TestRaycast() {
			epsilon = float(0.0001);
			// Create the world
			m_world = ETK_NEW(ephysics::CollisionWorld);
			// Body transform
			vec3 position(-3, 2, 7);
			etk::Quaternion orientation(M_PI / 5, M_PI / 6, M_PI / 7, 1.0f);
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
			m_triangleBody = m_world->createCollisionBody(m_bodyTransform);
			m_concaveMeshBody = m_world->createCollisionBody(m_bodyTransform);
			m_heightFieldBody = m_world->createCollisionBody(m_bodyTransform);

			// Collision shape transform
			vec3 shapePosition(1, -4, -3);
			etk::Quaternion shapeOrientation(3 * M_PI / 6 , -M_PI / 8, M_PI / 3, 1.0f);
			m_shapeTransform = etk::Transform3D(shapePosition, shapeOrientation);

			// Compute the the transform from a local shape point to world-space
			m_localShapeToWorld = m_bodyTransform * m_shapeTransform;

			// Create collision shapes
			m_boxShape = ETK_NEW(ephysics::BoxShape, vec3(2, 3, 4), 0);
			m_boxProxyShape = m_boxBody->addCollisionShape(m_boxShape, m_shapeTransform);

			m_sphereShape = ETK_NEW(ephysics::SphereShape, 3);
			m_sphereProxyShape = m_sphereBody->addCollisionShape(m_sphereShape, m_shapeTransform);

			const vec3 triangleVertex1(100, 100, 0);
			const vec3 triangleVertex2(105, 100, 0);
			const vec3 triangleVertex3(100, 103, 0);
			m_triangleShape = ETK_NEW(ephysics::TriangleShape, triangleVertex1, triangleVertex2, triangleVertex3);
			m_triangleProxyShape = m_triangleBody->addCollisionShape(m_triangleShape, m_shapeTransform);

			m_capsuleShape = ETK_NEW(ephysics::CapsuleShape, 2, 5);
			m_capsuleProxyShape = m_capsuleBody->addCollisionShape(m_capsuleShape, m_shapeTransform);

			m_coneShape = ETK_NEW(ephysics::ConeShape, 2, 6, 0);
			m_coneProxyShape = m_coneBody->addCollisionShape(m_coneShape, m_shapeTransform);

			// Box of dimension (2, 3, 4)
			m_convexMeshShape = ETK_NEW(ephysics::ConvexMeshShape, 0.0f);
			m_convexMeshShape->addVertex(vec3(-2, -3, -4));
			m_convexMeshShape->addVertex(vec3(2, -3, -4));
			m_convexMeshShape->addVertex(vec3(2, -3, 4));
			m_convexMeshShape->addVertex(vec3(-2, -3, 4));
			m_convexMeshShape->addVertex(vec3(-2, 3, -4));
			m_convexMeshShape->addVertex(vec3(2, 3, -4));
			m_convexMeshShape->addVertex(vec3(2, 3, 4));
			m_convexMeshShape->addVertex(vec3(-2, 3, 4));
			m_convexMeshProxyShape = m_convexMeshBody->addCollisionShape(m_convexMeshShape, m_shapeTransform);

			m_convexMeshShapeEdgesInfo = ETK_NEW(ephysics::ConvexMeshShape, 0.0f);
			m_convexMeshShapeEdgesInfo->addVertex(vec3(-2, -3, -4));
			m_convexMeshShapeEdgesInfo->addVertex(vec3(2, -3, -4));
			m_convexMeshShapeEdgesInfo->addVertex(vec3(2, -3, 4));
			m_convexMeshShapeEdgesInfo->addVertex(vec3(-2, -3, 4));
			m_convexMeshShapeEdgesInfo->addVertex(vec3(-2, 3, -4));
			m_convexMeshShapeEdgesInfo->addVertex(vec3(2, 3, -4));
			m_convexMeshShapeEdgesInfo->addVertex(vec3(2, 3, 4));
			m_convexMeshShapeEdgesInfo->addVertex(vec3(-2, 3, 4));
			m_convexMeshShapeEdgesInfo->addEdge(0, 1);
			m_convexMeshShapeEdgesInfo->addEdge(1, 2);
			m_convexMeshShapeEdgesInfo->addEdge(2, 3);
			m_convexMeshShapeEdgesInfo->addEdge(0, 3);
			m_convexMeshShapeEdgesInfo->addEdge(4, 5);
			m_convexMeshShapeEdgesInfo->addEdge(5, 6);
			m_convexMeshShapeEdgesInfo->addEdge(6, 7);
			m_convexMeshShapeEdgesInfo->addEdge(4, 7);
			m_convexMeshShapeEdgesInfo->addEdge(0, 4);
			m_convexMeshShapeEdgesInfo->addEdge(1, 5);
			m_convexMeshShapeEdgesInfo->addEdge(2, 6);
			m_convexMeshShapeEdgesInfo->addEdge(3, 7);
			m_convexMeshShapeEdgesInfo->setIsEdgesInformationUsed(true);
			m_convexMeshProxyShapeEdgesInfo = m_convexMeshBodyEdgesInfo->addCollisionShape(
																	 m_convexMeshShapeEdgesInfo,
																	 m_shapeTransform);

			m_cylinderShape = ETK_NEW(ephysics::CylinderShape, 2, 5, 0);
			m_cylinderProxyShape = m_cylinderBody->addCollisionShape(m_cylinderShape, m_shapeTransform);

			// Compound shape is a cylinder and a sphere
			vec3 positionShape2(vec3(4, 2, -3));
			etk::Quaternion orientationShape2(-3 *M_PI / 8, 1.5 * M_PI/ 3, M_PI / 13, 1.0f);
			etk::Transform3D shapeTransform2(positionShape2, orientationShape2);
			m_localShape2ToWorld = m_bodyTransform * shapeTransform2;
			m_compoundCylinderProxyShape = m_compoundBody->addCollisionShape(m_cylinderShape, m_shapeTransform);
			m_compoundSphereProxyShape = m_compoundBody->addCollisionShape(m_sphereShape, shapeTransform2);

			// Concave Mesh shape
			m_concaveMeshVertices.pushBack(vec3(-2, -3, -4));
			m_concaveMeshVertices.pushBack(vec3(2, -3, -4));
			m_concaveMeshVertices.pushBack(vec3(2, -3, 4));
			m_concaveMeshVertices.pushBack(vec3(-2, -3, 4));
			m_concaveMeshVertices.pushBack(vec3(-2, 3, -4));
			m_concaveMeshVertices.pushBack(vec3(2, 3, -4));
			m_concaveMeshVertices.pushBack(vec3(2, 3, 4));
			m_concaveMeshVertices.pushBack(vec3(-2, 3, 4));

			m_concaveMeshIndices.pushBack(0); m_concaveMeshIndices.pushBack(1); m_concaveMeshIndices.pushBack(2);
			m_concaveMeshIndices.pushBack(0); m_concaveMeshIndices.pushBack(2); m_concaveMeshIndices.pushBack(3);
			m_concaveMeshIndices.pushBack(1); m_concaveMeshIndices.pushBack(5); m_concaveMeshIndices.pushBack(2);
			m_concaveMeshIndices.pushBack(2); m_concaveMeshIndices.pushBack(5); m_concaveMeshIndices.pushBack(6);
			m_concaveMeshIndices.pushBack(2); m_concaveMeshIndices.pushBack(7); m_concaveMeshIndices.pushBack(3);
			m_concaveMeshIndices.pushBack(2); m_concaveMeshIndices.pushBack(6); m_concaveMeshIndices.pushBack(7);
			m_concaveMeshIndices.pushBack(0); m_concaveMeshIndices.pushBack(3); m_concaveMeshIndices.pushBack(4);
			m_concaveMeshIndices.pushBack(3); m_concaveMeshIndices.pushBack(7); m_concaveMeshIndices.pushBack(4);
			m_concaveMeshIndices.pushBack(0); m_concaveMeshIndices.pushBack(4); m_concaveMeshIndices.pushBack(1);
			m_concaveMeshIndices.pushBack(1); m_concaveMeshIndices.pushBack(4); m_concaveMeshIndices.pushBack(5);
			m_concaveMeshIndices.pushBack(5); m_concaveMeshIndices.pushBack(7); m_concaveMeshIndices.pushBack(6);
			m_concaveMeshIndices.pushBack(4); m_concaveMeshIndices.pushBack(7); m_concaveMeshIndices.pushBack(5);
			m_concaveMeshVertexArray = ETK_NEW(ephysics::TriangleVertexArray, m_concaveMeshVertices, m_concaveMeshIndices);


			// Add the triangle vertex array of the subpart to the triangle mesh
			m_concaveTriangleMesh.addSubpart(m_concaveMeshVertexArray);
			m_concaveMeshShape = ETK_NEW(ephysics::ConcaveMeshShape, &m_concaveTriangleMesh);
			m_concaveMeshProxyShape = m_concaveMeshBody->addCollisionShape(m_concaveMeshShape, m_shapeTransform);


			// Heightfield shape (plane height field at height=4)
			for (int32_t i=0; i<100; i++) {
				m_heightFieldData[i] = 4;
			}
			m_heightFieldShape = ETK_NEW(ephysics::HeightFieldShape, 10, 10, 0, 4, m_heightFieldData, ephysics::HeightFieldShape::HEIGHT_FLOAT_TYPE);
			m_heightFieldProxyShape = m_heightFieldBody->addCollisionShape(m_heightFieldShape, m_shapeTransform);

			// Assign proxy shapes to the different categories
			m_boxProxyShape->setCollisionCategoryBits(CATEGORY1);
			m_sphereProxyShape->setCollisionCategoryBits(CATEGORY1);
			m_capsuleProxyShape->setCollisionCategoryBits(CATEGORY1);
			m_coneProxyShape->setCollisionCategoryBits(CATEGORY2);
			m_convexMeshProxyShape->setCollisionCategoryBits(CATEGORY2);
			m_convexMeshProxyShapeEdgesInfo->setCollisionCategoryBits(CATEGORY2);
			m_cylinderProxyShape->setCollisionCategoryBits(CATEGORY2);
			m_compoundSphereProxyShape->setCollisionCategoryBits(CATEGORY2);
			m_compoundCylinderProxyShape->setCollisionCategoryBits(CATEGORY2);
			m_triangleProxyShape->setCollisionCategoryBits(CATEGORY1);
			m_concaveMeshProxyShape->setCollisionCategoryBits(CATEGORY2);
			m_heightFieldProxyShape->setCollisionCategoryBits(CATEGORY2);
		}

		/// Destructor
		~TestRaycast() {
			ETK_DELETE(ephysics::BoxShape, m_boxShape);
			ETK_DELETE(ephysics::SphereShape, m_sphereShape);
			ETK_DELETE(ephysics::CapsuleShape, m_capsuleShape);
			ETK_DELETE(ephysics::ConeShape, m_coneShape);
			ETK_DELETE(ephysics::ConvexMeshShape, m_convexMeshShape);
			ETK_DELETE(ephysics::ConvexMeshShape, m_convexMeshShapeEdgesInfo);
			ETK_DELETE(ephysics::CylinderShape, m_cylinderShape);
			ETK_DELETE(ephysics::TriangleShape, m_triangleShape);
			ETK_DELETE(ephysics::ConcaveShape, m_concaveMeshShape);
			ETK_DELETE(ephysics::HeightFieldShape, m_heightFieldShape);
			ETK_DELETE(ephysics::TriangleVertexArray, m_concaveMeshVertexArray);
			ETK_DELETE(ephysics::CollisionWorld, m_world);
		}
};

TEST(TestRay, box) {
	TestRaycast tmp;
	// ----- Test feedback data ----- //
	vec3 point1 = tmp.m_localShapeToWorld * vec3(1 , 2, 10);
	vec3 point2 = tmp.m_localShapeToWorld * vec3(1, 2, -20);
	ephysics::Ray ray(point1, point2);
	vec3 hitPoint = tmp.m_localShapeToWorld * vec3(1, 2, 4);

	tmp.m_callback.shapeToTest = tmp.m_boxProxyShape;

	// CollisionWorld::raycast()
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	EXPECT_EQ(true, tmp.m_callback.raycastInfo.body == tmp.m_boxBody);
	EXPECT_EQ(true, tmp.m_callback.raycastInfo.proxyShape == tmp.m_boxProxyShape);
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.hitFraction, float(0.2));
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.x(), hitPoint.x());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.y(), hitPoint.y());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.z(), hitPoint.z());

	// Correct category filter mask
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray, &tmp.m_callback, CATEGORY1);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	// Wrong category filter mask
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray, &tmp.m_callback, CATEGORY2);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	// CollisionBody::raycast()
	ephysics::RaycastInfo raycastInfo2;
	EXPECT_EQ(true, tmp.m_boxBody->raycast(ray, raycastInfo2));
	EXPECT_EQ(true, raycastInfo2.body == tmp.m_boxBody);
	EXPECT_EQ(true, raycastInfo2.proxyShape == tmp.m_boxProxyShape);
	EXPECT_FLOAT_EQ(raycastInfo2.hitFraction, float(0.2));
	EXPECT_FLOAT_EQ(raycastInfo2.worldPoint.x(), hitPoint.x());
	EXPECT_FLOAT_EQ(raycastInfo2.worldPoint.y(), hitPoint.y());
	EXPECT_FLOAT_EQ(raycastInfo2.worldPoint.z(), hitPoint.z());

	// ProxyCollisionShape::raycast()
	ephysics::RaycastInfo raycastInfo3;
	EXPECT_EQ(true, tmp.m_boxProxyShape->raycast(ray, raycastInfo3));
	EXPECT_EQ(true, raycastInfo3.body == tmp.m_boxBody);
	EXPECT_EQ(true, raycastInfo3.proxyShape == tmp.m_boxProxyShape);
	EXPECT_FLOAT_EQ(raycastInfo3.hitFraction, float(0.2));
	EXPECT_FLOAT_EQ(raycastInfo3.worldPoint.x(), hitPoint.x());
	EXPECT_FLOAT_EQ(raycastInfo3.worldPoint.y(), hitPoint.y());
	EXPECT_FLOAT_EQ(raycastInfo3.worldPoint.z(), hitPoint.z());

	ephysics::Ray ray1(tmp.m_localShapeToWorld * vec3(0, 0, 0), tmp.m_localShapeToWorld * vec3(5, 7, -1));
	ephysics::Ray ray2(tmp.m_localShapeToWorld * vec3(5, 11, 7), tmp.m_localShapeToWorld * vec3(17, 29, 28));
	ephysics::Ray ray3(tmp.m_localShapeToWorld * vec3(1, 2, 3), tmp.m_localShapeToWorld * vec3(-11, 2, 24));
	ephysics::Ray ray4(tmp.m_localShapeToWorld * vec3(10, 10, 10), tmp.m_localShapeToWorld * vec3(22, 28, 31));
	ephysics::Ray ray5(tmp.m_localShapeToWorld * vec3(3, 1, -5), tmp.m_localShapeToWorld * vec3(-30, 1, -5));
	ephysics::Ray ray6(tmp.m_localShapeToWorld * vec3(4, 4, 1), tmp.m_localShapeToWorld * vec3(4, -20, 1));
	ephysics::Ray ray7(tmp.m_localShapeToWorld * vec3(1, -4, 5), tmp.m_localShapeToWorld * vec3(1, -4, -20));
	ephysics::Ray ray8(tmp.m_localShapeToWorld * vec3(-4, 4, 0), tmp.m_localShapeToWorld * vec3(20, 4, 0));
	ephysics::Ray ray9(tmp.m_localShapeToWorld * vec3(0, -4, -7), tmp.m_localShapeToWorld * vec3(0, 50, -7));
	ephysics::Ray ray10(tmp.m_localShapeToWorld * vec3(-3, 0, -6), tmp.m_localShapeToWorld * vec3(-3, 0, 20));
	ephysics::Ray ray11(tmp.m_localShapeToWorld * vec3(3, 1, 2), tmp.m_localShapeToWorld * vec3(-20, 1, 2));
	ephysics::Ray ray12(tmp.m_localShapeToWorld * vec3(1, 4, -1), tmp.m_localShapeToWorld * vec3(1, -20, -1));
	ephysics::Ray ray13(tmp.m_localShapeToWorld * vec3(-1, 2, 5), tmp.m_localShapeToWorld * vec3(-1, 2, -20));
	ephysics::Ray ray14(tmp.m_localShapeToWorld * vec3(-3, 2, -2), tmp.m_localShapeToWorld * vec3(20, 2, -2));
	ephysics::Ray ray15(tmp.m_localShapeToWorld * vec3(0, -4, 1), tmp.m_localShapeToWorld * vec3(0, 20, 1));
	ephysics::Ray ray16(tmp.m_localShapeToWorld * vec3(-1, 2, -5), tmp.m_localShapeToWorld * vec3(-1, 2, 20));

	// ----- Test raycast miss ----- //
	EXPECT_EQ(false, tmp.m_boxBody->raycast(ray1, raycastInfo3));
	EXPECT_EQ(false, tmp.m_boxProxyShape->raycast(ray1, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray1, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray1.point1, ray1.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray1.point1, ray1.point2, float(100.0)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_boxBody->raycast(ray2, raycastInfo3));
	EXPECT_EQ(false, tmp.m_boxProxyShape->raycast(ray2, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray2, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_boxBody->raycast(ray3, raycastInfo3));
	EXPECT_EQ(false, tmp.m_boxProxyShape->raycast(ray3, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray3, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_boxBody->raycast(ray4, raycastInfo3));
	EXPECT_EQ(false, tmp.m_boxProxyShape->raycast(ray4, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray4, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_boxBody->raycast(ray5, raycastInfo3));
	EXPECT_EQ(false, tmp.m_boxProxyShape->raycast(ray5, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray5, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_boxBody->raycast(ray6, raycastInfo3));
	EXPECT_EQ(false, tmp.m_boxProxyShape->raycast(ray6, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray6, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_boxBody->raycast(ray7, raycastInfo3));
	EXPECT_EQ(false, tmp.m_boxProxyShape->raycast(ray7, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray7, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_boxBody->raycast(ray8, raycastInfo3));
	EXPECT_EQ(false, tmp.m_boxProxyShape->raycast(ray8, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray8, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_boxBody->raycast(ray9, raycastInfo3));
	EXPECT_EQ(false, tmp.m_boxProxyShape->raycast(ray9, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray9, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_boxBody->raycast(ray10, raycastInfo3));
	EXPECT_EQ(false, tmp.m_boxProxyShape->raycast(ray10, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray10, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray11.point1, ray11.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray12.point1, ray12.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray13.point1, ray13.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray14.point1, ray14.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray15.point1, ray15.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray16.point1, ray16.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	// ----- Test raycast hits ----- //
	EXPECT_EQ(true, tmp.m_boxBody->raycast(ray11, raycastInfo3));
	EXPECT_EQ(true, tmp.m_boxProxyShape->raycast(ray11, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray11, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray11.point1, ray11.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_boxBody->raycast(ray12, raycastInfo3));
	EXPECT_EQ(true, tmp.m_boxProxyShape->raycast(ray12, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray12, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray12.point1, ray12.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_boxBody->raycast(ray13, raycastInfo3));
	EXPECT_EQ(true, tmp.m_boxProxyShape->raycast(ray13, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray13, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray13.point1, ray13.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_boxBody->raycast(ray14, raycastInfo3));
	EXPECT_EQ(true, tmp.m_boxProxyShape->raycast(ray14, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray14, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray14.point1, ray14.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_boxBody->raycast(ray15, raycastInfo3));
	EXPECT_EQ(true, tmp.m_boxProxyShape->raycast(ray15, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray15, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray15.point1, ray15.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_boxBody->raycast(ray16, raycastInfo3));
	EXPECT_EQ(true, tmp.m_boxProxyShape->raycast(ray16, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray16, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray16.point1, ray16.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
}

TEST(TestRay, sphere) {
	TestRaycast tmp;
	// ----- Test feedback data ----- //
	vec3 point1 = tmp.m_localShapeToWorld * vec3(-5 , 0, 0);
	vec3 point2 = tmp.m_localShapeToWorld * vec3(5, 0, 0);
	ephysics::Ray ray(point1, point2);
	vec3 hitPoint = tmp.m_localShapeToWorld * vec3(-3, 0, 0);

	tmp.m_callback.shapeToTest = tmp.m_sphereProxyShape;

	// CollisionWorld::raycast()
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	EXPECT_EQ(true, tmp.m_callback.raycastInfo.body == tmp.m_sphereBody);
	EXPECT_EQ(true, tmp.m_callback.raycastInfo.proxyShape == tmp.m_sphereProxyShape);
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.hitFraction, 0.2);
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.x(), hitPoint.x());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.y(), hitPoint.y());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.z(), hitPoint.z());

	// Correct category filter mask
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray, &tmp.m_callback, CATEGORY1);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	// Wrong category filter mask
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray, &tmp.m_callback, CATEGORY2);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	// CollisionBody::raycast()
	ephysics::RaycastInfo raycastInfo2;
	EXPECT_EQ(true, tmp.m_sphereBody->raycast(ray, raycastInfo2));
	EXPECT_EQ(true, raycastInfo2.body == tmp.m_sphereBody);
	EXPECT_EQ(true, raycastInfo2.proxyShape == tmp.m_sphereProxyShape);
	EXPECT_FLOAT_EQ(raycastInfo2.hitFraction, 0.2);
	EXPECT_FLOAT_EQ(raycastInfo2.worldPoint.x(), hitPoint.x());
	EXPECT_FLOAT_EQ(raycastInfo2.worldPoint.y(), hitPoint.y());
	EXPECT_FLOAT_EQ(raycastInfo2.worldPoint.z(), hitPoint.z());

	// ProxyCollisionShape::raycast()
	ephysics::RaycastInfo raycastInfo3;
	EXPECT_EQ(true, tmp.m_sphereProxyShape->raycast(ray, raycastInfo3));
	EXPECT_EQ(true, raycastInfo3.body == tmp.m_sphereBody);
	EXPECT_EQ(true, raycastInfo3.proxyShape == tmp.m_sphereProxyShape);
	EXPECT_FLOAT_EQ(raycastInfo3.hitFraction, 0.2);
	EXPECT_FLOAT_EQ(raycastInfo3.worldPoint.x(), hitPoint.x());
	EXPECT_FLOAT_EQ(raycastInfo3.worldPoint.y(), hitPoint.y());
	EXPECT_FLOAT_EQ(raycastInfo3.worldPoint.z(), hitPoint.z());

	ephysics::Ray ray1(tmp.m_localShapeToWorld * vec3(0, 0, 0), tmp.m_localShapeToWorld * vec3(5, 7, -1));
	ephysics::Ray ray2(tmp.m_localShapeToWorld * vec3(5, 11, 7), tmp.m_localShapeToWorld * vec3(4, 6, 7));
	ephysics::Ray ray3(tmp.m_localShapeToWorld * vec3(1, 2, 2), tmp.m_localShapeToWorld * vec3(-4, 0, 7));
	ephysics::Ray ray4(tmp.m_localShapeToWorld * vec3(10, 10, 10), tmp.m_localShapeToWorld * vec3(4, 6, 7));
	ephysics::Ray ray5(tmp.m_localShapeToWorld * vec3(4, 1, -5), tmp.m_localShapeToWorld * vec3(-30, 1, -5));
	ephysics::Ray ray6(tmp.m_localShapeToWorld * vec3(4, 4, 1), tmp.m_localShapeToWorld * vec3(4, -30, 1));
	ephysics::Ray ray7(tmp.m_localShapeToWorld * vec3(1, -4, 5), tmp.m_localShapeToWorld * vec3(1, -4, -30));
	ephysics::Ray ray8(tmp.m_localShapeToWorld * vec3(-4, 4, 0), tmp.m_localShapeToWorld * vec3(30, 4, 0));
	ephysics::Ray ray9(tmp.m_localShapeToWorld * vec3(0, -4, -4), tmp.m_localShapeToWorld * vec3(0, 30, -4));
	ephysics::Ray ray10(tmp.m_localShapeToWorld * vec3(-4, 0, -6), tmp.m_localShapeToWorld * vec3(-4, 0, 30));
	ephysics::Ray ray11(tmp.m_localShapeToWorld * vec3(4, 1, 2), tmp.m_localShapeToWorld * vec3(-30, 1, 2));
	ephysics::Ray ray12(tmp.m_localShapeToWorld * vec3(1, 4, -1), tmp.m_localShapeToWorld * vec3(1, -30, -1));
	ephysics::Ray ray13(tmp.m_localShapeToWorld * vec3(-1, 2, 5), tmp.m_localShapeToWorld * vec3(-1, 2, -30));
	ephysics::Ray ray14(tmp.m_localShapeToWorld * vec3(-5, 2, -2), tmp.m_localShapeToWorld * vec3(30, 2, -2));
	ephysics::Ray ray15(tmp.m_localShapeToWorld * vec3(0, -4, 1), tmp.m_localShapeToWorld * vec3(0, 30, 1));
	ephysics::Ray ray16(tmp.m_localShapeToWorld * vec3(-1, 2, -11), tmp.m_localShapeToWorld * vec3(-1, 2, 30));

	// ----- Test raycast miss ----- //
	EXPECT_EQ(false, tmp.m_sphereBody->raycast(ray1, raycastInfo3));
	EXPECT_EQ(false, tmp.m_sphereProxyShape->raycast(ray1, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray1, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray1.point1, ray1.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray1.point1, ray1.point2, float(100.0)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_sphereBody->raycast(ray2, raycastInfo3));
	EXPECT_EQ(false, tmp.m_sphereProxyShape->raycast(ray2, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray2, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_sphereBody->raycast(ray3, raycastInfo3));
	EXPECT_EQ(false, tmp.m_sphereProxyShape->raycast(ray3, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray3, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_sphereBody->raycast(ray4, raycastInfo3));
	EXPECT_EQ(false, tmp.m_sphereProxyShape->raycast(ray4, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray4, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_sphereBody->raycast(ray5, raycastInfo3));
	EXPECT_EQ(false, tmp.m_sphereProxyShape->raycast(ray5, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray5, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_sphereBody->raycast(ray6, raycastInfo3));
	EXPECT_EQ(false, tmp.m_sphereProxyShape->raycast(ray6, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray6, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_sphereBody->raycast(ray7, raycastInfo3));
	EXPECT_EQ(false, tmp.m_sphereProxyShape->raycast(ray7, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray7, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_sphereBody->raycast(ray8, raycastInfo3));
	EXPECT_EQ(false, tmp.m_sphereProxyShape->raycast(ray8, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray8, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_sphereBody->raycast(ray9, raycastInfo3));
	EXPECT_EQ(false, tmp.m_sphereProxyShape->raycast(ray9, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray9, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_sphereBody->raycast(ray10, raycastInfo3));
	EXPECT_EQ(false, tmp.m_sphereProxyShape->raycast(ray10, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray10, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray11.point1, ray11.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray12.point1, ray12.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray13.point1, ray13.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray14.point1, ray14.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray15.point1, ray15.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray16.point1, ray16.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	// ----- Test raycast hits ----- //
	EXPECT_EQ(true, tmp.m_sphereBody->raycast(ray11, raycastInfo3));
	EXPECT_EQ(true, tmp.m_sphereProxyShape->raycast(ray11, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray11, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray11.point1, ray11.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_sphereBody->raycast(ray12, raycastInfo3));
	EXPECT_EQ(true, tmp.m_sphereProxyShape->raycast(ray12, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray12, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray12.point1, ray12.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_sphereBody->raycast(ray13, raycastInfo3));
	EXPECT_EQ(true, tmp.m_sphereProxyShape->raycast(ray13, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray13, &tmp.m_callback);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray13.point1, ray13.point2, float(0.8)), &tmp.m_callback);

	EXPECT_EQ(true, tmp.m_sphereBody->raycast(ray14, raycastInfo3));
	EXPECT_EQ(true, tmp.m_sphereProxyShape->raycast(ray14, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray14, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray14.point1, ray14.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_sphereBody->raycast(ray15, raycastInfo3));
	EXPECT_EQ(true, tmp.m_sphereProxyShape->raycast(ray15, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray15, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray15.point1, ray15.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_sphereBody->raycast(ray16, raycastInfo3));
	EXPECT_EQ(true, tmp.m_sphereProxyShape->raycast(ray16, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray16, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray16.point1, ray16.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
}

TEST(TestRay, capsule) {
	TestRaycast tmp;
	// ----- Test feedback data ----- //
	vec3 point1A = tmp.m_localShapeToWorld * vec3(4 , 1, 0);
	vec3 point1B = tmp.m_localShapeToWorld * vec3(-6, 1, 0);
	ephysics::Ray ray(point1A, point1B);
	vec3 hitPoint = tmp.m_localShapeToWorld * vec3(2, 1, 0);

	vec3 point2A = tmp.m_localShapeToWorld * vec3(0 , 6.5, 0);
	vec3 point2B = tmp.m_localShapeToWorld * vec3(0, -3.5, 0);
	ephysics::Ray rayTop(point2A, point2B);
	vec3 hitPointTop = tmp.m_localShapeToWorld * vec3(0, float(4.5), 0);

	vec3 point3A = tmp.m_localShapeToWorld * vec3(0 , -6.5, 0);
	vec3 point3B = tmp.m_localShapeToWorld * vec3(0, 3.5, 0);
	ephysics::Ray rayBottom(point3A, point3B);
	vec3 hitPointBottom = tmp.m_localShapeToWorld * vec3(0, float(-4.5), 0);

	tmp.m_callback.shapeToTest = tmp.m_capsuleProxyShape;

	// CollisionWorld::raycast()
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	EXPECT_EQ(true, tmp.m_callback.raycastInfo.body == tmp.m_capsuleBody);
	EXPECT_EQ(true, tmp.m_callback.raycastInfo.proxyShape == tmp.m_capsuleProxyShape);
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.hitFraction, float(0.2));
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.x(), hitPoint.x());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.y(), hitPoint.y());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.z(), hitPoint.z());

	// Correct category filter mask
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray, &tmp.m_callback, CATEGORY1);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	// Wrong category filter mask
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray, &tmp.m_callback, CATEGORY2);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	// CollisionBody::raycast()
	ephysics::RaycastInfo raycastInfo2;
	EXPECT_EQ(true, tmp.m_capsuleBody->raycast(ray, raycastInfo2));
	EXPECT_EQ(true, raycastInfo2.body == tmp.m_capsuleBody);
	EXPECT_EQ(true, raycastInfo2.proxyShape == tmp.m_capsuleProxyShape);
	EXPECT_FLOAT_EQ(raycastInfo2.hitFraction, float(0.2));
	EXPECT_FLOAT_EQ(raycastInfo2.worldPoint.x(), hitPoint.x());
	EXPECT_FLOAT_EQ(raycastInfo2.worldPoint.y(), hitPoint.y());
	EXPECT_FLOAT_EQ(raycastInfo2.worldPoint.z(), hitPoint.z());

	// ProxyCollisionShape::raycast()
	ephysics::RaycastInfo raycastInfo3;
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->raycast(ray, raycastInfo3));
	EXPECT_EQ(true, raycastInfo3.body == tmp.m_capsuleBody);
	EXPECT_EQ(true, raycastInfo3.proxyShape == tmp.m_capsuleProxyShape);
	EXPECT_FLOAT_EQ(raycastInfo3.hitFraction, float(0.2));
	EXPECT_FLOAT_EQ(raycastInfo3.worldPoint.x(), hitPoint.x());
	EXPECT_FLOAT_EQ(raycastInfo3.worldPoint.y(), hitPoint.y());
	EXPECT_FLOAT_EQ(raycastInfo3.worldPoint.z(), hitPoint.z());

	ephysics::RaycastInfo raycastInfo4;
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->raycast(rayTop, raycastInfo4));
	EXPECT_EQ(true, raycastInfo4.body == tmp.m_capsuleBody);
	EXPECT_EQ(true, raycastInfo4.proxyShape == tmp.m_capsuleProxyShape);
	EXPECT_FLOAT_EQ(raycastInfo4.hitFraction, float(0.2));
	EXPECT_FLOAT_EQ(raycastInfo4.worldPoint.x(), hitPointTop.x());
	EXPECT_FLOAT_EQ(raycastInfo4.worldPoint.y(), hitPointTop.y());
	EXPECT_FLOAT_EQ(raycastInfo4.worldPoint.z(), hitPointTop.z());

	// ProxyCollisionShape::raycast()
	ephysics::RaycastInfo raycastInfo5;
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->raycast(rayBottom, raycastInfo5));
	EXPECT_EQ(true, raycastInfo5.body == tmp.m_capsuleBody);
	EXPECT_EQ(true, raycastInfo5.proxyShape == tmp.m_capsuleProxyShape);
	EXPECT_FLOAT_EQ(raycastInfo5.hitFraction, float(0.2));
	EXPECT_FLOAT_EQ(raycastInfo5.worldPoint.x(), hitPointBottom.x());
	EXPECT_FLOAT_EQ(raycastInfo5.worldPoint.y(), hitPointBottom.y());
	EXPECT_FLOAT_EQ(raycastInfo5.worldPoint.z(), hitPointBottom.z());

	ephysics::Ray ray1(tmp.m_localShapeToWorld * vec3(0, 0, 0), tmp.m_localShapeToWorld * vec3(5, 7, -1));
	ephysics::Ray ray2(tmp.m_localShapeToWorld * vec3(5, 11, 7), tmp.m_localShapeToWorld * vec3(9, 17, 14));
	ephysics::Ray ray3(tmp.m_localShapeToWorld * vec3(1, 3, -1), tmp.m_localShapeToWorld * vec3(-3, 3, 6));
	ephysics::Ray ray4(tmp.m_localShapeToWorld * vec3(10, 10, 10), tmp.m_localShapeToWorld * vec3(14, 16, 17));
	ephysics::Ray ray5(tmp.m_localShapeToWorld * vec3(4, 1, -5), tmp.m_localShapeToWorld * vec3(1, 1, -5));
	ephysics::Ray ray6(tmp.m_localShapeToWorld * vec3(4, 9, 1), tmp.m_localShapeToWorld * vec3(4, 7, 1));
	ephysics::Ray ray7(tmp.m_localShapeToWorld * vec3(1, -9, 5), tmp.m_localShapeToWorld * vec3(1, -9, 3));
	ephysics::Ray ray8(tmp.m_localShapeToWorld * vec3(-4, 9, 0), tmp.m_localShapeToWorld * vec3(-3, 9, 0));
	ephysics::Ray ray9(tmp.m_localShapeToWorld * vec3(0, -9, -4), tmp.m_localShapeToWorld * vec3(0, -4, -4));
	ephysics::Ray ray10(tmp.m_localShapeToWorld * vec3(-4, 0, -6), tmp.m_localShapeToWorld * vec3(-4, 0, 2));
	ephysics::Ray ray11(tmp.m_localShapeToWorld * vec3(4, 1, 1.5), tmp.m_localShapeToWorld * vec3(-30, 1, 1.5));
	ephysics::Ray ray12(tmp.m_localShapeToWorld * vec3(1, 9, -1), tmp.m_localShapeToWorld * vec3(1, -30, -1));
	ephysics::Ray ray13(tmp.m_localShapeToWorld * vec3(-1, 2, 3), tmp.m_localShapeToWorld * vec3(-1, 2, -30));
	ephysics::Ray ray14(tmp.m_localShapeToWorld * vec3(-3, 2, -1.7), tmp.m_localShapeToWorld * vec3(30, 2, -1.7));
	ephysics::Ray ray15(tmp.m_localShapeToWorld * vec3(0, -9, 1), tmp.m_localShapeToWorld * vec3(0, 30, 1));
	ephysics::Ray ray16(tmp.m_localShapeToWorld * vec3(-1, 2, -7), tmp.m_localShapeToWorld * vec3(-1, 2, 30));

	// ----- Test raycast miss ----- //
	EXPECT_EQ(false, tmp.m_capsuleBody->raycast(ray1, raycastInfo3));
	EXPECT_EQ(false, tmp.m_capsuleProxyShape->raycast(ray1, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray1, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray1.point1, ray1.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray1.point1, ray1.point2, float(100.0)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_capsuleBody->raycast(ray2, raycastInfo3));
	EXPECT_EQ(false, tmp.m_capsuleProxyShape->raycast(ray2, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray2, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_capsuleBody->raycast(ray3, raycastInfo3));
	EXPECT_EQ(false, tmp.m_capsuleProxyShape->raycast(ray3, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray3, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_capsuleBody->raycast(ray4, raycastInfo3));
	EXPECT_EQ(false, tmp.m_capsuleProxyShape->raycast(ray4, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray4, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_capsuleBody->raycast(ray5, raycastInfo3));
	EXPECT_EQ(false, tmp.m_capsuleProxyShape->raycast(ray5, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray5, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_capsuleBody->raycast(ray6, raycastInfo3));
	EXPECT_EQ(false, tmp.m_capsuleProxyShape->raycast(ray6, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray6, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_capsuleBody->raycast(ray7, raycastInfo3));
	EXPECT_EQ(false, tmp.m_capsuleProxyShape->raycast(ray7, raycastInfo3));
	tmp.m_world->raycast(ray7, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_capsuleBody->raycast(ray8, raycastInfo3));
	EXPECT_EQ(false, tmp.m_capsuleProxyShape->raycast(ray8, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray8, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_capsuleBody->raycast(ray9, raycastInfo3));
	EXPECT_EQ(false, tmp.m_capsuleProxyShape->raycast(ray9, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray9, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_capsuleBody->raycast(ray10, raycastInfo3));
	EXPECT_EQ(false, tmp.m_capsuleProxyShape->raycast(ray10, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray10, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray11.point1, ray11.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray12.point1, ray12.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray13.point1, ray13.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray14.point1, ray14.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray15.point1, ray15.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray16.point1, ray16.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	// ----- Test raycast hits ----- //
	EXPECT_EQ(true, tmp.m_capsuleBody->raycast(ray11, raycastInfo3));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->raycast(ray11, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray11, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray11.point1, ray11.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_capsuleBody->raycast(ray12, raycastInfo3));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->raycast(ray12, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray12, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray12.point1, ray12.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_capsuleBody->raycast(ray13, raycastInfo3));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->raycast(ray13, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray13, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray13.point1, ray13.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_capsuleBody->raycast(ray14, raycastInfo3));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->raycast(ray14, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray14, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray14.point1, ray14.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_capsuleBody->raycast(ray15, raycastInfo3));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->raycast(ray15, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray15, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray15.point1, ray15.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_capsuleBody->raycast(ray16, raycastInfo3));
	EXPECT_EQ(true, tmp.m_capsuleProxyShape->raycast(ray16, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray16, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray16.point1, ray16.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
}

TEST(TestRay, triangle) {
	TestRaycast tmp;
	// ----- Test feedback data ----- //
	vec3 point1 = tmp.m_localShapeToWorld * vec3(101, 101, 400);
	vec3 point2 = tmp.m_localShapeToWorld * vec3(101, 101, -200);
	ephysics::Ray ray(point1, point2);			
	ephysics::Ray rayBackward(point2, point1);

	vec3 hitPoint = tmp.m_localShapeToWorld * vec3(101, 101, 0);
	vec3 hitNormal = tmp.m_localShapeToWorld.getOrientation() * vec3(0, 0, 1);
	hitNormal.normalize();
	tmp.m_callback.shapeToTest = tmp.m_triangleProxyShape;

	// CollisionWorld::raycast()
	tmp.m_callback.reset();
	tmp.m_triangleShape->setRaycastTestType(ephysics::FRONT);
	tmp.m_world->raycast(ray, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	EXPECT_EQ(true, tmp.m_callback.raycastInfo.body == tmp.m_triangleBody);
	EXPECT_EQ(true, tmp.m_callback.raycastInfo.proxyShape == tmp.m_triangleProxyShape);
	EXPECT_FLOAT_EQ_DELTA(tmp.m_callback.raycastInfo.hitFraction, 0.6666, 0.0001f);
	EXPECT_FLOAT_EQ_DELTA(tmp.m_callback.raycastInfo.worldPoint.x(), hitPoint.x(), 0.0001f);
	EXPECT_FLOAT_EQ_DELTA(tmp.m_callback.raycastInfo.worldPoint.y(), hitPoint.y(), 0.0001f);
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.z(), hitPoint.z());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldNormal.x(), hitNormal.x());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldNormal.y(), hitNormal.y());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldNormal.z(), hitNormal.z());

	tmp.m_callback.reset();
	tmp.m_triangleShape->setRaycastTestType(ephysics::BACK);
	tmp.m_world->raycast(rayBackward, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	EXPECT_EQ(true, tmp.m_callback.raycastInfo.body == tmp.m_triangleBody);
	EXPECT_EQ(true, tmp.m_callback.raycastInfo.proxyShape == tmp.m_triangleProxyShape);
	EXPECT_FLOAT_EQ_DELTA(tmp.m_callback.raycastInfo.hitFraction, 0.3333, 0.0001f);
	EXPECT_FLOAT_EQ_DELTA(tmp.m_callback.raycastInfo.worldPoint.x(), hitPoint.x(), 0.0001f);
	EXPECT_FLOAT_EQ_DELTA(tmp.m_callback.raycastInfo.worldPoint.y(), hitPoint.y(), 0.0001f);
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.z(), hitPoint.z());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldNormal.x(), -hitNormal.x());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldNormal.y(), -hitNormal.y());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldNormal.z(), -hitNormal.z());

	tmp.m_callback.reset();
	tmp.m_triangleShape->setRaycastTestType(ephysics::FRONT_AND_BACK);
	tmp.m_world->raycast(ray, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	EXPECT_EQ(true, tmp.m_callback.raycastInfo.body == tmp.m_triangleBody);
	EXPECT_EQ(true, tmp.m_callback.raycastInfo.proxyShape == tmp.m_triangleProxyShape);
	EXPECT_FLOAT_EQ_DELTA(tmp.m_callback.raycastInfo.hitFraction, 0.6666, 0.0001f);
	EXPECT_FLOAT_EQ_DELTA(tmp.m_callback.raycastInfo.worldPoint.x(), hitPoint.x(), 0.0001f);
	EXPECT_FLOAT_EQ_DELTA(tmp.m_callback.raycastInfo.worldPoint.y(), hitPoint.y(), 0.0001f);
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.z(), hitPoint.z());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldNormal.x(), hitNormal.x());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldNormal.y(), hitNormal.y());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldNormal.z(), hitNormal.z());

	tmp.m_callback.reset();
	tmp.m_triangleShape->setRaycastTestType(ephysics::FRONT_AND_BACK);
	tmp.m_world->raycast(rayBackward, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	EXPECT_EQ(true, tmp.m_callback.raycastInfo.body == tmp.m_triangleBody);
	EXPECT_EQ(true, tmp.m_callback.raycastInfo.proxyShape == tmp.m_triangleProxyShape);
	EXPECT_FLOAT_EQ_DELTA(tmp.m_callback.raycastInfo.hitFraction, 0.3333, 0.0001f);
	EXPECT_FLOAT_EQ_DELTA(tmp.m_callback.raycastInfo.worldPoint.x(), hitPoint.x(), 0.0001f);
	EXPECT_FLOAT_EQ_DELTA(tmp.m_callback.raycastInfo.worldPoint.y(), hitPoint.y(), 0.0001f);
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.z(), hitPoint.z());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldNormal.x(), -hitNormal.x());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldNormal.y(), -hitNormal.y());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldNormal.z(), -hitNormal.z());

	tmp.m_triangleShape->setRaycastTestType(ephysics::FRONT);

	// Correct category filter mask
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray, &tmp.m_callback, CATEGORY1);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	// Wrong category filter mask
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray, &tmp.m_callback, CATEGORY2);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	// CollisionBody::raycast()
	ephysics::RaycastInfo raycastInfo2;
	EXPECT_EQ(true, tmp.m_triangleBody->raycast(ray, raycastInfo2));
	EXPECT_EQ(true, raycastInfo2.body == tmp.m_triangleBody);
	EXPECT_EQ(true, raycastInfo2.proxyShape == tmp.m_triangleProxyShape);
	EXPECT_FLOAT_EQ_DELTA(raycastInfo2.hitFraction, 0.6666, 0.0001f);
	EXPECT_FLOAT_EQ_DELTA(raycastInfo2.worldPoint.x(), hitPoint.x(), 0.0001f);
	EXPECT_FLOAT_EQ_DELTA(raycastInfo2.worldPoint.y(), hitPoint.y(), 0.0001f);
	EXPECT_FLOAT_EQ(raycastInfo2.worldPoint.z(), hitPoint.z());

	// ProxyCollisionShape::raycast()
	ephysics::RaycastInfo raycastInfo3;
	EXPECT_EQ(true, tmp.m_triangleProxyShape->raycast(ray, raycastInfo3));
	EXPECT_EQ(true, raycastInfo3.body == tmp.m_triangleBody);
	EXPECT_EQ(true, raycastInfo3.proxyShape == tmp.m_triangleProxyShape);
	EXPECT_FLOAT_EQ_DELTA(raycastInfo3.hitFraction, 0.6666, 0.0001f);
	EXPECT_FLOAT_EQ_DELTA(raycastInfo3.worldPoint.x(), hitPoint.x(), 0.0001f);
	EXPECT_FLOAT_EQ_DELTA(raycastInfo3.worldPoint.y(), hitPoint.y(), 0.0001f);
	EXPECT_FLOAT_EQ(raycastInfo3.worldPoint.z(), hitPoint.z());

	ephysics::Ray ray1(tmp.m_localShapeToWorld * vec3(-10, 10, 4), tmp.m_localShapeToWorld * vec3(15, 6, -4));
	ephysics::Ray ray2(tmp.m_localShapeToWorld * vec3(102, 107, 5), tmp.m_localShapeToWorld * vec3(102, 107, -5));
	ephysics::Ray ray3(tmp.m_localShapeToWorld * vec3(106, 102, 6), tmp.m_localShapeToWorld * vec3(106, 102, -8));

	ephysics::Ray ray4(tmp.m_localShapeToWorld * vec3(100.2, 101, 5), tmp.m_localShapeToWorld * vec3(100.2, 101, -5));
	ephysics::Ray ray5(tmp.m_localShapeToWorld * vec3(100.5, 101.5, 4), tmp.m_localShapeToWorld * vec3(100.5, 101.5, -54));
	ephysics::Ray ray6(tmp.m_localShapeToWorld * vec3(102, 101, 1), tmp.m_localShapeToWorld * vec3(102, 102, -1));

	ephysics::Ray ray4Back(tmp.m_localShapeToWorld * vec3(100.2, 101, -5), tmp.m_localShapeToWorld * vec3(100.2, 101, 5));
	ephysics::Ray ray5Back(tmp.m_localShapeToWorld * vec3(100.5, 101.5, -54), tmp.m_localShapeToWorld * vec3(100.5, 101.5, 4));
	ephysics::Ray ray6Back(tmp.m_localShapeToWorld * vec3(102, 102, -1), tmp.m_localShapeToWorld * vec3(102, 101, 1));

	// ----- Test raycast miss ----- //
	EXPECT_EQ(false, tmp.m_triangleBody->raycast(ray1, raycastInfo3));
	EXPECT_EQ(false, tmp.m_triangleProxyShape->raycast(ray1, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray1, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray1.point1, ray1.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray1.point1, ray1.point2, float(100.0)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_triangleBody->raycast(ray2, raycastInfo3));
	EXPECT_EQ(false, tmp.m_triangleProxyShape->raycast(ray2, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray2, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_triangleBody->raycast(ray3, raycastInfo3));
	EXPECT_EQ(false, tmp.m_triangleProxyShape->raycast(ray3, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray3, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	// Test backward ray against front triangles (not hit should occur)
	tmp.m_triangleShape->setRaycastTestType(ephysics::FRONT);

	EXPECT_EQ(false, tmp.m_triangleBody->raycast(ray4Back, raycastInfo3));
	EXPECT_EQ(false, tmp.m_triangleProxyShape->raycast(ray4Back, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray4Back, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_triangleBody->raycast(ray5Back, raycastInfo3));
	EXPECT_EQ(false, tmp.m_triangleProxyShape->raycast(ray5Back, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray5Back, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_triangleBody->raycast(ray6Back, raycastInfo3));
	EXPECT_EQ(false, tmp.m_triangleProxyShape->raycast(ray6Back, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray6Back, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	// Test front ray against back triangles (not hit should occur)
	tmp.m_triangleShape->setRaycastTestType(ephysics::BACK);

	EXPECT_EQ(false, tmp.m_triangleBody->raycast(ray4, raycastInfo3));
	EXPECT_EQ(false, tmp.m_triangleProxyShape->raycast(ray4, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray4, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_triangleBody->raycast(ray5, raycastInfo3));
	EXPECT_EQ(false, tmp.m_triangleProxyShape->raycast(ray5, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray5, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_triangleBody->raycast(ray6, raycastInfo3));
	EXPECT_EQ(false, tmp.m_triangleProxyShape->raycast(ray6, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray6, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	// ----- Test raycast hits ----- //

	// Test front ray against front triangles
	tmp.m_triangleShape->setRaycastTestType(ephysics::FRONT);

	EXPECT_EQ(true, tmp.m_triangleBody->raycast(ray4, raycastInfo3));
	EXPECT_EQ(true, tmp.m_triangleProxyShape->raycast(ray4, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray4, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray4.point1, ray4.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_triangleBody->raycast(ray5, raycastInfo3));
	EXPECT_EQ(true, tmp.m_triangleProxyShape->raycast(ray5, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray5, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray5.point1, ray5.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_triangleBody->raycast(ray6, raycastInfo3));
	EXPECT_EQ(true, tmp.m_triangleProxyShape->raycast(ray6, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray6, &tmp.m_callback);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray6.point1, ray6.point2, float(0.8)), &tmp.m_callback);

	// Test back ray against back triangles
	tmp.m_triangleShape->setRaycastTestType(ephysics::BACK);

	EXPECT_EQ(true, tmp.m_triangleBody->raycast(ray4Back, raycastInfo3));
	EXPECT_EQ(true, tmp.m_triangleProxyShape->raycast(ray4Back, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray4Back, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray4Back.point1, ray4Back.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_triangleBody->raycast(ray5Back, raycastInfo3));
	EXPECT_EQ(true, tmp.m_triangleProxyShape->raycast(ray5Back, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray5Back, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray5Back.point1, ray5Back.point2, 1.0f), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_triangleBody->raycast(ray6Back, raycastInfo3));
	EXPECT_EQ(true, tmp.m_triangleProxyShape->raycast(ray6Back, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray6Back, &tmp.m_callback);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray6Back.point1, ray6Back.point2, float(0.8)), &tmp.m_callback);

	// Test front ray against front-back triangles
	tmp.m_triangleShape->setRaycastTestType(ephysics::FRONT_AND_BACK);

	EXPECT_EQ(true, tmp.m_triangleBody->raycast(ray4, raycastInfo3));
	EXPECT_EQ(true, tmp.m_triangleProxyShape->raycast(ray4, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray4, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray4.point1, ray4.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_triangleBody->raycast(ray5, raycastInfo3));
	EXPECT_EQ(true, tmp.m_triangleProxyShape->raycast(ray5, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray5, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray5.point1, ray5.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_triangleBody->raycast(ray6, raycastInfo3));
	EXPECT_EQ(true, tmp.m_triangleProxyShape->raycast(ray6, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray6, &tmp.m_callback);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray6.point1, ray6.point2, float(0.8)), &tmp.m_callback);

	// Test back ray against front-back triangles
	tmp.m_triangleShape->setRaycastTestType(ephysics::FRONT_AND_BACK);

	EXPECT_EQ(true, tmp.m_triangleBody->raycast(ray4Back, raycastInfo3));
	EXPECT_EQ(true, tmp.m_triangleProxyShape->raycast(ray4Back, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray4Back, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray4Back.point1, ray4Back.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_triangleBody->raycast(ray5Back, raycastInfo3));
	EXPECT_EQ(true, tmp.m_triangleProxyShape->raycast(ray5Back, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray5Back, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray5Back.point1, ray5Back.point2, 1.0f), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_triangleBody->raycast(ray6Back, raycastInfo3));
	EXPECT_EQ(true, tmp.m_triangleProxyShape->raycast(ray6Back, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray6Back, &tmp.m_callback);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray6Back.point1, ray6Back.point2, float(0.8)), &tmp.m_callback);
}

TEST(TestRay, cone) {
	TestRaycast tmp;
	// ----- Test feedback data ----- //
	vec3 point1A = tmp.m_localShapeToWorld * vec3(0 , 0, 3);
	vec3 point1B = tmp.m_localShapeToWorld * vec3(0, 0, -7);
	ephysics::Ray ray(point1A, point1B);
	vec3 hitPoint = tmp.m_localShapeToWorld * vec3(0, 0, 1);

	vec3 point2A = tmp.m_localShapeToWorld * vec3(1 , -5, 0);
	vec3 point2B = tmp.m_localShapeToWorld * vec3(1, 5, 0);
	ephysics::Ray rayBottom(point2A, point2B);
	vec3 hitPoint2 = tmp.m_localShapeToWorld * vec3(1, -3, 0);

	tmp.m_callback.shapeToTest = tmp.m_coneProxyShape;

	// CollisionWorld::raycast()
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	EXPECT_EQ(true, tmp.m_callback.raycastInfo.body == tmp.m_coneBody);
	EXPECT_EQ(true, tmp.m_callback.raycastInfo.proxyShape == tmp.m_coneProxyShape);
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.hitFraction, float(0.2));
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.x(), hitPoint.x());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.y(), hitPoint.y());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.z(), hitPoint.z());

	// Correct category filter mask
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray, &tmp.m_callback, CATEGORY2);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	// Wrong category filter mask
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray, &tmp.m_callback, CATEGORY1);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	// CollisionBody::raycast()
	ephysics::RaycastInfo raycastInfo2;
	EXPECT_EQ(true, tmp.m_coneBody->raycast(ray, raycastInfo2));
	EXPECT_EQ(true, raycastInfo2.body == tmp.m_coneBody);
	EXPECT_EQ(true, raycastInfo2.proxyShape == tmp.m_coneProxyShape);
	EXPECT_FLOAT_EQ(raycastInfo2.hitFraction, float(0.2));
	EXPECT_FLOAT_EQ(raycastInfo2.worldPoint.x(), hitPoint.x());
	EXPECT_FLOAT_EQ(raycastInfo2.worldPoint.y(), hitPoint.y());
	EXPECT_FLOAT_EQ(raycastInfo2.worldPoint.z(), hitPoint.z());

	// ProxyCollisionShape::raycast()
	ephysics::RaycastInfo raycastInfo3;
	EXPECT_EQ(true, tmp.m_coneProxyShape->raycast(ray, raycastInfo3));
	EXPECT_EQ(true, raycastInfo3.body == tmp.m_coneBody);
	EXPECT_EQ(true, raycastInfo3.proxyShape == tmp.m_coneProxyShape);
	EXPECT_FLOAT_EQ(raycastInfo3.hitFraction, float(0.2));
	EXPECT_FLOAT_EQ(raycastInfo3.worldPoint.x(), hitPoint.x());
	EXPECT_FLOAT_EQ(raycastInfo3.worldPoint.y(), hitPoint.y());
	EXPECT_FLOAT_EQ(raycastInfo3.worldPoint.z(), hitPoint.z());

	tmp.m_callback.reset();
	tmp.m_world->raycast(rayBottom, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	EXPECT_EQ(true, tmp.m_callback.raycastInfo.body == tmp.m_coneBody);
	EXPECT_EQ(true, tmp.m_callback.raycastInfo.proxyShape == tmp.m_coneProxyShape);
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.hitFraction, float(0.2));
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.x(), hitPoint2.x());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.y(), hitPoint2.y());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.z(), hitPoint2.z());

	// CollisionBody::raycast()
	ephysics::RaycastInfo raycastInfo5;
	EXPECT_EQ(true, tmp.m_coneBody->raycast(rayBottom, raycastInfo5));
	EXPECT_EQ(true, raycastInfo5.body == tmp.m_coneBody);
	EXPECT_EQ(true, raycastInfo5.proxyShape == tmp.m_coneProxyShape);
	EXPECT_FLOAT_EQ(raycastInfo5.hitFraction, float(0.2));
	EXPECT_FLOAT_EQ(raycastInfo5.worldPoint.x(), hitPoint2.x());
	EXPECT_FLOAT_EQ(raycastInfo5.worldPoint.y(), hitPoint2.y());
	EXPECT_FLOAT_EQ(raycastInfo5.worldPoint.z(), hitPoint2.z());

	// ProxyCollisionShape::raycast()
	ephysics::RaycastInfo raycastInfo6;
	EXPECT_EQ(true, tmp.m_coneProxyShape->raycast(rayBottom, raycastInfo6));
	EXPECT_EQ(true, raycastInfo6.body == tmp.m_coneBody);
	EXPECT_EQ(true, raycastInfo6.proxyShape == tmp.m_coneProxyShape);
	EXPECT_FLOAT_EQ(raycastInfo6.hitFraction, float(0.2));
	EXPECT_FLOAT_EQ(raycastInfo6.worldPoint.x(), hitPoint2.x());
	EXPECT_FLOAT_EQ(raycastInfo6.worldPoint.y(), hitPoint2.y());
	EXPECT_FLOAT_EQ(raycastInfo6.worldPoint.z(), hitPoint2.z());

	ephysics::Ray ray1(tmp.m_localShapeToWorld * vec3(0, 0, 0), tmp.m_localShapeToWorld * vec3(5, 7, -1));
	ephysics::Ray ray2(tmp.m_localShapeToWorld * vec3(5, 11, 7), tmp.m_localShapeToWorld * vec3(17, 29, 28));
	ephysics::Ray ray3(tmp.m_localShapeToWorld * vec3(-1, -2, 1), tmp.m_localShapeToWorld * vec3(-13, -2, 22));
	ephysics::Ray ray4(tmp.m_localShapeToWorld * vec3(10, 10, 10), tmp.m_localShapeToWorld * vec3(22, 28, 31));
	ephysics::Ray ray5(tmp.m_localShapeToWorld * vec3(4, 1, -1), tmp.m_localShapeToWorld * vec3(-26, 1, -1));
	ephysics::Ray ray6(tmp.m_localShapeToWorld * vec3(3, 4, 1), tmp.m_localShapeToWorld * vec3(3, -16, 1));
	ephysics::Ray ray7(tmp.m_localShapeToWorld * vec3(1, -4, 3), tmp.m_localShapeToWorld * vec3(1, -4, -17));
	ephysics::Ray ray8(tmp.m_localShapeToWorld * vec3(-4, 4, 0), tmp.m_localShapeToWorld * vec3(26, 4, 0));
	ephysics::Ray ray9(tmp.m_localShapeToWorld * vec3(0, -4, -7), tmp.m_localShapeToWorld * vec3(0, 46, -7));
	ephysics::Ray ray10(tmp.m_localShapeToWorld * vec3(-3, -2, -6), tmp.m_localShapeToWorld * vec3(-3, -2, 74));
	ephysics::Ray ray11(tmp.m_localShapeToWorld * vec3(3, -1, 0.5), tmp.m_localShapeToWorld * vec3(-27, -1, 0.5));
	ephysics::Ray ray12(tmp.m_localShapeToWorld * vec3(1, 4, -1), tmp.m_localShapeToWorld * vec3(1, -26, -1));
	ephysics::Ray ray13(tmp.m_localShapeToWorld * vec3(-1, -2, 3), tmp.m_localShapeToWorld * vec3(-1, -2, -27));
	ephysics::Ray ray14(tmp.m_localShapeToWorld * vec3(-2, 0, 0.8), tmp.m_localShapeToWorld * vec3(30, 0, 0.8));
	ephysics::Ray ray15(tmp.m_localShapeToWorld * vec3(0, -4, 1), tmp.m_localShapeToWorld * vec3(0, 30, 1));
	ephysics::Ray ray16(tmp.m_localShapeToWorld * vec3(-0.9, 0, -4), tmp.m_localShapeToWorld * vec3(-0.9, 0, 30));

	// ----- Test raycast miss ----- //
	EXPECT_EQ(false, tmp.m_coneBody->raycast(ray1, raycastInfo3));
	EXPECT_EQ(false, tmp.m_coneProxyShape->raycast(ray1, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray1, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray1.point1, ray1.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray1.point1, ray1.point2, float(100.0)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_coneBody->raycast(ray2, raycastInfo3));
	EXPECT_EQ(false, tmp.m_coneProxyShape->raycast(ray2, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray2, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_coneBody->raycast(ray3, raycastInfo3));
	EXPECT_EQ(false, tmp.m_coneProxyShape->raycast(ray3, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray3, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_coneBody->raycast(ray4, raycastInfo3));
	EXPECT_EQ(false, tmp.m_coneProxyShape->raycast(ray4, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray4, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_coneBody->raycast(ray5, raycastInfo3));
	EXPECT_EQ(false, tmp.m_coneProxyShape->raycast(ray5, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray5, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_coneBody->raycast(ray6, raycastInfo3));
	EXPECT_EQ(false, tmp.m_coneProxyShape->raycast(ray6, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray6, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_coneBody->raycast(ray7, raycastInfo3));
	EXPECT_EQ(false, tmp.m_coneProxyShape->raycast(ray7, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray7, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_coneBody->raycast(ray8, raycastInfo3));
	EXPECT_EQ(false, tmp.m_coneProxyShape->raycast(ray8, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray8, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_coneBody->raycast(ray9, raycastInfo3));
	EXPECT_EQ(false, tmp.m_coneProxyShape->raycast(ray9, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray9, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_coneBody->raycast(ray10, raycastInfo3));
	EXPECT_EQ(false, tmp.m_coneProxyShape->raycast(ray10, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray10, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray11.point1, ray11.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray12.point1, ray12.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray13.point1, ray13.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray14.point1, ray14.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray15.point1, ray15.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray16.point1, ray16.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	// ----- Test raycast hits ----- //
	EXPECT_EQ(true, tmp.m_coneBody->raycast(ray11, raycastInfo3));
	EXPECT_EQ(true, tmp.m_coneProxyShape->raycast(ray11, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray11, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray11.point1, ray11.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_coneBody->raycast(ray12, raycastInfo3));
	EXPECT_EQ(true, tmp.m_coneProxyShape->raycast(ray12, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray12, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray12.point1, ray12.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_coneBody->raycast(ray13, raycastInfo3));
	EXPECT_EQ(true, tmp.m_coneProxyShape->raycast(ray13, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray13, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray13.point1, ray13.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_coneBody->raycast(ray14, raycastInfo3));
	EXPECT_EQ(true, tmp.m_coneProxyShape->raycast(ray14, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray14, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray14.point1, ray14.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_coneBody->raycast(ray15, raycastInfo3));
	EXPECT_EQ(true, tmp.m_coneProxyShape->raycast(ray15, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray15, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray15.point1, ray15.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_coneBody->raycast(ray16, raycastInfo3));
	EXPECT_EQ(true, tmp.m_coneProxyShape->raycast(ray16, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray16, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray16.point1, ray16.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
}

TEST(TestRay, convexMash) {
	TestRaycast tmp;
	// ----- Test feedback data ----- //
	vec3 point1 = tmp.m_localShapeToWorld * vec3(1 , 2, 6);
	vec3 point2 = tmp.m_localShapeToWorld * vec3(1, 2, -4);
	ephysics::Ray ray(point1, point2);
	vec3 hitPoint = tmp.m_localShapeToWorld * vec3(1, 2, 4);

	tmp.m_callback.shapeToTest = tmp.m_convexMeshProxyShape;

	// CollisionWorld::raycast()
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	EXPECT_EQ(true, tmp.m_callback.raycastInfo.body == tmp.m_convexMeshBody);
	EXPECT_EQ(true, tmp.m_callback.raycastInfo.proxyShape == tmp.m_convexMeshProxyShape);
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.hitFraction, float(0.2));
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.x(), hitPoint.x());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.y(), hitPoint.y());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.z(), hitPoint.z());

	// Correct category filter mask
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray, &tmp.m_callback, CATEGORY2);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	// Wrong category filter mask
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray, &tmp.m_callback, CATEGORY1);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	// CollisionBody::raycast()
	ephysics::RaycastInfo raycastInfo2;
	EXPECT_EQ(true, tmp.m_convexMeshBody->raycast(ray, raycastInfo2));
	EXPECT_EQ(true, raycastInfo2.body == tmp.m_convexMeshBody);
	EXPECT_EQ(true, raycastInfo2.proxyShape == tmp.m_convexMeshProxyShape);
	EXPECT_FLOAT_EQ(raycastInfo2.hitFraction, float(0.2));
	EXPECT_FLOAT_EQ(raycastInfo2.worldPoint.x(), hitPoint.x());
	EXPECT_FLOAT_EQ(raycastInfo2.worldPoint.y(), hitPoint.y());
	EXPECT_FLOAT_EQ(raycastInfo2.worldPoint.z(), hitPoint.z());

	// ProxyCollisionShape::raycast()
	ephysics::RaycastInfo raycastInfo3;
	EXPECT_EQ(true, tmp.m_convexMeshBodyEdgesInfo->raycast(ray, raycastInfo3));
	EXPECT_EQ(true, raycastInfo3.body == tmp.m_convexMeshBodyEdgesInfo);
	EXPECT_EQ(true, raycastInfo3.proxyShape == tmp.m_convexMeshProxyShapeEdgesInfo);
	EXPECT_FLOAT_EQ(raycastInfo3.hitFraction, float(0.2));
	EXPECT_FLOAT_EQ(raycastInfo3.worldPoint.x(), hitPoint.x());
	EXPECT_FLOAT_EQ(raycastInfo3.worldPoint.y(), hitPoint.y());
	EXPECT_FLOAT_EQ(raycastInfo3.worldPoint.z(), hitPoint.z());

	// ProxyCollisionShape::raycast()
	ephysics::RaycastInfo raycastInfo4;
	EXPECT_EQ(true, tmp.m_convexMeshProxyShape->raycast(ray, raycastInfo4));
	EXPECT_EQ(true, raycastInfo4.body == tmp.m_convexMeshBody);
	EXPECT_EQ(true, raycastInfo4.proxyShape == tmp.m_convexMeshProxyShape);
	EXPECT_FLOAT_EQ(raycastInfo4.hitFraction, float(0.2));
	EXPECT_FLOAT_EQ(raycastInfo4.worldPoint.x(), hitPoint.x());
	EXPECT_FLOAT_EQ(raycastInfo4.worldPoint.y(), hitPoint.y());
	EXPECT_FLOAT_EQ(raycastInfo4.worldPoint.z(), hitPoint.z());

	// ProxyCollisionShape::raycast()
	ephysics::RaycastInfo raycastInfo5;
	EXPECT_EQ(true, tmp.m_convexMeshProxyShapeEdgesInfo->raycast(ray, raycastInfo5));
	EXPECT_EQ(true, raycastInfo5.body == tmp.m_convexMeshBodyEdgesInfo);
	EXPECT_EQ(true, raycastInfo5.proxyShape == tmp.m_convexMeshProxyShapeEdgesInfo);
	EXPECT_FLOAT_EQ(raycastInfo5.hitFraction, float(0.2));
	EXPECT_FLOAT_EQ(raycastInfo5.worldPoint.x(), hitPoint.x());
	EXPECT_FLOAT_EQ(raycastInfo5.worldPoint.y(), hitPoint.y());
	EXPECT_FLOAT_EQ(raycastInfo5.worldPoint.z(), hitPoint.z());

	ephysics::Ray ray1(tmp.m_localShapeToWorld * vec3(0, 0, 0), tmp.m_localShapeToWorld * vec3(5, 7, -1));
	ephysics::Ray ray2(tmp.m_localShapeToWorld * vec3(5, 11, 7), tmp.m_localShapeToWorld * vec3(17, 29, 28));
	ephysics::Ray ray3(tmp.m_localShapeToWorld * vec3(1, 2, 3), tmp.m_localShapeToWorld * vec3(-11, 2, 24));
	ephysics::Ray ray4(tmp.m_localShapeToWorld * vec3(10, 10, 10), tmp.m_localShapeToWorld * vec3(22, 28, 31));
	ephysics::Ray ray5(tmp.m_localShapeToWorld * vec3(3, 1, -5), tmp.m_localShapeToWorld * vec3(-30, 1, -5));
	ephysics::Ray ray6(tmp.m_localShapeToWorld * vec3(4, 4, 1), tmp.m_localShapeToWorld * vec3(4, -30, 1));
	ephysics::Ray ray7(tmp.m_localShapeToWorld * vec3(1, -4, 5), tmp.m_localShapeToWorld * vec3(1, -4, -30));
	ephysics::Ray ray8(tmp.m_localShapeToWorld * vec3(-4, 4, 0), tmp.m_localShapeToWorld * vec3(30, 4, 0));
	ephysics::Ray ray9(tmp.m_localShapeToWorld * vec3(0, -4, -7), tmp.m_localShapeToWorld * vec3(0, 30, -7));
	ephysics::Ray ray10(tmp.m_localShapeToWorld * vec3(-3, 0, -6), tmp.m_localShapeToWorld * vec3(-3, 0, 30));
	ephysics::Ray ray11(tmp.m_localShapeToWorld * vec3(3, 1, 2), tmp.m_localShapeToWorld * vec3(-30, 0, -6));
	ephysics::Ray ray12(tmp.m_localShapeToWorld * vec3(1, 4, -1), tmp.m_localShapeToWorld * vec3(1, -30, -1));
	ephysics::Ray ray13(tmp.m_localShapeToWorld * vec3(-1, 2, 5), tmp.m_localShapeToWorld * vec3(-1, 2, -30));
	ephysics::Ray ray14(tmp.m_localShapeToWorld * vec3(-3, 2, -2), tmp.m_localShapeToWorld * vec3(30, 2, -2));
	ephysics::Ray ray15(tmp.m_localShapeToWorld * vec3(0, -4, 1), tmp.m_localShapeToWorld * vec3(0, 30, 1));
	ephysics::Ray ray16(tmp.m_localShapeToWorld * vec3(-1, 2, -7), tmp.m_localShapeToWorld * vec3(-1, 2, 30));

	// ----- Test raycast miss ----- //
	EXPECT_EQ(false, tmp.m_convexMeshBody->raycast(ray1, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshBodyEdgesInfo->raycast(ray1, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShape->raycast(ray1, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShapeEdgesInfo->raycast(ray1, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray1, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray1.point1, ray1.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray1.point1, ray1.point2, float(100.0)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_convexMeshBody->raycast(ray2, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshBodyEdgesInfo->raycast(ray2, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShape->raycast(ray2, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShapeEdgesInfo->raycast(ray2, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray2, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_convexMeshBody->raycast(ray3, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshBodyEdgesInfo->raycast(ray3, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShape->raycast(ray3, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShapeEdgesInfo->raycast(ray3, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray3, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_convexMeshBody->raycast(ray4, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshBodyEdgesInfo->raycast(ray4, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShape->raycast(ray4, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShapeEdgesInfo->raycast(ray4, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray4, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_convexMeshBody->raycast(ray5, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshBodyEdgesInfo->raycast(ray5, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShape->raycast(ray5, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShapeEdgesInfo->raycast(ray5, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray5, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_convexMeshBody->raycast(ray6, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshBodyEdgesInfo->raycast(ray6, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShape->raycast(ray6, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShapeEdgesInfo->raycast(ray6, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray6, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_convexMeshBody->raycast(ray7, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshBodyEdgesInfo->raycast(ray7, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShape->raycast(ray7, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShapeEdgesInfo->raycast(ray7, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray7, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_convexMeshBody->raycast(ray8, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshBodyEdgesInfo->raycast(ray8, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShape->raycast(ray8, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShapeEdgesInfo->raycast(ray8, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray8, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_convexMeshBody->raycast(ray9, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshBodyEdgesInfo->raycast(ray9, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShape->raycast(ray9, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShapeEdgesInfo->raycast(ray9, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray9, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_convexMeshBody->raycast(ray10, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshBodyEdgesInfo->raycast(ray10, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShape->raycast(ray10, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShapeEdgesInfo->raycast(ray10, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray10, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray11.point1, ray11.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray12.point1, ray12.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray13.point1, ray13.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray14.point1, ray14.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray15.point1, ray15.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray16.point1, ray16.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	// ----- Test raycast hits ----- //
	EXPECT_EQ(true, tmp.m_convexMeshBody->raycast(ray11, raycastInfo3));
	EXPECT_EQ(true, tmp.m_convexMeshBodyEdgesInfo->raycast(ray11, raycastInfo3));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShape->raycast(ray11, raycastInfo3));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShapeEdgesInfo->raycast(ray11, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray11, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray11.point1, ray11.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_convexMeshBody->raycast(ray12, raycastInfo3));
	EXPECT_EQ(true, tmp.m_convexMeshBodyEdgesInfo->raycast(ray12, raycastInfo3));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShape->raycast(ray12, raycastInfo3));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShapeEdgesInfo->raycast(ray12, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray12, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray12.point1, ray12.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_convexMeshBody->raycast(ray13, raycastInfo3));
	EXPECT_EQ(true, tmp.m_convexMeshBodyEdgesInfo->raycast(ray13, raycastInfo3));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShape->raycast(ray13, raycastInfo3));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShapeEdgesInfo->raycast(ray13, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray13, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray13.point1, ray13.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_convexMeshBody->raycast(ray14, raycastInfo3));
	EXPECT_EQ(true, tmp.m_convexMeshBodyEdgesInfo->raycast(ray14, raycastInfo3));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShape->raycast(ray14, raycastInfo3));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShapeEdgesInfo->raycast(ray14, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray14, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray14.point1, ray14.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_convexMeshBody->raycast(ray15, raycastInfo3));
	EXPECT_EQ(true, tmp.m_convexMeshBodyEdgesInfo->raycast(ray15, raycastInfo3));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShape->raycast(ray15, raycastInfo3));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShapeEdgesInfo->raycast(ray15, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray15, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray15.point1, ray15.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_convexMeshBody->raycast(ray16, raycastInfo3));
	EXPECT_EQ(true, tmp.m_convexMeshBodyEdgesInfo->raycast(ray16, raycastInfo3));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShape->raycast(ray16, raycastInfo3));
	EXPECT_EQ(true, tmp.m_convexMeshProxyShapeEdgesInfo->raycast(ray16, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray16, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray16.point1, ray16.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
}

TEST(TestRay, cylinder) {
	TestRaycast tmp;
	// ----- Test feedback data ----- //
	vec3 point1A = tmp.m_localShapeToWorld * vec3(4 , 1, 0);
	vec3 point1B = tmp.m_localShapeToWorld * vec3(-6, 1, 0);
	ephysics::Ray ray(point1A, point1B);
	vec3 hitPoint = tmp.m_localShapeToWorld * vec3(2, 1, 0);

	vec3 point2A = tmp.m_localShapeToWorld * vec3(0 , 4.5, 0);
	vec3 point2B = tmp.m_localShapeToWorld * vec3(0, -5.5, 0);
	ephysics::Ray rayTop(point2A, point2B);
	vec3 hitPointTop = tmp.m_localShapeToWorld * vec3(0, float(2.5), 0);

	vec3 point3A = tmp.m_localShapeToWorld * vec3(0 , -4.5, 0);
	vec3 point3B = tmp.m_localShapeToWorld * vec3(0, 5.5, 0);
	ephysics::Ray rayBottom(point3A, point3B);
	vec3 hitPointBottom = tmp.m_localShapeToWorld * vec3(0, float(-2.5), 0);

	tmp.m_callback.shapeToTest = tmp.m_cylinderProxyShape;

	// CollisionWorld::raycast()
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	EXPECT_EQ(true, tmp.m_callback.raycastInfo.body == tmp.m_cylinderBody);
	EXPECT_EQ(true, tmp.m_callback.raycastInfo.proxyShape == tmp.m_cylinderProxyShape);
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.hitFraction, float(0.2));
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.x(), hitPoint.x());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.y(), hitPoint.y());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.z(), hitPoint.z());

	// Correct category filter mask
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray, &tmp.m_callback, CATEGORY2);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	// Wrong category filter mask
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray, &tmp.m_callback, CATEGORY1);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	// CollisionBody::raycast()
	ephysics::RaycastInfo raycastInfo2;
	EXPECT_EQ(true, tmp.m_cylinderBody->raycast(ray, raycastInfo2));
	EXPECT_EQ(true, raycastInfo2.body == tmp.m_cylinderBody);
	EXPECT_EQ(true, raycastInfo2.proxyShape == tmp.m_cylinderProxyShape);
	EXPECT_FLOAT_EQ(raycastInfo2.hitFraction, float(0.2));
	EXPECT_FLOAT_EQ(raycastInfo2.worldPoint.x(), hitPoint.x());
	EXPECT_FLOAT_EQ(raycastInfo2.worldPoint.y(), hitPoint.y());
	EXPECT_FLOAT_EQ(raycastInfo2.worldPoint.z(), hitPoint.z());

	// ProxyCollisionShape::raycast()
	ephysics::RaycastInfo raycastInfo3;
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->raycast(ray, raycastInfo3));
	EXPECT_EQ(true, raycastInfo3.body == tmp.m_cylinderBody);
	EXPECT_EQ(true, raycastInfo3.proxyShape == tmp.m_cylinderProxyShape);
	EXPECT_FLOAT_EQ(raycastInfo3.hitFraction, float(0.2));
	EXPECT_FLOAT_EQ(raycastInfo3.worldPoint.x(), hitPoint.x());
	EXPECT_FLOAT_EQ(raycastInfo3.worldPoint.y(), hitPoint.y());
	EXPECT_FLOAT_EQ(raycastInfo3.worldPoint.z(), hitPoint.z());

	// ProxyCollisionShape::raycast()
	ephysics::RaycastInfo raycastInfo5;
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->raycast(rayTop, raycastInfo5));
	EXPECT_EQ(true, raycastInfo5.body == tmp.m_cylinderBody);
	EXPECT_EQ(true, raycastInfo5.proxyShape == tmp.m_cylinderProxyShape);
	EXPECT_FLOAT_EQ(raycastInfo5.hitFraction, float(0.2));
	EXPECT_FLOAT_EQ(raycastInfo5.worldPoint.x(), hitPointTop.x());
	EXPECT_FLOAT_EQ(raycastInfo5.worldPoint.y(), hitPointTop.y());
	EXPECT_FLOAT_EQ(raycastInfo5.worldPoint.z(), hitPointTop.z());

	// ProxyCollisionShape::raycast()
	ephysics::RaycastInfo raycastInfo6;
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->raycast(rayBottom, raycastInfo6));
	EXPECT_EQ(true, raycastInfo6.body == tmp.m_cylinderBody);
	EXPECT_EQ(true, raycastInfo6.proxyShape == tmp.m_cylinderProxyShape);
	EXPECT_FLOAT_EQ(raycastInfo6.hitFraction, float(0.2));
	EXPECT_FLOAT_EQ(raycastInfo6.worldPoint.x(), hitPointBottom.x());
	EXPECT_FLOAT_EQ(raycastInfo6.worldPoint.y(), hitPointBottom.y());
	EXPECT_FLOAT_EQ(raycastInfo6.worldPoint.z(), hitPointBottom.z());

	ephysics::Ray ray1(tmp.m_localShapeToWorld * vec3(0, 0, 0), tmp.m_localShapeToWorld * vec3(5, 7, -1));
	ephysics::Ray ray2(tmp.m_localShapeToWorld * vec3(5, 11, 7), tmp.m_localShapeToWorld * vec3(17, 20, 28));
	ephysics::Ray ray3(tmp.m_localShapeToWorld * vec3(1, 3, -1), tmp.m_localShapeToWorld * vec3(-11,3, 20));
	ephysics::Ray ray4(tmp.m_localShapeToWorld * vec3(10, 10, 10), tmp.m_localShapeToWorld * vec3(22, 28, 31));
	ephysics::Ray ray5(tmp.m_localShapeToWorld * vec3(4, 1, -5), tmp.m_localShapeToWorld * vec3(-30, 1, -5));
	ephysics::Ray ray6(tmp.m_localShapeToWorld * vec3(4, 9, 1), tmp.m_localShapeToWorld * vec3(4, -30, 1));
	ephysics::Ray ray7(tmp.m_localShapeToWorld * vec3(1, -9, 5), tmp.m_localShapeToWorld * vec3(1, -9, -30));
	ephysics::Ray ray8(tmp.m_localShapeToWorld * vec3(-4, 9, 0), tmp.m_localShapeToWorld * vec3(30, 9, 0));
	ephysics::Ray ray9(tmp.m_localShapeToWorld * vec3(0, -9, -4), tmp.m_localShapeToWorld * vec3(0, 30, -4));
	ephysics::Ray ray10(tmp.m_localShapeToWorld * vec3(-4, 0, -6), tmp.m_localShapeToWorld * vec3(-4, 0, 30));
	ephysics::Ray ray11(tmp.m_localShapeToWorld * vec3(4, 1, 1.5), tmp.m_localShapeToWorld * vec3(-30, 1, 1.5));
	ephysics::Ray ray12(tmp.m_localShapeToWorld * vec3(1, 9, -1), tmp.m_localShapeToWorld * vec3(1, -30, -1));
	ephysics::Ray ray13(tmp.m_localShapeToWorld * vec3(-1, 2, 3), tmp.m_localShapeToWorld * vec3(-1, 2, -30));
	ephysics::Ray ray14(tmp.m_localShapeToWorld * vec3(-3, 2, -1.7), tmp.m_localShapeToWorld * vec3(30, 2, -1.7));
	ephysics::Ray ray15(tmp.m_localShapeToWorld * vec3(0, -9, 1), tmp.m_localShapeToWorld * vec3(0, 30, 1));
	ephysics::Ray ray16(tmp.m_localShapeToWorld * vec3(-1, 2, -7), tmp.m_localShapeToWorld * vec3(-1, 2, 30));

	// ----- Test raycast miss ----- //
	EXPECT_EQ(false, tmp.m_cylinderBody->raycast(ray1, raycastInfo3));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->raycast(ray1, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray1, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray1.point1, ray1.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray1.point1, ray1.point2, float(100.0)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_cylinderBody->raycast(ray2, raycastInfo3));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->raycast(ray2, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray2, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_cylinderBody->raycast(ray3, raycastInfo3));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->raycast(ray3, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray3, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_cylinderBody->raycast(ray4, raycastInfo3));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->raycast(ray4, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray4, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_cylinderBody->raycast(ray5, raycastInfo3));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->raycast(ray5, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray5, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_cylinderBody->raycast(ray6, raycastInfo3));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->raycast(ray6, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray6, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_cylinderBody->raycast(ray7, raycastInfo3));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->raycast(ray7, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray7, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_cylinderBody->raycast(ray8, raycastInfo3));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->raycast(ray8, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray8, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_cylinderBody->raycast(ray9, raycastInfo3));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->raycast(ray9, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray9, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_cylinderBody->raycast(ray10, raycastInfo3));
	EXPECT_EQ(false, tmp.m_cylinderProxyShape->raycast(ray10, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray10, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray11.point1, ray11.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray12.point1, ray12.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray13.point1, ray13.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray14.point1, ray14.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray15.point1, ray15.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray16.point1, ray16.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	// ----- Test raycast hits ----- //
	EXPECT_EQ(true, tmp.m_cylinderBody->raycast(ray11, raycastInfo3));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->raycast(ray11, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray11, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray11.point1, ray11.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_cylinderBody->raycast(ray12, raycastInfo3));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->raycast(ray12, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray12, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray12.point1, ray12.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_cylinderBody->raycast(ray13, raycastInfo3));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->raycast(ray13, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray13, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray13.point1, ray13.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_cylinderBody->raycast(ray14, raycastInfo3));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->raycast(ray14, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray14, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray14.point1, ray14.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_cylinderBody->raycast(ray15, raycastInfo3));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->raycast(ray15, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray15, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray15.point1, ray15.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_cylinderBody->raycast(ray16, raycastInfo3));
	EXPECT_EQ(true, tmp.m_cylinderProxyShape->raycast(ray16, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray16, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray16.point1, ray16.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
}

TEST(TestRay, compound) {
	TestRaycast tmp;
	// ----- Test feedback data ----- //

	// Raycast hit against the sphere shape
	ephysics::Ray ray1(tmp.m_localShape2ToWorld * vec3(4, 1, 2), tmp.m_localShape2ToWorld * vec3(-30, 1, 2));
	ephysics::Ray ray2(tmp.m_localShape2ToWorld * vec3(1, 4, -1), tmp.m_localShape2ToWorld * vec3(1, -30, -1));
	ephysics::Ray ray3(tmp.m_localShape2ToWorld * vec3(-1, 2, 5), tmp.m_localShape2ToWorld * vec3(-1, 2, -30));
	ephysics::Ray ray4(tmp.m_localShape2ToWorld * vec3(-5, 2, -2), tmp.m_localShape2ToWorld * vec3(30, 2, -2));
	ephysics::Ray ray5(tmp.m_localShape2ToWorld * vec3(0, -4, 1), tmp.m_localShape2ToWorld * vec3(0, 30, 1));
	ephysics::Ray ray6(tmp.m_localShape2ToWorld * vec3(-1, 2, -11), tmp.m_localShape2ToWorld * vec3(-1, 2, 30));

	tmp.m_callback.shapeToTest = tmp.m_compoundSphereProxyShape;

	// Correct category filter mask
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray1, &tmp.m_callback, CATEGORY2);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	// Wrong category filter mask
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray1, &tmp.m_callback, CATEGORY1);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	ephysics::RaycastInfo raycastInfo;
	EXPECT_EQ(true, tmp.m_compoundBody->raycast(ray1, raycastInfo));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray1, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray1.point1, ray1.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_compoundBody->raycast(ray2, raycastInfo));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray2, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray2.point1, ray2.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_compoundBody->raycast(ray3, raycastInfo));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray3, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray3.point1, ray3.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_compoundBody->raycast(ray4, raycastInfo));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray4, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray4.point1, ray4.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_compoundBody->raycast(ray5, raycastInfo));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray5, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray5.point1, ray5.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_compoundBody->raycast(ray6, raycastInfo));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray6, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray6.point1, ray6.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	// Raycast hit agains the cylinder shape
	ephysics::Ray ray11(tmp.m_localShapeToWorld * vec3(4, 1, 1.5), tmp.m_localShapeToWorld * vec3(-30, 1.5, 2));
	ephysics::Ray ray12(tmp.m_localShapeToWorld * vec3(1.5, 9, -1), tmp.m_localShapeToWorld * vec3(1.5, -30, -1));
	ephysics::Ray ray13(tmp.m_localShapeToWorld * vec3(-1, 2, 3), tmp.m_localShapeToWorld * vec3(-1, 2, -30));
	ephysics::Ray ray14(tmp.m_localShapeToWorld * vec3(-3, 2, -1.5), tmp.m_localShapeToWorld * vec3(30, 1, -1.5));
	ephysics::Ray ray15(tmp.m_localShapeToWorld * vec3(0, -9, 1), tmp.m_localShapeToWorld * vec3(0, 30, 1));
	ephysics::Ray ray16(tmp.m_localShapeToWorld * vec3(-1, 2, -7), tmp.m_localShapeToWorld * vec3(-1, 2, 30));

	tmp.m_callback.shapeToTest = tmp.m_compoundCylinderProxyShape;

	EXPECT_EQ(true, tmp.m_compoundBody->raycast(ray11, raycastInfo));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray11, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray11.point1, ray11.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_compoundBody->raycast(ray12, raycastInfo));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray12, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray12.point1, ray12.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_compoundBody->raycast(ray13, raycastInfo));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray13, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray13.point1, ray13.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_compoundBody->raycast(ray14, raycastInfo));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray14, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray14.point1, ray14.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_compoundBody->raycast(ray15, raycastInfo));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray15, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray15.point1, ray15.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_compoundBody->raycast(ray16, raycastInfo));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray16, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray16.point1, ray16.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
}


TEST(TestRay, concaveMesh) {
	TestRaycast tmp;
	// ----- Test feedback data ----- //
	vec3 point1 = tmp.m_localShapeToWorld * vec3(1 , 2, 6);
	vec3 point2 = tmp.m_localShapeToWorld * vec3(1, 2, -4);
	ephysics::Ray ray(point1, point2);
	vec3 hitPoint = tmp.m_localShapeToWorld * vec3(1, 2, 4);

	tmp.m_callback.shapeToTest = tmp.m_concaveMeshProxyShape;

	// CollisionWorld::raycast()
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	EXPECT_EQ(true, tmp.m_callback.raycastInfo.body == tmp.m_concaveMeshBody);
	EXPECT_EQ(true, tmp.m_callback.raycastInfo.proxyShape == tmp.m_concaveMeshProxyShape);
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.hitFraction, float(0.2));
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.x(), hitPoint.x());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.y(), hitPoint.y());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.z(), hitPoint.z());

	// Correct category filter mask
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray, &tmp.m_callback, CATEGORY2);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	// Wrong category filter mask
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray, &tmp.m_callback, CATEGORY1);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	// CollisionBody::raycast()
	ephysics::RaycastInfo raycastInfo2;
	EXPECT_EQ(true, tmp.m_concaveMeshBody->raycast(ray, raycastInfo2));
	EXPECT_EQ(true, raycastInfo2.body == tmp.m_concaveMeshBody);
	EXPECT_EQ(true, raycastInfo2.proxyShape == tmp.m_concaveMeshProxyShape);
	EXPECT_FLOAT_EQ(raycastInfo2.hitFraction, float(0.2));
	EXPECT_FLOAT_EQ(raycastInfo2.worldPoint.x(), hitPoint.x());
	EXPECT_FLOAT_EQ(raycastInfo2.worldPoint.y(), hitPoint.y());
	EXPECT_FLOAT_EQ(raycastInfo2.worldPoint.z(), hitPoint.z());

	// ProxyCollisionShape::raycast()
	ephysics::RaycastInfo raycastInfo3;
	EXPECT_EQ(true, tmp.m_concaveMeshBody->raycast(ray, raycastInfo3));
	EXPECT_EQ(true, raycastInfo3.body == tmp.m_concaveMeshBody);
	EXPECT_EQ(true, raycastInfo3.proxyShape == tmp.m_concaveMeshProxyShape);
	EXPECT_FLOAT_EQ(raycastInfo3.hitFraction, float(0.2));
	EXPECT_FLOAT_EQ(raycastInfo3.worldPoint.x(), hitPoint.x());
	EXPECT_FLOAT_EQ(raycastInfo3.worldPoint.y(), hitPoint.y());
	EXPECT_FLOAT_EQ(raycastInfo3.worldPoint.z(), hitPoint.z());

	// ProxyCollisionShape::raycast()
	ephysics::RaycastInfo raycastInfo4;
	EXPECT_EQ(true, tmp.m_concaveMeshBody->raycast(ray, raycastInfo4));
	EXPECT_EQ(true, raycastInfo4.body == tmp.m_concaveMeshBody);
	EXPECT_EQ(true, raycastInfo4.proxyShape == tmp.m_concaveMeshProxyShape);
	EXPECT_FLOAT_EQ(raycastInfo4.hitFraction, float(0.2));
	EXPECT_FLOAT_EQ(raycastInfo4.worldPoint.x(), hitPoint.x());
	EXPECT_FLOAT_EQ(raycastInfo4.worldPoint.y(), hitPoint.y());
	EXPECT_FLOAT_EQ(raycastInfo4.worldPoint.z(), hitPoint.z());

	// ProxyCollisionShape::raycast()
	ephysics::RaycastInfo raycastInfo5;
	EXPECT_EQ(true, tmp.m_concaveMeshBody->raycast(ray, raycastInfo5));
	EXPECT_EQ(true, raycastInfo5.body == tmp.m_concaveMeshBody);
	EXPECT_EQ(true, raycastInfo5.proxyShape == tmp.m_concaveMeshProxyShape);
	EXPECT_FLOAT_EQ(raycastInfo5.hitFraction, float(0.2));
	EXPECT_FLOAT_EQ(raycastInfo5.worldPoint.x(), hitPoint.x());
	EXPECT_FLOAT_EQ(raycastInfo5.worldPoint.y(), hitPoint.y());
	EXPECT_FLOAT_EQ(raycastInfo5.worldPoint.z(), hitPoint.z());

	ephysics::Ray ray1(tmp.m_localShapeToWorld * vec3(0, 0, 0), tmp.m_localShapeToWorld * vec3(5, 7, -1));
	ephysics::Ray ray2(tmp.m_localShapeToWorld * vec3(5, 11, 7), tmp.m_localShapeToWorld * vec3(17, 29, 28));
	ephysics::Ray ray3(tmp.m_localShapeToWorld * vec3(1, 2, 3), tmp.m_localShapeToWorld * vec3(-11, 2, 24));
	ephysics::Ray ray4(tmp.m_localShapeToWorld * vec3(10, 10, 10), tmp.m_localShapeToWorld * vec3(22, 28, 31));
	ephysics::Ray ray5(tmp.m_localShapeToWorld * vec3(3, 1, -5), tmp.m_localShapeToWorld * vec3(-30, 1, -5));
	ephysics::Ray ray6(tmp.m_localShapeToWorld * vec3(4, 4, 1), tmp.m_localShapeToWorld * vec3(4, -30, 1));
	ephysics::Ray ray7(tmp.m_localShapeToWorld * vec3(1, -4, 5), tmp.m_localShapeToWorld * vec3(1, -4, -30));
	ephysics::Ray ray8(tmp.m_localShapeToWorld * vec3(-4, 4, 0), tmp.m_localShapeToWorld * vec3(30, 4, 0));
	ephysics::Ray ray9(tmp.m_localShapeToWorld * vec3(0, -4, -7), tmp.m_localShapeToWorld * vec3(0, 30, -7));
	ephysics::Ray ray10(tmp.m_localShapeToWorld * vec3(-3, 0, -6), tmp.m_localShapeToWorld * vec3(-3, 0, 30));
	ephysics::Ray ray11(tmp.m_localShapeToWorld * vec3(3, 1, 2), tmp.m_localShapeToWorld * vec3(-30, 0, -6));
	ephysics::Ray ray12(tmp.m_localShapeToWorld * vec3(1, 4, -1), tmp.m_localShapeToWorld * vec3(1, -30, -1));
	ephysics::Ray ray13(tmp.m_localShapeToWorld * vec3(-1, 2, 5), tmp.m_localShapeToWorld * vec3(-1, 2, -30));
	ephysics::Ray ray14(tmp.m_localShapeToWorld * vec3(-3, 2, -2), tmp.m_localShapeToWorld * vec3(30, 2, -2));
	ephysics::Ray ray15(tmp.m_localShapeToWorld * vec3(0, -4, 1), tmp.m_localShapeToWorld * vec3(0, 30, 1));
	ephysics::Ray ray16(tmp.m_localShapeToWorld * vec3(-1, 2, -7), tmp.m_localShapeToWorld * vec3(-1, 2, 30));

	// ----- Test raycast miss ----- //
	EXPECT_EQ(false, tmp.m_concaveMeshBody->raycast(ray1, raycastInfo3));
	EXPECT_EQ(false, tmp.m_convexMeshProxyShape->raycast(ray1, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray1, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray1.point1, ray1.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray1.point1, ray1.point2, float(100.0)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_concaveMeshBody->raycast(ray2, raycastInfo3));
	EXPECT_EQ(false, tmp.m_concaveMeshProxyShape->raycast(ray2, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray2, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_concaveMeshBody->raycast(ray3, raycastInfo3));
	EXPECT_EQ(false, tmp.m_concaveMeshProxyShape->raycast(ray3, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray3, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_concaveMeshBody->raycast(ray4, raycastInfo3));
	EXPECT_EQ(false, tmp.m_concaveMeshProxyShape->raycast(ray4, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray4, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_concaveMeshBody->raycast(ray5, raycastInfo3));
	EXPECT_EQ(false, tmp.m_concaveMeshProxyShape->raycast(ray5, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray5, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_concaveMeshBody->raycast(ray6, raycastInfo3));
	EXPECT_EQ(false, tmp.m_concaveMeshProxyShape->raycast(ray6, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray6, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_concaveMeshBody->raycast(ray7, raycastInfo3));
	EXPECT_EQ(false, tmp.m_concaveMeshProxyShape->raycast(ray7, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray7, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_concaveMeshBody->raycast(ray8, raycastInfo3));
	EXPECT_EQ(false, tmp.m_concaveMeshProxyShape->raycast(ray8, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray8, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_concaveMeshBody->raycast(ray9, raycastInfo3));
	EXPECT_EQ(false, tmp.m_concaveMeshProxyShape->raycast(ray9, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray9, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_concaveMeshBody->raycast(ray10, raycastInfo3));
	EXPECT_EQ(false, tmp.m_concaveMeshProxyShape->raycast(ray10, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray10, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray11.point1, ray11.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray12.point1, ray12.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray13.point1, ray13.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray14.point1, ray14.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray15.point1, ray15.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray16.point1, ray16.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	// ----- Test raycast hits ----- //
	EXPECT_EQ(true, tmp.m_concaveMeshBody->raycast(ray11, raycastInfo3));
	EXPECT_EQ(true, tmp.m_concaveMeshProxyShape->raycast(ray11, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray11, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray11.point1, ray11.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_concaveMeshBody->raycast(ray12, raycastInfo3));
	EXPECT_EQ(true, tmp.m_concaveMeshProxyShape->raycast(ray12, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray12, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray12.point1, ray12.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_concaveMeshBody->raycast(ray13, raycastInfo3));
	EXPECT_EQ(true, tmp.m_concaveMeshProxyShape->raycast(ray13, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray13, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray13.point1, ray13.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_concaveMeshBody->raycast(ray14, raycastInfo3));
	EXPECT_EQ(true, tmp.m_concaveMeshProxyShape->raycast(ray14, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray14, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray14.point1, ray14.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_concaveMeshBody->raycast(ray15, raycastInfo3));
	EXPECT_EQ(true, tmp.m_concaveMeshProxyShape->raycast(ray15, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray15, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray15.point1, ray15.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_concaveMeshBody->raycast(ray16, raycastInfo3));
	EXPECT_EQ(true, tmp.m_concaveMeshProxyShape->raycast(ray16, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray16, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray16.point1, ray16.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
}

TEST(TestRay, heighField) {
	TestRaycast tmp;
	// ----- Test feedback data ----- //
	vec3 point1A = tmp.m_localShapeToWorld * vec3(0 , 10, 2);
	vec3 point1B = tmp.m_localShapeToWorld * vec3(0, -10, 2);
	ephysics::Ray ray(point1A, point1B);
	vec3 hitPoint = tmp.m_localShapeToWorld * vec3(0, 2, 2);

	vec3 point2A = tmp.m_localShapeToWorld * vec3(1 , 8, -4);
	vec3 point2B = tmp.m_localShapeToWorld * vec3(1, -8, -4);
	ephysics::Ray rayBottom(point2A, point2B);
	vec3 hitPoint2 = tmp.m_localShapeToWorld * vec3(1, 2, -4);

	tmp.m_callback.shapeToTest = tmp.m_heightFieldProxyShape;

	// CollisionWorld::raycast()
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	EXPECT_EQ(true, tmp.m_callback.raycastInfo.body == tmp.m_heightFieldBody);
	EXPECT_EQ(true, tmp.m_callback.raycastInfo.proxyShape == tmp.m_heightFieldProxyShape);
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.hitFraction, float(0.4));
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.x(), hitPoint.x());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.y(), hitPoint.y());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.z(), hitPoint.z());

	// Correct category filter mask
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray, &tmp.m_callback, CATEGORY2);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	// Wrong category filter mask
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray, &tmp.m_callback, CATEGORY1);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	// CollisionBody::raycast()
	ephysics::RaycastInfo raycastInfo2;
	EXPECT_EQ(true, tmp.m_heightFieldBody->raycast(ray, raycastInfo2));
	EXPECT_EQ(true, raycastInfo2.body == tmp.m_heightFieldBody);
	EXPECT_EQ(true, raycastInfo2.proxyShape == tmp.m_heightFieldProxyShape);
	EXPECT_FLOAT_EQ(raycastInfo2.hitFraction, float(0.4));
	EXPECT_FLOAT_EQ(raycastInfo2.worldPoint.x(), hitPoint.x());
	EXPECT_FLOAT_EQ(raycastInfo2.worldPoint.y(), hitPoint.y());
	EXPECT_FLOAT_EQ(raycastInfo2.worldPoint.z(), hitPoint.z());

	// ProxyCollisionShape::raycast()
	ephysics::RaycastInfo raycastInfo3;
	EXPECT_EQ(true, tmp.m_heightFieldProxyShape->raycast(ray, raycastInfo3));
	EXPECT_EQ(true, raycastInfo3.body == tmp.m_heightFieldBody);
	EXPECT_EQ(true, raycastInfo3.proxyShape == tmp.m_heightFieldProxyShape);
	EXPECT_FLOAT_EQ(raycastInfo3.hitFraction, float(0.4));
	EXPECT_FLOAT_EQ(raycastInfo3.worldPoint.x(), hitPoint.x());
	EXPECT_FLOAT_EQ(raycastInfo3.worldPoint.y(), hitPoint.y());
	EXPECT_FLOAT_EQ(raycastInfo3.worldPoint.z(), hitPoint.z());

	tmp.m_callback.reset();
	tmp.m_world->raycast(rayBottom, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	EXPECT_EQ(true, tmp.m_callback.raycastInfo.body == tmp.m_heightFieldBody);
	EXPECT_EQ(true, tmp.m_callback.raycastInfo.proxyShape == tmp.m_heightFieldProxyShape);
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.hitFraction, float(0.375));
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.x(), hitPoint2.x());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.y(), hitPoint2.y());
	EXPECT_FLOAT_EQ(tmp.m_callback.raycastInfo.worldPoint.z(), hitPoint2.z());

	// CollisionBody::raycast()
	ephysics::RaycastInfo raycastInfo5;
	EXPECT_EQ(true, tmp.m_heightFieldBody->raycast(rayBottom, raycastInfo5));
	EXPECT_EQ(true, raycastInfo5.body == tmp.m_heightFieldBody);
	EXPECT_EQ(true, raycastInfo5.proxyShape == tmp.m_heightFieldProxyShape);
	EXPECT_FLOAT_EQ(raycastInfo5.hitFraction, float(0.375));
	EXPECT_FLOAT_EQ(raycastInfo5.worldPoint.x(), hitPoint2.x());
	EXPECT_FLOAT_EQ(raycastInfo5.worldPoint.y(), hitPoint2.y());
	EXPECT_FLOAT_EQ(raycastInfo5.worldPoint.z(), hitPoint2.z());

	// ProxyCollisionShape::raycast()
	ephysics::RaycastInfo raycastInfo6;
	EXPECT_EQ(true, tmp.m_heightFieldProxyShape->raycast(rayBottom, raycastInfo6));
	EXPECT_EQ(true, raycastInfo6.body == tmp.m_heightFieldBody);
	EXPECT_EQ(true, raycastInfo6.proxyShape == tmp.m_heightFieldProxyShape);
	EXPECT_FLOAT_EQ(raycastInfo6.hitFraction, float(0.375));
	EXPECT_FLOAT_EQ(raycastInfo6.worldPoint.x(), hitPoint2.x());
	EXPECT_FLOAT_EQ(raycastInfo6.worldPoint.y(), hitPoint2.y());
	EXPECT_FLOAT_EQ(raycastInfo6.worldPoint.z(), hitPoint2.z());

	ephysics::Ray ray1(tmp.m_localShapeToWorld * vec3(0, 5, 0), tmp.m_localShapeToWorld * vec3(5, 7, 5));
	ephysics::Ray ray2(tmp.m_localShapeToWorld * vec3(-4, -4, 7), tmp.m_localShapeToWorld * vec3(-4, 15, 7));
	ephysics::Ray ray3(tmp.m_localShapeToWorld * vec3(23, 7, 2), tmp.m_localShapeToWorld * vec3(23, 1, 2));
	ephysics::Ray ray4(tmp.m_localShapeToWorld * vec3(10, 3, 10), tmp.m_localShapeToWorld * vec3(22, 3, 31));
	ephysics::Ray ray5(tmp.m_localShapeToWorld * vec3(4, 10, -1), tmp.m_localShapeToWorld * vec3(4, 3, -1));

	ephysics::Ray ray11(tmp.m_localShapeToWorld * vec3(3, 15, 0.5), tmp.m_localShapeToWorld * vec3(3, 1, 0.5));
	ephysics::Ray ray12(tmp.m_localShapeToWorld * vec3(0, 45, 0), tmp.m_localShapeToWorld * vec3(0, -5, 0));
	ephysics::Ray ray13(tmp.m_localShapeToWorld * vec3(1, 23, 2), tmp.m_localShapeToWorld * vec3(1, -23, 2));
	ephysics::Ray ray14(tmp.m_localShapeToWorld * vec3(3, 8, 3), tmp.m_localShapeToWorld * vec3(3, 0, 3));

	// ----- Test raycast miss ----- //
	EXPECT_EQ(false, tmp.m_heightFieldBody->raycast(ray1, raycastInfo3));
	EXPECT_EQ(false, tmp.m_heightFieldProxyShape->raycast(ray1, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray1, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray1.point1, ray1.point2, float(0.01)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray1.point1, ray1.point2, float(100.0)), &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_heightFieldBody->raycast(ray2, raycastInfo3));
	EXPECT_EQ(false, tmp.m_heightFieldProxyShape->raycast(ray2, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray2, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_heightFieldBody->raycast(ray3, raycastInfo3));
	EXPECT_EQ(false, tmp.m_heightFieldProxyShape->raycast(ray3, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray3, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_heightFieldBody->raycast(ray4, raycastInfo3));
	EXPECT_EQ(false, tmp.m_heightFieldProxyShape->raycast(ray4, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray4, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	EXPECT_EQ(false, tmp.m_heightFieldBody->raycast(ray5, raycastInfo3));
	EXPECT_EQ(false, tmp.m_heightFieldProxyShape->raycast(ray5, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray5, &tmp.m_callback);
	EXPECT_EQ(false, tmp.m_callback.isHit);

	tmp.m_callback.reset();

	// ----- Test raycast hits ----- //
	EXPECT_EQ(true, tmp.m_heightFieldBody->raycast(ray11, raycastInfo3));
	EXPECT_EQ(true, tmp.m_heightFieldProxyShape->raycast(ray11, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray11, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray11.point1, ray11.point2, float(0.95)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_heightFieldBody->raycast(ray12, raycastInfo3));
	EXPECT_EQ(true, tmp.m_heightFieldProxyShape->raycast(ray12, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray12, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray12.point1, ray12.point2, float(0.87)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_heightFieldBody->raycast(ray13, raycastInfo3));
	EXPECT_EQ(true, tmp.m_heightFieldProxyShape->raycast(ray13, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray13, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray13.point1, ray13.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);

	EXPECT_EQ(true, tmp.m_heightFieldBody->raycast(ray14, raycastInfo3));
	EXPECT_EQ(true, tmp.m_heightFieldProxyShape->raycast(ray14, raycastInfo3));
	tmp.m_callback.reset();
	tmp.m_world->raycast(ray14, &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
	tmp.m_callback.reset();
	tmp.m_world->raycast(ephysics::Ray(ray14.point1, ray14.point2, float(0.8)), &tmp.m_callback);
	EXPECT_EQ(true, tmp.m_callback.isHit);
}

