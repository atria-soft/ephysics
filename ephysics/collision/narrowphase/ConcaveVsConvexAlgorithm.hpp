/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

#include <ephysics/collision/narrowphase/NarrowPhaseAlgorithm.hpp>
#include <ephysics/collision/shapes/ConvexShape.hpp>
#include <ephysics/collision/shapes/ConcaveShape.hpp>
#include <unordered_map>

namespace ephysics {

	/**
	 * @brief This class is used to encapsulate a callback method for
	 * collision detection between the triangle of a concave mesh shape
	 * and a convex shape.
	 */
	class ConvexVsTriangleCallback : public TriangleCallback {
		protected:
			CollisionDetection* m_collisionDetection; //!< Pointer to the collision detection object
			NarrowPhaseCallback* m_narrowPhaseCallback; //!< Narrow-phase collision callback
			const ConvexShape* m_convexShape; //!< Convex collision shape to test collision with
			const ConcaveShape* m_concaveShape; //!< Concave collision shape
			ProxyShape* m_convexProxyShape; //!< Proxy shape of the convex collision shape
			ProxyShape* m_concaveProxyShape; //!< Proxy shape of the concave collision shape
			OverlappingPair* m_overlappingPair; //!< Broadphase overlapping pair
			static bool contactsDepthCompare(const ContactPointInfo& _contact1,
			                                 const ContactPointInfo& _contact2);
		public:
			/// Set the collision detection pointer
			void setCollisionDetection(CollisionDetection* _collisionDetection) {
				m_collisionDetection = _collisionDetection;
			}
			/// Set the narrow-phase collision callback
			void setNarrowPhaseCallback(NarrowPhaseCallback* _callback) {
				m_narrowPhaseCallback = _callback;
			}
			/// Set the convex collision shape to test collision with
			void setConvexShape(const ConvexShape* _convexShape) {
				m_convexShape = _convexShape;
			}
			/// Set the concave collision shape
			void setConcaveShape(const ConcaveShape* _concaveShape) {
				m_concaveShape = _concaveShape;
			}
			/// Set the broadphase overlapping pair
			void setOverlappingPair(OverlappingPair* _overlappingPair) {
				m_overlappingPair = _overlappingPair;
			}
			/// Set the proxy shapes of the two collision shapes
			void setProxyShapes(ProxyShape* _convexProxyShape, ProxyShape* _concaveProxyShape) {
				m_convexProxyShape = _convexProxyShape;
				m_concaveProxyShape = _concaveProxyShape;
			}
			/// Test collision between a triangle and the convex mesh shape
			virtual void testTriangle(const vec3* _trianglePoints);
	};

	/**
	 * @brief This class is used to store data about a contact with a triangle for the smooth
	 * mesh algorithm.
	 */
	class SmoothMeshContactInfo {
		public:
			ContactPointInfo contactInfo;
			bool isFirstShapeTriangle;
			vec3 triangleVertices[3];
			/// Constructor
			SmoothMeshContactInfo(const ContactPointInfo& _contact,
			                      bool _firstShapeTriangle,
			                      const vec3& _trianglePoint1,
			                      const vec3& _trianglePoint2,
			                      const vec3& _trianglePoint3):
			  contactInfo(_contact) {
				isFirstShapeTriangle = _firstShapeTriangle;
				triangleVertices[0] = _trianglePoint1;
				triangleVertices[1] = _trianglePoint2;
				triangleVertices[2] = _trianglePoint3;
			}
	};

	struct ContactsDepthCompare {
		bool operator()(const SmoothMeshContactInfo& _contact1, const SmoothMeshContactInfo& _contact2) {
			return _contact1.contactInfo.penetrationDepth < _contact2.contactInfo.penetrationDepth;
		}
	};

	/**
	 * @brief This class is used as a narrow-phase callback to get narrow-phase contacts
	 * of the concave triangle mesh to temporary store them in order to be used in
	 * the smooth mesh collision algorithm if this one is enabled.
	 */
	class SmoothCollisionNarrowPhaseCallback : public NarrowPhaseCallback {
		private:
			std::vector<SmoothMeshContactInfo>& m_contactPoints;
		public:
			// Constructor
			SmoothCollisionNarrowPhaseCallback(std::vector<SmoothMeshContactInfo>& _contactPoints):
			  m_contactPoints(_contactPoints) {
				
			}
			/// Called by a narrow-phase collision algorithm when a new contact has been found
			virtual void notifyContact(OverlappingPair* _overlappingPair, const ContactPointInfo& _contactInfo);
	};

	/**
	 * @brief This class is used to compute the narrow-phase collision detection
	 * between a concave collision shape and a convex collision shape. The idea is
	 * to use the GJK collision detection algorithm to compute the collision between
	 * the convex shape and each of the triangles in the concave shape.
	 */
	class ConcaveVsConvexAlgorithm : public NarrowPhaseAlgorithm {
		protected :
			/// Private copy-constructor
			ConcaveVsConvexAlgorithm(const ConcaveVsConvexAlgorithm& _algorithm);
			/// Private assignment operator
			ConcaveVsConvexAlgorithm& operator=(const ConcaveVsConvexAlgorithm& _algorithm);
			/// Process the concave triangle mesh collision using the smooth mesh collision algorithm
			void processSmoothMeshCollision(OverlappingPair* _overlappingPair,
			                                std::vector<SmoothMeshContactInfo> _contactPoints,
			                                NarrowPhaseCallback* _narrowPhaseCallback);
			/// Add a triangle vertex int32_to the set of processed triangles
			void addProcessedVertex(std::unordered_multimap<int32_t, vec3>& _processTriangleVertices,
			                        const vec3& _vertex) {
				processTriangleVertices.insert(std::make_pair(int32_t(vertex.x() * vertex.y() * vertex.z()), vertex));
			}
			/// Return true if the vertex is in the set of already processed vertices
			bool hasVertexBeenProcessed(const std::unordered_multimap<int32_t, vec3>& _processTriangleVertices,
			                            const vec3& _vertex) const;
		public :
			/// Constructor
			ConcaveVsConvexAlgorithm();
			/// Destructor
			virtual ~ConcaveVsConvexAlgorithm();
			/// Compute a contact info if the two bounding volume collide
			virtual void testCollision(const CollisionShapeInfo& _shape1Info,
			                           const CollisionShapeInfo& _shape2Info,
			                           NarrowPhaseCallback* _narrowPhaseCallback);
	};

}

