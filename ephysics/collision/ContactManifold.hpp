/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#pragma once

#include <etk/Vector.hpp>
#include <ephysics/body/CollisionBody.hpp>
#include <ephysics/collision/ProxyShape.hpp>
#include <ephysics/constraint/ContactPoint.hpp>

namespace ephysics {

	const uint32_t MAX_CONTACT_POINTS_IN_MANIFOLD = 4; //!< Maximum number of contacts in the manifold
	
	class ContactManifold;
	
	/**
	 * @brief This structure represents a single element of a linked list of contact manifolds
	 */
	struct ContactManifoldListElement {
		public:
			ContactManifold* contactManifold; //!< Pointer to the actual contact manifold
			ContactManifoldListElement* next; //!< Next element of the list
			ContactManifoldListElement(ContactManifold* _initContactManifold,
			                           ContactManifoldListElement* _initNext):
			  contactManifold(_initContactManifold),
			  next(_initNext) {
				
			}
	};
	
	/**
	 * @brief This class represents the set of contact points between two bodies.
	 * The contact manifold is implemented in a way to cache the contact
	 * points among the frames for better stability following the
	 * "Contact Generation" presentation of Erwin Coumans at GDC 2010
	 * conference (bullet.googlecode.com/files/GDC10_Coumans_Erwin_Contact.pdf).
	 * Some code of this class is based on the implementation of the
	 * btPersistentManifold class from Bullet physics engine (www.http://bulletphysics.org).
	 * The contacts between two bodies are added one after the other in the cache.
	 * When the cache is full, we have to remove one point. The idea is to keep
	 * the point with the deepest penetration depth and also to keep the
	 * points producing the larger area (for a more stable contact manifold).
	 * The new added point is always kept.
	 */
	class ContactManifold {
		public:
			/// Constructor
			ContactManifold(ProxyShape* _shape1,
			                ProxyShape* _shape2,
			                int16_t _normalDirectionId);
			/// Destructor
			~ContactManifold();
			/// DELETE copy-constructor
			ContactManifold(const ContactManifold& _contactManifold) = delete;
			/// DELETE assignment operator
			ContactManifold& operator=(const ContactManifold& _contactManifold) = delete;
		private:
			ProxyShape* m_shape1; //!< Pointer to the first proxy shape of the contact
			ProxyShape* m_shape2; //!< Pointer to the second proxy shape of the contact
			ContactPoint* m_contactPoints[MAX_CONTACT_POINTS_IN_MANIFOLD]; //!< Contact points in the manifold
			int16_t m_normalDirectionId; //!< Normal direction Id (Unique Id representing the normal direction)
			uint32_t m_nbContactPoints; //!< Number of contacts in the cache
			vec3 m_frictionVector1; //!< First friction vector of the contact manifold
			vec3 m_frictionvec2; //!< Second friction vector of the contact manifold
			float m_frictionImpulse1; //!< First friction constraint accumulated impulse
			float m_frictionImpulse2; //!< Second friction constraint accumulated impulse
			float m_frictionTwistImpulse; //!< Twist friction constraint accumulated impulse
			vec3 m_rollingResistanceImpulse; //!< Accumulated rolling resistance impulse
			bool m_isAlreadyInIsland; //!< True if the contact manifold has already been added int32_to an island
			/// Return the index of maximum area
			int32_t getMaxArea(float _area0, float _area1, float _area2, float _area3) const;
			/**
			 * @brief Return the index of the contact with the larger penetration depth.
			 *
			 * This corresponding contact will be kept in the cache. The method returns -1 is
			 * the new contact is the deepest.
			 */
			int32_t getIndexOfDeepestPenetration(ContactPoint* _newContact) const;
			/**
			 * @brief Return the index that will be removed.
			 * The index of the contact point with the larger penetration
			 * depth is given as a parameter. This contact won't be removed. Given this contact, we compute
			 * the different area and we want to keep the contacts with the largest area. The new point is also
			 * kept. In order to compute the area of a quadrilateral, we use the formula :
			 * Area = 0.5 * | AC x BD | where AC and BD form the diagonals of the quadrilateral. Note that
			 * when we compute this area, we do not calculate it exactly but we
			 * only estimate it because we do not compute the actual diagonals of the quadrialteral. Therefore,
			 * this is only a guess that is faster to compute. This idea comes from the Bullet Physics library
			 * by Erwin Coumans (http://wwww.bulletphysics.org).
			int32_t getIndexToRemove(int32_t _indexMaxPenetration, const vec3& _newPoint) const;
			/// Remove a contact point from the manifold
			void removeContactPoint(uint32_t _index);
			/// Return true if the contact manifold has already been added int32_to an island
			bool isAlreadyInIsland() const;
		public:
			/// Return a pointer to the first proxy shape of the contact
			ProxyShape* getShape1() const;
			/// Return a pointer to the second proxy shape of the contact
			ProxyShape* getShape2() const;
			/// Return a pointer to the first body of the contact manifold
			CollisionBody* getBody1() const;
			/// Return a pointer to the second body of the contact manifold
			CollisionBody* getBody2() const;
			/// Return the normal direction Id
			int16_t getNormalDirectionId() const;
			/// Add a contact point to the manifold
			void addContactPoint(ContactPoint* _contact);
			/**
			 * @brief Update the contact manifold.
			 * 
			 * First the world space coordinates of the current contacts in the manifold are recomputed from
			 * the corresponding transforms of the bodies because they have moved. Then we remove the contacts
			 * with a negative penetration depth (meaning that the bodies are not penetrating anymore) and also
			 * the contacts with a too large distance between the contact points in the plane orthogonal to the
			 * contact normal.
			 */
			void update(const etk::Transform3D& _transform1,
			            const etk::Transform3D& _transform2);
			/// Clear the contact manifold
			void clear();
			/// Return the number of contact points in the manifold
			uint32_t getNbContactPoints() const;
			/// Return the first friction vector at the center of the contact manifold
			const vec3& getFrictionVector1() const;
			/// set the first friction vector at the center of the contact manifold
			void setFrictionVector1(const vec3& _frictionVector1);
			/// Return the second friction vector at the center of the contact manifold
			const vec3& getFrictionvec2() const;
			/// set the second friction vector at the center of the contact manifold
			void setFrictionvec2(const vec3& _frictionvec2);
			/// Return the first friction accumulated impulse
			float getFrictionImpulse1() const;
			/// Set the first friction accumulated impulse
			void setFrictionImpulse1(float _frictionImpulse1);
			/// Return the second friction accumulated impulse
			float getFrictionImpulse2() const;
			/// Set the second friction accumulated impulse
			void setFrictionImpulse2(float _frictionImpulse2);
			/// Return the friction twist accumulated impulse
			float getFrictionTwistImpulse() const;
			/// Set the friction twist accumulated impulse
			void setFrictionTwistImpulse(float _frictionTwistImpulse);
			/// Set the accumulated rolling resistance impulse
			void setRollingResistanceImpulse(const vec3& _rollingResistanceImpulse);
			/// Return a contact point of the manifold
			ContactPoint* getContactPoint(uint32_t _index) const;
			/// Return the normalized averaged normal vector
			vec3 getAverageContactNormal() const;
			/// Return the largest depth of all the contact points
			float getLargestContactDepth() const;
			friend class DynamicsWorld;
			friend class Island;
			friend class CollisionBody;
	};

}


