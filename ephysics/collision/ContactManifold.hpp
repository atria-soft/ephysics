/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

#include <vector>
#include <ephysics/body/CollisionBody.hpp>
#include <ephysics/collision/ProxyShape.hpp>
#include <ephysics/constraint/ContactPoint.hpp>
#include <ephysics/memory/MemoryAllocator.hpp>

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
			MemoryAllocator& m_memoryAllocator; //!< Reference to the memory allocator
			/// Private copy-constructor
			ContactManifold(const ContactManifold& _contactManifold) = delete;
			/// Private assignment operator
			ContactManifold& operator=(const ContactManifold& _contactManifold) = delete;
			/// Return the index of maximum area
			int32_t getMaxArea(float _area0, float _area1, float _area2, float _area3) const;
			/// Return the index of the contact with the larger penetration depth.
			int32_t getIndexOfDeepestPenetration(ContactPoint* _newContact) const;
			/// Return the index that will be removed.
			int32_t getIndexToRemove(int32_t _indexMaxPenetration, const vec3& _newPoint) const;
			/// Remove a contact point from the manifold
			void removeContactPoint(uint32_t _index);
			/// Return true if the contact manifold has already been added int32_to an island
			bool isAlreadyInIsland() const;
		public:
			/// Constructor
			ContactManifold(ProxyShape* _shape1,
			                ProxyShape* _shape2,
			                MemoryAllocator& _memoryAllocator,
			                int16_t _normalDirectionId);
			/// Destructor
			~ContactManifold();
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
			/// Update the contact manifold.
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

// Return a pointer to the first proxy shape of the contact
ProxyShape* ContactManifold::getShape1() const {
	return m_shape1;
}

// Return a pointer to the second proxy shape of the contact
ProxyShape* ContactManifold::getShape2() const {
	return m_shape2;
}

// Return a pointer to the first body of the contact manifold
CollisionBody* ContactManifold::getBody1() const {
	return m_shape1->getBody();
}

// Return a pointer to the second body of the contact manifold
CollisionBody* ContactManifold::getBody2() const {
	return m_shape2->getBody();
}

// Return the normal direction Id
int16_t ContactManifold::getNormalDirectionId() const {
	return m_normalDirectionId;
}

// Return the number of contact points in the manifold
uint32_t ContactManifold::getNbContactPoints() const {
	return m_nbContactPoints;
}

// Return the first friction vector at the center of the contact manifold
const vec3& ContactManifold::getFrictionVector1() const {
	return m_frictionVector1;
}

// set the first friction vector at the center of the contact manifold
void ContactManifold::setFrictionVector1(const vec3& frictionVector1) {
	m_frictionVector1 = frictionVector1;
}

// Return the second friction vector at the center of the contact manifold
const vec3& ContactManifold::getFrictionvec2() const {
	return m_frictionvec2;
}

// set the second friction vector at the center of the contact manifold
void ContactManifold::setFrictionvec2(const vec3& frictionvec2) {
	m_frictionvec2 = frictionvec2;
}

// Return the first friction accumulated impulse
float ContactManifold::getFrictionImpulse1() const {
	return m_frictionImpulse1;
}

// Set the first friction accumulated impulse
void ContactManifold::setFrictionImpulse1(float frictionImpulse1) {
	m_frictionImpulse1 = frictionImpulse1;
}

// Return the second friction accumulated impulse
float ContactManifold::getFrictionImpulse2() const {
	return m_frictionImpulse2;
}

// Set the second friction accumulated impulse
void ContactManifold::setFrictionImpulse2(float frictionImpulse2) {
	m_frictionImpulse2 = frictionImpulse2;
}

// Return the friction twist accumulated impulse
float ContactManifold::getFrictionTwistImpulse() const {
	return m_frictionTwistImpulse;
}

// Set the friction twist accumulated impulse
void ContactManifold::setFrictionTwistImpulse(float frictionTwistImpulse) {
	m_frictionTwistImpulse = frictionTwistImpulse;
}

// Set the accumulated rolling resistance impulse
void ContactManifold::setRollingResistanceImpulse(const vec3& rollingResistanceImpulse) {
	m_rollingResistanceImpulse = rollingResistanceImpulse;
}

// Return a contact point of the manifold
ContactPoint* ContactManifold::getContactPoint(uint32_t index) const {
	assert(index < m_nbContactPoints);
	return m_contactPoints[index];
}

// Return true if the contact manifold has already been added int32_to an island
bool ContactManifold::isAlreadyInIsland() const {
	return m_isAlreadyInIsland;
}

// Return the normalized averaged normal vector
vec3 ContactManifold::getAverageContactNormal() const {
	vec3 averageNormal;
	for (uint32_t i=0; i<m_nbContactPoints; i++) {
		averageNormal += m_contactPoints[i]->getNormal();
	}
	return averageNormal.safeNormalized();
}

// Return the largest depth of all the contact points
float ContactManifold::getLargestContactDepth() const {
	float largestDepth = 0.0f;
	for (uint32_t i=0; i<m_nbContactPoints; i++) {
		float depth = m_contactPoints[i]->getPenetrationDepth();
		if (depth > largestDepth) {
			largestDepth = depth;
		}
	}
	return largestDepth;
}

}

