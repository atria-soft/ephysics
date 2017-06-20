/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

#include <ephysics/body/CollisionBody.hpp>
#include <ephysics/collision/CollisionShapeInfo.hpp>
#include <ephysics/configuration.hpp>
#include <ephysics/mathematics/mathematics.hpp>
#include <ephysics/configuration.hpp>

namespace ephysics {

	/**
	 * @brief This structure contains informations about a collision contact
	 * computed during the narrow-phase collision detection. Those
	 * informations are used to compute the contact set for a contact
	 * between two bodies.
	 */
	struct ContactPointInfo {
		public:
			/// First proxy shape of the contact
			ProxyShape* shape1;
			/// Second proxy shape of the contact
			ProxyShape* shape2;
			/// First collision shape
			const CollisionShape* collisionShape1;
			/// Second collision shape
			const CollisionShape* collisionShape2;
			/// Normalized normal vector of the collision contact in world space
			vec3 normal;
			/// Penetration depth of the contact
			float penetrationDepth;
			/// Contact point of body 1 in local space of body 1
			vec3 localPoint1;
			/// Contact point of body 2 in local space of body 2
			vec3 localPoint2;
			/// Constructor
			ContactPointInfo(ProxyShape* _proxyShape1,
			                 ProxyShape* _proxyShape2,
			                 const CollisionShape* _collShape1,
			                 const CollisionShape* _collShape2,
			                 const vec3& _normal,
			                 float _penetrationDepth,
			                 const vec3& _localPoint1,
			                 const vec3& _localPoint2):
			  shape1(_proxyShape1),
			  shape2(_proxyShape2),
			  collisionShape1(_collShape1),
			  collisionShape2(_collShape2),
			  normal(_normal),
			  penetrationDepth(_penetrationDepth),
			  localPoint1(_localPoint1),
			  localPoint2(_localPoint2) {
				
			}
	};

	/**
	 * @brief This class represents a collision contact point between two
	 * bodies in the physics engine.
	 */
	class ContactPoint {
		private :
			CollisionBody* m_body1; //!< First rigid body of the contact
			CollisionBody* m_body2; //!< Second rigid body of the contact
			const vec3 m_normal; //!< Normalized normal vector of the contact (from body1 toward body2) in world space
			float m_penetrationDepth; //!< Penetration depth
			const vec3 m_localPointOnBody1; //!< Contact point on body 1 in local space of body 1
			const vec3 m_localPointOnBody2; //!< Contact point on body 2 in local space of body 2
			vec3 m_worldPointOnBody1; //!< Contact point on body 1 in world space
			vec3 m_worldPointOnBody2; //!< Contact point on body 2 in world space
			bool m_isRestingContact; //!< True if the contact is a resting contact (exists for more than one time step)
			vec3 m_frictionVectors[2]; //!< Two orthogonal vectors that span the tangential friction plane
			float m_penetrationImpulse; //!< Cached penetration impulse
			float m_frictionImpulse1; //!< Cached first friction impulse
			float m_frictionImpulse2; //!< Cached second friction impulse
			vec3 m_rollingResistanceImpulse; //!< Cached rolling resistance impulse
			/// Private copy-constructor
			ContactPoint(const ContactPoint& contact) = delete;
			/// Private assignment operator
			ContactPoint& operator=(const ContactPoint& contact) = delete;
		public :
			/// Constructor
			ContactPoint(const ContactPointInfo& contactInfo);
			/// Destructor
			~ContactPoint();
			/// Return the reference to the body 1
			CollisionBody* getBody1() const;
			/// Return the reference to the body 2
			CollisionBody* getBody2() const;
			/// Return the normal vector of the contact
			vec3 getNormal() const;
			/// Set the penetration depth of the contact
			void setPenetrationDepth(float _penetrationDepth);
			/// Return the contact local point on body 1
			vec3 getLocalPointOnBody1() const;
			/// Return the contact local point on body 2
			vec3 getLocalPointOnBody2() const;
			/// Return the contact world point on body 1
			vec3 getWorldPointOnBody1() const;
			/// Return the contact world point on body 2
			vec3 getWorldPointOnBody2() const;
			/// Return the cached penetration impulse
			float getPenetrationImpulse() const;
			/// Return the cached first friction impulse
			float getFrictionImpulse1() const;
			/// Return the cached second friction impulse
			float getFrictionImpulse2() const;
			/// Return the cached rolling resistance impulse
			vec3 getRollingResistanceImpulse() const;
			/// Set the cached penetration impulse
			void setPenetrationImpulse(float _impulse);
			/// Set the first cached friction impulse
			void setFrictionImpulse1(float _impulse);
			/// Set the second cached friction impulse
			void setFrictionImpulse2(float _impulse);
			/// Set the cached rolling resistance impulse
			void setRollingResistanceImpulse(const vec3& _impulse);
			/// Set the contact world point on body 1
			void setWorldPointOnBody1(const vec3& _worldPoint);
			/// Set the contact world point on body 2
			void setWorldPointOnBody2(const vec3& _worldPoint);
			/// Return true if the contact is a resting contact
			bool getIsRestingContact() const;
			/// Set the m_isRestingContact variable
			void setIsRestingContact(bool _isRestingContact);
			/// Get the first friction vector
			vec3 getFrictionVector1() const;
			/// Set the first friction vector
			void setFrictionVector1(const vec3& _frictionVector1);
			/// Get the second friction vector
			vec3 getFrictionvec2() const;
			/// Set the second friction vector
			void setFrictionvec2(const vec3& _frictionvec2);
			/// Return the penetration depth
			float getPenetrationDepth() const;
			/// Return the number of bytes used by the contact point
			size_t getSizeInBytes() const;
	};

// Return the reference to the body 1
CollisionBody* ContactPoint::getBody1() const {
	return m_body1;
}

// Return the reference to the body 2
CollisionBody* ContactPoint::getBody2() const {
	return m_body2;
}

// Return the normal vector of the contact
vec3 ContactPoint::getNormal() const {
	return m_normal;
}

// Set the penetration depth of the contact
void ContactPoint::setPenetrationDepth(float penetrationDepth) {
	this->m_penetrationDepth = penetrationDepth;
}

// Return the contact point on body 1
vec3 ContactPoint::getLocalPointOnBody1() const {
	return m_localPointOnBody1;
}

// Return the contact point on body 2
vec3 ContactPoint::getLocalPointOnBody2() const {
	return m_localPointOnBody2;
}

// Return the contact world point on body 1
vec3 ContactPoint::getWorldPointOnBody1() const {
	return m_worldPointOnBody1;
}

// Return the contact world point on body 2
vec3 ContactPoint::getWorldPointOnBody2() const {
	return m_worldPointOnBody2;
}

// Return the cached penetration impulse
float ContactPoint::getPenetrationImpulse() const {
	return m_penetrationImpulse;
}

// Return the cached first friction impulse
float ContactPoint::getFrictionImpulse1() const {
	return m_frictionImpulse1;
}

// Return the cached second friction impulse
float ContactPoint::getFrictionImpulse2() const {
	return m_frictionImpulse2;
}

// Return the cached rolling resistance impulse
vec3 ContactPoint::getRollingResistanceImpulse() const {
	return m_rollingResistanceImpulse;
}

// Set the cached penetration impulse
void ContactPoint::setPenetrationImpulse(float impulse) {
	m_penetrationImpulse = impulse;
}

// Set the first cached friction impulse
void ContactPoint::setFrictionImpulse1(float impulse) {
	m_frictionImpulse1 = impulse;
}

// Set the second cached friction impulse
void ContactPoint::setFrictionImpulse2(float impulse) {
	m_frictionImpulse2 = impulse;
}

// Set the cached rolling resistance impulse
void ContactPoint::setRollingResistanceImpulse(const vec3& impulse) {
	m_rollingResistanceImpulse = impulse;
}

// Set the contact world point on body 1
void ContactPoint::setWorldPointOnBody1(const vec3& worldPoint) {
	m_worldPointOnBody1 = worldPoint;
}

// Set the contact world point on body 2
void ContactPoint::setWorldPointOnBody2(const vec3& worldPoint) {
	m_worldPointOnBody2 = worldPoint;
}

// Return true if the contact is a resting contact
bool ContactPoint::getIsRestingContact() const {
	return m_isRestingContact;
}

// Set the m_isRestingContact variable
void ContactPoint::setIsRestingContact(bool isRestingContact) {
	m_isRestingContact = isRestingContact;
}

// Get the first friction vector
vec3 ContactPoint::getFrictionVector1() const {
	return m_frictionVectors[0];
}

// Set the first friction vector
void ContactPoint::setFrictionVector1(const vec3& frictionVector1) {
	m_frictionVectors[0] = frictionVector1;
}

// Get the second friction vector
vec3 ContactPoint::getFrictionvec2() const {
	return m_frictionVectors[1];
}

// Set the second friction vector
void ContactPoint::setFrictionvec2(const vec3& frictionvec2) {
	m_frictionVectors[1] = frictionvec2;
}

// Return the penetration depth of the contact
float ContactPoint::getPenetrationDepth() const {
	return m_penetrationDepth;
}

// Return the number of bytes used by the contact point
size_t ContactPoint::getSizeInBytes() const {
	return sizeof(ContactPoint);
}

}