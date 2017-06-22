/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/constraint/ContactPoint.hpp>
#include <ephysics/collision/ProxyShape.hpp>

using namespace ephysics;
using namespace std;

// Constructor
ContactPoint::ContactPoint(const ContactPointInfo& contactInfo)
			 : m_body1(contactInfo.shape1->getBody()), m_body2(contactInfo.shape2->getBody()),
			   m_normal(contactInfo.normal),
			   m_penetrationDepth(contactInfo.penetrationDepth),
			   m_localPointOnBody1(contactInfo.localPoint1),
			   m_localPointOnBody2(contactInfo.localPoint2),
			   m_worldPointOnBody1(contactInfo.shape1->getBody()->getTransform() *
								  contactInfo.shape1->getLocalToBodyTransform() *
								  contactInfo.localPoint1),
			   m_worldPointOnBody2(contactInfo.shape2->getBody()->getTransform() *
								  contactInfo.shape2->getLocalToBodyTransform() *
								  contactInfo.localPoint2),
			   m_isRestingContact(false) {

	m_frictionVectors[0] = vec3(0, 0, 0);
	m_frictionVectors[1] = vec3(0, 0, 0);

	assert(m_penetrationDepth > 0.0);

}

// Destructor
ContactPoint::~ContactPoint() {

}

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
