/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <stdexcept>
#include <cassert>
#include <ephysics/configuration.h>

/// Namespace reactphysics3d
namespace reactphysics3d {

// TODO : Make this class abstract
// Class Body
/**
 * This class to represent a body of the physics engine. You should not
 * instantiante this class but instantiate the CollisionBody or RigidBody
 * classes instead.
 */
class Body {

	protected :

		// -------------------- Attributes -------------------- //

		/// ID of the body
		bodyindex m_id;

		/// True if the body has already been added in an island (for sleeping technique)
		bool m_isAlreadyInIsland;

		/// True if the body is allowed to go to sleep for better efficiency
		bool m_isAllowedToSleep;

		/// True if the body is active.
		/// An inactive body does not participate in collision detection,
		/// is not simulated and will not be hit in a ray casting query.
		/// A body is active by default. If you set this
		/// value to "false", all the proxy shapes of this body will be
		/// removed from the broad-phase. If you set this value to "true",
		/// all the proxy shapes will be added to the broad-phase. A joint
		/// connected to an inactive body will also be inactive.
		bool m_isActive;

		/// True if the body is sleeping (for sleeping technique)
		bool m_isSleeping;

		/// Elapsed time since the body velocity was bellow the sleep velocity
		float m_sleepTime;

		/// Pointer that can be used to attach user data to the body
		void* m_userData;

		// -------------------- Methods -------------------- //

		/// Private copy-constructor
		Body(const Body& body);

		/// Private assignment operator
		Body& operator=(const Body& body);

	public :

		// -------------------- Methods -------------------- //

		/// Constructor
		Body(bodyindex id);

		/// Destructor
		virtual ~Body();

		/// Return the ID of the body
		bodyindex getID() const;

		/// Return whether or not the body is allowed to sleep
		bool isAllowedToSleep() const;

		/// Set whether or not the body is allowed to go to sleep
		void setIsAllowedToSleep(bool isAllowedToSleep);

		/// Set the variable to know whether or not the body is sleeping
		virtual void setIsSleeping(bool isSleeping);

		/// Return whether or not the body is sleeping
		bool isSleeping() const;

		/// Return true if the body is active
		bool isActive() const;

		/// Set whether or not the body is active
		virtual void setIsActive(bool isActive);

		/// Return a pointer to the user data attached to this body
		void* getUserData() const;

		/// Attach user data to this body
		void setUserData(void* userData);

		/// Smaller than operator
		bool operator<(const Body& body2) const;

		/// Larger than operator
		bool operator>(const Body& body2) const;

		/// Equal operator
		bool operator==(const Body& body2) const;

		/// Not equal operator
		bool operator!=(const Body& body2) const;

		// -------------------- Friendship -------------------- //

		friend class DynamicsWorld;
};

// Return the id of the body
/**
 * @return The ID of the body
 */
inline bodyindex Body::getID() const {
	return m_id;
}

// Return whether or not the body is allowed to sleep
/**
 * @return True if the body is allowed to sleep and false otherwise
 */
inline bool Body::isAllowedToSleep() const {
	return m_isAllowedToSleep;
}

// Set whether or not the body is allowed to go to sleep
/**
 * @param isAllowedToSleep True if the body is allowed to sleep
 */
inline void Body::setIsAllowedToSleep(bool isAllowedToSleep) {
	m_isAllowedToSleep = isAllowedToSleep;

	if (!m_isAllowedToSleep) setIsSleeping(false);
}

// Return whether or not the body is sleeping
/**
 * @return True if the body is currently sleeping and false otherwise
 */
inline bool Body::isSleeping() const {
	return m_isSleeping;
}

// Return true if the body is active
/**
 * @return True if the body currently active and false otherwise
 */
inline bool Body::isActive() const {
	return m_isActive;
}

// Set whether or not the body is active
/**
 * @param isActive True if you want to activate the body
 */
inline void Body::setIsActive(bool isActive) {
	m_isActive = isActive;
}

// Set the variable to know whether or not the body is sleeping
inline void Body::setIsSleeping(bool isSleeping) {

	if (isSleeping) {
		m_sleepTime = 0.0f;
	}
	else {
		if (m_isSleeping) {
			m_sleepTime = 0.0f;
		}
	}

	m_isSleeping = isSleeping;
}

// Return a pointer to the user data attached to this body
/**
 * @return A pointer to the user data you have attached to the body
 */
inline void* Body::getUserData() const {
	return m_userData;
}

// Attach user data to this body
/**
 * @param userData A pointer to the user data you want to attach to the body
 */
inline void Body::setUserData(void* userData) {
	m_userData = userData;
}

// Smaller than operator
inline bool Body::operator<(const Body& body2) const {
	return (m_id < body2.m_id);
} 

// Larger than operator
inline bool Body::operator>(const Body& body2) const {
	return (m_id > body2.m_id);
} 

// Equal operator
inline bool Body::operator==(const Body& body2) const {
	return (m_id == body2.m_id);
}
		
// Not equal operator
inline bool Body::operator!=(const Body& body2) const {
	return (m_id != body2.m_id);
}			   

}
