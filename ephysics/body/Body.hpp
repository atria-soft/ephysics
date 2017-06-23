/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

#include <stdexcept>
#include <cassert>
#include <ephysics/configuration.hpp>

namespace ephysics {
	/**
	 * @brief Represent a body of the physics engine. You should not
	 * instantiante this class but instantiate the CollisionBody or RigidBody
	 * classes instead.
	 */
	class Body {
		protected :
			bodyindex m_id; //!< ID of the body
			bool m_isAlreadyInIsland; //!< True if the body has already been added in an island (for sleeping technique)
			bool m_isAllowedToSleep; //!< True if the body is allowed to go to sleep for better efficiency
			/**
			 * @brief True if the body is active.
			 * An inactive body does not participate in collision detection, is not simulated and will not be hit in a ray casting query.
			 * A body is active by default. If you set this value to "false", all the proxy shapes of this body will be removed from the broad-phase.
			 * If you set this value to "true", all the proxy shapes will be added to the broad-phase.
			 * A joint connected to an inactive body will also be inactive.
			 */
			bool m_isActive;
			bool m_isSleeping; //!< True if the body is sleeping (for sleeping technique)
			float m_sleepTime; //!< Elapsed time since the body velocity was bellow the sleep velocity
			void* m_userData; //!< Pointer that can be used to attach user data to the body
			/// Private copy-constructor
			Body(const Body& body) = delete;
			/// Private assignment operator
			Body& operator=(const Body& body) = delete;
		public :
			/**
			 * @brief Constructor
			 * @param[in] _id ID of the new body
			 */
			Body(bodyindex _id);
			/**
			 * @brtief Virtualize Destructor
			 */
			virtual ~Body() = default;
			/**
			 * @brief Return the id of the body
			 * @return The ID of the body
			 */
			bodyindex getID() const {
				return m_id;
			}
			/**
			 * @brief Return whether or not the body is allowed to sleep
			 * @return True if the body is allowed to sleep and false otherwise
			 */
			bool isAllowedToSleep() const {
				return m_isAllowedToSleep;
			}
			/**
			 * @brief Set whether or not the body is allowed to go to sleep
			 * @param[in] _isAllowedToSleep True if the body is allowed to sleep
			 */
			void setIsAllowedToSleep(bool _isAllowedToSleep) {
				m_isAllowedToSleep = _isAllowedToSleep;
				if (!m_isAllowedToSleep) {
					setIsSleeping(false);
				}
			}
			/**
			 * @brief Return whether or not the body is sleeping
			 * @return True if the body is currently sleeping and false otherwise
			 */
			bool isSleeping() const {
				return m_isSleeping;
			}
			/**
			 * @brief Return true if the body is active
			 * @return True if the body currently active and false otherwise
			 */
			bool isActive() const {
				return m_isActive;
			}
			/**
			 * @brief Set whether or not the body is active
			 * @param[in] _isActive True if you want to activate the body
			 */
			void setIsActive(bool _isActive) {
				m_isActive = _isActive;
			}
			/**
			 * @brief Set the variable to know whether or not the body is sleeping
			 * @param[in] _isSleeping Set the new status
			 */
			void setIsSleeping(bool _isSleeping) {
				if (_isSleeping) {
					m_sleepTime = 0.0f;
				} else {
					if (m_isSleeping) {
						m_sleepTime = 0.0f;
					}
				}
				m_isSleeping = _isSleeping;
			}
			/**
			 * @brief Return a pointer to the user data attached to this body
			 * @return A pointer to the user data you have attached to the body
			 */
			void* getUserData() const {
				return m_userData;
			}
			/**
			 * @brief Attach user data to this body
			 * @param[in] _userData A pointer to the user data you want to attach to the body
			 */
			void setUserData(void* _userData) {
				m_userData = _userData;
			}
			/**
			 * @brief Smaller than operator
			 * @param[in] _obj Other object to compare
			 * @return true if the current element is smaller
			 */
			bool operator<(const Body& _obj) const {
				return (m_id < _obj.m_id);
			}
			/**
			 * @brief Larger than operator
			 * @param[in] _obj Other object to compare
			 * @return true if the current element is Bigger
			 */
			bool operator>(const Body& _obj) const {
				return (m_id > _obj.m_id);
			}
			/**
			 * @brief Equal operator
			 * @param[in] _obj Other object to compare
			 * @return true if the curretn element is equal
			 */
			bool operator==(const Body& _obj) const {
				return (m_id == _obj.m_id);
			}
			/**
			 * @brief Not equal operator
			 * @param[in] _obj Other object to compare
			 * @return true if the curretn element is NOT equal
			 */
			bool operator!=(const Body& _obj) const {
				return (m_id != _obj.m_id);
			}
			friend class DynamicsWorld;
	};

}
