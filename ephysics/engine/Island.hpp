/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#pragma once

#include <ephysics/body/RigidBody.hpp>
#include <ephysics/constraint/Joint.hpp>
#include <ephysics/collision/ContactManifold.hpp>

namespace ephysics {
	/**
	 * @brief An island represent an isolated group of awake bodies that are connected with each other by
	 * some contraints (contacts or joints).
	 */
	class Island {
		private:
			etk::Vector<RigidBody*> m_bodies; //!< Array with all the bodies of the island
			etk::Vector<ContactManifold*> m_contactManifolds; //!< Array with all the contact manifolds between bodies of the island
			etk::Vector<Joint*> m_joints; //!< Array with all the joints between bodies of the island
			//! Remove assignment operator
			Island& operator=(const Island& island) = delete;
			//! Remove copy-constructor
			Island(const Island& island) = delete;
		public:
			/**
			 * @brief Constructor
			 */
			Island(size_t nbMaxBodies, size_t nbMaxContactManifolds, size_t nbMaxJoints);
			/**
			 * @brief Destructor
			 */
			~Island() = default;
			/** 
			 * Add a body.
			 */
			void addBody(RigidBody* _body);
			/** 
			 * Add a contact manifold.
			 */
			void addContactManifold(ContactManifold* _contactManifold);
			/** 
			 * Add a joint.
			 */
			void addJoint(Joint* _joint);
			/** 
			 * @brief Get the number of body
			 * @return Number of bodies.
			 */
			size_t getNbBodies() const;
			/** 
			 * @ Get the number of contact manifolds
			 * Return the number of contact manifolds in the island
			 */
			size_t getNbContactManifolds() const;
			/** 
			 * Return the number of joints in the island
			 */
			size_t getNbJoints() const;
			/** 
			 * Return a pointer to the array of bodies
			 */
			RigidBody** getBodies();
			/** 
			 * Return a pointer to the array of contact manifolds
			 */
			ContactManifold** getContactManifold();
			/** 
			 * Return a pointer to the array of joints
			 */
			Joint** getJoints();
			/**
			 * @brief Reset the isAlreadyIsland variable of the static bodies so that they can also be included in the other islands
			 */
			void resetStaticBobyNotInIsland();
	};


}
