/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

#include <ephysics/memory/MemoryAllocator.hpp>
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
			RigidBody** m_bodies; //!< Array with all the bodies of the island
			ContactManifold** m_contactManifolds; //!< Array with all the contact manifolds between bodies of the island
			Joint** m_joints; //!< Array with all the joints between bodies of the island
			uint32_t m_numberBodies; //!< Current number of bodies in the island
			uint32_t m_numberContactManifolds; //!< Current number of contact manifold in the island
			uint32_t m_numberJoints; //!< Current number of joints in the island
			MemoryAllocator& m_memoryAllocator; //!< Reference to the memory allocator
			size_t m_numberAllocatedBytesBodies; //!< Number of bytes allocated for the bodies array
			size_t m_numberAllocatedBytesContactManifolds; //!< Number of bytes allocated for the contact manifolds array
			size_t m_numberAllocatedBytesJoints; //!< Number of bytes allocated for the joints array
			/// Private assignment operator
			Island& operator=(const Island& island);
			/// Private copy-constructor
			Island(const Island& island);
		public:
			/// Constructor
			Island(uint32_t nbMaxBodies, uint32_t nbMaxContactManifolds, uint32_t nbMaxJoints,
				   MemoryAllocator& memoryAllocator);
			/// Destructor
			~Island();
			/// Add a body int32_to the island
			void addBody(RigidBody* body);
			/// Add a contact manifold int32_to the island
			void addContactManifold(ContactManifold* contactManifold);
			/// Add a joint int32_to the island
			void addJoint(Joint* joint);
			/// Return the number of bodies in the island
			uint32_t getNbBodies() const;
			/// Return the number of contact manifolds in the island
			uint32_t getNbContactManifolds() const;
			/// Return the number of joints in the island
			uint32_t getNbJoints() const;
			/// Return a pointer to the array of bodies
			RigidBody** getBodies();
			/// Return a pointer to the array of contact manifolds
			ContactManifold** getContactManifold();
			/// Return a pointer to the array of joints
			Joint** getJoints();
			friend class DynamicsWorld;
	};


}
