/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#pragma once
#include <ephysics/collision/ContactManifoldSet.hpp>
#include <ephysics/collision/ProxyShape.hpp>
#include <ephysics/collision/shapes/CollisionShape.hpp>

namespace ephysics {
	// Type for the overlapping pair ID
	typedef etk::Pair<uint32_t, uint32_t> overlappingpairid;
	/**
	 * @brief This class represents a pair of two proxy collision shapes that are overlapping
	 * during the broad-phase collision detection. It is created when
	 * the two proxy collision shapes start to overlap and is destroyed when they do not
	 * overlap anymore. This class contains a contact manifold that
	 * store all the contact points between the two bodies.
	 */
	class OverlappingPair {
		private:
			ContactManifoldSet m_contactManifoldSet; //!< Set of persistent contact manifolds
			vec3 m_cachedSeparatingAxis; //!< Cached previous separating axis
			/// Private copy-constructor
			OverlappingPair(const OverlappingPair& pair);
			/// Private assignment operator
			OverlappingPair& operator=(const OverlappingPair& pair);
		public:
			/// Constructor
			OverlappingPair(ProxyShape* shape1,
			                ProxyShape* shape2,
			                int32_t nbMaxContactManifolds);
			/// Return the pointer to first proxy collision shape
			ProxyShape* getShape1() const;
			/// Return the pointer to second body
			ProxyShape* getShape2() const;
			/// Add a contact to the contact cache
			void addContact(ContactPoint* contact);
			/// Update the contact cache
			void update();
			/// Return the cached separating axis
			vec3 getCachedSeparatingAxis() const;
			/// Set the cached separating axis
			void setCachedSeparatingAxis(const vec3& axis);
			/// Return the number of contacts in the cache
			uint32_t getNbContactPoints() const;
			/// Return the a reference to the contact manifold set
			const ContactManifoldSet& getContactManifoldSet();
			/// Clear the contact points of the contact manifold
			void clearContactPoints();
			/// Return the pair of bodies index
			static overlappingpairid computeID(ProxyShape* shape1, ProxyShape* shape2);
			/// Return the pair of bodies index of the pair
			static bodyindexpair computeBodiesIndexPair(CollisionBody* body1, CollisionBody* body2);
			friend class DynamicsWorld;
	};

}

