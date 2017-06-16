/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/collision/ContactManifoldSet.hpp>
#include <ephysics/collision/ProxyShape.hpp>
#include <ephysics/collision/shapes/CollisionShape.hpp>

/// ReactPhysics3D namespace
namespace ephysics {

// Type for the overlapping pair ID
typedef std::pair<uint32_t, uint32_t> overlappingpairid;

// Class OverlappingPair
/**
 * This class represents a pair of two proxy collision shapes that are overlapping
 * during the broad-phase collision detection. It is created when
 * the two proxy collision shapes start to overlap and is destroyed when they do not
 * overlap anymore. This class contains a contact manifold that
 * store all the contact points between the two bodies.
 */
class OverlappingPair {

	private:

		// -------------------- Attributes -------------------- //

		/// Set of persistent contact manifolds
		ContactManifoldSet m_contactManifoldSet;

		/// Cached previous separating axis
		vec3 m_cachedSeparatingAxis;
		
		// -------------------- Methods -------------------- //

		/// Private copy-constructor
		OverlappingPair(const OverlappingPair& pair);

		/// Private assignment operator
		OverlappingPair& operator=(const OverlappingPair& pair);

	public:

		// -------------------- Methods -------------------- //

		/// Constructor
		OverlappingPair(ProxyShape* shape1, ProxyShape* shape2,
						int32_t nbMaxContactManifolds, MemoryAllocator& memoryAllocator);

		/// Destructor
		~OverlappingPair();
		
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

		// -------------------- Friendship -------------------- //

		friend class DynamicsWorld;
};

// Return the pointer to first body
inline ProxyShape* OverlappingPair::getShape1() const {
	return m_contactManifoldSet.getShape1();
}		  

// Return the pointer to second body
inline ProxyShape* OverlappingPair::getShape2() const {
	return m_contactManifoldSet.getShape2();
}				

// Add a contact to the contact manifold
inline void OverlappingPair::addContact(ContactPoint* contact) {
	m_contactManifoldSet.addContactPoint(contact);
}

// Update the contact manifold
inline void OverlappingPair::update() {
	m_contactManifoldSet.update();
}

// Return the cached separating axis
inline vec3 OverlappingPair::getCachedSeparatingAxis() const {
	return m_cachedSeparatingAxis;
}

// Set the cached separating axis
inline void OverlappingPair::setCachedSeparatingAxis(const vec3& axis) {
	m_cachedSeparatingAxis = axis;
}


// Return the number of contact points in the contact manifold
inline uint32_t OverlappingPair::getNbContactPoints() const {
	return m_contactManifoldSet.getTotalNbContactPoints();
}

// Return the contact manifold
inline const ContactManifoldSet& OverlappingPair::getContactManifoldSet() {
	return m_contactManifoldSet;
}

// Return the pair of bodies index
inline overlappingpairid OverlappingPair::computeID(ProxyShape* shape1, ProxyShape* shape2) {
	assert(shape1->m_broadPhaseID >= 0 && shape2->m_broadPhaseID >= 0);

	// Construct the pair of body index
	overlappingpairid pairID = shape1->m_broadPhaseID < shape2->m_broadPhaseID ?
							 std::make_pair(shape1->m_broadPhaseID, shape2->m_broadPhaseID) :
							 std::make_pair(shape2->m_broadPhaseID, shape1->m_broadPhaseID);
	assert(pairID.first != pairID.second);
	return pairID;
}

// Return the pair of bodies index
inline bodyindexpair OverlappingPair::computeBodiesIndexPair(CollisionBody* body1,
															 CollisionBody* body2) {

	// Construct the pair of body index
	bodyindexpair indexPair = body1->getID() < body2->getID() ?
								 std::make_pair(body1->getID(), body2->getID()) :
								 std::make_pair(body2->getID(), body1->getID());
	assert(indexPair.first != indexPair.second);
	return indexPair;
}

// Clear the contact points of the contact manifold
inline void OverlappingPair::clearContactPoints() {
   m_contactManifoldSet.clear();
}

}

