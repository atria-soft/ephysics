/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/collision/ContactManifold.h>

namespace reactphysics3d {

// Constants
const int32_t MAX_MANIFOLDS_IN_CONTACT_MANIFOLD_SET = 3;   // Maximum number of contact manifolds in the set
const int32_t CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS = 3;	// N Number for the N x N subdivisions of the cubemap

// Class ContactManifoldSet
/**
 * This class represents a set of one or several contact manifolds. Typically a
 * convex/convex collision will have a set with a single manifold and a convex-concave
 * collision can have more than one manifolds. Note that a contact manifold can
 * contains several contact points.
 */
class ContactManifoldSet {

	private:

		// -------------------- Attributes -------------------- //

		/// Maximum number of contact manifolds in the set
		int32_t m_nbMaxManifolds;

		/// Current number of contact manifolds in the set
		int32_t m_nbManifolds;

		/// Pointer to the first proxy shape of the contact
		ProxyShape* m_shape1;

		/// Pointer to the second proxy shape of the contact
		ProxyShape* m_shape2;

		/// Reference to the memory allocator
		MemoryAllocator& m_memoryAllocator;

		/// Contact manifolds of the set
		ContactManifold* m_manifolds[MAX_MANIFOLDS_IN_CONTACT_MANIFOLD_SET];

		// -------------------- Methods -------------------- //

		/// Create a new contact manifold and add it to the set
		void createManifold(short normalDirectionId);

		/// Remove a contact manifold from the set
		void removeManifold(int32_t index);

		// Return the index of the contact manifold with a similar average normal.
		int32_t selectManifoldWithSimilarNormal(int16_t normalDirectionId) const;

		// Map the normal vector int32_to a cubemap face bucket (a face contains 4x4 buckets)
		// Each face of the cube is divided int32_to 4x4 buckets. This method maps the
		// normal vector int32_to of the of the bucket and returns a unique Id for the bucket
		int16_t computeCubemapNormalId(const vec3& normal) const;

	public:

		// -------------------- Methods -------------------- //

		/// Constructor
		ContactManifoldSet(ProxyShape* shape1, ProxyShape* shape2,
						   MemoryAllocator& memoryAllocator, int32_t nbMaxManifolds);

		/// Destructor
		~ContactManifoldSet();

		/// Return the first proxy shape
		ProxyShape* getShape1() const;

		/// Return the second proxy shape
		ProxyShape* getShape2() const;

		/// Add a contact point to the manifold set
		void addContactPoint(ContactPoint* contact);

		/// Update the contact manifolds
		void update();

		/// Clear the contact manifold set
		void clear();

		/// Return the number of manifolds in the set
		int32_t getNbContactManifolds() const;

		/// Return a given contact manifold
		ContactManifold* getContactManifold(int32_t index) const;

		/// Return the total number of contact points in the set of manifolds
		int32_t getTotalNbContactPoints() const;
};

// Return the first proxy shape
inline ProxyShape* ContactManifoldSet::getShape1() const {
	return m_shape1;
}

// Return the second proxy shape
inline ProxyShape* ContactManifoldSet::getShape2() const {
	return m_shape2;
}

// Return the number of manifolds in the set
inline int32_t ContactManifoldSet::getNbContactManifolds() const {
	return m_nbManifolds;
}

// Return a given contact manifold
inline ContactManifold* ContactManifoldSet::getContactManifold(int32_t index) const {
	assert(index >= 0 && index < m_nbManifolds);
	return m_manifolds[index];
}

// Return the total number of contact points in the set of manifolds
inline int32_t ContactManifoldSet::getTotalNbContactPoints() const {
	int32_t nbPoints = 0;
	for (int32_t i=0; i<m_nbManifolds; i++) {
		nbPoints += m_manifolds[i]->getNbContactPoints();
	}
	return nbPoints;
}

}

