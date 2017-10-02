/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#pragma once

#include <ephysics/collision/ContactManifold.hpp>

namespace ephysics {
	const int32_t MAX_MANIFOLDS_IN_CONTACT_MANIFOLD_SET = 3;   // Maximum number of contact manifolds in the set
	const int32_t CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS = 3;	// N Number for the N x N subdivisions of the cubemap
	/**
	 * @brief This class represents a set of one or several contact manifolds. Typically a
	 * convex/convex collision will have a set with a single manifold and a convex-concave
	 * collision can have more than one manifolds. Note that a contact manifold can
	 * contains several contact points.
	 */
	class ContactManifoldSet {
		private:
			int32_t m_nbMaxManifolds; //!< Maximum number of contact manifolds in the set
			int32_t m_nbManifolds; //!< Current number of contact manifolds in the set
			ProxyShape* m_shape1; //!< Pointer to the first proxy shape of the contact
			ProxyShape* m_shape2; //!< Pointer to the second proxy shape of the contact
			ContactManifold* m_manifolds[MAX_MANIFOLDS_IN_CONTACT_MANIFOLD_SET]; //!< Contact manifolds of the set
			/// Create a new contact manifold and add it to the set
			void createManifold(short _normalDirectionId);
			/// Remove a contact manifold from the set
			void removeManifold(int32_t _index);
			// Return the index of the contact manifold with a similar average normal.
			int32_t selectManifoldWithSimilarNormal(int16_t _normalDirectionId) const;
			// Map the normal vector int32_to a cubemap face bucket (a face contains 4x4 buckets)
			// Each face of the cube is divided int32_to 4x4 buckets. This method maps the
			// normal vector int32_to of the of the bucket and returns a unique Id for the bucket
			int16_t computeCubemapNormalId(const vec3& _normal) const;
		public:
			/// Constructor
			ContactManifoldSet(ProxyShape* _shape1,
			                   ProxyShape* _shape2,
			                   int32_t _nbMaxManifolds);
			/// Destructor
			~ContactManifoldSet();
			/// Return the first proxy shape
			ProxyShape* getShape1() const;
			/// Return the second proxy shape
			ProxyShape* getShape2() const;
			/// Add a contact point to the manifold set
			void addContactPoint(ContactPoint* _contact);
			/// Update the contact manifolds
			void update();
			/// Clear the contact manifold set
			void clear();
			/// Return the number of manifolds in the set
			int32_t getNbContactManifolds() const;
			/// Return a given contact manifold
			ContactManifold* getContactManifold(int32_t _index) const;
			/// Return the total number of contact points in the set of manifolds
			int32_t getTotalNbContactPoints() const;
	};

}

