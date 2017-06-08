/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <vector>
#include <ephysics/body/CollisionBody.h>
#include <ephysics/collision/ProxyShape.h>
#include <ephysics/constraint/ContactPoint.h>
#include <ephysics/memory/MemoryAllocator.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Constants
const uint32_t MAX_CONTACT_POINTS_IN_MANIFOLD = 4;   // Maximum number of contacts in the manifold

// Class declarations
class ContactManifold;

// Structure ContactManifoldListElement
/**
 * This structure represents a single element of a linked list of contact manifolds
 */
struct ContactManifoldListElement {

	public:

		// -------------------- Attributes -------------------- //

		/// Pointer to the actual contact manifold
		ContactManifold* contactManifold;

		/// Next element of the list
		ContactManifoldListElement* next;

		// -------------------- Methods -------------------- //

		/// Constructor
		ContactManifoldListElement(ContactManifold* initContactManifold,
								   ContactManifoldListElement* initNext)
								  :contactManifold(initContactManifold), next(initNext) {

		}
};

// Class ContactManifold
/**
 * This class represents the set of contact points between two bodies.
 * The contact manifold is implemented in a way to cache the contact
 * points among the frames for better stability following the
 * "Contact Generation" presentation of Erwin Coumans at GDC 2010
 * conference (bullet.googlecode.com/files/GDC10_Coumans_Erwin_Contact.pdf).
 * Some code of this class is based on the implementation of the
 * btPersistentManifold class from Bullet physics engine (www.http://bulletphysics.org).
 * The contacts between two bodies are added one after the other in the cache.
 * When the cache is full, we have to remove one point. The idea is to keep
 * the point with the deepest penetration depth and also to keep the
 * points producing the larger area (for a more stable contact manifold).
 * The new added point is always kept.
 */
class ContactManifold {

	private:

		// -------------------- Attributes -------------------- //

		/// Pointer to the first proxy shape of the contact
		ProxyShape* mShape1;

		/// Pointer to the second proxy shape of the contact
		ProxyShape* mShape2;

		/// Contact points in the manifold
		ContactPoint* mContactPoints[MAX_CONTACT_POINTS_IN_MANIFOLD];

		/// Normal direction Id (Unique Id representing the normal direction)
		int16_t mNormalDirectionId;

		/// Number of contacts in the cache
		uint32_t mNbContactPoints;

		/// First friction vector of the contact manifold
		Vector3 mFrictionVector1;

		/// Second friction vector of the contact manifold
		Vector3 mFrictionVector2;

		/// First friction constraint accumulated impulse
		float mFrictionImpulse1;

		/// Second friction constraint accumulated impulse
		float mFrictionImpulse2;

		/// Twist friction constraint accumulated impulse
		float mFrictionTwistImpulse;

		/// Accumulated rolling resistance impulse
		Vector3 mRollingResistanceImpulse;

		/// True if the contact manifold has already been added int32_to an island
		bool mIsAlreadyInIsland;

		/// Reference to the memory allocator
		MemoryAllocator& mMemoryAllocator;

		// -------------------- Methods -------------------- //

		/// Private copy-constructor
		ContactManifold(const ContactManifold& contactManifold);

		/// Private assignment operator
		ContactManifold& operator=(const ContactManifold& contactManifold);

		/// Return the index of maximum area
		int32_t getMaxArea(float area0, float area1, float area2, float area3) const;

		/// Return the index of the contact with the larger penetration depth.
		int32_t getIndexOfDeepestPenetration(ContactPoint* newContact) const;

		/// Return the index that will be removed.
		int32_t getIndexToRemove(int32_t indexMaxPenetration, const Vector3& newPoint) const;

		/// Remove a contact point from the manifold
		void removeContactPoint(uint32_t index);

		/// Return true if the contact manifold has already been added int32_to an island
		bool isAlreadyInIsland() const;
		
	public:

		// -------------------- Methods -------------------- //

		/// Constructor
		ContactManifold(ProxyShape* shape1, ProxyShape* shape2,
						MemoryAllocator& memoryAllocator, int16_t normalDirectionId);

		/// Destructor
		~ContactManifold();

		/// Return a pointer to the first proxy shape of the contact
		ProxyShape* getShape1() const;

		/// Return a pointer to the second proxy shape of the contact
		ProxyShape* getShape2() const;

		/// Return a pointer to the first body of the contact manifold
		CollisionBody* getBody1() const;

		/// Return a pointer to the second body of the contact manifold
		CollisionBody* getBody2() const;

		/// Return the normal direction Id
		int16_t getNormalDirectionId() const;

		/// Add a contact point to the manifold
		void addContactPoint(ContactPoint* contact);

		/// Update the contact manifold.
		void update(const Transform& transform1, const Transform& transform2);

		/// Clear the contact manifold
		void clear();

		/// Return the number of contact points in the manifold
		uint32_t getNbContactPoints() const;

		/// Return the first friction vector at the center of the contact manifold
		const Vector3& getFrictionVector1() const;

		/// set the first friction vector at the center of the contact manifold
		void setFrictionVector1(const Vector3& mFrictionVector1);

		/// Return the second friction vector at the center of the contact manifold
		const Vector3& getFrictionVector2() const;

		/// set the second friction vector at the center of the contact manifold
		void setFrictionVector2(const Vector3& mFrictionVector2);

		/// Return the first friction accumulated impulse
		float getFrictionImpulse1() const;

		/// Set the first friction accumulated impulse
		void setFrictionImpulse1(float frictionImpulse1);

		/// Return the second friction accumulated impulse
		float getFrictionImpulse2() const;

		/// Set the second friction accumulated impulse
		void setFrictionImpulse2(float frictionImpulse2);

		/// Return the friction twist accumulated impulse
		float getFrictionTwistImpulse() const;

		/// Set the friction twist accumulated impulse
		void setFrictionTwistImpulse(float frictionTwistImpulse);

		/// Set the accumulated rolling resistance impulse
		void setRollingResistanceImpulse(const Vector3& rollingResistanceImpulse);

		/// Return a contact point of the manifold
		ContactPoint* getContactPoint(uint32_t index) const;

		/// Return the normalized averaged normal vector
		Vector3 getAverageContactNormal() const;

		/// Return the largest depth of all the contact points
		float getLargestContactDepth() const;

		// -------------------- Friendship -------------------- //

		friend class DynamicsWorld;
		friend class Island;
		friend class CollisionBody;
};

// Return a pointer to the first proxy shape of the contact
inline ProxyShape* ContactManifold::getShape1() const {
	return mShape1;
}

// Return a pointer to the second proxy shape of the contact
inline ProxyShape* ContactManifold::getShape2() const {
	return mShape2;
}

// Return a pointer to the first body of the contact manifold
inline CollisionBody* ContactManifold::getBody1() const {
	return mShape1->getBody();
}

// Return a pointer to the second body of the contact manifold
inline CollisionBody* ContactManifold::getBody2() const {
	return mShape2->getBody();
}

// Return the normal direction Id
inline int16_t ContactManifold::getNormalDirectionId() const {
	return mNormalDirectionId;
}

// Return the number of contact points in the manifold
inline uint32_t ContactManifold::getNbContactPoints() const {
	return mNbContactPoints;
}

// Return the first friction vector at the center of the contact manifold
inline const Vector3& ContactManifold::getFrictionVector1() const {
	return mFrictionVector1;
}

// set the first friction vector at the center of the contact manifold
inline void ContactManifold::setFrictionVector1(const Vector3& frictionVector1) {
	mFrictionVector1 = frictionVector1;
}

// Return the second friction vector at the center of the contact manifold
inline const Vector3& ContactManifold::getFrictionVector2() const {
	return mFrictionVector2;
}

// set the second friction vector at the center of the contact manifold
inline void ContactManifold::setFrictionVector2(const Vector3& frictionVector2) {
	mFrictionVector2 = frictionVector2;
}

// Return the first friction accumulated impulse
inline float ContactManifold::getFrictionImpulse1() const {
	return mFrictionImpulse1;
}

// Set the first friction accumulated impulse
inline void ContactManifold::setFrictionImpulse1(float frictionImpulse1) {
	mFrictionImpulse1 = frictionImpulse1;
}

// Return the second friction accumulated impulse
inline float ContactManifold::getFrictionImpulse2() const {
	return mFrictionImpulse2;
}

// Set the second friction accumulated impulse
inline void ContactManifold::setFrictionImpulse2(float frictionImpulse2) {
	mFrictionImpulse2 = frictionImpulse2;
}

// Return the friction twist accumulated impulse
inline float ContactManifold::getFrictionTwistImpulse() const {
	return mFrictionTwistImpulse;
}

// Set the friction twist accumulated impulse
inline void ContactManifold::setFrictionTwistImpulse(float frictionTwistImpulse) {
	mFrictionTwistImpulse = frictionTwistImpulse;
}

// Set the accumulated rolling resistance impulse
inline void ContactManifold::setRollingResistanceImpulse(const Vector3& rollingResistanceImpulse) {
	mRollingResistanceImpulse = rollingResistanceImpulse;
}

// Return a contact point of the manifold
inline ContactPoint* ContactManifold::getContactPoint(uint32_t index) const {
	assert(index < mNbContactPoints);
	return mContactPoints[index];
}

// Return true if the contact manifold has already been added int32_to an island
inline bool ContactManifold::isAlreadyInIsland() const {
	return mIsAlreadyInIsland;
}

// Return the normalized averaged normal vector
inline Vector3 ContactManifold::getAverageContactNormal() const {
	Vector3 averageNormal;

	for (uint32_t i=0; i<mNbContactPoints; i++) {
		averageNormal += mContactPoints[i]->getNormal();
	}

	return averageNormal.getUnit();
}

// Return the largest depth of all the contact points
inline float ContactManifold::getLargestContactDepth() const {
	float largestDepth = 0.0f;

	for (uint32_t i=0; i<mNbContactPoints; i++) {
		float depth = mContactPoints[i]->getPenetrationDepth();
		if (depth > largestDepth) {
			largestDepth = depth;
		}
	}

	return largestDepth;
}

}


