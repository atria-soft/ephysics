/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/ContactManifoldSet.h>

using namespace reactphysics3d;

// Constructor
ContactManifoldSet::ContactManifoldSet(ProxyShape* shape1, ProxyShape* shape2,
									   MemoryAllocator& memoryAllocator, int32_t nbMaxManifolds)
				   : mNbMaxManifolds(nbMaxManifolds), mNbManifolds(0), mShape1(shape1),
					 mShape2(shape2), mMemoryAllocator(memoryAllocator) {
	assert(nbMaxManifolds >= 1);
}

// Destructor
ContactManifoldSet::~ContactManifoldSet() {

	// Clear all the contact manifolds
	clear();
}

// Add a contact point to the manifold set
void ContactManifoldSet::addContactPoint(ContactPoint* contact) {

	// Compute an Id corresponding to the normal direction (using a cubemap)
	int16_t normalDirectionId = computeCubemapNormalId(contact->getNormal());

	// If there is no contact manifold yet
	if (mNbManifolds == 0) {

		createManifold(normalDirectionId);
		mManifolds[0]->addContactPoint(contact);
		assert(mManifolds[mNbManifolds-1]->getNbContactPoints() > 0);
		for (int32_t i=0; i<mNbManifolds; i++) {
			assert(mManifolds[i]->getNbContactPoints() > 0);
		}

		return;
	}

	// Select the manifold with the most similar normal (if exists)
	int32_t similarManifoldIndex = 0;
	if (mNbMaxManifolds > 1) {
		similarManifoldIndex = selectManifoldWithSimilarNormal(normalDirectionId);
	}

	// If a similar manifold has been found
	if (similarManifoldIndex != -1) {

		// Add the contact point to that similar manifold
		mManifolds[similarManifoldIndex]->addContactPoint(contact);
		assert(mManifolds[similarManifoldIndex]->getNbContactPoints() > 0);

		return;
	}

	// If the maximum number of manifold has not been reached yet
	if (mNbManifolds < mNbMaxManifolds) {

		// Create a new manifold for the contact point
		createManifold(normalDirectionId);
		mManifolds[mNbManifolds-1]->addContactPoint(contact);
		for (int32_t i=0; i<mNbManifolds; i++) {
			assert(mManifolds[i]->getNbContactPoints() > 0);
		}

		return;
	}

	// The contact point will be in a new contact manifold, we now have too much
	// manifolds condidates. We need to remove one. We choose to keep the manifolds
	// with the largest contact depth among their points
	int32_t smallestDepthIndex = -1;
	float minDepth = contact->getPenetrationDepth();
	assert(mNbManifolds == mNbMaxManifolds);
	for (int32_t i=0; i<mNbManifolds; i++) {
		float depth = mManifolds[i]->getLargestContactDepth();
		if (depth < minDepth) {
			minDepth = depth;
			smallestDepthIndex = i;
		}
	}

	// If we do not want to keep to new manifold (not created yet) with the
	// new contact point
	if (smallestDepthIndex == -1) {

		// Delete the new contact
		contact->~ContactPoint();
		mMemoryAllocator.release(contact, sizeof(ContactPoint));

		return;
	}

	assert(smallestDepthIndex >= 0 && smallestDepthIndex < mNbManifolds);

	// Here we need to replace an existing manifold with a new one (that contains
	// the new contact point)
	removeManifold(smallestDepthIndex);
	createManifold(normalDirectionId);
	mManifolds[mNbManifolds-1]->addContactPoint(contact);
	assert(mManifolds[mNbManifolds-1]->getNbContactPoints() > 0);
	for (int32_t i=0; i<mNbManifolds; i++) {
		assert(mManifolds[i]->getNbContactPoints() > 0);
	}

	return;
}

// Return the index of the contact manifold with a similar average normal.
// If no manifold has close enough average normal, it returns -1
int32_t ContactManifoldSet::selectManifoldWithSimilarNormal(int16_t normalDirectionId) const {

	// Return the Id of the manifold with the same normal direction id (if exists)
	for (int32_t i=0; i<mNbManifolds; i++) {
		if (normalDirectionId == mManifolds[i]->getNormalDirectionId()) {
			return i;
		}
	}

	return -1;
}

// Map the normal vector int32_to a cubemap face bucket (a face contains 4x4 buckets)
// Each face of the cube is divided int32_to 4x4 buckets. This method maps the
// normal vector int32_to of the of the bucket and returns a unique Id for the bucket
int16_t ContactManifoldSet::computeCubemapNormalId(const Vector3& normal) const {

	assert(normal.lengthSquare() > MACHINE_EPSILON);

	int32_t faceNo;
	float u, v;
	float max = max3(fabs(normal.x), fabs(normal.y), fabs(normal.z));
	Vector3 normalScaled = normal / max;

	if (normalScaled.x >= normalScaled.y && normalScaled.x >= normalScaled.z) {
		faceNo = normalScaled.x > 0 ? 0 : 1;
		u = normalScaled.y;
		v = normalScaled.z;
	}
	else if (normalScaled.y >= normalScaled.x && normalScaled.y >= normalScaled.z) {
		faceNo = normalScaled.y > 0 ? 2 : 3;
		u = normalScaled.x;
		v = normalScaled.z;
	}
	else {
		faceNo = normalScaled.z > 0 ? 4 : 5;
		u = normalScaled.x;
		v = normalScaled.y;
	}

	int32_t indexU = floor(((u + 1)/2) * CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS);
	int32_t indexV = floor(((v + 1)/2) * CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS);
	if (indexU == CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS) indexU--;
	if (indexV == CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS) indexV--;

	const int32_t nbSubDivInFace = CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS * CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS;
	return faceNo * 200 + indexU * nbSubDivInFace + indexV;
}

// Update the contact manifolds
void ContactManifoldSet::update() {

	for (int32_t i=mNbManifolds-1; i>=0; i--) {

		// Update the contact manifold
		mManifolds[i]->update(mShape1->getBody()->getTransform() * mShape1->getLocalToBodyTransform(),
							  mShape2->getBody()->getTransform() * mShape2->getLocalToBodyTransform());

		// Remove the contact manifold if has no contact points anymore
		if (mManifolds[i]->getNbContactPoints() == 0) {
			removeManifold(i);
		}
	}
}

// Clear the contact manifold set
void ContactManifoldSet::clear() {

	// Destroy all the contact manifolds
	for (int32_t i=mNbManifolds-1; i>=0; i--) {
		removeManifold(i);
	}

	assert(mNbManifolds == 0);
}

// Create a new contact manifold and add it to the set
void ContactManifoldSet::createManifold(int16_t normalDirectionId) {
	assert(mNbManifolds < mNbMaxManifolds);

	mManifolds[mNbManifolds] = new (mMemoryAllocator.allocate(sizeof(ContactManifold)))
									ContactManifold(mShape1, mShape2, mMemoryAllocator, normalDirectionId);
	mNbManifolds++;
}

// Remove a contact manifold from the set
void ContactManifoldSet::removeManifold(int32_t index) {

	assert(mNbManifolds > 0);
	assert(index >= 0 && index < mNbManifolds);

	// Delete the new contact
	mManifolds[index]->~ContactManifold();
	mMemoryAllocator.release(mManifolds[index], sizeof(ContactManifold));

	for (int32_t i=index; (i+1) < mNbManifolds; i++) {
		mManifolds[i] = mManifolds[i+1];
	}

	mNbManifolds--;
}
