/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/ContactManifoldSet.hpp>

using namespace ephysics;

// Constructor
ContactManifoldSet::ContactManifoldSet(ProxyShape* shape1, ProxyShape* shape2,
									   MemoryAllocator& memoryAllocator, int32_t nbMaxManifolds)
				   : m_nbMaxManifolds(nbMaxManifolds), m_nbManifolds(0), m_shape1(shape1),
					 m_shape2(shape2), m_memoryAllocator(memoryAllocator) {
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
	if (m_nbManifolds == 0) {

		createManifold(normalDirectionId);
		m_manifolds[0]->addContactPoint(contact);
		assert(m_manifolds[m_nbManifolds-1]->getNbContactPoints() > 0);
		for (int32_t i=0; i<m_nbManifolds; i++) {
			assert(m_manifolds[i]->getNbContactPoints() > 0);
		}

		return;
	}

	// Select the manifold with the most similar normal (if exists)
	int32_t similarManifoldIndex = 0;
	if (m_nbMaxManifolds > 1) {
		similarManifoldIndex = selectManifoldWithSimilarNormal(normalDirectionId);
	}

	// If a similar manifold has been found
	if (similarManifoldIndex != -1) {

		// Add the contact point to that similar manifold
		m_manifolds[similarManifoldIndex]->addContactPoint(contact);
		assert(m_manifolds[similarManifoldIndex]->getNbContactPoints() > 0);

		return;
	}

	// If the maximum number of manifold has not been reached yet
	if (m_nbManifolds < m_nbMaxManifolds) {

		// Create a new manifold for the contact point
		createManifold(normalDirectionId);
		m_manifolds[m_nbManifolds-1]->addContactPoint(contact);
		for (int32_t i=0; i<m_nbManifolds; i++) {
			assert(m_manifolds[i]->getNbContactPoints() > 0);
		}

		return;
	}

	// The contact point will be in a new contact manifold, we now have too much
	// manifolds condidates. We need to remove one. We choose to keep the manifolds
	// with the largest contact depth among their points
	int32_t smallestDepthIndex = -1;
	float minDepth = contact->getPenetrationDepth();
	assert(m_nbManifolds == m_nbMaxManifolds);
	for (int32_t i=0; i<m_nbManifolds; i++) {
		float depth = m_manifolds[i]->getLargestContactDepth();
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
		m_memoryAllocator.release(contact, sizeof(ContactPoint));

		return;
	}

	assert(smallestDepthIndex >= 0 && smallestDepthIndex < m_nbManifolds);

	// Here we need to replace an existing manifold with a new one (that contains
	// the new contact point)
	removeManifold(smallestDepthIndex);
	createManifold(normalDirectionId);
	m_manifolds[m_nbManifolds-1]->addContactPoint(contact);
	assert(m_manifolds[m_nbManifolds-1]->getNbContactPoints() > 0);
	for (int32_t i=0; i<m_nbManifolds; i++) {
		assert(m_manifolds[i]->getNbContactPoints() > 0);
	}

	return;
}

// Return the index of the contact manifold with a similar average normal.
// If no manifold has close enough average normal, it returns -1
int32_t ContactManifoldSet::selectManifoldWithSimilarNormal(int16_t normalDirectionId) const {

	// Return the Id of the manifold with the same normal direction id (if exists)
	for (int32_t i=0; i<m_nbManifolds; i++) {
		if (normalDirectionId == m_manifolds[i]->getNormalDirectionId()) {
			return i;
		}
	}

	return -1;
}

// Map the normal vector int32_to a cubemap face bucket (a face contains 4x4 buckets)
// Each face of the cube is divided int32_to 4x4 buckets. This method maps the
// normal vector int32_to of the of the bucket and returns a unique Id for the bucket
int16_t ContactManifoldSet::computeCubemapNormalId(const vec3& normal) const {

	assert(normal.length2() > MACHINE_EPSILON);

	int32_t faceNo;
	float u, v;
	float max = max3(fabs(normal.x()), fabs(normal.y()), fabs(normal.z()));
	vec3 normalScaled = normal / max;

	if (normalScaled.x() >= normalScaled.y() && normalScaled.x() >= normalScaled.z()) {
		faceNo = normalScaled.x() > 0 ? 0 : 1;
		u = normalScaled.y();
		v = normalScaled.z();
	}
	else if (normalScaled.y() >= normalScaled.x() && normalScaled.y() >= normalScaled.z()) {
		faceNo = normalScaled.y() > 0 ? 2 : 3;
		u = normalScaled.x();
		v = normalScaled.z();
	}
	else {
		faceNo = normalScaled.z() > 0 ? 4 : 5;
		u = normalScaled.x();
		v = normalScaled.y();
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

	for (int32_t i=m_nbManifolds-1; i>=0; i--) {

		// Update the contact manifold
		m_manifolds[i]->update(m_shape1->getBody()->getTransform() * m_shape1->getLocalToBodyTransform(),
							  m_shape2->getBody()->getTransform() * m_shape2->getLocalToBodyTransform());

		// Remove the contact manifold if has no contact points anymore
		if (m_manifolds[i]->getNbContactPoints() == 0) {
			removeManifold(i);
		}
	}
}

// Clear the contact manifold set
void ContactManifoldSet::clear() {

	// Destroy all the contact manifolds
	for (int32_t i=m_nbManifolds-1; i>=0; i--) {
		removeManifold(i);
	}

	assert(m_nbManifolds == 0);
}

// Create a new contact manifold and add it to the set
void ContactManifoldSet::createManifold(int16_t normalDirectionId) {
	assert(m_nbManifolds < m_nbMaxManifolds);

	m_manifolds[m_nbManifolds] = new (m_memoryAllocator.allocate(sizeof(ContactManifold)))
									ContactManifold(m_shape1, m_shape2, m_memoryAllocator, normalDirectionId);
	m_nbManifolds++;
}

// Remove a contact manifold from the set
void ContactManifoldSet::removeManifold(int32_t index) {

	assert(m_nbManifolds > 0);
	assert(index >= 0 && index < m_nbManifolds);

	// Delete the new contact
	m_manifolds[index]->~ContactManifold();
	m_memoryAllocator.release(m_manifolds[index], sizeof(ContactManifold));

	for (int32_t i=index; (i+1) < m_nbManifolds; i++) {
		m_manifolds[i] = m_manifolds[i+1];
	}

	m_nbManifolds--;
}

// Return the first proxy shape
ProxyShape* ContactManifoldSet::getShape1() const {
	return m_shape1;
}

// Return the second proxy shape
ProxyShape* ContactManifoldSet::getShape2() const {
	return m_shape2;
}

// Return the number of manifolds in the set
int32_t ContactManifoldSet::getNbContactManifolds() const {
	return m_nbManifolds;
}

// Return a given contact manifold
ContactManifold* ContactManifoldSet::getContactManifold(int32_t index) const {
	assert(index >= 0 && index < m_nbManifolds);
	return m_manifolds[index];
}

// Return the total number of contact points in the set of manifolds
int32_t ContactManifoldSet::getTotalNbContactPoints() const {
	int32_t nbPoints = 0;
	for (int32_t i=0; i<m_nbManifolds; i++) {
		nbPoints += m_manifolds[i]->getNbContactPoints();
	}
	return nbPoints;
}
