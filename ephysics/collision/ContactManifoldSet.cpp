/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#include <ephysics/collision/ContactManifoldSet.hpp>

using namespace ephysics;

ContactManifoldSet::ContactManifoldSet(ProxyShape* _shape1,
                                       ProxyShape* _shape2,
                                       int32_t _nbMaxManifolds):
  m_nbMaxManifolds(_nbMaxManifolds),
  m_nbManifolds(0),
  m_shape1(_shape1),
  m_shape2(_shape2) {
	assert(_nbMaxManifolds >= 1);
}

ContactManifoldSet::~ContactManifoldSet() {
	clear();
}

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
		ETK_DELETE(ContactPoint, contact);
		contact = null;
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

int32_t ContactManifoldSet::selectManifoldWithSimilarNormal(int16_t normalDirectionId) const {
	// Return the Id of the manifold with the same normal direction id (if exists)
	for (int32_t i=0; i<m_nbManifolds; i++) {
		if (normalDirectionId == m_manifolds[i]->getNormalDirectionId()) {
			return i;
		}
	}
	return -1;
}

int16_t ContactManifoldSet::computeCubemapNormalId(const vec3& normal) const {
	assert(normal.length2() > FLT_EPSILON);
	int32_t faceNo;
	float u, v;
	float max = max3(fabs(normal.x()), fabs(normal.y()), fabs(normal.z()));
	vec3 normalScaled = normal / max;
	if (normalScaled.x() >= normalScaled.y() && normalScaled.x() >= normalScaled.z()) {
		faceNo = normalScaled.x() > 0 ? 0 : 1;
		u = normalScaled.y();
		v = normalScaled.z();
	} else if (normalScaled.y() >= normalScaled.x() && normalScaled.y() >= normalScaled.z()) {
		faceNo = normalScaled.y() > 0 ? 2 : 3;
		u = normalScaled.x();
		v = normalScaled.z();
	} else {
		faceNo = normalScaled.z() > 0 ? 4 : 5;
		u = normalScaled.x();
		v = normalScaled.y();
	}
	int32_t indexU = floor(((u + 1)/2) * CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS);
	int32_t indexV = floor(((v + 1)/2) * CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS);
	if (indexU == CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS) {
		indexU--;
	}
	if (indexV == CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS) {
		indexV--;
	}
	const int32_t nbSubDivInFace = CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS * CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS;
	return faceNo * 200 + indexU * nbSubDivInFace + indexV;
}

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

void ContactManifoldSet::clear() {
	for (int32_t i=m_nbManifolds-1; i>=0; i--) {
		removeManifold(i);
	}
	assert(m_nbManifolds == 0);
}

void ContactManifoldSet::createManifold(int16_t normalDirectionId) {
	assert(m_nbManifolds < m_nbMaxManifolds);
	m_manifolds[m_nbManifolds] = ETK_NEW(ContactManifold, m_shape1, m_shape2, normalDirectionId);
	m_nbManifolds++;
}

void ContactManifoldSet::removeManifold(int32_t index) {
	assert(m_nbManifolds > 0);
	assert(index >= 0 && index < m_nbManifolds);
	// Delete the new contact
	ETK_DELETE(ContactManifold, m_manifolds[index]);
	m_manifolds[index] = null;
	for (int32_t i=index; (i+1) < m_nbManifolds; i++) {
		m_manifolds[i] = m_manifolds[i+1];
	}
	m_nbManifolds--;
}

ProxyShape* ContactManifoldSet::getShape1() const {
	return m_shape1;
}

ProxyShape* ContactManifoldSet::getShape2() const {
	return m_shape2;
}

int32_t ContactManifoldSet::getNbContactManifolds() const {
	return m_nbManifolds;
}

ContactManifold* ContactManifoldSet::getContactManifold(int32_t index) const {
	assert(index >= 0 && index < m_nbManifolds);
	return m_manifolds[index];
}

int32_t ContactManifoldSet::getTotalNbContactPoints() const {
	int32_t nbPoints = 0;
	for (int32_t i=0; i<m_nbManifolds; i++) {
		nbPoints += m_manifolds[i]->getNbContactPoints();
	}
	return nbPoints;
}
