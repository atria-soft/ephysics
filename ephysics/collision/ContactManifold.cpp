/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#include <ephysics/collision/ContactManifold.hpp>

using namespace ephysics;

// Constructor
ContactManifold::ContactManifold(ProxyShape* _shape1,
                                 ProxyShape* _shape2,
                                 short _normalDirectionId):
  m_shape1(_shape1),
  m_shape2(_shape2),
  m_normalDirectionId(_normalDirectionId),
  m_nbContactPoints(0),
  m_frictionImpulse1(0.0),
  m_frictionImpulse2(0.0),
  m_frictionTwistImpulse(0.0),
  m_isAlreadyInIsland(false) {
	
}

// Destructor
ContactManifold::~ContactManifold() {
	clear();
}

// Add a contact point in the manifold
void ContactManifold::addContactPoint(ContactPoint* contact) {
	
	// For contact already in the manifold
	for (uint32_t i=0; i<m_nbContactPoints; i++) {

		// Check if the new point point does not correspond to a same contact point
		// already in the manifold.
		float distance = (m_contactPoints[i]->getWorldPointOnBody1() -
							contact->getWorldPointOnBody1()).length2();
		if (distance <= PERSISTENT_CONTACT_DIST_THRESHOLD*PERSISTENT_CONTACT_DIST_THRESHOLD) {
			// Delete the new contact
			delete contact;
			assert(m_nbContactPoints > 0);
			return;
		}
	}
	
	// If the contact manifold is full
	if (m_nbContactPoints == MAX_CONTACT_POINTS_IN_MANIFOLD) {
		int32_t indexMaxPenetration = getIndexOfDeepestPenetration(contact);
		int32_t indexToRemove = getIndexToRemove(indexMaxPenetration, contact->getLocalPointOnBody1());
		removeContactPoint(indexToRemove);
	}

	// Add the new contact point in the manifold
	m_contactPoints[m_nbContactPoints] = contact;
	m_nbContactPoints++;

	assert(m_nbContactPoints > 0);
}

// Remove a contact point from the manifold
void ContactManifold::removeContactPoint(uint32_t index) {
	assert(index < m_nbContactPoints);
	assert(m_nbContactPoints > 0);
	
	// Call the destructor explicitly and tell the memory allocator that
	// the corresponding memory block is now free
	delete m_contactPoints[index];
	m_contactPoints[index] = nullptr;
	// If we don't remove the last index
	if (index < m_nbContactPoints - 1) {
		m_contactPoints[index] = m_contactPoints[m_nbContactPoints - 1];
	}

	m_nbContactPoints--;
}

// Update the contact manifold
/// First the world space coordinates of the current contacts in the manifold are recomputed from
/// the corresponding transforms of the bodies because they have moved. Then we remove the contacts
/// with a negative penetration depth (meaning that the bodies are not penetrating anymore) and also
/// the contacts with a too large distance between the contact points in the plane orthogonal to the
/// contact normal.
void ContactManifold::update(const etk::Transform3D& transform1, const etk::Transform3D& transform2) {

	if (m_nbContactPoints == 0) return;

	// Update the world coordinates and penetration depth of the contact points in the manifold
	for (uint32_t i=0; i<m_nbContactPoints; i++) {
		m_contactPoints[i]->setWorldPointOnBody1(transform1 *
												m_contactPoints[i]->getLocalPointOnBody1());
		m_contactPoints[i]->setWorldPointOnBody2(transform2 *
												m_contactPoints[i]->getLocalPointOnBody2());
		m_contactPoints[i]->setPenetrationDepth((m_contactPoints[i]->getWorldPointOnBody1() -
				  m_contactPoints[i]->getWorldPointOnBody2()).dot(m_contactPoints[i]->getNormal()));
	}

	const float squarePersistentContactThreshold = PERSISTENT_CONTACT_DIST_THRESHOLD *
													 PERSISTENT_CONTACT_DIST_THRESHOLD;

	// Remove the contact points that don't represent very well the contact manifold
	for (int32_t i=static_cast<int32_t>(m_nbContactPoints)-1; i>=0; i--) {
		assert(i < static_cast<int32_t>(m_nbContactPoints));

		// Compute the distance between contact points in the normal direction
		float distanceNormal = -m_contactPoints[i]->getPenetrationDepth();
		
		// If the contacts points are too far from each other in the normal direction
		if (distanceNormal > squarePersistentContactThreshold) {
			removeContactPoint(i);
		}
		else {
			// Compute the distance of the two contact points in the plane
			// orthogonal to the contact normal
			vec3 projOfPoint1 = m_contactPoints[i]->getWorldPointOnBody1() +
								   m_contactPoints[i]->getNormal() * distanceNormal;
			vec3 projDifference = m_contactPoints[i]->getWorldPointOnBody2() - projOfPoint1;

			// If the orthogonal distance is larger than the valid distance
			// threshold, we remove the contact
			if (projDifference.length2() > squarePersistentContactThreshold) {
				removeContactPoint(i);
			}
		}
	}	
}

// Return the index of the contact point with the larger penetration depth.
/// This corresponding contact will be kept in the cache. The method returns -1 is
/// the new contact is the deepest.
int32_t ContactManifold::getIndexOfDeepestPenetration(ContactPoint* newContact) const {
	assert(m_nbContactPoints == MAX_CONTACT_POINTS_IN_MANIFOLD);
	int32_t indexMaxPenetrationDepth = -1;
	float maxPenetrationDepth = newContact->getPenetrationDepth();

	// For each contact in the cache
	for (uint32_t i=0; i<m_nbContactPoints; i++) {

		// If the current contact has a larger penetration depth
		if (m_contactPoints[i]->getPenetrationDepth() > maxPenetrationDepth) {
			maxPenetrationDepth = m_contactPoints[i]->getPenetrationDepth();
			indexMaxPenetrationDepth = i;
		}
	}

	// Return the index of largest penetration depth
	return indexMaxPenetrationDepth;
}

// Return the index that will be removed.
/// The index of the contact point with the larger penetration
/// depth is given as a parameter. This contact won't be removed. Given this contact, we compute
/// the different area and we want to keep the contacts with the largest area. The new point is also
/// kept. In order to compute the area of a quadrilateral, we use the formula :
/// Area = 0.5 * | AC x BD | where AC and BD form the diagonals of the quadrilateral. Note that
/// when we compute this area, we do not calculate it exactly but we
/// only estimate it because we do not compute the actual diagonals of the quadrialteral. Therefore,
/// this is only a guess that is faster to compute. This idea comes from the Bullet Physics library
/// by Erwin Coumans (http://wwww.bulletphysics.org).
int32_t ContactManifold::getIndexToRemove(int32_t indexMaxPenetration, const vec3& newPoint) const {

	assert(m_nbContactPoints == MAX_CONTACT_POINTS_IN_MANIFOLD);

	float area0 = 0.0;	   // Area with contact 1,2,3 and newPoint
	float area1 = 0.0;	   // Area with contact 0,2,3 and newPoint
	float area2 = 0.0;	   // Area with contact 0,1,3 and newPoint
	float area3 = 0.0;	   // Area with contact 0,1,2 and newPoint

	if (indexMaxPenetration != 0) {
		// Compute the area
		vec3 vector1 = newPoint - m_contactPoints[1]->getLocalPointOnBody1();
		vec3 vector2 = m_contactPoints[3]->getLocalPointOnBody1() -
						  m_contactPoints[2]->getLocalPointOnBody1();
		vec3 crossProduct = vector1.cross(vector2);
		area0 = crossProduct.length2();
	}
	if (indexMaxPenetration != 1) {
		// Compute the area
		vec3 vector1 = newPoint - m_contactPoints[0]->getLocalPointOnBody1();
		vec3 vector2 = m_contactPoints[3]->getLocalPointOnBody1() -
						  m_contactPoints[2]->getLocalPointOnBody1();
		vec3 crossProduct = vector1.cross(vector2);
		area1 = crossProduct.length2();
	}
	if (indexMaxPenetration != 2) {
		// Compute the area
		vec3 vector1 = newPoint - m_contactPoints[0]->getLocalPointOnBody1();
		vec3 vector2 = m_contactPoints[3]->getLocalPointOnBody1() -
						  m_contactPoints[1]->getLocalPointOnBody1();
		vec3 crossProduct = vector1.cross(vector2);
		area2 = crossProduct.length2();
	}
	if (indexMaxPenetration != 3) {
		// Compute the area
		vec3 vector1 = newPoint - m_contactPoints[0]->getLocalPointOnBody1();
		vec3 vector2 = m_contactPoints[2]->getLocalPointOnBody1() -
						  m_contactPoints[1]->getLocalPointOnBody1();
		vec3 crossProduct = vector1.cross(vector2);
		area3 = crossProduct.length2();
	}
	
	// Return the index of the contact to remove
	return getMaxArea(area0, area1, area2, area3);
}

// Return the index of maximum area
int32_t ContactManifold::getMaxArea(float area0, float area1, float area2, float area3) const {
	if (area0 < area1) {
		if (area1 < area2) {
			if (area2 < area3) return 3;
			else return 2;
		}
		else {
			if (area1 < area3) return 3;
			else return 1;
		}
	}
	else {
		if (area0 < area2) {
			if (area2 < area3) return 3;
			else return 2;
		}
		else {
			if (area0 < area3) return 3;
			else return 0;
		}
	}
}

// Clear the contact manifold
void ContactManifold::clear() {
	for (uint32_t iii=0; iii<m_nbContactPoints; ++iii) {
		
		// Call the destructor explicitly and tell the memory allocator that
		// the corresponding memory block is now free
		delete m_contactPoints[iii];
		m_contactPoints[iii] = nullptr;
	}
	m_nbContactPoints = 0;
}

// Return a pointer to the first proxy shape of the contact
ProxyShape* ContactManifold::getShape1() const {
	return m_shape1;
}

// Return a pointer to the second proxy shape of the contact
ProxyShape* ContactManifold::getShape2() const {
	return m_shape2;
}

// Return a pointer to the first body of the contact manifold
CollisionBody* ContactManifold::getBody1() const {
	return m_shape1->getBody();
}

// Return a pointer to the second body of the contact manifold
CollisionBody* ContactManifold::getBody2() const {
	return m_shape2->getBody();
}

// Return the normal direction Id
int16_t ContactManifold::getNormalDirectionId() const {
	return m_normalDirectionId;
}

// Return the number of contact points in the manifold
uint32_t ContactManifold::getNbContactPoints() const {
	return m_nbContactPoints;
}

// Return the first friction vector at the center of the contact manifold
const vec3& ContactManifold::getFrictionVector1() const {
	return m_frictionVector1;
}

// set the first friction vector at the center of the contact manifold
void ContactManifold::setFrictionVector1(const vec3& frictionVector1) {
	m_frictionVector1 = frictionVector1;
}

// Return the second friction vector at the center of the contact manifold
const vec3& ContactManifold::getFrictionvec2() const {
	return m_frictionvec2;
}

// set the second friction vector at the center of the contact manifold
void ContactManifold::setFrictionvec2(const vec3& frictionvec2) {
	m_frictionvec2 = frictionvec2;
}

// Return the first friction accumulated impulse
float ContactManifold::getFrictionImpulse1() const {
	return m_frictionImpulse1;
}

// Set the first friction accumulated impulse
void ContactManifold::setFrictionImpulse1(float frictionImpulse1) {
	m_frictionImpulse1 = frictionImpulse1;
}

// Return the second friction accumulated impulse
float ContactManifold::getFrictionImpulse2() const {
	return m_frictionImpulse2;
}

// Set the second friction accumulated impulse
void ContactManifold::setFrictionImpulse2(float frictionImpulse2) {
	m_frictionImpulse2 = frictionImpulse2;
}

// Return the friction twist accumulated impulse
float ContactManifold::getFrictionTwistImpulse() const {
	return m_frictionTwistImpulse;
}

// Set the friction twist accumulated impulse
void ContactManifold::setFrictionTwistImpulse(float frictionTwistImpulse) {
	m_frictionTwistImpulse = frictionTwistImpulse;
}

// Set the accumulated rolling resistance impulse
void ContactManifold::setRollingResistanceImpulse(const vec3& rollingResistanceImpulse) {
	m_rollingResistanceImpulse = rollingResistanceImpulse;
}

// Return a contact point of the manifold
ContactPoint* ContactManifold::getContactPoint(uint32_t index) const {
	assert(index < m_nbContactPoints);
	return m_contactPoints[index];
}

// Return true if the contact manifold has already been added int32_to an island
bool ContactManifold::isAlreadyInIsland() const {
	return m_isAlreadyInIsland;
}

// Return the normalized averaged normal vector
vec3 ContactManifold::getAverageContactNormal() const {
	vec3 averageNormal;
	for (uint32_t i=0; i<m_nbContactPoints; i++) {
		averageNormal += m_contactPoints[i]->getNormal();
	}
	return averageNormal.safeNormalized();
}

// Return the largest depth of all the contact points
float ContactManifold::getLargestContactDepth() const {
	float largestDepth = 0.0f;
	for (uint32_t i=0; i<m_nbContactPoints; i++) {
		float depth = m_contactPoints[i]->getPenetrationDepth();
		if (depth > largestDepth) {
			largestDepth = depth;
		}
	}
	return largestDepth;
}
