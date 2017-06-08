/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/constraint/ContactPoint.h>
#include <ephysics/collision/ProxyShape.h>

using namespace reactphysics3d;
using namespace std;

// Constructor
ContactPoint::ContactPoint(const ContactPointInfo& contactInfo)
			 : mBody1(contactInfo.shape1->getBody()), mBody2(contactInfo.shape2->getBody()),
			   mNormal(contactInfo.normal),
			   mPenetrationDepth(contactInfo.penetrationDepth),
			   mLocalPointOnBody1(contactInfo.localPoint1),
			   mLocalPointOnBody2(contactInfo.localPoint2),
			   mWorldPointOnBody1(contactInfo.shape1->getBody()->getTransform() *
								  contactInfo.shape1->getLocalToBodyTransform() *
								  contactInfo.localPoint1),
			   mWorldPointOnBody2(contactInfo.shape2->getBody()->getTransform() *
								  contactInfo.shape2->getLocalToBodyTransform() *
								  contactInfo.localPoint2),
			   mIsRestingContact(false) {

	mFrictionVectors[0] = Vector3(0, 0, 0);
	mFrictionVectors[1] = Vector3(0, 0, 0);

	assert(mPenetrationDepth > 0.0);

}

// Destructor
ContactPoint::~ContactPoint() {

}
