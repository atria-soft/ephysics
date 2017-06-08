/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/mathematics/Transform.h>

// Namespaces
using namespace reactphysics3d;

// Constructor
Transform::Transform() : mPosition(Vector3(0.0, 0.0, 0.0)), mOrientation(Quaternion::identity()) {

}

// Constructor
Transform::Transform(const Vector3& position, const Matrix3x3& orientation)
		  : mPosition(position), mOrientation(Quaternion(orientation)) {

}

// Constructor
Transform::Transform(const Vector3& position, const Quaternion& orientation)
		  : mPosition(position), mOrientation(orientation) {

}

// Copy-constructor
Transform::Transform(const Transform& transform)
		  : mPosition(transform.mPosition), mOrientation(transform.mOrientation) {

}

// Destructor
Transform::~Transform() {
	
}
