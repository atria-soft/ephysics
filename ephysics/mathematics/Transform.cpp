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
Transform::Transform() : m_position(Vector3(0.0, 0.0, 0.0)), m_orientation(Quaternion::identity()) {

}

// Constructor
Transform::Transform(const Vector3& position, const Matrix3x3& orientation)
		  : m_position(position), m_orientation(Quaternion(orientation)) {

}

// Constructor
Transform::Transform(const Vector3& position, const Quaternion& orientation)
		  : m_position(position), m_orientation(orientation) {

}

// Copy-constructor
Transform::Transform(const Transform& transform)
		  : m_position(transform.m_position), m_orientation(transform.m_orientation) {

}

// Destructor
Transform::~Transform() {
	
}
