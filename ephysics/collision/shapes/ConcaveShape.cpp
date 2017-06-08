/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/collision/shapes/ConcaveShape.h>


// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
ConcaveShape::ConcaveShape(CollisionShapeType type)
			 : CollisionShape(type), m_isSmoothMeshCollisionEnabled(false),
			   m_triangleMargin(0), m_raycastTestType(FRONT) {

}

// Destructor
ConcaveShape::~ConcaveShape() {

}
