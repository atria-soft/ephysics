/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/configuration.h>
#include <ephysics/mathematics/mathematics.h>
#include <ephysics/body/CollisionBody.h>
#include <ephysics/body/RigidBody.h>
#include <ephysics/engine/DynamicsWorld.h>
#include <ephysics/engine/CollisionWorld.h>
#include <ephysics/engine/Material.h>
#include <ephysics/engine/EventListener.h>
#include <ephysics/collision/shapes/CollisionShape.h>
#include <ephysics/collision/shapes/BoxShape.h>
#include <ephysics/collision/shapes/SphereShape.h>
#include <ephysics/collision/shapes/ConeShape.h>
#include <ephysics/collision/shapes/CylinderShape.h>
#include <ephysics/collision/shapes/CapsuleShape.h>
#include <ephysics/collision/shapes/ConvexMeshShape.h>
#include <ephysics/collision/shapes/ConcaveMeshShape.h>
#include <ephysics/collision/shapes/HeightFieldShape.h>
#include <ephysics/collision/shapes/AABB.h>
#include <ephysics/collision/ProxyShape.h>
#include <ephysics/collision/RaycastInfo.h>
#include <ephysics/collision/TriangleMesh.h>
#include <ephysics/collision/TriangleVertexArray.h>
#include <ephysics/constraint/BallAndSocketJoint.h>
#include <ephysics/constraint/SliderJoint.h>
#include <ephysics/constraint/HingeJoint.h>
#include <ephysics/constraint/FixedJoint.h>

/// Alias to the ReactPhysics3D namespace TODO: Remove this ...
namespace rp3d = reactphysics3d;
