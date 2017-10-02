/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#pragma once

// Libraries
#include <ephysics/configuration.hpp>
#include <ephysics/mathematics/mathematics.hpp>
#include <ephysics/body/CollisionBody.hpp>
#include <ephysics/body/RigidBody.hpp>
#include <ephysics/engine/DynamicsWorld.hpp>
#include <ephysics/engine/CollisionWorld.hpp>
#include <ephysics/engine/Material.hpp>
#include <ephysics/engine/EventListener.hpp>
#include <ephysics/collision/shapes/CollisionShape.hpp>
#include <ephysics/collision/shapes/BoxShape.hpp>
#include <ephysics/collision/shapes/SphereShape.hpp>
#include <ephysics/collision/shapes/ConeShape.hpp>
#include <ephysics/collision/shapes/CylinderShape.hpp>
#include <ephysics/collision/shapes/CapsuleShape.hpp>
#include <ephysics/collision/shapes/ConvexMeshShape.hpp>
#include <ephysics/collision/shapes/ConcaveMeshShape.hpp>
#include <ephysics/collision/shapes/HeightFieldShape.hpp>
#include <ephysics/collision/shapes/AABB.hpp>
#include <ephysics/collision/ProxyShape.hpp>
#include <ephysics/collision/RaycastInfo.hpp>
#include <ephysics/collision/TriangleMesh.hpp>
#include <ephysics/collision/TriangleVertexArray.hpp>
#include <ephysics/constraint/BallAndSocketJoint.hpp>
#include <ephysics/constraint/SliderJoint.hpp>
#include <ephysics/constraint/HingeJoint.hpp>
#include <ephysics/constraint/FixedJoint.hpp>

