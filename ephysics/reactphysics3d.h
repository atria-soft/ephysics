/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2016 Daniel Chappuis                                       *
*********************************************************************************
*                                                                               *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.                                                         *
*                                                                               *
* Permission is granted to anyone to use this software for any purpose,         *
* including commercial applications, and to alter it and redistribute it        *
* freely, subject to the following restrictions:                                *
*                                                                               *
* 1. The origin of this software must not be misrepresented; you must not claim *
*    that you wrote the original software. If you use this software in a        *
*    product, an acknowledgment in the product documentation would be           *
*    appreciated but is not required.                                           *
*                                                                               *
* 2. Altered source versions must be plainly marked as such, and must not be    *
*    misrepresented as being the original software.                             *
*                                                                               *
* 3. This notice may not be removed or altered from any source distribution.    *
*                                                                               *
********************************************************************************/


/********************************************************************************
* ReactPhysics3D                                                                *
* Version 0.6.0                                                                 *
* http://www.reactphysics3d.com                                                 *
* Daniel Chappuis                                                               *
********************************************************************************/

#ifndef REACTPHYSICS3D_H
#define REACTPHYSICS3D_H

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

/// Alias to the ReactPhysics3D namespace
namespace rp3d = reactphysics3d;

#endif
