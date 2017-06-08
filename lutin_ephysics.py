#!/usr/bin/python
import lutin.debug as debug
import lutin.tools as tools


def get_type():
	return "LIBRARY"

def get_desc():
	return "Ewol Physic engine"

def get_licence():
	return "BSD-3"

def get_compagny_type():
	return "com"

def get_compagny_name():
	return "atria-soft"

def get_maintainer():
	return "authors.txt"

def get_version():
	return "version.txt"

def configure(target, my_module):
	my_module.add_extra_flags()
	# add the file to compile:
	my_module.add_src_file([
		'ephysics/memory/MemoryAllocator.cpp',
		'ephysics/constraint/Joint.cpp',
		'ephysics/constraint/HingeJoint.cpp',
		'ephysics/constraint/ContactPoint.cpp',
		'ephysics/constraint/BallAndSocketJoint.cpp',
		'ephysics/constraint/SliderJoint.cpp',
		'ephysics/constraint/FixedJoint.cpp',
		'ephysics/collision/ContactManifoldSet.cpp',
		'ephysics/collision/RaycastInfo.cpp',
		'ephysics/collision/narrowphase/GJK/Simplex.cpp',
		'ephysics/collision/narrowphase/GJK/GJKAlgorithm.cpp',
		'ephysics/collision/narrowphase/DefaultCollisionDispatch.cpp',
		'ephysics/collision/narrowphase/SphereVsSphereAlgorithm.cpp',
		'ephysics/collision/narrowphase/NarrowPhaseAlgorithm.cpp',
		'ephysics/collision/narrowphase/ConcaveVsConvexAlgorithm.cpp',
		'ephysics/collision/narrowphase/EPA/EPAAlgorithm.cpp',
		'ephysics/collision/narrowphase/EPA/TrianglesStore.cpp',
		'ephysics/collision/narrowphase/EPA/TriangleEPA.cpp',
		'ephysics/collision/narrowphase/EPA/EdgeEPA.cpp',
		'ephysics/collision/ProxyShape.cpp',
		'ephysics/collision/shapes/ConcaveShape.cpp',
		'ephysics/collision/shapes/CylinderShape.cpp',
		'ephysics/collision/shapes/SphereShape.cpp',
		'ephysics/collision/shapes/CapsuleShape.cpp',
		'ephysics/collision/shapes/ConvexMeshShape.cpp',
		'ephysics/collision/shapes/CollisionShape.cpp',
		'ephysics/collision/shapes/BoxShape.cpp',
		'ephysics/collision/shapes/TriangleShape.cpp',
		'ephysics/collision/shapes/HeightFieldShape.cpp',
		'ephysics/collision/shapes/ConvexShape.cpp',
		'ephysics/collision/shapes/ConeShape.cpp',
		'ephysics/collision/shapes/ConcaveMeshShape.cpp',
		'ephysics/collision/shapes/AABB.cpp',
		'ephysics/collision/TriangleMesh.cpp',
		'ephysics/collision/CollisionDetection.cpp',
		'ephysics/collision/TriangleVertexArray.cpp',
		'ephysics/collision/ContactManifold.cpp',
		'ephysics/collision/broadphase/DynamicAABBTree.cpp',
		'ephysics/collision/broadphase/BroadPhaseAlgorithm.cpp',
		'ephysics/body/RigidBody.cpp',
		'ephysics/body/Body.cpp',
		'ephysics/body/CollisionBody.cpp',
		'ephysics/mathematics/Vector2.cpp',
		'ephysics/mathematics/Vector3.cpp',
		'ephysics/mathematics/Transform.cpp',
		'ephysics/mathematics/Matrix2x2.cpp',
		'ephysics/mathematics/mathematics_functions.cpp',
		'ephysics/mathematics/Matrix3x3.cpp',
		'ephysics/mathematics/Quaternion.cpp',
		'ephysics/engine/CollisionWorld.cpp',
		'ephysics/engine/OverlappingPair.cpp',
		'ephysics/engine/Material.cpp',
		'ephysics/engine/Island.cpp',
		'ephysics/engine/Profiler.cpp',
		'ephysics/engine/ConstraintSolver.cpp',
		'ephysics/engine/DynamicsWorld.cpp',
		'ephysics/engine/ContactSolver.cpp',
		'ephysics/engine/Timer.cpp',
		])
	
	my_module.add_header_file([
		'ephysics/memory/MemoryAllocator.h',
		'ephysics/memory/Stack.h',
		'ephysics/constraint/BallAndSocketJoint.h',
		'ephysics/constraint/Joint.h',
		'ephysics/constraint/FixedJoint.h',
		'ephysics/constraint/HingeJoint.h',
		'ephysics/constraint/ContactPoint.h',
		'ephysics/constraint/SliderJoint.h',
		'ephysics/collision/TriangleVertexArray.h',
		'ephysics/collision/ContactManifold.h',
		'ephysics/collision/ContactManifoldSet.h',
		'ephysics/collision/narrowphase/SphereVsSphereAlgorithm.h',
		'ephysics/collision/narrowphase/GJK/Simplex.h',
		'ephysics/collision/narrowphase/GJK/GJKAlgorithm.h',
		'ephysics/collision/narrowphase/ConcaveVsConvexAlgorithm.h',
		'ephysics/collision/narrowphase/CollisionDispatch.h',
		'ephysics/collision/narrowphase/DefaultCollisionDispatch.h',
		'ephysics/collision/narrowphase/NarrowPhaseAlgorithm.h',
		'ephysics/collision/narrowphase/EPA/EdgeEPA.h',
		'ephysics/collision/narrowphase/EPA/EPAAlgorithm.h',
		'ephysics/collision/narrowphase/EPA/TrianglesStore.h',
		'ephysics/collision/narrowphase/EPA/TriangleEPA.h',
		'ephysics/collision/CollisionDetection.h',
		'ephysics/collision/shapes/TriangleShape.h',
		'ephysics/collision/shapes/AABB.h',
		'ephysics/collision/shapes/CapsuleShape.h',
		'ephysics/collision/shapes/SphereShape.h',
		'ephysics/collision/shapes/CollisionShape.h',
		'ephysics/collision/shapes/BoxShape.h',
		'ephysics/collision/shapes/ConcaveMeshShape.h',
		'ephysics/collision/shapes/ConvexMeshShape.h',
		'ephysics/collision/shapes/HeightFieldShape.h',
		'ephysics/collision/shapes/CylinderShape.h',
		'ephysics/collision/shapes/ConeShape.h',
		'ephysics/collision/shapes/ConvexShape.h',
		'ephysics/collision/shapes/ConcaveShape.h',
		'ephysics/collision/CollisionShapeInfo.h',
		'ephysics/collision/TriangleMesh.h',
		'ephysics/collision/RaycastInfo.h',
		'ephysics/collision/ProxyShape.h',
		'ephysics/collision/broadphase/DynamicAABBTree.h',
		'ephysics/collision/broadphase/BroadPhaseAlgorithm.h',
		'ephysics/configuration.h',
		'ephysics/reactphysics3d.h',
		'ephysics/body/Body.h',
		'ephysics/body/RigidBody.h',
		'ephysics/body/CollisionBody.h',
		'ephysics/mathematics/mathematics.h',
		'ephysics/mathematics/Ray.h',
		'ephysics/mathematics/Matrix2x2.h',
		'ephysics/mathematics/Vector2.h',
		'ephysics/mathematics/mathematics_functions.h',
		'ephysics/mathematics/Vector3.h',
		'ephysics/mathematics/Quaternion.h',
		'ephysics/mathematics/Transform.h',
		'ephysics/mathematics/Matrix3x3.h',
		'ephysics/engine/CollisionWorld.h',
		'ephysics/engine/DynamicsWorld.h',
		'ephysics/engine/ConstraintSolver.h',
		'ephysics/engine/OverlappingPair.h',
		'ephysics/engine/Island.h',
		'ephysics/engine/ContactSolver.h',
		'ephysics/engine/Material.h',
		'ephysics/engine/Profiler.h',
		'ephysics/engine/Timer.h',
		'ephysics/engine/Impulse.h',
		'ephysics/engine/EventListener.h'
		])
	
	# build in C++ mode
	my_module.compile_version("c++", 2011)
	# add dependency of the generic C++ library:
	my_module.add_depend([
		'cxx',
		'm',
		'elog',
		'etk',
		'ememory'
		])
	# TODO: Remove this ...
	my_module.add_flag('c++', "-Wno-overloaded-virtual", export=True)
	my_module.add_path(".")
	return True


