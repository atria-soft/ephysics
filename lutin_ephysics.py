#!/usr/bin/python
import realog.debug as debug
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
		'ephysics/debug.cpp',
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
		'ephysics/mathematics/mathematics_functions.cpp',
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
		'ephysics/debug.hpp',
		'ephysics/memory/Stack.hpp',
		'ephysics/constraint/BallAndSocketJoint.hpp',
		'ephysics/constraint/Joint.hpp',
		'ephysics/constraint/FixedJoint.hpp',
		'ephysics/constraint/HingeJoint.hpp',
		'ephysics/constraint/ContactPoint.hpp',
		'ephysics/constraint/SliderJoint.hpp',
		'ephysics/collision/TriangleVertexArray.hpp',
		'ephysics/collision/ContactManifold.hpp',
		'ephysics/collision/ContactManifoldSet.hpp',
		'ephysics/collision/narrowphase/SphereVsSphereAlgorithm.hpp',
		'ephysics/collision/narrowphase/GJK/Simplex.hpp',
		'ephysics/collision/narrowphase/GJK/GJKAlgorithm.hpp',
		'ephysics/collision/narrowphase/ConcaveVsConvexAlgorithm.hpp',
		'ephysics/collision/narrowphase/CollisionDispatch.hpp',
		'ephysics/collision/narrowphase/DefaultCollisionDispatch.hpp',
		'ephysics/collision/narrowphase/NarrowPhaseAlgorithm.hpp',
		'ephysics/collision/narrowphase/EPA/EdgeEPA.hpp',
		'ephysics/collision/narrowphase/EPA/EPAAlgorithm.hpp',
		'ephysics/collision/narrowphase/EPA/TrianglesStore.hpp',
		'ephysics/collision/narrowphase/EPA/TriangleEPA.hpp',
		'ephysics/collision/CollisionDetection.hpp',
		'ephysics/collision/shapes/TriangleShape.hpp',
		'ephysics/collision/shapes/AABB.hpp',
		'ephysics/collision/shapes/CapsuleShape.hpp',
		'ephysics/collision/shapes/SphereShape.hpp',
		'ephysics/collision/shapes/CollisionShape.hpp',
		'ephysics/collision/shapes/BoxShape.hpp',
		'ephysics/collision/shapes/ConcaveMeshShape.hpp',
		'ephysics/collision/shapes/ConvexMeshShape.hpp',
		'ephysics/collision/shapes/HeightFieldShape.hpp',
		'ephysics/collision/shapes/CylinderShape.hpp',
		'ephysics/collision/shapes/ConeShape.hpp',
		'ephysics/collision/shapes/ConvexShape.hpp',
		'ephysics/collision/shapes/ConcaveShape.hpp',
		'ephysics/collision/CollisionShapeInfo.hpp',
		'ephysics/collision/TriangleMesh.hpp',
		'ephysics/collision/RaycastInfo.hpp',
		'ephysics/collision/ProxyShape.hpp',
		'ephysics/collision/broadphase/DynamicAABBTree.hpp',
		'ephysics/collision/broadphase/BroadPhaseAlgorithm.hpp',
		'ephysics/configuration.hpp',
		'ephysics/ephysics.hpp',
		'ephysics/body/Body.hpp',
		'ephysics/body/RigidBody.hpp',
		'ephysics/body/CollisionBody.hpp',
		'ephysics/mathematics/mathematics.hpp',
		'ephysics/mathematics/Ray.hpp',
		'ephysics/mathematics/mathematics_functions.hpp',
		'ephysics/engine/CollisionWorld.hpp',
		'ephysics/engine/DynamicsWorld.hpp',
		'ephysics/engine/ConstraintSolver.hpp',
		'ephysics/engine/OverlappingPair.hpp',
		'ephysics/engine/Island.hpp',
		'ephysics/engine/ContactSolver.hpp',
		'ephysics/engine/Material.hpp',
		'ephysics/engine/Profiler.hpp',
		'ephysics/engine/Timer.hpp',
		'ephysics/engine/Impulse.hpp',
		'ephysics/engine/EventListener.hpp'
		])
	
	# build in C++ mode
	my_module.compile_version("c++", 2011)
	# add dependency of the generic C++ library:
	my_module.add_depend([
		'm',
		'elog',
		'etk',
		'ememory',
		'echrono'
		])
	# TODO: Remove this ...
	#my_module.add_flag('c++', "-Wno-overloaded-virtual", export=True)
	#my_module.add_path(".")
	return True


