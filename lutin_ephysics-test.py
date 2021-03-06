#!/usr/bin/python
import realog.debug as debug
import lutin.tools as tools


def get_type():
	return "BINARY"

def get_sub_type():
	return "TEST"

def get_desc():
	return "Ewol Physic engine TEST UNIT"

def get_licence():
	return "BSD-3"

def get_compagny_type():
	return "com"

def get_compagny_name():
	return "atria-soft"

def get_maintainer():
	return "authors.txt"

def configure(target, my_module):
	my_module.add_src_file([
		'test/main.cpp',
		'test/testAABB.cpp',
		'test/testCollisionWorld.cpp',
		'test/testDynamicAABBTree.cpp',
		'test/testPointInside.cpp',
		'test/testRaycast.cpp',
		])
	my_module.add_depend([
		'ephysics',
		'etest',
		'test-debug'
		])
	my_module.add_path(".")
	return True

