#!/usr/bin/python
import lutin.debug as debug
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
	    'test/Test.cpp',
	    'test/TestSuite.cpp',
	    'test/main.cpp'
	    ])
	my_module.add_depend([
	    'ephysics',
	    'gtest',
	    'test-debug'
	    ])
	my_module.add_path(".")
	return True

