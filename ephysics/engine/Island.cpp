/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/engine/Island.h>

using namespace reactphysics3d;

// Constructor
Island::Island(uint32_t nbMaxBodies, uint32_t nbMaxContactManifolds, uint32_t nbMaxJoints,
			   MemoryAllocator& memoryAllocator)
	   : mBodies(NULL), mContactManifolds(NULL), mJoints(NULL), mNbBodies(0),
		 mNbContactManifolds(0), mNbJoints(0), mMemoryAllocator(memoryAllocator) {

	// Allocate memory for the arrays
	mNbAllocatedBytesBodies = sizeof(RigidBody*) * nbMaxBodies;
	mBodies = (RigidBody**) mMemoryAllocator.allocate(mNbAllocatedBytesBodies);
	mNbAllocatedBytesContactManifolds = sizeof(ContactManifold*) * nbMaxContactManifolds;
	mContactManifolds = (ContactManifold**) mMemoryAllocator.allocate(
																mNbAllocatedBytesContactManifolds);
	mNbAllocatedBytesJoints = sizeof(Joint*) * nbMaxJoints;
	mJoints = (Joint**) mMemoryAllocator.allocate(mNbAllocatedBytesJoints);
}

// Destructor
Island::~Island() {

	// Release the memory of the arrays
	mMemoryAllocator.release(mBodies, mNbAllocatedBytesBodies);
	mMemoryAllocator.release(mContactManifolds, mNbAllocatedBytesContactManifolds);
	mMemoryAllocator.release(mJoints, mNbAllocatedBytesJoints);
}
