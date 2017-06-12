/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/memory/MemoryAllocator.h>
#include <cstdlib>
#include <cassert>

using namespace reactphysics3d;

// Initialization of static variables
bool MemoryAllocator::isMapSizeToHeadIndexInitialized = false;
size_t MemoryAllocator::m_unitSizes[NB_HEAPS];
int32_t MemoryAllocator::m_mapSizeToHeapIndex[MAX_UNIT_SIZE + 1];

// Constructor
MemoryAllocator::MemoryAllocator() {

	// Allocate some memory to manage the blocks
	m_numberAllocatedMemoryBlocks = 64;
	m_numberCurrentMemoryBlocks = 0;
	const size_t sizeToAllocate = m_numberAllocatedMemoryBlocks * sizeof(MemoryBlock);
	m_memoryBlocks = (MemoryBlock*) malloc(sizeToAllocate);
	memset(m_memoryBlocks, 0, sizeToAllocate);
	memset(m_freeMemoryUnits, 0, sizeof(m_freeMemoryUnits));

#ifndef NDEBUG
		m_numberTimesAllocateMethodCalled = 0;
#endif

	// If the m_mapSizeToHeapIndex has not been initialized yet
	if (!isMapSizeToHeadIndexInitialized) {

		// Initialize the array that contains the sizes the memory units that will
		// be allocated in each different heap
		for (int32_t i=0; i < NB_HEAPS; i++) {
			m_unitSizes[i] = (i+1) * 8;
		}

		// Initialize the lookup table that maps the size to allocated to the
		// corresponding heap we will use for the allocation
		uint32_t j = 0;
		m_mapSizeToHeapIndex[0] = -1;	// This element should not be used
		for (uint32_t i=1; i <= MAX_UNIT_SIZE; i++) {
			if (i <= m_unitSizes[j]) {
				m_mapSizeToHeapIndex[i] = j;
			}
			else {
				j++;
				m_mapSizeToHeapIndex[i] = j;
			}
		}

		isMapSizeToHeadIndexInitialized = true;
	}
}

// Destructor
MemoryAllocator::~MemoryAllocator() {

	// Release the memory allocated for each block
	for (uint32_t i=0; i<m_numberCurrentMemoryBlocks; i++) {
		free(m_memoryBlocks[i].memoryUnits);
	}

	free(m_memoryBlocks);

#ifndef NDEBUG
		// Check that the allocate() and release() methods have been called the same
		// number of times to avoid memory leaks.
		assert(m_numberTimesAllocateMethodCalled == 0);
#endif
}

// Allocate memory of a given size (in bytes) and return a pointer to the
// allocated memory.
void* MemoryAllocator::allocate(size_t size) {

	// We cannot allocate zero bytes
	if (size == 0) return NULL;

#ifndef NDEBUG
		m_numberTimesAllocateMethodCalled++;
#endif

	// If we need to allocate more than the maximum memory unit size
	if (size > MAX_UNIT_SIZE) {

		// Allocate memory using standard malloc() function
		return malloc(size);
	}

	// Get the index of the heap that will take care of the allocation request
	int32_t indexHeap = m_mapSizeToHeapIndex[size];
	assert(indexHeap >= 0 && indexHeap < NB_HEAPS);

	// If there still are free memory units in the corresponding heap
	if (m_freeMemoryUnits[indexHeap] != NULL) {

		// Return a pointer to the memory unit
		MemoryUnit* unit = m_freeMemoryUnits[indexHeap];
		m_freeMemoryUnits[indexHeap] = unit->nextUnit;
		return unit;
	}
	else {  // If there is no more free memory units in the corresponding heap

		// If we need to allocate more memory to containsthe blocks
		if (m_numberCurrentMemoryBlocks == m_numberAllocatedMemoryBlocks) {

			// Allocate more memory to contain the blocks
			MemoryBlock* currentMemoryBlocks = m_memoryBlocks;
			m_numberAllocatedMemoryBlocks += 64;
			m_memoryBlocks = (MemoryBlock*) malloc(m_numberAllocatedMemoryBlocks * sizeof(MemoryBlock));
			memcpy(m_memoryBlocks, currentMemoryBlocks,m_numberCurrentMemoryBlocks * sizeof(MemoryBlock));
			memset(m_memoryBlocks + m_numberCurrentMemoryBlocks, 0, 64 * sizeof(MemoryBlock));
			free(currentMemoryBlocks);
		}

		// Allocate a new memory blocks for the corresponding heap and divide it in many
		// memory units
		MemoryBlock* newBlock = m_memoryBlocks + m_numberCurrentMemoryBlocks;
		newBlock->memoryUnits = (MemoryUnit*) malloc(BLOCK_SIZE);
		assert(newBlock->memoryUnits != NULL);
		size_t unitSize = m_unitSizes[indexHeap];
		uint32_t nbUnits = BLOCK_SIZE / unitSize;
		assert(nbUnits * unitSize <= BLOCK_SIZE);
		for (size_t i=0; i < nbUnits - 1; i++) {
			MemoryUnit* unit = (MemoryUnit*) ((size_t)newBlock->memoryUnits + unitSize * i);
			MemoryUnit* nextUnit = (MemoryUnit*) ((size_t)newBlock->memoryUnits + unitSize * (i+1));
			unit->nextUnit = nextUnit;
		}
		MemoryUnit* lastUnit = (MemoryUnit*) ((size_t)newBlock->memoryUnits + unitSize*(nbUnits-1));
		lastUnit->nextUnit = NULL;

		// Add the new allocated block int32_to the list of free memory units in the heap
		m_freeMemoryUnits[indexHeap] = newBlock->memoryUnits->nextUnit;
		m_numberCurrentMemoryBlocks++;

		// Return the pointer to the first memory unit of the new allocated block
		return newBlock->memoryUnits;
	}
}

// Release previously allocated memory.
void MemoryAllocator::release(void* pointer, size_t size) {

	// Cannot release a 0-byte allocated memory
	if (size == 0) return;

#ifndef NDEBUG
		m_numberTimesAllocateMethodCalled--;
#endif

	// If the size is larger than the maximum memory unit size
	if (size > MAX_UNIT_SIZE) {

		// Release the memory using the standard free() function
		free(pointer);
		return;
	}

	// Get the index of the heap that has handled the corresponding allocation request
	int32_t indexHeap = m_mapSizeToHeapIndex[size];
	assert(indexHeap >= 0 && indexHeap < NB_HEAPS);

	// Insert the released memory unit int32_to the list of free memory units of the
	// corresponding heap
	MemoryUnit* releasedUnit = (MemoryUnit*) pointer;
	releasedUnit->nextUnit = m_freeMemoryUnits[indexHeap];
	m_freeMemoryUnits[indexHeap] = releasedUnit;
}
