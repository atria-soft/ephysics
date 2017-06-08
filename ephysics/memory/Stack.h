/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/configuration.h>

namespace reactphysics3d {

// Class Stack
/**
 * This class represents a simple generic stack with an initial capacity. If the number
 * of elements exceeds the capacity, the heap will be used to allocated more memory.
  */
template<typename T, uint32_t capacity>
class Stack {

	private:

		// -------------------- Attributes -------------------- //

		/// Initial array that contains the elements of the stack
		T mInitArray[capacity];

		/// Pointer to the first element of the stack
		T* mElements;

		/// Number of elements in the stack
		uint32_t mNbElements;

		/// Number of allocated elements in the stack
		uint32_t mNbAllocatedElements;

	public:

		// -------------------- Methods -------------------- //

		/// Constructor
		Stack() : mElements(mInitArray), mNbElements(0), mNbAllocatedElements(capacity) {

		}

		/// Destructor
		~Stack() {

			// If elements have been allocated on the heap
			if (mInitArray != mElements) {

				// Release the memory allocated on the heap
				free(mElements);
			}
		}

		/// Push an element int32_to the stack
		void push(const T& element);

		/// Pop an element from the stack (remove it from the stack and return it)
		T pop();

		/// Return the number of elments in the stack
		uint32_t getNbElements() const;

};

// Push an element int32_to the stack
template<typename T, uint32_t capacity>
inline void Stack<T, capacity>::push(const T& element) {

	// If we need to allocate more elements
	if (mNbElements == mNbAllocatedElements) {
		T* oldElements = mElements;
		mNbAllocatedElements *= 2;
		mElements = (T*) malloc(mNbAllocatedElements * sizeof(T));
		assert(mElements);
		memcpy(mElements, oldElements, mNbElements * sizeof(T));
		if (oldElements != mInitArray) {
			free(oldElements);
		}
	}

	mElements[mNbElements] = element;
	mNbElements++;
}

// Pop an element from the stack (remove it from the stack and return it)
template<typename T, uint32_t capacity>
inline T Stack<T, capacity>::pop() {
	assert(mNbElements > 0);
	mNbElements--;
	return mElements[mNbElements];
}

// Return the number of elments in the stack
template<typename T, uint32_t capacity>
inline uint32_t Stack<T, capacity>::getNbElements() const {
	return mNbElements;
}

}
