/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

#ifdef IS_PROFILING_ACTIVE

// Libraries
#include <ephysics/configuration.h>
#include <ephysics/Timer.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class ProfileNode
/**
 * It represents a profile sample in the profiler tree.
 */
class ProfileNode {

	private :

		// -------------------- Attributes -------------------- //

		/// Name of the node
		const char* mName;

		/// Total number of calls of this node
		uint32_t mNbTotalCalls;

		/// Starting time of the sampling of corresponding block of code
		long double mStartingTime;

		/// Total time spent in the block of code
		long double mTotalTime;

		/// Recursion counter
		int32_t mRecursionCounter;

		/// Pointer to the parent node
		ProfileNode* mParentNode;

		/// Pointer to a child node
		ProfileNode* mChildNode;

		/// Pointer to a sibling node
		ProfileNode* mSiblingNode;

	public :

		// -------------------- Methods -------------------- //

		/// Constructor
		ProfileNode(const char* name, ProfileNode* parentNode);

		/// Destructor
		~ProfileNode();

		/// Return a pointer to a sub node
		ProfileNode* findSubNode(const char* name);

		/// Return a pointer to the parent node
		ProfileNode* getParentNode();

		/// Return a pointer to a sibling node
		ProfileNode* getSiblingNode();

		/// Return a pointer to a child node
		ProfileNode* getChildNode();

		/// Return the name of the node
		const char* getName();

		/// Return the total number of call of the corresponding block of code
		uint32_t getNbTotalCalls() const;

		/// Return the total time spent in the block of code
		long double getTotalTime() const;

		/// Called when we enter the block of code corresponding to this profile node
		void enterBlockOfCode();

		/// Called when we exit the block of code corresponding to this profile node
		bool exitBlockOfCode();

		/// Reset the profiling of the node
		void reset();

		/// Destroy the node
		void destroy();
};

// Class ProfileNodeIterator
/**
 * This class allows us to iterator over the profiler tree.
 */
class ProfileNodeIterator {

	private :

		// -------------------- Attributes -------------------- //

		/// Current parent node
		ProfileNode* mCurrentParentNode;

		/// Current child node
		ProfileNode* mCurrentChildNode;

	public :

		// -------------------- Methods -------------------- //

		/// Constructor
		ProfileNodeIterator(ProfileNode* startingNode);

		/// Go to the first node
		void first();

		/// Go to the next node
		void next();

		/// Enter a given child node
		void enterChild(int32_t index);

		/// Enter a given parent node
		void enterParent();

		/// Return true if we are at the root of the profiler tree
		bool isRoot();

		/// Return true if we are at the end of a branch of the profiler tree
		bool isEnd();

		/// Return the name of the current node
		const char* getCurrentName();

		/// Return the total time of the current node
		long double getCurrentTotalTime();

		/// Return the total number of calls of the current node
		uint32_t getCurrentNbTotalCalls();

		/// Return the name of the current parent node
		const char* getCurrentParentName();

		/// Return the total time of the current parent node
		long double getCurrentParentTotalTime();

		/// Return the total number of calls of the current parent node
		uint32_t getCurrentParentNbTotalCalls();
};

// Class Profiler
/**
 * This is the main class of the profiler. This profiler is based on "Real-Time Hierarchical
 * Profiling" article from "Game Programming Gems 3" by Greg Hjelstrom and Byon Garrabrant.
 */
class Profiler {

	private :

		// -------------------- Attributes -------------------- //

		/// Root node of the profiler tree
		static ProfileNode mRootNode;

		/// Current node in the current execution
		static ProfileNode* mCurrentNode;

		/// Frame counter
		static uint32_t mFrameCounter;

		/// Starting profiling time
		static long double mProfilingStartTime;

		/// Recursively print32_t the report of a given node of the profiler tree
		static void print32_tRecursiveNodeReport(ProfileNodeIterator* iterator,
											 int32_t spacing,
											 std::ostream& outputStream);

	public :

		// -------------------- Methods -------------------- //

		/// Method called when we want to start profiling a block of code.
		static void startProfilingBlock(const char *name);

		/// Method called at the end of the scope where the
		/// startProfilingBlock() method has been called.
		static void stopProfilingBlock();

		/// Reset the timing data of the profiler (but not the profiler tree structure)
		static void reset();

		/// Return the number of frames
		static uint32_t getNbFrames();

		/// Return the elasped time since the start/reset of the profiling
		static long double getElapsedTimeSinceStart();

		/// Increment the frame counter
		static void incrementFrameCounter();

		/// Return an iterator over the profiler tree starting at the root
		static ProfileNodeIterator* getIterator();

		/// Print32_t the report of the profiler in a given output stream
		static void print32_tReport(std::ostream& outputStream);

		/// Destroy a previously allocated iterator
		static void destroyIterator(ProfileNodeIterator* iterator);

		/// Destroy the profiler (release the memory)
		static void destroy();
};

// Class ProfileSample
/**
 * This class is used to represent a profile sample. It is constructed at the
 * beginning of a code block we want to profile and destructed at the end of the
 * scope to profile.
 */
class ProfileSample {

	public :

		// -------------------- Methods -------------------- //

		/// Constructor
		ProfileSample(const char* name) {

			// Ask the profiler to start profiling a block of code
			Profiler::startProfilingBlock(name);
		}

		/// Destructor
		~ProfileSample() {

			// Tell the profiler to stop profiling a block of code
			Profiler::stopProfilingBlock();
		}
};

// Use this macro to start profile a block of code
#define PROFILE(name) ProfileSample profileSample(name)

// Return true if we are at the root of the profiler tree
inline bool ProfileNodeIterator::isRoot() {
	return (mCurrentParentNode->getParentNode() == NULL);
}

// Return true if we are at the end of a branch of the profiler tree
inline bool ProfileNodeIterator::isEnd() {
	return (mCurrentChildNode == NULL);
}

// Return the name of the current node
inline const char* ProfileNodeIterator::getCurrentName() {
	return mCurrentChildNode->getName();
}

// Return the total time of the current node
inline long double ProfileNodeIterator::getCurrentTotalTime() {
	return mCurrentChildNode->getTotalTime();
}

// Return the total number of calls of the current node
inline uint32_t ProfileNodeIterator::getCurrentNbTotalCalls() {
	return mCurrentChildNode->getNbTotalCalls();
}

// Return the name of the current parent node
inline const char* ProfileNodeIterator::getCurrentParentName() {
	return mCurrentParentNode->getName();
}

// Return the total time of the current parent node
inline long double ProfileNodeIterator::getCurrentParentTotalTime() {
	return mCurrentParentNode->getTotalTime();
}

// Return the total number of calls of the current parent node
inline uint32_t ProfileNodeIterator::getCurrentParentNbTotalCalls() {
	return mCurrentParentNode->getNbTotalCalls();
}

// Go to the first node
inline void ProfileNodeIterator::first() {
	mCurrentChildNode = mCurrentParentNode->getChildNode();
}

// Go to the next node
inline void ProfileNodeIterator::next() {
	mCurrentChildNode = mCurrentChildNode->getSiblingNode();
}

// Return a pointer to the parent node
inline ProfileNode* ProfileNode::getParentNode() {
	return mParentNode;
}

// Return a pointer to a sibling node
inline ProfileNode* ProfileNode::getSiblingNode() {
	return mSiblingNode;
}

// Return a pointer to a child node
inline ProfileNode* ProfileNode::getChildNode() {
	return mChildNode;
}

// Return the name of the node
inline const char* ProfileNode::getName() {
	return mName;
}

// Return the total number of call of the corresponding block of code
inline uint32_t ProfileNode::getNbTotalCalls() const {
	return mNbTotalCalls;
}

// Return the total time spent in the block of code
inline long double ProfileNode::getTotalTime() const {
	return mTotalTime;
}

// Return the number of frames
inline uint32_t Profiler::getNbFrames() {
	return mFrameCounter;
}

// Return the elasped time since the start/reset of the profiling
inline long double Profiler::getElapsedTimeSinceStart() {
	long double currentTime = Timer::getCurrentSystemTime() * 1000.0;
	return currentTime - mProfilingStartTime;
}

// Increment the frame counter
inline void Profiler::incrementFrameCounter() {
	mFrameCounter++;
}

// Return an iterator over the profiler tree starting at the root
inline ProfileNodeIterator* Profiler::getIterator() {
	return new ProfileNodeIterator(&mRootNode);
}

// Destroy a previously allocated iterator
inline void Profiler::destroyIterator(ProfileNodeIterator* iterator) {
	delete iterator;
}

// Destroy the profiler (release the memory)
inline void Profiler::destroy() {
	mRootNode.destroy();
}

}

#else   // In profile is not active

// Empty macro in case profiling is not active
#define PROFILE(name)

#endif
