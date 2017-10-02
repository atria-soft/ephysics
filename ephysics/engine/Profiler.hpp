/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#pragma once

#ifdef IS_PROFILING_ACTIVE
#include <ephysics/configuration.hpp>
#include <ephysics/Timer.hpp>
namespace ephysics {
	/**
	 * @brief It represents a profile sample in the profiler tree.
	 */
	class ProfileNode {
		private :
			const char* m_name; //!< Name of the node
			uint32_t m_numberTotalCalls; //!< Total number of calls of this node
			long double m_startTime; //!< Starting time of the sampling of corresponding block of code
			long double m_totalTime; //!< Total time spent in the block of code
			int32_t m_recursionCounter; //!< Recursion counter
			ProfileNode* m_parentNode; //!< Pointer to the parent node
			ProfileNode* m_childNode; //!< Pointer to a child node
			ProfileNode* m_siblingNode; //!< Pointer to a sibling node
		public :
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
	/**
	 * @brief This class allows us to iterator over the profiler tree.
	 */
	class ProfileNodeIterator {
		private :
			ProfileNode* m_currentParentNode; //!< Current parent node
			ProfileNode* m_currentChildNode; //!< Current child node
		public :
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

	/**
	 * @brief This is the main class of the profiler. This profiler is based on "Real-Time Hierarchical
	 * Profiling" article from "Game Programming Gems 3" by Greg Hjelstrom and Byon Garrabrant.
	 */
	class Profiler {
		private :
			static ProfileNode m_rootNode; //!< Root node of the profiler tree
			static ProfileNode* m_currentNode; //!< Current node in the current execution
			static uint32_t m_frameCounter; //!< Frame counter
			static long double m_profilingStartTime; //!< Starting profiling time
		private:
			static void print32_tRecursiveNodeReport(ProfileNodeIterator* iterator,
			                                         int32_t spacing,
			                                         etk::Stream& outputStream);
		public :
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
			static void print32_tReport(etk::Stream& outputStream);
			/// Destroy a previously allocated iterator
			static void destroyIterator(ProfileNodeIterator* iterator);
			/// Destroy the profiler (release the memory)
			static void destroy();
	};

	/**
	 * @brief This class is used to represent a profile sample. It is constructed at the
	 * beginning of a code block we want to profile and destructed at the end of the
	 * scope to profile.
	 */
	class ProfileSample {
		public :
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
}
#else
	// Empty macro in case profiling is not active
	#define PROFILE(name)
#endif
