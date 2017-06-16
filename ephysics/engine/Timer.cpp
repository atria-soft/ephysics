/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/engine/Timer.hpp>

// We want to use the ReactPhysics3D namespace
using namespace ephysics;

// Constructor
Timer::Timer(double timeStep) : m_timeStep(timeStep), m_isRunning(false) {
	assert(timeStep > 0.0);
}

// Destructor
Timer::~Timer() {

}

// Return the current time of the system in seconds
long double Timer::getCurrentSystemTime() {

	#if defined(__TARGET_OS__Windows)
		LARGE_INTEGER ticksPerSecond;
		LARGE_INTEGER ticks;
		QueryPerformanceFrequency(&ticksPerSecond);
		QueryPerformanceCounter(&ticks);
		return (long double(ticks.QuadPart) / long double(ticksPerSecond.QuadPart));
	#else
		// Initialize the lastUpdateTime with the current time in seconds
		timeval timeValue;
		gettimeofday(&timeValue, NULL);
		return (timeValue.tv_sec + (timeValue.tv_usec / 1000000.0));
	#endif
}









