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



// Return the timestep of the physics engine
double Timer::getTimeStep() const {
	return m_timeStep;
}

// Set the timestep of the physics engine
void Timer::setTimeStep(double timeStep) {
	assert(timeStep > 0.0f);
	m_timeStep = timeStep;
}

// Return the current time
long double Timer::getPhysicsTime() const {
	return m_lastUpdateTime;
}

// Return if the timer is running
bool Timer::getIsRunning() const {
	return m_isRunning;
}

// Start the timer
void Timer::start() {
	if (!m_isRunning) {
		// Get the current system time
		m_lastUpdateTime = getCurrentSystemTime();
		m_accumulator = 0.0;
		m_isRunning = true;
	}
}

// Stop the timer
void Timer::stop() {
	m_isRunning = false;
}

// True if it's possible to take a new step
bool Timer::isPossibleToTakeStep() const {
	return (m_accumulator >= m_timeStep);
}

// Take a new step => update the timer by adding the timeStep value to the current time
void Timer::nextStep() {
	assert(m_isRunning);

	// Update the accumulator value
	m_accumulator -= m_timeStep;
}

// Compute the int32_terpolation factor
float Timer::computeInterpolationFactor() {
	return (float(m_accumulator / m_timeStep));
}

// Compute the time since the last update() call and add it to the accumulator
void Timer::update() {
	long double currentTime = getCurrentSystemTime();
	m_deltaTime = currentTime - m_lastUpdateTime;
	m_lastUpdateTime = currentTime;
	m_accumulator += m_deltaTime;
}







