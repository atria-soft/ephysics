/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/engine/Timer.hpp>
#include <echrono/Clock.hpp>


ephysics::Timer::Timer(double timeStep) : m_timeStep(timeStep), m_isRunning(false) {
	assert(timeStep > 0.0);
}

ephysics::Timer::~Timer() {
	
}

long double ephysics::Timer::getCurrentSystemTime() {
	return (long double)(echrono::Clock::now().get()) / 1000000000.0;
}

double ephysics::Timer::getTimeStep() const {
	return m_timeStep;
}

void ephysics::Timer::setTimeStep(double timeStep) {
	assert(timeStep > 0.0f);
	m_timeStep = timeStep;
}

long double ephysics::Timer::getPhysicsTime() const {
	return m_lastUpdateTime;
}

bool ephysics::Timer::getIsRunning() const {
	return m_isRunning;
}

void ephysics::Timer::start() {
	if (!m_isRunning) {
		// Get the current system time
		m_lastUpdateTime = getCurrentSystemTime();
		m_accumulator = 0.0;
		m_isRunning = true;
	}
}

void ephysics::Timer::stop() {
	m_isRunning = false;
}

bool ephysics::Timer::isPossibleToTakeStep() const {
	return (m_accumulator >= m_timeStep);
}

void ephysics::Timer::nextStep() {
	assert(m_isRunning);
	// Update the accumulator value
	m_accumulator -= m_timeStep;
}

float ephysics::Timer::computeInterpolationFactor() {
	return (float(m_accumulator / m_timeStep));
}

void ephysics::Timer::update() {
	long double currentTime = getCurrentSystemTime();
	m_deltaTime = currentTime - m_lastUpdateTime;
	m_lastUpdateTime = currentTime;
	m_accumulator += m_deltaTime;
}

