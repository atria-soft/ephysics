/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <stdexcept>
#include <iostream>
#include <ctime>
#include <cassert>
#include <ephysics/configuration.hpp>

#if defined(__TARGET_OS__Windows)
   #define NOMINMAX	   // This is used to avoid definition of max() and min() macros
   #include <windows.h>
#else								   // For Mac OS or Linux platform
   #include <sys/time.h>
#endif


namespace ephysics {
	/**
	 * @brief This class will take care of the time in the physics engine. It
	 * uses functions that depend on the current platform to get the
	 * current time.
	 */
	class Timer {
		private :
			double m_timeStep; //!< Timestep dt of the physics engine (timestep > 0.0)
			long double m_lastUpdateTime; //!< Last time the timer has been updated
			long double m_deltaTime; //!< Time difference between the two last timer update() calls
			double m_accumulator; //!< Used to fix the time step and avoid strange time effects
			bool m_isRunning; //!< True if the timer is running
			/// Private copy-constructor
			Timer(const Timer& timer);
			/// Private assignment operator
			Timer& operator=(const Timer& timer);
		public :
			/// Constructor
			Timer(double timeStep);
			/// Destructor
			virtual ~Timer();
			/// Return the timestep of the physics engine
			double getTimeStep() const;
			/// Set the timestep of the physics engine
			void setTimeStep(double timeStep);
			/// Return the current time of the physics engine
			long double getPhysicsTime() const;
			/// Start the timer
			void start();
			/// Stop the timer
			void stop();
			/// Return true if the timer is running
			bool getIsRunning() const;
			/// True if it's possible to take a new step
			bool isPossibleToTakeStep() const;
			/// Compute the time since the last update() call and add it to the accumulator
			void update();
			/// Take a new step => update the timer by adding the timeStep value to the current time
			void nextStep();
			/// Compute the int32_terpolation factor
			float computeInterpolationFactor();
			/// Return the current time of the system in seconds
			static long double getCurrentSystemTime();
	};

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

}
