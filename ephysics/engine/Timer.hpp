/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/configuration.hpp>
#include <eechrono/echrono.hpp>


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

}
