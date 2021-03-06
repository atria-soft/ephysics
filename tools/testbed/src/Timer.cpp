/********************************************************************************
* ReactPhysics3D physics library, http://www.ephysics.com				 *
* Copyright (c) 2010-2016 Daniel Chappuis									   *
*********************************************************************************
*																			   *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.														 *
*																			   *
* Permission is granted to anyone to use this software for any purpose,		 *
* including commercial applications, and to alter it and redistribute it		*
* freely, subject to the following restrictions:								*
*																			   *
* 1. The origin of this software must not be misrepresented; you must not claim *
*	that you wrote the original software. If you use this software in a		*
*	product, an acknowledgment in the product documentation would be		   *
*	appreciated but is not required.										   *
*																			   *
* 2. Altered source versions must be plainly marked as such, and must not be	*
*	misrepresented as being the original software.							 *
*																			   *
* 3. This notice may not be removed or altered from any source distribution.	*
*																			   *
********************************************************************************/

// Libraries
#include <ephysics/Timer.hpp>


// Constructor
Timer::Timer() : m_isRunning(false) {

}

// Destructor
Timer::~Timer() {

}

// Return the current time of the system in seconds
long double Timer::getCurrentSystemTime() {

	#if defined(WINDOWS_OS)
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









