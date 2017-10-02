/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#pragma once

#include <ephysics/constraint/ContactPoint.hpp>

namespace ephysics {
	/**
	 * @brief This class can be used to receive event callbacks from the physics engine.
	 * In order to receive callbacks, you need to create a new class that inherits from
	 * this one and you must override the methods you need. Then, you need to register your
	 * new event listener class to the physics world using the DynamicsWorld::setEventListener()
	 * method.
	 */
	class EventListener {
		public :
			/**
			 * @brief Generic Constructor
			 */
			EventListener() {}
			/**
			 * @brief Generic Desstructor take it virtual
			 */
			virtual ~EventListener() =default;
			/**
			 * @brief Called when a new contact point is found between two bodies that were separated before
			 * @param contact Information about the contact
			 */
			virtual void beginContact(const ContactPointInfo& contact) {};
			/**
			 * @brief Called when a new contact point is found between two bodies
			 * @param contact Information about the contact
			 */
			virtual void newContact(const ContactPointInfo& contact) {}
			/**
			 * @brief Called at the beginning of an int32_ternal tick of the simulation step.
			 * Each time the DynamicsWorld::update() method is called, the physics
			 * engine will do several int32_ternal simulation steps. This method is
			 * called at the beginning of each int32_ternal simulation step.
			 */
			virtual void beginInternalTick() {}
			/**
			 * @brief Called at the end of an int32_ternal tick of the simulation step.
			 * Each time the DynamicsWorld::update() metho is called, the physics
			 * engine will do several int32_ternal simulation steps. This method is
			 * called at the end of each int32_ternal simulation step.
			 */
			virtual void endInternalTick() {}
	};
}
