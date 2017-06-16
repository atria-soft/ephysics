/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/constraint/ContactPoint.hpp>

namespace ephysics {

// Class EventListener
/**
 * This class can be used to receive event callbacks from the physics engine.
 * In order to receive callbacks, you need to create a new class that inherits from
 * this one and you must override the methods you need. Then, you need to register your
 * new event listener class to the physics world using the DynamicsWorld::setEventListener()
 * method.
 */
class EventListener {

	public :

		/// Constructor
		EventListener() {}

		/// Destructor
		virtual ~EventListener() {}

		/// Called when a new contact point is found between two bodies that were separated before
		/**
		 * @param contact Information about the contact
		 */
		virtual void beginContact(const ContactPointInfo& contact) {};

		/// Called when a new contact point is found between two bodies
		/**
		 * @param contact Information about the contact
		 */
		virtual void newContact(const ContactPointInfo& contact) {}

		/// Called at the beginning of an int32_ternal tick of the simulation step.
		/// Each time the DynamicsWorld::update() method is called, the physics
		/// engine will do several int32_ternal simulation steps. This method is
		/// called at the beginning of each int32_ternal simulation step.
		virtual void beginInternalTick() {}

		/// Called at the end of an int32_ternal tick of the simulation step.
		/// Each time the DynamicsWorld::update() metho is called, the physics
		/// engine will do several int32_ternal simulation steps. This method is
		/// called at the end of each int32_ternal simulation step.
		virtual void endInternalTick() {}
};

}
