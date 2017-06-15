/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/mathematics/mathematics.h>

namespace reactphysics3d {

// Structure Impulse
/**
 * Represents an impulse that we can apply to bodies in the contact or constraint solver.
 */
struct Impulse {

	private:

		// -------------------- Methods -------------------- //

		/// Private assignment operator
		Impulse& operator=(const Impulse& impulse);

	public:

		// -------------------- Attributes -------------------- //

		/// Linear impulse applied to the first body
		const vec3 linearImpulseBody1;

		/// Angular impulse applied to the first body
		const vec3 angularImpulseBody1;

		/// Linear impulse applied to the second body
		const vec3 linearImpulseBody2;

		/// Angular impulse applied to the second body
		const vec3 angularImpulseBody2;

		// -------------------- Methods -------------------- //

		/// Constructor
		Impulse(const vec3& initLinearImpulseBody1, const vec3& initAngularImpulseBody1,
				const vec3& initLinearImpulseBody2, const vec3& initAngularImpulseBody2)
			: linearImpulseBody1(initLinearImpulseBody1),
			  angularImpulseBody1(initAngularImpulseBody1),
			  linearImpulseBody2(initLinearImpulseBody2),
			  angularImpulseBody2(initAngularImpulseBody2) {

		}

		/// Copy-constructor
		Impulse(const Impulse& impulse)
			  : linearImpulseBody1(impulse.linearImpulseBody1),
				angularImpulseBody1(impulse.angularImpulseBody1),
				linearImpulseBody2(impulse.linearImpulseBody2),
				angularImpulseBody2(impulse.angularImpulseBody2) {
;
		}
};

}
