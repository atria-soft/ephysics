/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#pragma once

#include <ephysics/mathematics/mathematics.hpp>

namespace ephysics {
	/**
	 * @brief Represents an impulse that we can apply to bodies in the contact or constraint solver.
	 */
	class Impulse {
		private:
			/// Private assignment operator
			Impulse& operator=(const Impulse& _impulse);
		public:
			const vec3 linearImpulseBody1; //!< Linear impulse applied to the first body
			const vec3 angularImpulseBody1; //!< Angular impulse applied to the first body
			const vec3 linearImpulseBody2; //!< Linear impulse applied to the second body
			const vec3 angularImpulseBody2; //!< Angular impulse applied to the second body
			/// Constructor
			Impulse(const vec3& _initLinearImpulseBody1,
			        const vec3& _initAngularImpulseBody1,
			        const vec3& _initLinearImpulseBody2,
			        const vec3& _initAngularImpulseBody2):
			  linearImpulseBody1(_initLinearImpulseBody1),
			  angularImpulseBody1(_initAngularImpulseBody1),
			  linearImpulseBody2(_initLinearImpulseBody2),
			  angularImpulseBody2(_initAngularImpulseBody2) {
				
			}
			/// Copy-constructor
			Impulse(const Impulse& _impulse):
			  linearImpulseBody1(_impulse.linearImpulseBody1),
			  angularImpulseBody1(_impulse.angularImpulseBody1),
			  linearImpulseBody2(_impulse.linearImpulseBody2),
			  angularImpulseBody2(_impulse.angularImpulseBody2) {
				
			}
	};
}
