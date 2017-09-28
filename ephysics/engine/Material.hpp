/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once
extern "C" {
	#include <assert.h>
}
#include <ephysics/configuration.hpp>

namespace ephysics {
	/**
	 * This class contains the material properties of a rigid body that will be use for
	 * the dynamics simulation like the friction coefficient or the bounciness of the rigid
	 * body.
	 */
	class Material {
		private :
			float m_frictionCoefficient; //!< Friction coefficient (positive value)
			float m_rollingResistance; //!< Rolling resistance factor (positive value)
			float m_bounciness; //!< Bounciness during collisions (between 0 and 1) where 1 is for a very bouncy body
		public :
			/// Constructor
			Material();
			/// Copy-constructor
			Material(const Material& material);
			/// Return the bounciness
			float getBounciness() const;
			/// Set the bounciness.
			void setBounciness(float bounciness);
			/// Return the friction coefficient
			float getFrictionCoefficient() const;
			/// Set the friction coefficient.
			void setFrictionCoefficient(float frictionCoefficient);
			/// Return the rolling resistance factor
			float getRollingResistance() const;
			/// Set the rolling resistance factor
			void setRollingResistance(float rollingResistance);
			/// Overloaded assignment operator
			Material& operator=(const Material& material);
	};

}
