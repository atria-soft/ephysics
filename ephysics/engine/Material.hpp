/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once
#include <cassert>
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

// Return the bounciness
/**
 * @return Bounciness factor (between 0 and 1) where 1 is very bouncy
 */
inline float Material::getBounciness() const {
	return m_bounciness;
}

// Set the bounciness.
/// The bounciness should be a value between 0 and 1. The value 1 is used for a
/// very bouncy body and zero is used for a body that is not bouncy at all.
/**
 * @param bounciness Bounciness factor (between 0 and 1) where 1 is very bouncy
 */
inline void Material::setBounciness(float bounciness) {
	assert(bounciness >= 0.0f && bounciness <= 1.0f);
	m_bounciness = bounciness;
}

// Return the friction coefficient
/**
 * @return Friction coefficient (positive value)
 */
inline float Material::getFrictionCoefficient() const {
	return m_frictionCoefficient;
}

// Set the friction coefficient.
/// The friction coefficient has to be a positive value. The value zero is used for no
/// friction at all.
/**
 * @param frictionCoefficient Friction coefficient (positive value)
 */
inline void Material::setFrictionCoefficient(float frictionCoefficient) {
	assert(frictionCoefficient >= 0.0f);
	m_frictionCoefficient = frictionCoefficient;
}

// Return the rolling resistance factor. If this value is larger than zero,
// it will be used to slow down the body when it is rolling
// against another body.
/**
 * @return The rolling resistance factor (positive value)
 */
inline float Material::getRollingResistance() const {
	return m_rollingResistance;
}

// Set the rolling resistance factor. If this value is larger than zero,
// it will be used to slow down the body when it is rolling
// against another body.
/**
 * @param rollingResistance The rolling resistance factor
 */
inline void Material::setRollingResistance(float rollingResistance) {
	assert(rollingResistance >= 0);
	m_rollingResistance = rollingResistance;
}

// Overloaded assignment operator
inline Material& Material::operator=(const Material& material) {

	// Check for self-assignment
	if (this != &material) {
		m_frictionCoefficient = material.m_frictionCoefficient;
		m_bounciness = material.m_bounciness;
		m_rollingResistance = material.m_rollingResistance;
	}

	// Return this material
	return *this;
}

}
