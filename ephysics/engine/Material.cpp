/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

// Libraries
#include <ephysics/engine/Material.h>

using namespace reactphysics3d;

// Constructor
Material::Material()
		 : m_frictionCoefficient(DEFAULT_FRICTION_COEFFICIENT),
		   m_rollingResistance(DEFAULT_ROLLING_RESISTANCE),
		   m_bounciness(DEFAULT_BOUNCINESS) {

}

// Copy-constructor
Material::Material(const Material& material)
		 : m_frictionCoefficient(material.m_frictionCoefficient),
		   m_rollingResistance(material.m_rollingResistance), m_bounciness(material.m_bounciness) {

}

// Destructor
Material::~Material() {

}
