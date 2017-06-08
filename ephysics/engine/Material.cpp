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
		 : mFrictionCoefficient(DEFAULT_FRICTION_COEFFICIENT),
		   mRollingResistance(DEFAULT_ROLLING_RESISTANCE),
		   mBounciness(DEFAULT_BOUNCINESS) {

}

// Copy-constructor
Material::Material(const Material& material)
		 : mFrictionCoefficient(material.mFrictionCoefficient),
		   mRollingResistance(material.mRollingResistance), mBounciness(material.mBounciness) {

}

// Destructor
Material::~Material() {

}
