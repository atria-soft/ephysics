/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <ephysics/mathematics/Vector3.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class Ray
/**
 * This structure represents a 3D ray represented by two points.
 * The ray goes from point1 to point1 + maxFraction * (point2 - point1).
 * The points are specified in world-space coordinates.
 */
struct Ray {

	public:

		// -------------------- Attributes -------------------- //

		/// First point of the ray (origin)
		Vector3 point1;

		/// Second point of the ray
		Vector3 point2;

		/// Maximum fraction value
		float maxFraction;

		// -------------------- Methods -------------------- //

		/// Constructor with arguments
		Ray(const Vector3& p1, const Vector3& p2, float maxFrac = float(1.0))
		   : point1(p1), point2(p2), maxFraction(maxFrac) {

		}

		/// Copy-constructor
		Ray(const Ray& ray) : point1(ray.point1), point2(ray.point2), maxFraction(ray.maxFraction) {

		}

		/// Destructor
		~Ray() {

		}

		/// Overloaded assignment operator
		Ray& operator=(const Ray& ray) {
			if (&ray != this) {
				point1 = ray.point1;
				point2 = ray.point2;
				maxFraction = ray.maxFraction;
			}
			return *this;
		}
};

}
