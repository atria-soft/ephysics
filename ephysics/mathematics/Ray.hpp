/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

#include <etk/math/Vector3D.hpp>

namespace ephysics {
	/**
	 * This structure represents a 3D ray represented by two points.
	 * The ray goes from point1 to point1 + maxFraction * (point2 - point1).
	 * The points are specified in world-space coordinates.
	 */
	struct Ray {
		public:
			vec3 point1; //!<First point of the ray (origin)
			vec3 point2; //!< Second point of the ray
			float maxFraction; //!< Maximum fraction value
			/// Constructor with arguments
			Ray(const vec3& _p1, const vec3& _p2, float _maxFrac = 1.0f):
			  point1(_p1),
			  point2(_p2),
			  maxFraction(_maxFrac) {
				
			}
			/// Copy-constructor
			Ray(const Ray& _ray):
			  point1(_ray.point1),
			  point2(_ray.point2),
			  maxFraction(_ray.maxFraction) {
				
			}
			/// Overloaded assignment operator
			Ray& operator=(const Ray& _ray) {
				if (&_ray != this) {
					point1 = _ray.point1;
					point2 = _ray.point2;
					maxFraction = _ray.maxFraction;
				}
				return *this;
			}
	};
}
