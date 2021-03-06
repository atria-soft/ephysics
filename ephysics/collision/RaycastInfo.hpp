/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#pragma once

#include <etk/math/Vector3D.hpp>
#include <ephysics/mathematics/Ray.hpp>

namespace ephysics {
	class CollisionBody;
	class ProxyShape;
	class CollisionShape;
	/**
	 * @brief It contains the information about a raycast hit.
	 */
	struct RaycastInfo {
		private:
			/// Private copy constructor
			RaycastInfo(const RaycastInfo&) = delete;
			/// Private assignment operator
			RaycastInfo& operator=(const RaycastInfo&) = delete;
		public:
			vec3 worldPoint; //!< Hit point in world-space coordinates
			vec3 worldNormal; //!< Surface normal at hit point in world-space coordinates
			float hitFraction; //!< Fraction distance of the hit point between point1 and point2 of the ray. The hit point "p" is such that p = point1 + hitFraction * (point2 - point1)
			int32_t meshSubpart; //!< Mesh subpart index that has been hit (only used for triangles mesh and -1 otherwise)
			int32_t triangleIndex; //!< Hit triangle index (only used for triangles mesh and -1 otherwise)
			CollisionBody* body; //!< Pointer to the hit collision body
			ProxyShape* proxyShape; //!< Pointer to the hit proxy collision shape
			/// Constructor
			RaycastInfo() :
			  meshSubpart(-1),
			  triangleIndex(-1),
			  body(null),
			  proxyShape(null) {
				
			}
	
			/// Destructor
			virtual ~RaycastInfo() = default;
	};
	
	/**
	 * @brief It can be used to register a callback for ray casting queries.
	 * You should implement your own class inherited from this one and implement
	 * the notifyRaycastHit() method. This method will be called for each ProxyShape
	 * that is hit by the ray.
	 */
	class RaycastCallback {
		public:
			/// Destructor
			virtual ~RaycastCallback() = default;
			/**
			 * @brief This method will be called for each ProxyShape that is hit by the
			 * ray. You cannot make any assumptions about the order of the
			 * calls. You should use the return value to control the continuation
			 * of the ray. The returned value is the next maxFraction value to use.
			 * If you return a fraction of 0.0, it means that the raycast should
			 * terminate. If you return a fraction of 1.0, it indicates that the
			 * ray is not clipped and the ray cast should continue as if no hit
			 * occurred. If you return the fraction in the parameter (hitFraction
			 * value in the RaycastInfo object), the current ray will be clipped
			 * to this fraction in the next queries. If you return -1.0, it will
			 * ignore this ProxyShape and continue the ray cast.
			 * @param[in] _raycastInfo Information about the raycast hit
			 * @return Value that controls the continuation of the ray after a hit
			 */
			virtual float notifyRaycastHit(const RaycastInfo& _raycastInfo)=0;
	};
	
	struct RaycastTest {
		public:
			RaycastCallback* userCallback; //!< User callback class
			/// Constructor
			RaycastTest(RaycastCallback* _callback) {
				userCallback = _callback;
			}
			/// Ray cast test against a proxy shape
			float raycastAgainstShape(ProxyShape* _shape, const Ray& _ray);
	};

}

