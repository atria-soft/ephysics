/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */
#pragma once

// Libraries
#include <etk/math/Vector3D.hpp>
#include <ephysics/mathematics/Ray.hpp>

/// ReactPhysics3D namespace
namespace ephysics {

// Declarations
class CollisionBody;
class ProxyShape;
class CollisionShape;

// Structure RaycastInfo
/**
 * This structure contains the information about a raycast hit.
 */
struct RaycastInfo {

	private:

		// -------------------- Methods -------------------- //

		/// Private copy constructor
		RaycastInfo(const RaycastInfo& raycastInfo);

		/// Private assignment operator
		RaycastInfo& operator=(const RaycastInfo& raycastInfo);

	public:

		// -------------------- Attributes -------------------- //

		/// Hit point in world-space coordinates
		vec3 worldPoint;

		/// Surface normal at hit point in world-space coordinates
		vec3 worldNormal;

		/// Fraction distance of the hit point between point1 and point2 of the ray
		/// The hit point "p" is such that p = point1 + hitFraction * (point2 - point1)
		float hitFraction;

		/// Mesh subpart index that has been hit (only used for triangles mesh and -1 otherwise)
		int32_t meshSubpart;

		/// Hit triangle index (only used for triangles mesh and -1 otherwise)
		int32_t triangleIndex;

		/// Pointer to the hit collision body
		CollisionBody* body;

		/// Pointer to the hit proxy collision shape
		ProxyShape* proxyShape;

		// -------------------- Methods -------------------- //

		/// Constructor
		RaycastInfo() : meshSubpart(-1), triangleIndex(-1), body(NULL), proxyShape(NULL) {

		}

		/// Destructor
		virtual ~RaycastInfo() {

		}
};

// Class RaycastCallback
/**
 * This class can be used to register a callback for ray casting queries.
 * You should implement your own class inherited from this one and implement
 * the notifyRaycastHit() method. This method will be called for each ProxyShape
 * that is hit by the ray.
 */
class RaycastCallback {

	public:

		// -------------------- Methods -------------------- //

		/// Destructor
		virtual ~RaycastCallback() {

		}

		/// This method will be called for each ProxyShape that is hit by the
		/// ray. You cannot make any assumptions about the order of the
		/// calls. You should use the return value to control the continuation
		/// of the ray. The returned value is the next maxFraction value to use.
		/// If you return a fraction of 0.0, it means that the raycast should
		/// terminate. If you return a fraction of 1.0, it indicates that the
		/// ray is not clipped and the ray cast should continue as if no hit
		/// occurred. If you return the fraction in the parameter (hitFraction
		/// value in the RaycastInfo object), the current ray will be clipped
		/// to this fraction in the next queries. If you return -1.0, it will
		/// ignore this ProxyShape and continue the ray cast.
		/**
		 * @param raycastInfo Information about the raycast hit
		 * @return Value that controls the continuation of the ray after a hit
		 */
		virtual float notifyRaycastHit(const RaycastInfo& raycastInfo)=0;

};

/// Structure RaycastTest
struct RaycastTest {

	public:

		/// User callback class
		RaycastCallback* userCallback;

		/// Constructor
		RaycastTest(RaycastCallback* callback) {
			userCallback = callback;
		}

		/// Ray cast test against a proxy shape
		float raycastAgainstShape(ProxyShape* shape, const Ray& ray);
};

}

