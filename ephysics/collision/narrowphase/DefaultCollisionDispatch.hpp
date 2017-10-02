/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#pragma once

#include <ephysics/collision/narrowphase/CollisionDispatch.hpp>
#include <ephysics/collision/narrowphase/ConcaveVsConvexAlgorithm.hpp>
#include <ephysics/collision/narrowphase/SphereVsSphereAlgorithm.hpp>
#include <ephysics/collision/narrowphase/GJK/GJKAlgorithm.hpp>

namespace ephysics {
	/**
	 * @brief This is the default collision dispatch configuration use in ephysics.
	 * Collision dispatching decides which collision
	 * algorithm to use given two types of proxy collision shapes.
	 */
	class DefaultCollisionDispatch : public CollisionDispatch {
		protected:
			SphereVsSphereAlgorithm m_sphereVsSphereAlgorithm; //!< Sphere vs Sphere collision algorithm
			ConcaveVsConvexAlgorithm m_concaveVsConvexAlgorithm; //!< Concave vs Convex collision algorithm
			GJKAlgorithm m_GJKAlgorithm; //!< GJK Algorithm
		public:
			/**
			 * @brief Constructor
			 */
			DefaultCollisionDispatch();
			void init(CollisionDetection* _collisionDetection) override;
			NarrowPhaseAlgorithm* selectAlgorithm(int32_t _type1, int32_t _type2) override;
	};
}


