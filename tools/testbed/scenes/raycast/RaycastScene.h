/********************************************************************************
* ReactPhysics3D physics library, http://www.ephysics.com				 *
* Copyright (c) 2010-2016 Daniel Chappuis									   *
*********************************************************************************
*																			   *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.														 *
*																			   *
* Permission is granted to anyone to use this software for any purpose,		 *
* including commercial applications, and to alter it and redistribute it		*
* freely, subject to the following restrictions:								*
*																			   *
* 1. The origin of this software must not be misrepresented; you must not claim *
*	that you wrote the original software. If you use this software in a		*
*	product, an acknowledgment in the product documentation would be		   *
*	appreciated but is not required.										   *
*																			   *
* 2. Altered source versions must be plainly marked as such, and must not be	*
*	misrepresented as being the original software.							 *
*																			   *
* 3. This notice may not be removed or altered from any source distribution.	*
*																			   *
********************************************************************************/

#ifndef RAYCAST_SCENE_H
#define RAYCAST_SCENE_H

// Libraries
#define _USE_MATH_DEFINES
#include <cmath>
#include <ephysics/openglframework.hpp>
#include <ephysics/ephysics.hpp>
#include <ephysics/SceneDemo.hpp>
#include <ephysics/Sphere.hpp>
#include <ephysics/Box.hpp>
#include <ephysics/Cone.hpp>
#include <ephysics/Cylinder.hpp>
#include <ephysics/Capsule.hpp>
#include <ephysics/Line.hpp>
#include <ephysics/ConvexMesh.hpp>
#include <ephysics/ConcaveMesh.hpp>
#include <ephysics/HeightField.hpp>
#include <ephysics/Dumbbell.hpp>
#include <ephysics/VisualContactPoint.hpp>

namespace raycastscene {

// Constants
const float SCENE_RADIUS = 30.0f;
const openglframework::vec3 BOX_SIZE(4, 2, 1);
const float SPHERE_RADIUS = 3.0f;
const float CONE_RADIUS = 3.0f;
const float CONE_HEIGHT = 5.0f;
const float CYLINDER_RADIUS = 3.0f;
const float CYLINDER_HEIGHT = 5.0f;
const float CAPSULE_RADIUS = 3.0f;
const float CAPSULE_HEIGHT = 5.0f;
const float DUMBBELL_HEIGHT = 5.0f;
const int32_t NB_RAYS = 100;
const float RAY_LENGTH = 30.0f;
const int32_t NB_BODIES = 9;

// Raycast manager
class RaycastManager : public ephysics::RaycastCallback {

	private:

		/// All the visual contact points
		std::vector<ContactPoint> mHitPoints;

		/// All the normals at hit points
		std::vector<Line*> mNormals;

		/// Contact point mesh folder path
		std::string mMeshFolderPath;

   public:

		RaycastManager(openglframework::Shader& shader,
					   const std::string& meshFolderPath)
			: mMeshFolderPath(meshFolderPath) {

		}

		virtual ephysics::float notifyRaycastHit(const ephysics::RaycastInfo& raycastInfo) {
			ephysics::vec3 hitPos = raycastInfo.worldPoint;
			openglframework::vec3 position(hitPos.x(), hitPos.y(), hitPos.z());
			mHitPoints.push_back(ContactPoint(position));

			// Create a line to display the normal at hit point
			ephysics::vec3 n = raycastInfo.worldNormal;
			openglframework::vec3 normal(n.x(), n.y(), n.z());
			Line* normalLine = new Line(position, position + normal);
			mNormals.push_back(normalLine);

			return raycastInfo.hitFraction;
		}

		void resetPoints() {

			mHitPoints.clear();

			// Destroy all the normals
			for (std::vector<Line*>::iterator it = mNormals.begin();
				 it != mNormals.end(); ++it) {
				delete (*it);
			}
			mNormals.clear();
		}

		std::vector<ContactPoint> getHitPoints() const {
			return mHitPoints;
		}
};

// Class RaycastScene
class RaycastScene : public SceneDemo {

	private :

		// -------------------- Attributes -------------------- //

		/// Contact point mesh folder path
		std::string mMeshFolderPath;

		/// Raycast manager
		RaycastManager m_raycastManager;

		/// All the raycast lines
		std::vector<Line*> mLines;

		/// Current body index
		int32_t mCurrentBodyIndex;

		/// True if the hit points normals are displayed
		bool mAreNormalsDisplayed;

		/// Raycast manager

		/// All objects on the scene
		Box* mBox;
		Sphere* mSphere;
		Cone* mCone;
		Cylinder* mCylinder;
		Capsule* mCapsule;
		ConvexMesh* mConvexMesh;
		Dumbbell* mDumbbell;
		ConcaveMesh* mConcaveMesh;
		HeightField* mHeightField;

		/// Collision world used for the physics simulation
		ephysics::CollisionWorld* mCollisionWorld;

		/// All the points to render the lines
		std::vector<openglframework::vec3> mLinePoints;

		/// Vertex Buffer Object for the vertices data
		openglframework::VertexBufferObject mVBOVertices;

		/// Vertex Array Object for the vertex data
		openglframework::VertexArrayObject mVAO;

		/// Create the raycast lines
		void createLines();

		// Create the Vertex Buffer Objects used to render with OpenGL.
		void createVBOAndVAO(openglframework::Shader& shader);


	public:

		// -------------------- Methods -------------------- //

		/// Constructor
		RaycastScene(const std::string& name);

		/// Destructor
		virtual ~RaycastScene();

		/// Update the physics world (take a simulation step)
		/// Can be called several times per frame
		virtual void updatePhysics();

		/// Take a step for the simulation
		virtual void update();

		/// Render the scene in a single pass
		virtual void renderSinglePass(openglframework::Shader& shader,
									  const openglframework::Matrix4& worldToCameraMatrix);

		/// Reset the scene
		virtual void reset();

		/// Change the body to raycast
		void changeBody();

		/// Display or not the surface normals at hit points
		void showHideNormals();

		/// Called when a keyboard event occurs
		virtual bool keyboardEvent(int32_t key, int32_t scancode, int32_t action, int32_t mods);

		/// Enabled/Disable the shadow mapping
		void virtual setIsShadowMappingEnabled(bool isShadowMappingEnabled);

		/// Display/Hide the contact points
		void virtual setIsContactPointsDisplayed(bool display);

		/// Return all the contact points of the scene
		virtual std::vector<ContactPoint> getContactPoints() const;
};

// Display or not the surface normals at hit points
inline void RaycastScene::showHideNormals() {
	mAreNormalsDisplayed = !mAreNormalsDisplayed;
}

// Enabled/Disable the shadow mapping
inline void RaycastScene::setIsShadowMappingEnabled(bool isShadowMappingEnabled) {
	SceneDemo::setIsShadowMappingEnabled(false);
}

// Display/Hide the contact points
inline void RaycastScene::setIsContactPointsDisplayed(bool display) {
	SceneDemo::setIsContactPointsDisplayed(true);
}

// Return all the contact points of the scene
inline std::vector<ContactPoint> RaycastScene::getContactPoints() const {
	return m_raycastManager.getHitPoints();
}

}

#endif
