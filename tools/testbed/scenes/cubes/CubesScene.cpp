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

// Libraries
#include <ephysics/CubesScene.hpp>

// Namespaces
using namespace openglframework;
using namespace cubesscene;

// Constructor
CubesScene::CubesScene(const std::string& name)
	  : SceneDemo(name, SCENE_RADIUS) {

	// Compute the radius and the center of the scene
	openglframework::vec3 center(0, 5, 0);

	// Set the center of the scene
	setScenePosition(center, SCENE_RADIUS);

	// Gravity vector in the dynamics world
	ephysics::vec3 gravity(0, ephysics::float(-9.81), 0);

	// Create the dynamics world for the physics simulation
	m_dynamicsWorld = new ephysics::DynamicsWorld(gravity);

	// Set the number of iterations of the constraint solver
	m_dynamicsWorld->setNbIterationsVelocitySolver(15);

	float radius = 2.0f;

	// Create all the cubes of the scene
	for (int32_t i=0; i<NB_CUBES; i++) {

		// Position of the cubes
		float angle = i * 30.0f;
		openglframework::vec3 position(radius * cos(angle),
										  30 + i * (BOX_SIZE.y() + 0.3f),
										  0);

		// Create a cube and a corresponding rigid in the dynamics world
		Box* cube = new Box(BOX_SIZE, position , BOX_MASS, m_dynamicsWorld);

		// Set the box color
		cube->setColor(mDemoColors[i % mNbDemoColors]);
		cube->setSleepingColor(mRedColorDemo);

		// Change the material properties of the rigid body
		ephysics::Material& material = cube->getRigidBody()->getMaterial();
		material.setBounciness(ephysics::float(0.4));

		// Add the box the list of box in the scene
		mBoxes.push_back(cube);
	}

	// Create the floor
	openglframework::vec3 floorPosition(0, 0, 0);
	mFloor = new Box(FLOOR_SIZE, floorPosition, FLOOR_MASS, m_dynamicsWorld);
	mFloor->setColor(mGreyColorDemo);
	mFloor->setSleepingColor(mGreyColorDemo);

	// The floor must be a static rigid body
	mFloor->getRigidBody()->setType(ephysics::STATIC);

	// Change the material properties of the floor rigid body
	ephysics::Material& material = mFloor->getRigidBody()->getMaterial();
	material.setBounciness(ephysics::float(0.3));

	// Get the physics engine parameters
	mEngineSettings.isGravityEnabled = m_dynamicsWorld->isGravityEnabled();
	ephysics::vec3 gravityVector = m_dynamicsWorld->getGravity();
	mEngineSettings.gravity = openglframework::vec3(gravityVector.x(), gravityVector.y(), gravityVector.z());
	mEngineSettings.isSleepingEnabled = m_dynamicsWorld->isSleepingEnabled();
	mEngineSettings.sleepLinearVelocity = m_dynamicsWorld->getSleepLinearVelocity();
	mEngineSettings.sleepAngularVelocity = m_dynamicsWorld->getSleepAngularVelocity();
	mEngineSettings.nbPositionSolverIterations = m_dynamicsWorld->getNbIterationsPositionSolver();
	mEngineSettings.nbVelocitySolverIterations = m_dynamicsWorld->getNbIterationsVelocitySolver();
	mEngineSettings.timeBeforeSleep = m_dynamicsWorld->getTimeBeforeSleep();
}

// Destructor
CubesScene::~CubesScene() {

	// Destroy all the cubes of the scene
	for (std::vector<Box*>::iterator it = mBoxes.begin(); it != mBoxes.end(); ++it) {

		// Destroy the corresponding rigid body from the dynamics world
		m_dynamicsWorld->destroyRigidBody((*it)->getRigidBody());

		// Destroy the cube
		delete (*it);
	}

	// Destroy the rigid body of the floor
	m_dynamicsWorld->destroyRigidBody(mFloor->getRigidBody());

	// Destroy the floor
	delete mFloor;

	// Destroy the dynamics world
	delete m_dynamicsWorld;
}

// Update the physics world (take a simulation step)
void CubesScene::updatePhysics() {

	// Update the physics engine parameters
	m_dynamicsWorld->setIsGratityEnabled(mEngineSettings.isGravityEnabled);
	ephysics::vec3 gravity(mEngineSettings.gravity.x(), mEngineSettings.gravity.y(),
									 mEngineSettings.gravity.z());
	m_dynamicsWorld->setGravity(gravity);
	m_dynamicsWorld->enableSleeping(mEngineSettings.isSleepingEnabled);
	m_dynamicsWorld->setSleepLinearVelocity(mEngineSettings.sleepLinearVelocity);
	m_dynamicsWorld->setSleepAngularVelocity(mEngineSettings.sleepAngularVelocity);
	m_dynamicsWorld->setNbIterationsPositionSolver(mEngineSettings.nbPositionSolverIterations);
	m_dynamicsWorld->setNbIterationsVelocitySolver(mEngineSettings.nbVelocitySolverIterations);
	m_dynamicsWorld->setTimeBeforeSleep(mEngineSettings.timeBeforeSleep);

	// Take a simulation step
	m_dynamicsWorld->update(mEngineSettings.timeStep);
}

// Update the scene
void CubesScene::update() {

	SceneDemo::update();

	// Update the position and orientation of the boxes
	for (std::vector<Box*>::iterator it = mBoxes.begin(); it != mBoxes.end(); ++it) {

		// Update the transform used for the rendering
		(*it)->updateetk::Transform3D(mInterpolationFactor);
	}

	mFloor->updateetk::Transform3D(mInterpolationFactor);
}

// Render the scene in a single pass
void CubesScene::renderSinglePass(Shader& shader, const openglframework::Matrix4& worldToCameraMatrix) {

	// Bind the shader
	shader.bind();

	// Render all the cubes of the scene
	for (std::vector<Box*>::iterator it = mBoxes.begin(); it != mBoxes.end(); ++it) {
		(*it)->render(shader, worldToCameraMatrix);
	}

	// Render the floor
	mFloor->render(shader, worldToCameraMatrix);

	// Unbind the shader
	shader.unbind();
}

// Reset the scene
void CubesScene::reset() {

	float radius = 2.0f;

	for (int32_t i=0; i<NB_CUBES; i++) {

		// Position of the cubes
		float angle = i * 30.0f;
		openglframework::vec3 position(radius * cos(angle),
										  10 + i * (BOX_SIZE.y() + 0.3f),
										  0);

		// Initial position and orientation of the rigid body
		ephysics::vec3 initPosition(position.x(), position.y(), position.z());
		ephysics::etk::Quaternion initOrientation = ephysics::Quaternion::identity();
		ephysics::etk::Transform3D transform(initPosition, initOrientation);

		mBoxes[i]->resetTransform(transform);
	}
}
