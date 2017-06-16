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
#include <ephysics/Cone.hpp>

openglframework::VertexBufferObject Cone::mVBOVertices(GL_ARRAY_BUFFER);
openglframework::VertexBufferObject Cone::mVBONormals(GL_ARRAY_BUFFER);
openglframework::VertexBufferObject Cone::mVBOTextureCoords(GL_ARRAY_BUFFER);
openglframework::VertexBufferObject Cone::mVBOIndices(GL_ELEMENT_ARRAY_BUFFER);
openglframework::VertexArrayObject Cone::mVAO;
int32_t Cone::totalNbCones = 0;

// Constructor
Cone::Cone(float radius, float height, const openglframework::vec3 &position,
		   ephysics::CollisionWorld* world,
		   const std::string& meshFolderPath)
	 : openglframework::Mesh(), mRadius(radius), mHeight(height) {

	// Load the mesh from a file
	openglframework::MeshReaderWriter::loadMeshFromFile(meshFolderPath + "cone.obj", *this);

	// Calculate the normals of the mesh
	calculateNormals();

	// Compute the scaling matrix
	m_scalingMatrix = openglframework::Matrix4(mRadius, 0, 0, 0,
											  0, mHeight, 0, 0,
											  0, 0, mRadius, 0,
											  0, 0, 0, 1);

	// Initialize the position where the cone will be rendered
	translateWorld(position);

	// Create the collision shape for the rigid body (cone shape) and do
	// not forget to delete it at the end
	mConeShape = new ephysics::ConeShape(mRadius, mHeight);

	// Initial position and orientation of the rigid body
	ephysics::vec3 initPosition(position.x(), position.y(), position.z());
	ephysics::etk::Quaternion initOrientation = ephysics::Quaternion::identity();
	ephysics::etk::Transform3D transform(initPosition, initOrientation);

	mPreviousetk::Transform3D = transform;

	// Create a rigid body corresponding to the cone in the dynamics world
	m_body = world->createCollisionBody(transform);

	// Add a collision shape to the body and specify the mass of the shape
	m_proxyShape = m_body->addCollisionShape(mConeShape, ephysics::etk::Transform3D::identity());

	m_transformMatrix = m_transformMatrix * m_scalingMatrix;

	// Create the VBOs and VAO
	if (totalNbCones == 0) {
		createVBOAndVAO();
	}

	totalNbCones++;
}

// Constructor
Cone::Cone(float radius, float height, const openglframework::vec3 &position,
		   float mass, ephysics::DynamicsWorld* dynamicsWorld,
		   const std::string& meshFolderPath)
	 : openglframework::Mesh(), mRadius(radius), mHeight(height) {

	// Load the mesh from a file
	openglframework::MeshReaderWriter::loadMeshFromFile(meshFolderPath + "cone.obj", *this);

	// Calculate the normals of the mesh
	calculateNormals();

	// Compute the scaling matrix
	m_scalingMatrix = openglframework::Matrix4(mRadius, 0, 0, 0,
											  0, mHeight, 0, 0,
											  0, 0, mRadius, 0,
											  0, 0, 0, 1);

	// Initialize the position where the cone will be rendered
	translateWorld(position);

	// Create the collision shape for the rigid body (cone shape) and do not
	// forget to delete it at the end
	mConeShape = new ephysics::ConeShape(mRadius, mHeight);

	// Initial position and orientation of the rigid body
	ephysics::vec3 initPosition(position.x(), position.y(), position.z());
	ephysics::etk::Quaternion initOrientation = ephysics::Quaternion::identity();
	ephysics::etk::Transform3D transform(initPosition, initOrientation);

	// Create a rigid body corresponding to the cone in the dynamics world
	ephysics::RigidBody* body = dynamicsWorld->createRigidBody(transform);

	// Add a collision shape to the body and specify the mass of the shape
	m_proxyShape = body->addCollisionShape(mConeShape, ephysics::etk::Transform3D::identity(), mass);

	m_body = body;

	m_transformMatrix = m_transformMatrix * m_scalingMatrix;

	// Create the VBOs and VAO
	if (totalNbCones == 0) {
		createVBOAndVAO();
	}

	totalNbCones++;
}

// Destructor
Cone::~Cone() {

	if (totalNbCones == 1) {
		// Destroy the mesh
		destroy();

		// Destroy the VBOs and VAO
		mVBOIndices.destroy();
		mVBOVertices.destroy();
		mVBONormals.destroy();
		mVBOTextureCoords.destroy();
		mVAO.destroy();
	}
	delete mConeShape;
	totalNbCones--;
}

// Render the cone at the correct position and with the correct orientation
void Cone::render(openglframework::Shader& shader,
				  const openglframework::Matrix4& worldToCameraMatrix) {

	// Bind the shader
	shader.bind();

	// Set the model to camera matrix
	shader.setMatrix4x4Uniform("localToWorldMatrix", m_transformMatrix);
	shader.setMatrix4x4Uniform("worldToCameraMatrix", worldToCameraMatrix);

	// Set the normal matrix (inverse transpose of the 3x3 upper-left sub matrix of the
	// model-view matrix)
	const openglframework::Matrix4 localToCameraMatrix = worldToCameraMatrix * m_transformMatrix;
	const openglframework::Matrix3 normalMatrix =
					   localToCameraMatrix.getUpperLeft3x3Matrix().getInverse().getTranspose();
	shader.setetk::Matrix3x3Uniform("normalMatrix", normalMatrix, false);

	// Set the vertex color
	openglframework::Color currentColor = m_body->isSleeping() ? mSleepingColor : mColor;
	openglframework::Vector4 color(currentColor.r, currentColor.g, currentColor.b, currentColor.a);
	shader.setVector4Uniform("vertexColor", color, false);

	// Bind the VAO
	mVAO.bind();

	mVBOVertices.bind();

	// Get the location of shader attribute variables
	GLint32_t vertexPositionLoc = shader.getAttribLocation("vertexPosition");
	GLint32_t vertexNormalLoc = shader.getAttribLocation("vertexNormal", false);

	glEnableVertexAttribArray(vertexPositionLoc);
	glVertexAttribPointer(vertexPositionLoc, 3, GL_FLOAT, GL_FALSE, 0, (char*)NULL);

	mVBONormals.bind();

	if (vertexNormalLoc != -1) glVertexAttribPointer(vertexNormalLoc, 3, GL_FLOAT, GL_FALSE, 0, (char*)NULL);
	if (vertexNormalLoc != -1) glEnableVertexAttribArray(vertexNormalLoc);

	// For each part of the mesh
	for (uint32_t i=0; i<getNbParts(); i++) {
		glDrawElements(GL_TRIANGLES, getNbFaces(i) * 3, GL_UNSIGNED_INT, (char*)NULL);
	}

	glDisableVertexAttribArray(vertexPositionLoc);
	if (vertexNormalLoc != -1) glDisableVertexAttribArray(vertexNormalLoc);

	mVBONormals.unbind();
	mVBOVertices.unbind();

	// Unbind the VAO
	mVAO.unbind();

	// Unbind the shader
	shader.unbind();
}

// Create the Vertex Buffer Objects used to render with OpenGL.
/// We create two VBOs (one for vertices and one for indices)
void Cone::createVBOAndVAO() {

	// Create the VBO for the vertices data
	mVBOVertices.create();
	mVBOVertices.bind();
	size_t sizeVertices = m_vertices.size() * sizeof(openglframework::vec3);
	mVBOVertices.copyDataIntoVBO(sizeVertices, getVerticesPointer(), GL_STATIC_DRAW);
	mVBOVertices.unbind();

	// Create the VBO for the normals data
	mVBONormals.create();
	mVBONormals.bind();
	size_t sizeNormals = mNormals.size() * sizeof(openglframework::vec3);
	mVBONormals.copyDataIntoVBO(sizeNormals, getNormalsPointer(), GL_STATIC_DRAW);
	mVBONormals.unbind();

	if (hasTexture()) {
		// Create the VBO for the texture co data
		mVBOTextureCoords.create();
		mVBOTextureCoords.bind();
		size_t sizeTextureCoords = mUVs.size() * sizeof(openglframework::vec2);
		mVBOTextureCoords.copyDataIntoVBO(sizeTextureCoords, getUVTextureCoordinatesPointer(), GL_STATIC_DRAW);
		mVBOTextureCoords.unbind();
	}

	// Create th VBO for the indices data
	mVBOIndices.create();
	mVBOIndices.bind();
	size_t sizeIndices = mIndices[0].size() * sizeof(uint32_t);
	mVBOIndices.copyDataIntoVBO(sizeIndices, getIndicesPointer(), GL_STATIC_DRAW);
	mVBOIndices.unbind();

	// Create the VAO for both VBOs
	mVAO.create();
	mVAO.bind();

	// Bind the VBO of vertices
	mVBOVertices.bind();

	// Bind the VBO of normals
	mVBONormals.bind();

	if (hasTexture()) {
		// Bind the VBO of texture coords
		mVBOTextureCoords.bind();
	}

	// Bind the VBO of indices
	mVBOIndices.bind();

	// Unbind the VAO
	mVAO.unbind();
}

// Reset the transform
void Cone::resetTransform(const ephysics::Transform& transform) {

	// Reset the transform
	m_body->setTransform(transform);

	m_body->setIsSleeping(false);

	// Reset the velocity of the rigid body
	ephysics::RigidBody* rigidBody = dynamic_cast<ephysics::RigidBody*>(m_body);
	if (rigidBody != NULL) {
		rigidBody->setLinearVelocity(ephysics::vec3(0, 0, 0));
		rigidBody->setAngularVelocity(ephysics::vec3(0, 0, 0));
	}

	updateetk::Transform3D(1.0f);
}

// Set the scaling of the object
void Cone::setScaling(const openglframework::vec3& scaling) {

	// Scale the collision shape
	m_proxyShape->setLocalScaling(ephysics::vec3(scaling.x(), scaling.y(), scaling.z()));

	// Scale the graphics object
	m_scalingMatrix = openglframework::Matrix4(mRadius * scaling.x(), 0, 0, 0,
											  0, mHeight * scaling.y(), 0, 0,
											  0, 0, mRadius * scaling.z(), 0,
											  0, 0, 0, 1);
}

