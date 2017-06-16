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

#ifndef CAPSULE_H
#define CAPSULE_H

// Libraries
#include <ephysics/openglframework.hpp>
#include <ephysics/ephysics.hpp>
#include <ephysics/PhysicsObject.hpp>

// Class Sphere
class Capsule : public openglframework::Mesh, public PhysicsObject {

	private :

		// -------------------- Attributes -------------------- //

		/// Radius of the capsule
		float mRadius;

		/// Height of the capsule
		float mHeight;

		/// Scaling matrix (applied to a sphere to obtain the correct sphere dimensions)
		openglframework::Matrix4 m_scalingMatrix;

		/// Collision shape
		ephysics::CapsuleShape* mCapsuleShape;
		ephysics::ProxyShape* m_proxyShape;

		/// Previous transform (for int32_terpolation)
		ephysics::etk::Transform3D mPreviousTransform;

		/// Vertex Buffer Object for the vertices data
		static openglframework::VertexBufferObject mVBOVertices;

		/// Vertex Buffer Object for the normals data
		static openglframework::VertexBufferObject mVBONormals;

		/// Vertex Buffer Object for the texture coords
		static openglframework::VertexBufferObject mVBOTextureCoords;

		/// Vertex Buffer Object for the indices
		static openglframework::VertexBufferObject mVBOIndices;

		/// Vertex Array Object for the vertex data
		static openglframework::VertexArrayObject mVAO;

		// Total number of capsules created
		static int32_t totalNbCapsules;

		// -------------------- Methods -------------------- //

		// Create the Vertex Buffer Objects used to render with OpenGL.
		void createVBOAndVAO();

	public :

		// -------------------- Methods -------------------- //

		/// Constructor
		Capsule(float radius, float height, const openglframework::vec3& position,
				ephysics::CollisionWorld* world, const std::string& meshFolderPath);

		/// Constructor
		Capsule(float radius, float height, const openglframework::vec3& position,
				float mass, ephysics::DynamicsWorld* dynamicsWorld,
				const std::string& meshFolderPath);

		/// Destructor
		~Capsule();

		/// Render the sphere at the correct position and with the correct orientation
		void render(openglframework::Shader& shader,
				const openglframework::Matrix4& worldToCameraMatrix);

		/// Set the position of the box
		void resetTransform(const ephysics::Transform& transform);

		/// Update the transform matrix of the object
		virtual void updateetk::Transform3D(float int32_terpolationFactor);

		/// Set the scaling of the object
		void setScaling(const openglframework::vec3& scaling);
};

// Update the transform matrix of the object
inline void Capsule::updateetk::Transform3D(float int32_terpolationFactor) {
	m_transformMatrix = computeetk::Transform3D(int32_terpolationFactor, m_scalingMatrix);
}

#endif
