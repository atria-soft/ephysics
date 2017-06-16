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

#ifndef CONVEX_MESH_H
#define CONVEX_MESH_H

// Libraries
#include <ephysics/openglframework.hpp>
#include <ephysics/ephysics.hpp>
#include <ephysics/PhysicsObject.hpp>

// Class ConvexMesh
class ConvexMesh : public openglframework::Mesh, public PhysicsObject {

	private :

		// -------------------- Attributes -------------------- //

		/// Previous transform (for int32_terpolation)
		ephysics::etk::Transform3D mPreviousTransform;

		ephysics::TriangleVertexArray* mPhysicsTriangleVertexArray;

		/// Collision shape
		ephysics::ConvexMeshShape* mConvexShape;
		ephysics::ProxyShape* m_proxyShape;

		/// Scaling matrix
		openglframework::Matrix4 m_scalingMatrix;

		/// Vertex Buffer Object for the vertices data
		openglframework::VertexBufferObject mVBOVertices;

		/// Vertex Buffer Object for the normals data
		openglframework::VertexBufferObject mVBONormals;

		/// Vertex Buffer Object for the texture coords
		openglframework::VertexBufferObject mVBOTextureCoords;

		/// Vertex Buffer Object for the indices
		openglframework::VertexBufferObject mVBOIndices;

		/// Vertex Array Object for the vertex data
		openglframework::VertexArrayObject mVAO;

		// -------------------- Methods -------------------- //

		// Create the Vertex Buffer Objects used to render with OpenGL.
		void createVBOAndVAO();

	public :

		// -------------------- Methods -------------------- //

		/// Constructor
		ConvexMesh(const openglframework::vec3& position,
				   ephysics::CollisionWorld* world, const std::string& meshPath);

		/// Constructor
		ConvexMesh(const openglframework::vec3& position, float mass,
				   ephysics::DynamicsWorld* dynamicsWorld, const std::string& meshPath);

		/// Destructor
		~ConvexMesh();

		/// Render the mesh at the correct position and with the correct orientation
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
inline void ConvexMesh::updateetk::Transform3D(float int32_terpolationFactor) {
	m_transformMatrix = computeetk::Transform3D(int32_terpolationFactor, m_scalingMatrix);
}

#endif
