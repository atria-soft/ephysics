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
#include <ephysics/Line.hpp>

// Constructor
Line::Line(const openglframework::vec3& worldPoint1,
		   const openglframework::vec3& worldPoint2)
	 : m_worldPoint1(worldPoint1), m_worldPoint2(worldPoint2) {

}

// Destructor
Line::~Line() {


}

// Render the sphere at the correct position and with the correct orientation
void Line::render(openglframework::Shader& shader,
					const openglframework::Matrix4& worldToCameraMatrix) {

	// Bind the shader
	shader.bind();

	// Set the model to camera matrix
	shader.setMatrix4x4Uniform("localToWorldMatrix", openglframework::Matrix4::identity());
	shader.setMatrix4x4Uniform("worldToCameraMatrix", worldToCameraMatrix);

	// Set the vertex color
	openglframework::Vector4 color(1, 0, 0, 1);
	shader.setVector4Uniform("vertexColor", color, false);

	/*
	glBegin(GL_LINES);
		glVertex3f(m_worldPoint1.x(), m_worldPoint1.y(), m_worldPoint1.z());
		glVertex3f(m_worldPoint2.x(), m_worldPoint2.y(), m_worldPoint2.z());
	glEnd();
	*/

	// Unbind the shader
	shader.unbind();
}
