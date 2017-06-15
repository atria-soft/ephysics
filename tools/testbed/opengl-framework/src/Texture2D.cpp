/********************************************************************************
* OpenGL-Framework															  *
* Copyright (c) 2013 Daniel Chappuis											*
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
#include <ephysics/Texture2D.h>
#include <fstream>
#include <iostream>
#include <string>

// Namespaces
using namespace openglframework;

// Constructor
Texture2D::Texture2D() : m_id(0), mUnit(0), m_width(0), mHeight(0) {

}

// Constructor
Texture2D::Texture2D(uint32_t width, uint32_t height, uint32_t int32_ternalFormat, uint32_t format, uint32_t type)
		  : m_id(0), mUnit(0), m_width(0), mHeight(0) {

	// Create the texture
	create(width, height, int32_ternalFormat, format, type);
}

// Destructor
Texture2D::~Texture2D() {
	destroy();
}

// Create the texture
void Texture2D::create(uint32_t width, uint32_t height, uint32_t int32_ternalFormat, uint32_t format, uint32_t type,
					   void* data) {

	// Destroy the current texture
	destroy();

	m_width = width;
	mHeight = height;

	// Create the OpenGL texture
	glGenTextures(1, &m_id);
	assert(m_id != 0);
	glBindTexture(GL_TEXTURE_2D, m_id);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, int32_ternalFormat, m_width, mHeight, 0, format, type, data);
	glBindTexture(GL_TEXTURE_2D, 0);
}

// Create the texture
void Texture2D::create(uint32_t width, uint32_t height, uint32_t int32_ternalFormat, uint32_t format, uint32_t type,
					   uint32_t minFilter, uint32_t maxFilter, uint32_t wrapS, uint32_t wrapT, void* data) {

	// Destroy the current texture
	destroy();

	m_width = width;
	mHeight = height;

	// Create the OpenGL texture
	glGenTextures(1, &m_id);
	assert(m_id != 0);
	glBindTexture(GL_TEXTURE_2D, m_id);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapS);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, maxFilter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
	glTexImage2D(GL_TEXTURE_2D, 0, int32_ternalFormat, m_width, mHeight, 0, format, type, data);
	glBindTexture(GL_TEXTURE_2D, 0);
}

// Destroy the texture
void Texture2D::destroy() {
	if (m_id != 0) {
		glDeleteTextures(1, &m_id);
		m_id = 0;
		mUnit = 0;
		m_width = 0;
		mHeight = 0;
	}
}
