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

#ifndef TEXTURE2D_H
#define TEXTURE2D_H

// Libraries
#include <string>
#include <cassert>
#include <ephysics/definitions.h>
#include <nanogui/opengl.h>

namespace openglframework {

// Class Texture2D
// This class represents a 2D texture
class Texture2D {

	private:

		// -------------------- Attributes -------------------- //

		// OpenGL texture ID
		GLuint32_t mID;

		// Current texture unit for this texture
		GLuint32_t mUnit;

		// Width
		uint32_t mWidth;

		// Height
		uint32_t mHeight;

	public:

		// -------------------- Methods -------------------- //

		// Constructor
		Texture2D();

		// Constructor
		Texture2D(uint32_t width, uint32_t height, uint32_t int32_ternalFormat, uint32_t format, uint32_t type);

		// Destructor
		~Texture2D();

		// Create the texture
		void create(uint32_t width, uint32_t height, uint32_t int32_ternalFormat, uint32_t format, uint32_t type,
					void* data = NULL);

		// Create the texture
		void create(uint32_t width, uint32_t height, uint32_t int32_ternalFormat, uint32_t format, uint32_t type,
					uint32_t minFilter, uint32_t maxFilter, uint32_t wrapS, uint32_t wrapT, void* data);

		// Destroy the texture
		void destroy();

		// Bind the texture
		void bind() const;

		// Unbind the texture
		void unbind() const;

		// Get the OpenGL texture ID
		uint32_t getID() const;

		// Get the unit of the texture
		uint32_t getUnit() const;

		// Set the unit of the texture
		void setUnit(uint32_t unit);

		// Get the width
		uint32_t getWidth() const;

		// Get the height
		uint32_t getHeight() const;
};

// Bind the texture
inline void Texture2D::bind() const {
	assert(mID != 0);
	glActiveTexture(GL_TEXTURE0 + mUnit);
	glBindTexture(GL_TEXTURE_2D, mID);
}

// Unbind the texture
inline void Texture2D::unbind() const {
	assert(mID != 0);
	glActiveTexture(GL_TEXTURE0 + mUnit);
	glBindTexture(GL_TEXTURE_2D, 0);
}

// Get the OpenGL texture ID
inline uint32_t Texture2D::getID() const {
	return mID;
}

// Get the unit of the texture
inline uint32_t Texture2D::getUnit() const {
	return mUnit;
}

// Set the unit of the texture
inline void Texture2D::setUnit(uint32_t unit) {
	mUnit = unit;
}

// Get the width
inline uint32_t Texture2D::getWidth() const {
	return mWidth;
}

// Get the height
inline uint32_t Texture2D::getHeight() const {
	return mHeight;
}

}

#endif
