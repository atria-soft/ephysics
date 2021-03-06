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
#include <ephysics/MeshReaderWriter.hpp>
#include <fstream>
#include <sstream>
#include <locale>
#include <cctype>
#include <etk/Map.hpp>
#include <algorithm>

using namespace openglframework;
using namespace std;

// Constructor
MeshReaderWriter::MeshReaderWriter() {

}

// Load a mesh from a file and returns true if the mesh has been sucessfully loaded
void MeshReaderWriter::loadMeshFromFile(const etk::String& filename, Mesh& meshToCreate) {

	// Get the extension of the file
	uint32_t startPosExtension = filename.find_last_of(".");
	string extension = filename.substr(startPosExtension+1);

	// Load the file using the correct method
	if (extension == "obj") {
		loadOBJFile(filename, meshToCreate);
	}
	else {

		// Display an error message and throw an exception
		string errorMessage("Error : the MeshReaderWriter class cannot load a file with the extension .");
		errorMessage += extension;
		std::cerr << errorMessage << std::endl;
		throw std::invalid_argument(errorMessage.c_str());
	}
}

// Write a mesh to a file
void MeshReaderWriter::writeMeshToFile(const etk::String& filename, const Mesh& meshToWrite) {

	// Get the extension of the file
	uint32_t startPosExtension = filename.find_last_of(".");
	string extension = filename.substr(startPosExtension+1);

	// Load the file using the correct method
	if (extension == "obj") {
		writeOBJFile(filename, meshToWrite);
	}
	else {

		// Display an error message and throw an exception
		string errorMessage("Error : the MeshReaderWriter class cannot store a mesh file with the extension .");
		errorMessage += extension;
		std::cerr << errorMessage << std::endl;
		throw std::invalid_argument(errorMessage.c_str());
	}
}

// Load an OBJ file with a triangular or quad mesh
void MeshReaderWriter::loadOBJFile(const string &filename, Mesh& meshToCreate) {

	// Open the file
	std::ifstream meshFile(filename.c_str());

	// If we cannot open the file
	if(!meshFile.is_open()) {

		// Throw an exception and display an error message
		string errorMessage("Error : Cannot open the file " + filename);
		std::cerr << errorMessage << std::endl;
		throw runtime_error(errorMessage);
	}

	etk::String buffer;
	string line, tmp;
	int32_t id1, id2, id3, id4;
	int32_t nId1, nId2, nId3, nId4;
	int32_t tId1, tId2, tId3, tId4;
	float v1, v2, v3;
	size_t found1, found2;
	etk::Vector<bool> isQuad;
	etk::Vector<vec3> vertices;
	etk::Vector<vec3> normals;
	etk::Vector<vec2> uvs;
	etk::Vector<uint32_t> verticesIndices;
	etk::Vector<uint32_t> normalsIndices;
	etk::Vector<uint32_t> uvsIndices;

	// ---------- Collect the data from the file ---------- //

	// For each line of the file
	while(std::getline(meshFile, buffer)) {

		std::istringstream lineStream(buffer);
		etk::String word;
		lineStream >> word;
		std::transform(word.begin(), word.end(), word.begin(), ::tolower);
		if(word == "usemtl") {  // Material definition

			// Loading of MTL file is not implemented

		}
		else if(word == "v") {  // Vertex position
			sscanf(buffer.c_str(), "%*s %f %f %f", &v1, &v2, &v3);
			vertices.pushBack(vec3(v1, v2, v3));
		}
		else if(word == "vt") { // Vertex texture coordinate
			sscanf(buffer.c_str(), "%*s %f %f", &v1, &v2);
			uvs.pushBack(vec2(v1,v2));
		}
		else if(word == "vn") { // Vertex normal
			sscanf(buffer.c_str(), "%*s %f %f %f", &v1, &v2, &v3);
			normals.pushBack(vec3(v1 ,v2, v3));
		}
		else if (word == "f") { // Face
			line = buffer;
			found1 = (int32_t)line.find("/");
			bool isFaceQuad = false;
			int32_t foundNext = (int32_t)line.substr(found1+1).find("/");

			// If the face definition is of the form "f v1 v2 v3 v4"
			if(found1 == string::npos) {
				int32_t nbVertices = sscanf(buffer.c_str(), "%*s %d %d %d %d", &id1, &id2, &id3, &id4);
				if (nbVertices == 4) isFaceQuad = true;
			}
			// If the face definition is of the form "f v1// v2// v3// v4//"
			else if (foundNext == 0) {
				int32_t nbVertices = sscanf(buffer.c_str(), "%*s %d// %d// %d// %d//", &id1, &id2, &id3, &id4);
				if (nbVertices == 4) isFaceQuad = true;
			}
			else {  // If the face definition contains vertices and texture coordinates

				//get the part of the string until the second index
				tmp = line.substr(found1+1);
				found2 = (int32_t)tmp.find(" ");
				tmp = tmp.substr(0,found2);
				found2 = (int32_t)tmp.find("/");

				// If the face definition is of the form "f vert1/textcoord1 vert2/textcoord2 ..."
				if(found2 == string::npos) {
					int32_t n = sscanf(buffer.c_str(), "%*s %d/%d %d/%d %d/%d %d/%d", &id1, &tId1, &id2, &tId2, &id3, &tId3, &id4, &tId4);
					if (n == 8) isFaceQuad = true;
					uvsIndices.pushBack(tId1-1);
					uvsIndices.pushBack(tId2-1);
					uvsIndices.pushBack(tId3-1);
					if (isFaceQuad) uvsIndices.pushBack(tId4-1);
				}
				else {
					tmp = line.substr(found1+1);
					found2 = (int32_t)tmp.find("/");

					// If the face definition is of the form "f vert1/normal1 vert2/normal2 ..."
					if(found2 == 0) {
						int32_t n = sscanf(buffer.c_str(), "%*s %d//%d %d//%d %d//%d %d//%d", &id1, &nId1, &id2, &nId2, &id3, &nId3, &id4, &nId4);
						if (n == 8) isFaceQuad = true;
					}
					// If the face definition is of the form "f vert1/textcoord1/normal1 ..."
					else {
						int32_t n = sscanf(buffer.c_str(), "%*s %d/%d/%d %d/%d/%d %d/%d/%d %d/%d/%d", &id1, &tId1, &nId1, &id2, &tId2, &nId2, &id3, &tId3, &nId3, &id4, &tId4, &nId4);
						if (n == 12) isFaceQuad = true;
						uvsIndices.pushBack(tId1-1);
						uvsIndices.pushBack(tId2-1);
						uvsIndices.pushBack(tId3-1);
						if (isFaceQuad) uvsIndices.pushBack(tId4-1);
					}
					normalsIndices.pushBack(nId1-1);
					normalsIndices.pushBack(nId2-1);
					normalsIndices.pushBack(nId3-1);
					if (isFaceQuad) normalsIndices.pushBack(nId4-1);
				}
			}
			verticesIndices.pushBack(id1-1);
			verticesIndices.pushBack(id2-1);
			verticesIndices.pushBack(id3-1);
			if (isFaceQuad) verticesIndices.pushBack((id4-1));
			isQuad.pushBack(isFaceQuad);
		}
	}

	assert(!verticesIndices.empty());
	assert(normalsIndices.empty() || normalsIndices.size() == verticesIndices.size());
	assert(uvsIndices.empty() || uvsIndices.size() == verticesIndices.size());
	meshFile.close();

	// ---------- Merge the data that we have collected from the file ---------- //

	// Destroy the current mesh
	meshToCreate.destroy();

	// Mesh data
	vector<etk::Vector<uint32_t> > meshIndices;
	vector<vec3> meshNormals;
	if (!normals.empty()) meshNormals = vector<vec3>(vertices.size(), vec3(0, 0, 0));
	vector<vec2> meshUVs;
	if (!uvs.empty()) meshUVs = vector<vec2>(vertices.size(), vec2(0, 0));

	// We cannot load mesh with several parts for the moment
	uint32_t meshPart = 0;

	// Fill in the vertex indices
	// We also triangulate each quad face
	meshIndices.pushBack(etk::Vector<uint32_t>());
	for(size_t i = 0, j = 0; i < verticesIndices.size(); j++) {

		// Get the current vertex IDs
		uint32_t i1 = verticesIndices[i];
		uint32_t i2 = verticesIndices[i+1];
		uint32_t i3 = verticesIndices[i+2];

		// Add the vertex normal
		if (!normalsIndices.empty() && !normals.empty()) {
			meshNormals[i1] = normals[normalsIndices[i]];
			meshNormals[i2] = normals[normalsIndices[i+1]];
			meshNormals[i3] = normals[normalsIndices[i+2]];
		}

		// Add the vertex UV texture coordinates
		if (!uvsIndices.empty() && !uvs.empty()) {
			meshUVs[i1] = uvs[uvsIndices[i]];
			meshUVs[i2] = uvs[uvsIndices[i+1]];
			meshUVs[i3] = uvs[uvsIndices[i+2]];
		}

		// If the current vertex not in a quad (it is part of a triangle)
		if (!isQuad[j]) {

			// Add the vertex indices
			meshIndices[meshPart].pushBack(i1);
			meshIndices[meshPart].pushBack(i2);
			meshIndices[meshPart].pushBack(i3);

			i+=3;
		}
		else {  // If the current vertex is in a quad

			vec3 v1 = vertices[i1];
			vec3 v2 = vertices[i2];
			vec3 v3 = vertices[i3];
			uint32_t i4 = verticesIndices[i+3];
			vec3 v4 = vertices[i4];

			vec3 v13 = v3-v1;
			vec3 v12 = v2-v1;
			vec3 v14 = v4-v1;

			float a1 = v13.dot(v12);
			float a2 = v13.dot(v14);
			if((a1 >= 0 && a2 <= 0) || (a1 <= 0 && a2 >= 0)) {
				meshIndices[meshPart].pushBack(i1);
				meshIndices[meshPart].pushBack(i2);
				meshIndices[meshPart].pushBack(i3);
				meshIndices[meshPart].pushBack(i1);
				meshIndices[meshPart].pushBack(i3);
				meshIndices[meshPart].pushBack(i4);
			}
			else {
				meshIndices[meshPart].pushBack(i1);
				meshIndices[meshPart].pushBack(i2);
				meshIndices[meshPart].pushBack(i4);
				meshIndices[meshPart].pushBack(i2);
				meshIndices[meshPart].pushBack(i3);
				meshIndices[meshPart].pushBack(i4);
			}

			// Add the vertex normal
			if (!normalsIndices.empty() && !normals.empty()) {
				meshNormals[i4] = normals[normalsIndices[i]];
			}

			// Add the vertex UV texture coordinates
			if (!uvsIndices.empty() && !uvs.empty()) {
				meshUVs[i4] = uvs[uvsIndices[i]];
			}

			i+=4;
		}
	}

	assert(meshNormals.empty() || meshNormals.size() == vertices.size());
	assert(meshUVs.empty() || meshUVs.size() == vertices.size());

	// Set the data to the mesh
	meshToCreate.setIndices(meshIndices);
	meshToCreate.setVertices(vertices);
	meshToCreate.setNormals(meshNormals);
	meshToCreate.setUVs(meshUVs);
}

// Store a mesh int32_to a OBJ file
void MeshReaderWriter::writeOBJFile(const etk::String& filename, const Mesh& meshToWrite) {
	std::ofstream file(filename.c_str());

	// Geth the mesh data
	const etk::Vector<vec3>& vertices = meshToWrite.getVertices();
	const etk::Vector<vec3>& normals = meshToWrite.getNormals();
	const etk::Vector<vec2>& uvs = meshToWrite.getUVs();

	// If we can open the file
	if (file.is_open()) {

		assert(meshToWrite.getNbVertices() == vertices.size());

		// Write the vertices
		for (uint32_t v=0; v<vertices.size(); v++) {

			file << "v " << vertices[v].x() << " " << vertices[v].y() << " " << vertices[v].z() <<
					std::endl;
		}

		// Write the normals
		if (meshToWrite.hasNormals()) {
			file << std::endl;

			assert(meshToWrite.getNbVertices() == normals.size());

			for (uint32_t v=0; v<normals.size(); v++) {

				file << "vn " << normals[v].x() << " " << normals[v].y() << " " << normals[v].z() <<
						std::endl;
			}
		}

		// Write the UVs texture coordinates
		if (meshToWrite.hasUVTextureCoordinates()) {
			file << std::endl;

			assert(meshToWrite.getNbVertices() == uvs.size());

			for (uint32_t v=0; v<uvs.size(); v++) {

				file << "vt " << uvs[v].x() << " " << uvs[v].y() << std::endl;
			}
		}

		// Write the faces
		file << std::endl;
		for (uint32_t p=0; p<meshToWrite.getNbParts(); p++) {

			// Get the indices of the part
			const etk::Vector<uint32_t>& indices = meshToWrite.getIndices(p);

			// For each index of the part
			for (uint32_t i=0; i<indices.size(); i+=3) {

				if (meshToWrite.hasNormals() && meshToWrite.hasUVTextureCoordinates()) {
					file << "f " <<indices[i]+1 << "/" << indices[i]+1 << "/" << indices[i]+1 <<
							" " << indices[i+1]+1 << "/" << indices[i+1]+1 << "/" << indices[i+1]+1 <<
							" " << indices[i+2]+1 << "/" << indices[i+2]+1 << "/" << indices[i+2]+1 <<
							std::endl;
				}
				else if (meshToWrite.hasNormals() || meshToWrite.hasUVTextureCoordinates()) {
					file << "f " <<indices[i]+1 << "/" << indices[i]+1 <<
							" " << indices[i+1]+1 << "/" << indices[i+1]+1 <<
							" " << indices[i+2]+1 << "/" << indices[i+2]+1 << std::endl;
				}
				else {
					file << "f " << indices[i]+1 << " " << indices[i+1]+1 << " " << indices[i+2]+1 <<
							std::endl;
				}
			}
		}
	}
	else {
		std::cerr << "Error : Cannot open the file " << filename << std::endl;
		exit(1);
	}
}
