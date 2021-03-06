/** @file
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://www.reactphysics3d.com/> This code is re-licensed with permission from ReactPhysics3D author.
 * @author Daniel CHAPPUIS
 * @author Edouard DUPIN
 * @copyright 2010-2016, Daniel Chappuis
 * @copyright 2017, Edouard DUPIN
 * @license MPL v2.0 (see license file)
 */
#ifdef IS_PROFILING_ACTIVE

// Libraries
#include <ephysics/Profiler.hpp>
#include <ephysics/debug.hpp>

using namespace ephysics;

// Initialization of static variables
ProfileNode Profiler::m_rootNode("Root", null);
ProfileNode* Profiler::m_currentNode = &Profiler::m_rootNode;
long double Profiler::m_profilingStartTime = Timer::getCurrentSystemTime() * 1000.0;
uint32_t Profiler::m_frameCounter = 0;

// Constructor
ProfileNode::ProfileNode(const char* name, ProfileNode* parentNode)
	:m_name(name), m_numberTotalCalls(0), m_startTime(0), m_totalTime(0),
	 m_recursionCounter(0), m_parentNode(parentNode), m_childNode(null),
	 m_siblingNode(null) {
	reset();
}

// Destructor
ProfileNode::~ProfileNode() {
	ETK_DELETE(ProfileNode, m_childNode);
	m_childNode = null;
	ETK_DELETE(ProfileNode, m_siblingNode);
	m_siblingNode = null;
}

// Return a pointer to a sub node with a given name
ProfileNode* ProfileNode::findSubNode(const char* name) {

	// Try to find the node among the child nodes
	ProfileNode* child = m_childNode;
	while (child != null) {
		if (child->m_name == name) {
			return child;
		}
		child = child->m_siblingNode;
	}

	// The nose has not been found. Therefore, we create it
	// and add it to the profiler tree
	ProfileNode* newNode = ETK_NEW(ProfileNode, name, this);
	newNode->m_siblingNode = m_childNode;
	m_childNode = newNode;

	return newNode;
}

// Called when we enter the block of code corresponding to this profile node
void ProfileNode::enterBlockOfCode() {
	m_numberTotalCalls++;

	// If the current code is not called recursively
	if (m_recursionCounter == 0) {

		// Get the current system time to initialize the starting time of
		// the profiling of the current block of code
		m_startTime = Timer::getCurrentSystemTime() * 1000.0;
	}

	m_recursionCounter++;
}

// Called when we exit the block of code corresponding to this profile node
bool ProfileNode::exitBlockOfCode() {
	m_recursionCounter--;

	if (m_recursionCounter == 0 && m_numberTotalCalls != 0) {

		// Get the current system time
		long double currentTime = Timer::getCurrentSystemTime() * 1000.0;

		// Increase the total elasped time in the current block of code
		m_totalTime += currentTime - m_startTime;
	}

	// Return true if the current code is not recursing
	return (m_recursionCounter == 0);
}

// Reset the profiling of the node
void ProfileNode::reset() {
	m_numberTotalCalls = 0;
	m_totalTime = 0.0;

	// Reset the child node
	if (m_childNode != null) {
		m_childNode->reset();
	}

	// Reset the sibling node
	if (m_siblingNode != null) {
		m_siblingNode->reset();
	}
}

// Destroy the node
void ProfileNode::destroy() {
	ETK_DELETE(ProfileNode, m_childNode);
	m_childNode = null;
	ETK_DELETE(ProfileNode, m_siblingNode);
	m_siblingNode = null;
}

// Constructor
ProfileNodeIterator::ProfileNodeIterator(ProfileNode* startingNode)
	:m_currentParentNode(startingNode),
	  m_currentChildNode(m_currentParentNode->getChildNode()){

}

// Enter a given child node
void ProfileNodeIterator::enterChild(int32_t index) {
	m_currentChildNode = m_currentParentNode->getChildNode();
	while ((m_currentChildNode != null) && (index != 0)) {
		index--;
		m_currentChildNode = m_currentChildNode->getSiblingNode();
	}

	if (m_currentChildNode != null) {
		m_currentParentNode = m_currentChildNode;
		m_currentChildNode = m_currentParentNode->getChildNode();
	}
}

// Enter a given parent node
void ProfileNodeIterator::enterParent() {
	if (m_currentParentNode->getParentNode() != null) {
		m_currentParentNode = m_currentParentNode->getParentNode();
	}
	m_currentChildNode = m_currentParentNode->getChildNode();
}

// Method called when we want to start profiling a block of code.
void Profiler::startProfilingBlock(const char* name) {

	// Look for the node in the tree that corresponds to the block of
	// code to profile
	if (name != m_currentNode->getName()) {
		m_currentNode = mCurrentNode->findSubNode(name);
	}

	// Start profile the node
	m_currentNode->enterBlockOfCode();
}

// Method called at the end of the scope where the
// startProfilingBlock() method has been called.
void Profiler::stopProfilingBlock() {

	// Go to the parent node unless if the current block
	// of code is recursing
	if (m_currentNode->exitBlockOfCode()) {
		m_currentNode = mCurrentNode->getParentNode();
	}
}

// Reset the timing data of the profiler (but not the profiler tree structure)
void Profiler::reset() {
	m_rootNode.reset();
	m_rootNode.enterBlockOfCode();
	m_frameCounter = 0;
	m_profilingStartTime = Timer::getCurrentSystemTime() * 1000.0;
}

// Print32_t the report of the profiler in a given output stream
void Profiler::print32_tReport(etk::Stream& _stream) {
	ProfileNodeIterator* iterator = Profiler::getIterator();

	// Recursively print32_t the report of each node of the profiler tree
	print32_tRecursiveNodeReport(iterator, 0, _stream);

	// Destroy the iterator
	destroyIterator(iterator);
}

// Recursively print32_t the report of a given node of the profiler tree
void Profiler::print32_tRecursiveNodeReport(ProfileNodeIterator* iterator,
                                            int32_t spacing,
                                            etk::Stream& _stream) {
	iterator->first();

	// If we are at the end of a branch in the profiler tree
	if (iterator->isEnd()) {
		return;
	}

	long double parentTime = iterator->isRoot() ? getElapsedTimeSinceStart() :
												  iterator->getCurrentParentTotalTime();
	long double accumulatedTime = 0.0;
	uint32_t nbFrames = Profiler::getNbFrames();
	for (int32_t i=0; i<spacing; i++) {
		_stream << " ";
	}
	_stream << "---------------\n";
	for (int32_t i=0; i<spacing; i++) _stream << " ";
	_stream << "| Profiling : " << iterator->getCurrentParentName() <<
					" (total running time : " << parentTime << " ms) ---\n";
	long double totalTime = 0.0;

	// Recurse over the children of the current node
	int32_t nbChildren = 0;
	for (int32_t i=0; !iterator->isEnd(); i++, iterator->next()) {
		nbChildren++;
		long double currentTotalTime = iterator->getCurrentTotalTime();
		accumulatedTime += currentTotalTime;
		long double fraction = parentTime > DLB_EPSILON ?
							   (currentTotalTime / parentTime) * 100.0 : 0.0;
		for (int32_t j=0; j<spacing; j++) _stream << " ";
		_stream << "|   " << i << " -- " << iterator->getCurrentName() << " : " <<
						fraction << " % | " << (currentTotalTime / (long double) (nbFrames)) <<
						" ms/frame (" << iterator->getCurrentNbTotalCalls() << " calls)\n";
		totalTime += currentTotalTime;
	}

	if (parentTime < accumulatedTime) {
		_stream << "Something is wrong !\n";
	}
	for (int32_t i=0; i<spacing; i++) _stream << " ";
	long double percentage = parentTime > DLB_EPSILON ?
				((parentTime - accumulatedTime) / parentTime) * 100.0 : 0.0;
	long double difference = parentTime - accumulatedTime;
	_stream << "| Unaccounted : " << difference << " ms (" << percentage << " %)\n";

	for (int32_t i=0; i<nbChildren; i++){
		iterator->enterChild(i);
		print32_tRecursiveNodeReport(iterator, spacing + 3, _stream);
		iterator->enterParent();
	}
}

// Return true if we are at the root of the profiler tree
bool ProfileNodeIterator::isRoot() {
	return (m_currentParentNode->getParentNode() == null);
}

// Return true if we are at the end of a branch of the profiler tree
bool ProfileNodeIterator::isEnd() {
	return (m_currentChildNode == null);
}

// Return the name of the current node
const char* ProfileNodeIterator::getCurrentName() {
	return m_currentChildNode->getName();
}

// Return the total time of the current node
long double ProfileNodeIterator::getCurrentTotalTime() {
	return m_currentChildNode->getTotalTime();
}

// Return the total number of calls of the current node
uint32_t ProfileNodeIterator::getCurrentNbTotalCalls() {
	return m_currentChildNode->getNbTotalCalls();
}

// Return the name of the current parent node
const char* ProfileNodeIterator::getCurrentParentName() {
	return m_currentParentNode->getName();
}

// Return the total time of the current parent node
long double ProfileNodeIterator::getCurrentParentTotalTime() {
	return m_currentParentNode->getTotalTime();
}

// Return the total number of calls of the current parent node
uint32_t ProfileNodeIterator::getCurrentParentNbTotalCalls() {
	return m_currentParentNode->getNbTotalCalls();
}

// Go to the first node
void ProfileNodeIterator::first() {
	m_currentChildNode = m_currentParentNode->getChildNode();
}

// Go to the next node
void ProfileNodeIterator::next() {
	m_currentChildNode = m_currentChildNode->getSiblingNode();
}

// Return a pointer to the parent node
ProfileNode* ProfileNode::getParentNode() {
	return m_parentNode;
}

// Return a pointer to a sibling node
ProfileNode* ProfileNode::getSiblingNode() {
	return m_siblingNode;
}

// Return a pointer to a child node
ProfileNode* ProfileNode::getChildNode() {
	return m_childNode;
}

// Return the name of the node
const char* ProfileNode::getName() {
	return m_name;
}

// Return the total number of call of the corresponding block of code
uint32_t ProfileNode::getNbTotalCalls() const {
	return m_numberTotalCalls;
}

// Return the total time spent in the block of code
long double ProfileNode::getTotalTime() const {
	return m_totalTime;
}

// Return the number of frames
uint32_t Profiler::getNbFrames() {
	return m_frameCounter;
}

// Return the elasped time since the start/reset of the profiling
long double Profiler::getElapsedTimeSinceStart() {
	long double currentTime = Timer::getCurrentSystemTime() * 1000.0;
	return currentTime - m_profilingStartTime;
}

// Increment the frame counter
void Profiler::incrementFrameCounter() {
	m_frameCounter++;
}

// Return an iterator over the profiler tree starting at the root
ProfileNodeIterator* Profiler::getIterator() {
	return ETK_NEW(ProfileNodeIterator(&m_rootNode));
}

// Destroy a previously allocated iterator
void Profiler::destroyIterator(ProfileNodeIterator* iterator) {
	ETK_DELETE(ProfileNodeIterator, iterator);
}

// Destroy the profiler (release the memory)
void Profiler::destroy() {
	m_rootNode.destroy();
}
#endif
