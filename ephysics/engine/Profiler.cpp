/** @file
 * @author Daniel Chappuis
 * @copyright 2010-2016 Daniel Chappuis
 * @license BSD 3 clauses (see license file)
 */

#ifdef IS_PROFILING_ACTIVE

// Libraries
#include <ephysics/Profiler.h>

using namespace reactphysics3d;

// Initialization of static variables
ProfileNode Profiler::m_rootNode("Root", NULL);
ProfileNode* Profiler::m_currentNode = &Profiler::m_rootNode;
long double Profiler::m_profilingStartTime = Timer::getCurrentSystemTime() * 1000.0;
uint32_t Profiler::m_frameCounter = 0;

// Constructor
ProfileNode::ProfileNode(const char* name, ProfileNode* parentNode)
	:m_name(name), m_numberTotalCalls(0), m_startTime(0), m_totalTime(0),
	 m_recursionCounter(0), m_parentNode(parentNode), m_childNode(NULL),
	 m_siblingNode(NULL) {
	reset();
}

// Destructor
ProfileNode::~ProfileNode() {

	delete m_childNode;
	delete m_siblingNode;
}

// Return a pointer to a sub node with a given name
ProfileNode* ProfileNode::findSubNode(const char* name) {

	// Try to find the node among the child nodes
	ProfileNode* child = m_childNode;
	while (child != NULL) {
		if (child->m_name == name) {
			return child;
		}
		child = child->m_siblingNode;
	}

	// The nose has not been found. Therefore, we create it
	// and add it to the profiler tree
	ProfileNode* newNode = new ProfileNode(name, this);
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
	if (m_childNode != NULL) {
		m_childNode->reset();
	}

	// Reset the sibling node
	if (m_siblingNode != NULL) {
		m_siblingNode->reset();
	}
}

// Destroy the node
void ProfileNode::destroy() {
	delete m_childNode;
	m_childNode = NULL;
	delete m_siblingNode;
	m_siblingNode = NULL;
}

// Constructor
ProfileNodeIterator::ProfileNodeIterator(ProfileNode* startingNode)
	:m_currentParentNode(startingNode),
	  m_currentChildNode(m_currentParentNode->getChildNode()){

}

// Enter a given child node
void ProfileNodeIterator::enterChild(int32_t index) {
	m_currentChildNode = m_currentParentNode->getChildNode();
	while ((m_currentChildNode != NULL) && (index != 0)) {
		index--;
		m_currentChildNode = m_currentChildNode->getSiblingNode();
	}

	if (m_currentChildNode != NULL) {
		m_currentParentNode = m_currentChildNode;
		m_currentChildNode = m_currentParentNode->getChildNode();
	}
}

// Enter a given parent node
void ProfileNodeIterator::enterParent() {
	if (m_currentParentNode->getParentNode() != NULL) {
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
void Profiler::print32_tReport(std::ostream& outputStream) {
	ProfileNodeIterator* iterator = Profiler::getIterator();

	// Recursively print32_t the report of each node of the profiler tree
	print32_tRecursiveNodeReport(iterator, 0, outputStream);

	// Destroy the iterator
	destroyIterator(iterator);
}

// Recursively print32_t the report of a given node of the profiler tree
void Profiler::print32_tRecursiveNodeReport(ProfileNodeIterator* iterator,
										int32_t spacing,
										std::ostream& outputStream) {
	iterator->first();

	// If we are at the end of a branch in the profiler tree
	if (iterator->isEnd()) {
		return;
	}

	long double parentTime = iterator->isRoot() ? getElapsedTimeSinceStart() :
												  iterator->getCurrentParentTotalTime();
	long double accumulatedTime = 0.0;
	uint32_t nbFrames = Profiler::getNbFrames();
	for (int32_t i=0; i<spacing; i++) outputStream << " ";
	outputStream << "---------------" << std::endl;
	for (int32_t i=0; i<spacing; i++) outputStream << " ";
	outputStream << "| Profiling : " << iterator->getCurrentParentName() <<
					" (total running time : " << parentTime << " ms) ---" << std::endl;
	long double totalTime = 0.0;

	// Recurse over the children of the current node
	int32_t nbChildren = 0;
	for (int32_t i=0; !iterator->isEnd(); i++, iterator->next()) {
		nbChildren++;
		long double currentTotalTime = iterator->getCurrentTotalTime();
		accumulatedTime += currentTotalTime;
		long double fraction = parentTime > std::numeric_limits<long double>::epsilon() ?
							   (currentTotalTime / parentTime) * 100.0 : 0.0;
		for (int32_t j=0; j<spacing; j++) outputStream << " ";
		outputStream << "|   " << i << " -- " << iterator->getCurrentName() << " : " <<
						fraction << " % | " << (currentTotalTime / (long double) (nbFrames)) <<
						" ms/frame (" << iterator->getCurrentNbTotalCalls() << " calls)" <<
						std::endl;
		totalTime += currentTotalTime;
	}

	if (parentTime < accumulatedTime) {
		outputStream << "Something is wrong !" << std::endl;
	}
	for (int32_t i=0; i<spacing; i++) outputStream << " ";
	long double percentage = parentTime > std::numeric_limits<long double>::epsilon() ?
				((parentTime - accumulatedTime) / parentTime) * 100.0 : 0.0;
	long double difference = parentTime - accumulatedTime;
	outputStream << "| Unaccounted : " << difference << " ms (" << percentage << " %)" << std::endl;

	for (int32_t i=0; i<nbChildren; i++){
		iterator->enterChild(i);
		print32_tRecursiveNodeReport(iterator, spacing + 3, outputStream);
		iterator->enterParent();
	}
}

#endif
