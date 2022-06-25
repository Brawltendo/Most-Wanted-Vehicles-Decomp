#include "Speed/Indep/Src/Main/Event.h"


// MATCHING
void* Event::operator new(size_t size)
{
	char* event = gCreationPoint;
	gCreationPoint += (size + 0xF) & 0xFFFFFFF0;
	return event;
}

// MATCHING
void Event::operator delete(void* ptr)
{
	gDeletionPoint += reinterpret_cast<Event*>(ptr)->fEventSize;
}