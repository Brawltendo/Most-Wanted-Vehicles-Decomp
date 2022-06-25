#pragma once
#include "speedcommon.h"


char* gCreationPoint = NULL;
char* gDeletionPoint = NULL;

class Event
{
public:
	virtual ~Event();
	virtual const char* GetEventName() { return ""; }

	void* operator new(size_t size);
	void operator delete(void* ptr);

	struct StaticData
	{
		uint32_t fEventSize;
	};

private:
	uint32_t fEventSize;
};