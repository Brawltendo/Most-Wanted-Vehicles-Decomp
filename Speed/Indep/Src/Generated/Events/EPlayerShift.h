#pragma once
#include "Speed/Indep/Src/Main/Event.h"
#include "Physics/Physicstypes.h"


class EPlayerShift : public Event
{
public:
	virtual ~EPlayerShift();
	EPlayerShift(class HSIMABLE__ *phSimable, ShiftStatus pStatus, bool pAutomatic, GearID pFrom, GearID pTo);
	const char* GetEventName() { return "EPlayerShift"; }

	struct StaticData : public Event::StaticData
	{
		class HSIMABLE__* fhSimable;
		ShiftStatus fStatus;
		bool fAutomatic;
		GearID fFrom;
		GearID fTo;
	};

	enum
	{
		kEventID = 1171244884
	};

private:
	class HSIMABLE__* fhSimable;
	ShiftStatus fStatus;
	bool fAutomatic;
	GearID fFrom;
	GearID fTo;

};

void EPlayerShift_MakeEvent_Callback(const void* staticData);