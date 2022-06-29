#include "Speed/Indep/Src/Generated/Events/EPlayerShift.h"


void EPlayerShift_MakeEvent_Callback(const void* staticData)
{
	EPlayerShift::StaticData* data = (EPlayerShift::StaticData*)staticData;
	new EPlayerShift(data->fhSimable, data->fStatus, data->fAutomatic, data->fFrom, data->fTo);
}