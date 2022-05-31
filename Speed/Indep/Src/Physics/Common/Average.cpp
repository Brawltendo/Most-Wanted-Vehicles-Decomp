#include "physics/common/average.h"

// MATCHING
// Queues an input value and a time value into their respective buffers and averages the total.
void AverageWindow::Record(const float fValue, const float fTimeNow)
{
	if (pData[nCurrentSlot] == 0.f && pTimeData[nCurrentSlot] == 0.f)
		++nSamples;
	else
		fTotal -= pData[nCurrentSlot];

	fTotal += fValue;
	// enqueue values
	pData[nCurrentSlot] = fValue;
	pTimeData[nCurrentSlot] = fTimeNow;

	// refresh oldest value using the time buffer and clear if necessary
	while (fTimeNow - pTimeData[iOldestValue] > fTimeWindow)
	{
		if (pTimeData[iOldestValue] > 0.f)
		{
			fTotal -= pData[iOldestValue];
			pData[iOldestValue] = 0.f;
			pTimeData[iOldestValue] = 0.f;
			--nSamples;
		}

		++iOldestValue;
		if (iOldestValue >= nSlots)
			iOldestValue = 0;
	}

	fAverage = fTotal / nSamples;
	++nCurrentSlot;
	if (nCurrentSlot >= nSlots)
		nCurrentSlot = 0;
}