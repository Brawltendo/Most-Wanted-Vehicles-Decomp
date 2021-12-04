#include "misc/envelope.h"

// NOT MATCHING
// for some reason some labels are lower but have the same instructions as the original
// so it's nearly matching since they're still executed in the same order anyway
int tEnvelope::GetIndex(float t)
{
	int numPts = nPoints;
	int i = 0;
	if (numPts >= 4)
	{
		sEnvelopePoint* pt = &pPoints[2];
		while (!(t < pt[-2].fTime))
		{
			if (t < pt[-1].fTime)
			{
				++i;
				break;
			}
			if (t < pt[0].fTime)
			{
				i += 2;
				break;
			}
			if (t < pt[1].fTime)
			{
				i += 3;
				break;
			}
			i += 4;
			pt += 4;
			if (i < numPts - 3)
				continue;
			else
				goto LoopLessThan4Pts;
		}
	}
	else
	{
	LoopLessThan4Pts:
		if (i < numPts)
		{
			sEnvelopePoint* pt = &pPoints[i];
			do
			{
				if (t < pt->fTime)
					break;
				++i;
				++pt;
			} while (i < numPts);
		}
	}
	if (i > 0)
		--i;

	return i;
}