#include "speedcommon.h"

struct sEnvelopePoint
{
	float fTime;
	float fValue;
};

struct tEnvelope
{
	int GetIndex(float t);

	sEnvelopePoint *pPoints;
	int nPoints;
};