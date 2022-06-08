#include "packages/snd/9/source/library/cmn/sattrdef.h"


// MATCHING
int SND_attrsetdef(struct SNDSAMPLEATTR* pssa)
{
	pssa->priority = 0;
	pssa->detune = 0;
	pssa->vol = 127;
	pssa->pan = 64;
	pssa->fxlevel0 = 0;
	pssa->bendrange = 0;
	pssa->platformver = 0;
	pssa->rendermode = 0;

	{
		int i;
		for (i = 0; i < 4; ++i)
		{
			pssa->puserdata[i] = 0;
			pssa->userdatasize[i] = 0;
		}
		for (i = 0; i < 6; ++i)
		{
			pssa->azimuth[i] = 0;
			pssa->ptsdata[i] = 0;
		}
	}
	return 0;
}