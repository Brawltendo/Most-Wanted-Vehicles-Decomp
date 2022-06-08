#include "packages/snd/9/source/library/cmn/splysdef.h"


// MATCHING
int SNDplaysetdef(struct SNDPLAYOPTS* pspo)
{
	pspo->lowpasscutoff = -1;
	pspo->keynum = 60;
	pspo->pitchmult = 4096;
	pspo->timemult = 4096;
	pspo->tempomult = 4096;
	pspo->highpasscutoff = 0;
	pspo->velocity = 127;
	pspo->vol = 127;
	pspo->drylevel = 127;
	pspo->bend = 64;
	pspo->fxlevel0 = 0;
	pspo->azimuth = 0;
	pspo->elevation = 0;
	return 0;
}