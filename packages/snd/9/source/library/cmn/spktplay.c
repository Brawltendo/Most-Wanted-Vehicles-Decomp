#include "packages/snd/9/source/library/cmn/spktplay.h"


char unk_spktplay_var = 1;
int SNDPKTPLAY_submit(int packetinstancehandle, struct SNDPACKET* psp)
{
	struct SNDPACKETCHAN* ppchan;
	struct SNDPACKETENTRY* pentry;
	int packethandle = 0;

	if (unk_spktplay_var == 0)
		return -10;

	ppchan = spktchannels[packetinstancehandle];
	if (ppchan->sampleformat.channels > 0)
	{
		int i;
		for (i = 0; ppchan->outstandingpackets[i] < ppchan->maxpackets - 1; ++i)
		{
			if (i >= ppchan->sampleformat.channels)
        		goto abort;
		}
		packethandle = -13;
	}
	else
	{
	abort:
		pentry = &ppchan->pe[ppchan->submitpacket];
		pentry->numframes = psp->numframes;
		pentry->continuation = psp->continuation;
		pentry->handle = ppchan->lastpackethandle;
		if (ppchan->sampleformat.channels > 0)
		{
			int i;
			for (i = 0; i < ppchan->sampleformat.channels; ++i)
			{
				pentry->psamples[i] = psp->psamples[i];
				++ppchan->outstandingpackets[i];
			}
		}
		ppchan->outstandingframes += psp->numframes;
		packethandle = ppchan->lastpackethandle;
		ppchan->lastpackethandle = packethandle + 1;
		if (++ppchan->submitpacket >= ppchan->maxpackets)
			ppchan->submitpacket = 0;
	}

	return packethandle;
}