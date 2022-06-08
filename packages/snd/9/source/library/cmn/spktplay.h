#pragma once
#include "packages/snd/9/source/library/cmn/sndcmn.h"


int SNDPKTPLAY_start(int packetinstancehandle, struct SNDSAMPLEFORMAT* pssf, struct SNDSAMPLEATTR* pssa, struct SNDPLAYOPTS* pspo);
int SNDPKTPLAY_submit(int packetinstancehandle, struct SNDPACKET* psp);
int SNDPKTPLAY_stop(int packetinstancehandle);