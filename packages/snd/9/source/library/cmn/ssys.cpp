#include "packages/snd/9/source/library/cmn/ssys.h"
#include "packages/snd/9/source/library/cmn/sndgs.h"


void Snd::DefaultMutexLockFn()
{
	SNDI_mutexlock();
}

void Snd::DefaultMutexUnlockFn()
{
	SNDI_mutexunlock();
}

// bridge functions to C
#ifdef __cplusplus
extern "C"
{
#endif

void SNDSYS_entercritical()
{
	Snd::gMutexLockFn();
	++sndgs.incritical;
}

void SNDSYS_leavecritical()
{
	--sndgs.incritical;
	Snd::gMutexUnlockFn();
}

#ifdef __cplusplus
}
#endif