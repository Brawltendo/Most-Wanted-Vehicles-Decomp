#include "packages\snd\9\source\library\cmn\ssys.h"


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

char mutexLocked = 0;
void SNDSYS_entercritical()
{
	Snd::gMutexLockFn();
	++mutexLocked;
}

void SNDSYS_leavecritical()
{
	--mutexLocked;
	Snd::gMutexUnlockFn();
}

#ifdef __cplusplus
}
#endif