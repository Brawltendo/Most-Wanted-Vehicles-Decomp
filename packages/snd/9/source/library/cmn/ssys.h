#pragma once


#ifdef __cplusplus
namespace Snd
{

void DefaultMutexLockFn();
void DefaultMutexUnlockFn();
void (*gMutexLockFn)()   = DefaultMutexLockFn;
void (*gMutexUnlockFn)() = DefaultMutexUnlockFn;

} // namespace Snd
#endif

// bridge functions for C
#ifdef __cplusplus
extern "C"
{
#endif

void SNDSYS_entercritical();
void SNDSYS_leavecritical();

#ifdef __cplusplus
}
#endif

extern "C"
{

void SNDI_mutexlock();
void SNDI_mutexunlock();

}

