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

#ifdef __cplusplus
extern "C"
{
#endif

void SNDI_mutexlock();
void SNDI_mutexunlock();

#ifdef __cplusplus
}
#endif

typedef struct SNDSYSCAP
{
	unsigned short outputratemin;
	unsigned short outputratemax;
	unsigned char outputchannelsmin;
	unsigned char outputchannelsmax;
	unsigned char voicesmaincpumax;
	unsigned char voicesiopcpumax;
	unsigned char voicesspumax;
	unsigned char voicesds2dhwmax;
	unsigned char voicesdspmax;
	unsigned char eax;
	unsigned char numrendermodes;
	unsigned short rendermode[6];
};

typedef struct SNDSYSSET
{
	int packetbufsize;
	unsigned int randomseed;
	unsigned short maxbanks;
	unsigned short outputrate;
	unsigned short outputrateiopcpu;
	unsigned char voicesmaincpu;
	unsigned char voicesiopcpu;
	unsigned char voicesspu;
	unsigned char voicesds2dhw;
	unsigned char voicesdsp;
	char compatibility;
	float updateperiod;
	unsigned char stealequalpriorityvoices;
	unsigned char usesse;
	unsigned char useeax;
	unsigned char loadiopmodules;
	unsigned char resamplequality;
	unsigned char iopcpuresamplequality;
	unsigned char routemaincpufxtoiopcpu;
	unsigned char routespufxtoiopcpu;
	unsigned char emulationsubtype;
	unsigned char initaram;
	unsigned char maxstreams;
	unsigned char outputchannels;
	unsigned char outputlfe;
	unsigned char outputspdifmode;
	unsigned char dtsquality;
	unsigned char sndheapthreshold;
	unsigned char numrendermodes;
	unsigned char useASIOdriver;
	unsigned short rendermode[6];
	unsigned int arampooladdr;
	int arampoolsize;
	void* fastramaddr;
	int fastramsize;
	char* iopmodulepath;
	char* ASIOconfig;
	void* snd2asio;
	unsigned short virtualspkrcfg3d[6][6];
};

typedef struct SNDSYSVEC
{
	unsigned int (*aramalloc)(unsigned int);
	void (*aramfree)(unsigned int);
	void (*abortmsg)(char*);
	void (*profileenter)(const char*);
	void (*profileleave)(const char*);
};

typedef struct SNDSYSOPTS
{
	struct SNDSYSCAP cap;
	struct SNDSYSSET set;
	struct SNDSYSVEC vec;
};