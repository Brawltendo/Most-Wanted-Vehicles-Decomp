#pragma once
#include "packages/snd/9/source/library/cmn/ssys.h"


typedef struct FXVOLUME
{
	float fxLevel;
};

typedef struct ENVELOPE
{
	int duration;
	int targetvol;
};

typedef struct SCALINGTABLE
{
	signed char xlate[128];
};

typedef struct CHANPUB
{
	int handle;
	short voices[6];
	unsigned short patnum;
	short bhandle;
	unsigned int frames;
	int sustainend;
	unsigned short azimuth;
	short elevation;
	unsigned short samplerate;
	unsigned char samplerep;
	unsigned char numchan;
	unsigned short rendermode;
	unsigned char patchkey;
	unsigned char ismaster;
	short masterchan;
	unsigned char priority;
	signed char builtinvol;
	unsigned int timestamp;
	float fadePer100Hz;
	float fadeTargetVol;
	float programmedVol;
	int envpertick;
	int envvol;
	int envduration;
	float finalvol;
	unsigned short builtinazimuth;
	unsigned char isLfe;
	char pad[1];
	unsigned short azimuthOffsets[6];
	signed char numenvelopes;
	signed char curenvelope;
	signed char releaseenvelope;
	signed char drylevel;
	signed char progfxlevel;
	char pad1[3];
	struct FXVOLUME* pFxVolume;
	signed char pitchbend;
	unsigned char vollfolength;
	unsigned char pitchlfolength;
	unsigned char curvollfoentry;
	unsigned char curpitchlfoentry;
	signed char status;
	unsigned short timemult;
	struct ENVELOPE* paenvelope;
	struct SCALINGTABLE* pvoltable;
	struct SCALINGTABLE* pbendtable;
	struct SCALINGTABLE* pvollfo;
	struct SCALINGTABLE* ppitchlfo;
	short pitchlfodepth;
	short bendrange;
	short initialdetune;
	unsigned short detunepitch;
	unsigned short progpitch;
	unsigned short finalpitch;
};

typedef struct SNDGLOBALSTATE
{
	struct SNDSYSOPTS sso;
	struct SNDSYSSET prevset;
	signed char installed;
	volatile signed char incritical;
	signed char numserverclients100hz;
	signed char numserverclients;
	short voicestotal;
	signed char numuserdataclients;
	unsigned char cputypes;
	volatile unsigned int audiotick;
	void (*serverclient100hz[6])();
	void (*serverclient[6])();
	void (*userdataclient[4])(struct SNDUSERDATACBINFO*);
	void (*aemsrestore)();
	int (*aemsstopmodulebanks)();
	int (*aemsstreamrestore)();
	int (*eventrestore)();
	int (*bankremove)();
	int (*midirestore)();
	int (*streamrestore)();
	int (*aemsstreampurge)();
	struct CHANPUB* chan;
	struct BANKLIST* banklist;
	struct SNDMEMSTATE* mm;
	unsigned short srcchancfg3d[6][6];
	void (*profileenter)();
	void (*profileleave)();
	volatile unsigned int servertick;
	volatile short currenttimerhz;
	volatile short servicesskipped;
	char aborted;
	char logmixing;
	void (*logcallbackfn)(unsigned short, void*);
	char debugpad[2];
};

struct SNDGLOBALSTATE sndgs;