#pragma once


struct SNDSAMPLEFORMAT
{
	unsigned short samplerate;
	unsigned char channels;
	unsigned char samplerep;
};

struct SNDPACKETENTRY
{
	int handle;
	unsigned int numframes : 31;
	unsigned int continuation : 1;
	void* psamples[6];
};

struct SNDPACKETCHAN
{
	volatile int shandle;
	volatile int lastpackethandle;
	volatile unsigned int masterReleasePacket;
	volatile unsigned int masterCurrentPacket;
	volatile unsigned int numPacketsProcessed[6];
	volatile short playpacket[6];
	volatile short outstandingpackets[6];
	volatile short maxpackets;
	volatile signed char masterpacketchan;
	char pad;
	volatile short releasepacket;
	volatile short submitpacket;
	volatile unsigned int outstandingframes;
	volatile unsigned int outstandingplatformframes;
	void* pplatformmem;
	void (*preleasefunc)(void*, void*);
	void (*pframesfunc)(int, int, void*);
	void* pclientdata;
	struct SNDSAMPLEFORMAT sampleformat;
	void* ptsdata[6];
	struct SNDPACKETENTRY pe[];
};

struct SNDSAMPLEATTR
{
	short detune;
	signed char priority;
	signed char vol;
	signed char pan;
	signed char fxlevel0;
	signed char bendrange;
	signed char platformver;
	unsigned short rendermode;
	char padchar[2];
	unsigned short azimuth[6];
	void* ptsdata[6];
	int tsdatasize[6];
	void* puserdata[4];
	int userdatasize[4];
};

struct SNDPLAYOPTS
{
	signed char vol;
	signed char bend;
	signed char keynum;
	signed char velocity;
	signed char drylevel;
	signed char fxlevel0;
	char pad[2];
	unsigned short azimuth;
	short elevation;
	unsigned short pitchmult;
	unsigned short timemult;
	unsigned short tempomult;
	unsigned short pad2;
	unsigned short lowpasscutoff;
	unsigned short highpasscutoff;
};

struct SNDPACKET
{
	struct SNDSAMPLEFORMAT ssf;
	unsigned int numframes : 31;
	unsigned int continuation : 1;
	int sizeofsamples;
	void* psamples[6];
};

typedef struct SNDUSERDATACBINFO
{
	int type;
	void* pdata;
	int size;
	int shandle;
	int sndstrmrequest;
};

typedef struct TAGGEDPATCH
{
	short id;
	unsigned char platform;
	unsigned char flags;
	int hdrsize;
};

typedef struct BANKVER5
{
	int id;
	unsigned char ver;
	char resolved;
	unsigned short numpatches;
	int hdrsize;
	int spusize;
	int iopcpusize;
	struct TAGGEDPATCH patch[512];
};

typedef struct BANKLIST
{
	struct BANKVER5* phdr;
	void* pspuram;
	signed char locked;
	char pad[3];
};

struct SNDPACKETCHAN* spktchannels[31];