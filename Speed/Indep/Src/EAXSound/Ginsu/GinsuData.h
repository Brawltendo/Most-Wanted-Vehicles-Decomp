#pragma once
#include "speedcommon.h"


// ---- Decoder consts ----

// The amount of samples per block
const int BLOCKSIZE_SAMPLES = 32;
// The size of each block in bytes
const int BLOCKSIZE_BYTES = 19;

// This struct maps directly to the data found inside of the Ginsu file
struct GinsuDataLayout
{
	// Valid Ginsu ID should be "Gnsu"
	char id[4];

	// Valid Ginsu version should be "20" (v2.0)
	char ver[2];

	int16_t flags;

	// The minimum audio RPM for this engine ramp
	float minFrequency;

	// The maximum audio RPM for this engine ramp
	float maxFrequency;

	// The number of segments in the data
	int segCount;

	// The number of cycles in the data
	int cycleCount;

	// The number of samples in the data
	int sampleCount;

	// The audio sample rate for this data
	int sampleRate;

	uint8_t data[];
};

class GinsuSynthData
{
public:
	GinsuSynthData();

	// Getters
	float GetMinFrequency() { return mMinFrequency; }
	float GetMaxFrequency() { return mMaxFrequency; }
	int GetSampleRate() { return mSampleRate; }

	virtual ~GinsuSynthData();
	void DecodeBlock(int block);
	bool BindToData(void* ptr);
	int FrequencyToSample(float freq);
	int CycleToSample(float cycle);
	float CyclePeriod(float cycle);
	float SampleToCycle(int sample);
	bool GetSamples(int startSample, int numSamples, short* dest);

private:
	float mMinFrequency;
	float mMaxFrequency;
	int mSegCount;
	int mCycleCount;
	int mSampleCount;
	int mSampleRate;
	int *mFreqPos;
	int *mCyclePos;

	// A pointer to the sample data from the loaded Ginsu file
	uint8_t *mSampleData;

	float mMinPeriod;

	// The current sample chunk to be played
	float mSample[BLOCKSIZE_SAMPLES];

	// The index of the current sample block
	int mCurrentBlock;
};
