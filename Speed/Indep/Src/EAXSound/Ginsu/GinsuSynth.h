#pragma once


class GinsuSynthesis
{
public:
	//GinsuSynthesis();
	virtual ~GinsuSynthesis();
	static void PacketReleaseCallback(void* samples, void* clientdata);
	void HandlePacketRelease(short* samples);
	bool SetSynthData(class GinsuSynthData& data);
	int StartSynthesis(float startFreq);
	bool UpdateFrequency(float targetFreq, float latency);
	float GetCurrentPitch();
	void StopSynthesis();

private:
	short* mPacketData[2];
	int mMaxPacketSize;
	int mNoJumpSize;
	int mPacketSize;
	int mOverlapSize;
	int mPacketHandle;
	int mSndHandle;
	int mSampleRate;
	class GinsuSynthData* mSynthData;
	int mPlaybackPos;
	int mCurrentPos;
	int mTargetPos;
	int mPacketCountdown;
	int mNoJumpRemaining;
	float mCurrentCycle;
};