// A group of classes used to return the average and total values of an input buffer
#include "speedcommon.h"

class AverageBase
{
public:
	uint8_t nSize;
    uint8_t nSlots;
    uint8_t nSamples;
    uint8_t nCurrentSlot;
};

class Average : public AverageBase
{
public:
	virtual void Recalculate();
	void Record(const float fValue);
	float GetValue() { return fAverage; }
	float GetTotal() { return fTotal; }

    float fTotal;
    float fAverage;
    float *pData;
    float SmallDataBuffer[5];
};

class AverageWindow : public Average
{
public:
	float GetOldestValue() { return pData[iOldestValue]; }
	float GetOldestTimeValue() { return pTimeData[iOldestValue]; }
	void Record(const float fValue, const float fTimeNow);

    float fTimeWindow;
    int iOldestValue;
    float *pTimeData;
    uint32_t AllocSize;
};