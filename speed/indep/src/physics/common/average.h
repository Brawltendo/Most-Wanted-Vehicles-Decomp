// A group of classes used to return the average and total values of an input buffer

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
    float fTotal;
    float fAverage;
    float *pData;
    float SmallDataBuffer[5];
};

class AverageWindow : public Average
{
public:
    float fTimeWindow;
    int iOldestValue;
    float *pTimeData;
    uint32_t AllocSize;
};