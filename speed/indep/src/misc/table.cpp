#include "misc/table.hpp"

// MATCHING
float Table::GetValue(float input)
{
    const int length = NumEntries;
    const float index = (input - MinArg) * IndexMultiplier;
    const int i = (int)index;
    
    if (i < 0 || index < 0.f)
        return pTable[0];
    if (i >= (length - 1))
        return pTable[length - 1];

    float ind = i;
    if (ind > index)
        ind -= 1.f;
    
    float* curVal = &pTable[i];
    float delta = index - ind;
    return (1.f - delta) * curVal[0] + delta * curVal[1];
}