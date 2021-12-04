#include "world/wcollisiontri.h"

struct WWorldPos
{
    WCollisionTri fFace;
    union
    {
        bool fFaceValid;
        uint32_t fMissCount;
        uint32_t fUsageCount;
    };
    float fYOffset;
    struct SimSurface *fSurface;
};
//const int offset = offsetof(WWorldPos, fYOffset);