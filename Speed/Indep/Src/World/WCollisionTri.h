#include "math/vector.h"
#include "world/wcollision.h"

__declspec(align(1)) struct WCollisionTri
{
    UMath::Vector3 fPt0;
    struct SimSurface *fSurfaceRef;
    UMath::Vector3 fPt1;
    uint32_t fFlags;
    UMath::Vector3 fPt2;
    WSurface fSurface;
};
const int offset = offsetof(WCollisionTri, fSurfaceRef);