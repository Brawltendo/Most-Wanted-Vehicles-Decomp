#include "generated/attribsys/classes/simsurface.h"
#include "math/vector.h"
#include "world/wworldpos.h"

__declspec(align(16)) struct Wheel
{
    WWorldPos mWorldPos;
    UMath::Vector4 mNormal;
    UMath::Vector3 mPosition;
    enum eWheelFlags 
    {
        WF_SMOOTHING
    } mFlags;
    UMath::Vector3 mForce;
    float mAirTime;
    UMath::Vector3 mLocalArm;
    float compression;
    UMath::Vector3 mWorldArm;
    UMath::Vector3 mVelocity;
    Attrib::Gen::simsurface mSurface;
    float mSurfaceStick;
    void *mIntegral;
};
//const int offset = offsetof(Wheel, mNormal);