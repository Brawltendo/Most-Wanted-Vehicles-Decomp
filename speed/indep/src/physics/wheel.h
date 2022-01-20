#include "generated/attribsys/classes/simsurface.h"
#include "math/matrix.h"
#include "math/vector.h"
#include "world/wworldpos.h"

__declspec(align(16)) struct Wheel
{
	bool UpdatePosition(const UMath::Vector3& body_av, const UMath::Vector3& body_lv, const UMath::Matrix4& body_matrix, const UMath::Vector3& cog, float dT, float wheel_radius, bool usecache, const void* collider, float vehicle_height);

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
    float mCompression;
    UMath::Vector3 mWorldArm;
	int pad;
    UMath::Vector3 mVelocity;
	int pad2;
    Attrib::Gen::simsurface mSurface;
    float mSurfaceStick;
    void *mIntegral;
};
//const int offset = offsetof(Wheel, mNormal);