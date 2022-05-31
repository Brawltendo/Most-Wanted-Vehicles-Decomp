#pragma once
#include "speedcommon.h"
#include "math/vector.h"
#include "Speed/indep/Libs/Support/Utility/UCOM.h"


SPEED_INTERFACE ISuspension : public UTL::COM::IUnknown
{
public:
	virtual void _PADDING();
    virtual float GetWheelTraction(uint32_t wheelIndex);
    virtual uint32_t GetNumWheels() { return 4; }
    virtual UMath::Vector3* GetWheelPos(uint32_t wheelIndex);
    virtual UMath::Vector3* GetWheelLocalPos(uint32_t wheelIndex);
    virtual UMath::Vector3* GetWheelCenterPos(uint32_t wheelIndex);
    virtual float GetWheelLoad(uint32_t wheelIndex);
	virtual void ApplyVehicleEntryForces(bool enteringVehicle, const UMath::Vector3& pos, bool calledfromEvent);
    virtual float GetWheelRoadHeight(uint32_t wheelIndex);
    virtual bool IsWheelOnGround(uint32_t wheelIndex);
    virtual float GetCompression(uint32_t wheelIndex);
    virtual float GuessCompression(uint32_t wheelIndex, float downforce);
    virtual float GetWheelSlip(uint32_t wheelIndex);
    virtual float GetToleratedSlip(uint32_t wheelIndex);
    virtual float GetWheelSkid(uint32_t wheelIndex);
    virtual float GetWheelSlipAngle(uint32_t wheelIndex);
    virtual UMath::Vector4* GetWheelRoadNormal(uint32_t wheelIndex);
    virtual void* GetWheelRoadSurface(uint32_t wheelIndex);
    virtual UMath::Vector3* GetWheelVelocity(uint32_t wheelIndex);
    virtual uint32_t GetNumWheelsOnGround();
    virtual float GetWheelAngularVelocity(uint32_t wheelIndex);
    virtual void SetWheelAngularVelocity(uint32_t wheelIndex, float av);
    virtual float GetWheelSteer(uint32_t wheelIndex);
    virtual float GetRideHeight(uint32_t wheelIndex);
    virtual float GetWheelRadius(uint32_t wheelIndex);
    virtual float GetMaxSteering();
};