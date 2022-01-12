#pragma once
//#include "speedcommon.h"
#include "math/matrix.h"
#include "math/vector.h"

class IPlayer
{
public:
	virtual void _PADDING();
    virtual void* GetSimable();
    virtual bool IsLocal();
    virtual UMath::Vector3& GetPosition();
    virtual void SetPosition(const UMath::Vector3&);
    virtual void* GetSettings();
    virtual void SetSettings(int);
    virtual int GetSettingsIndex();
    virtual void* GetHud();
    virtual void SetHud(int);
    virtual void SetRenderPort(int);
    virtual int GetRenderPort();
    virtual void SetControllerPort(int);
    virtual int GetControllerPort();
    virtual void* GetFFB();
    virtual void* GetSteeringDevice();
    virtual bool InGameBreaker();
    virtual bool CanRechargeNOS();
    virtual void ResetGameBreaker(bool);
    virtual void ChargeGameBreaker(float);
    virtual bool ToggleGameBreaker();
};