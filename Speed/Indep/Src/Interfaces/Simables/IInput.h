#pragma once
#include "speedcommon.h"
#include "Speed/Indep/Libs/Support/Utility/UCOM.h"


class InputControls;

SPEED_INTERFACE IInput : public UTL::COM::IUnknown
{
public:
    virtual void ClearInput();
    virtual class InputControls& GetControls();
    virtual float GetControlHandBrake();
    virtual bool GetControlActionButton();
    virtual void SetControlGas(float);
    virtual void SetControlBrake(float);
    virtual void SetControlNOS(bool);
    virtual void SetControlHandBrake(float);
    virtual void SetControlSteering(float);
    virtual void SetControlActionButton(bool);
    virtual void SetControlSteeringVertical(float);
    virtual void SetControlBanking(float);
    virtual float GetControlBanking();
    virtual bool IsLookBackButtonPressed();
    virtual bool IsPullBackButtonPressed();
    virtual bool IsAutomaticShift();
};