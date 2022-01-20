#pragma once
//#include "speedcommon.h"
#include "math/matrix.h"
#include "math/vector.h"


enum eControllerConfig
{
	CC_CONFIG_1,
	CC_CONFIG_2,
	CC_CONFIG_3,
	CC_CONFIG_4,
	CC_CONFIG_5,
	NUM_CONTROLLER_CONFIGS,
	MIN_CONFIG = 0,
	MAX_CONFIG = 4,
};

struct PlayerSettings
{
	bool GaugesOn;
	bool PositionOn;
	bool LapInfoOn;
	bool ScoreOn;
	bool Rumble;
	bool LeaderboardOn;
	bool TransmissionPromptOn;
	bool DriveWithAnalog;
	eControllerConfig Config;
	enum ePlayerSettingsCameras
	{
		PSC_BUMPER,
		PSC_HOOD,
		PSC_CLOSE,
		PSC_FAR,
		PSC_SUPER_FAR,
		PSC_DRIFT,
		PSC_PURSUIT,
		NUM_CAMERAS_IN_OPTIONS,
		PSC_DEFAULT = 2,
	} CurCam;
	uint8_t SplitTimeType;
	uint8_t Transmission;
	uint8_t Handling;
};

class IPlayer
{
public:
	virtual void _PADDING();
    virtual void* GetSimable();
    virtual bool IsLocal();
    virtual UMath::Vector3& GetPosition();
    virtual void SetPosition(const UMath::Vector3&);
    virtual PlayerSettings* GetSettings();
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