#include "speedcommon.h"
#include "math/vector.h"
#include "physics/physicstunings.h"
#include "physics/physicstypes.h"

class IVehicle
{
public:
	virtual void _PADDING();
    virtual void* GetSimable() const;
    virtual void* GetSimable();
    virtual UMath::Vector3& GetPosition();
    virtual void SetBehaviorOverride(uint32_t, uint32_t);
    virtual void RemoveBehaviorOverride(uint32_t);
    virtual void CommitBehaviorOverrides();
    virtual void SetStaging(bool staging);
    virtual bool IsStaging();
    virtual void Launch();
	virtual float GetPerfectLaunch();
	virtual void SetDriverStyle(DriverStyle style);
	virtual DriverStyle GetDriverStyle();
	virtual void SetPhysicsMode(enum PhysicsMode mode);
	virtual enum PhysicsMode GetPhysicsMode();
	virtual int GetModelType();
	virtual bool IsSpooled();
	virtual int GetVehicleClass();
	virtual int GetVehicleAttributes();
	virtual char* GetVehicleName();
	virtual uint32_t GetVehicleKey();
	virtual void SetDriverClass(DriverClass dc);
	virtual DriverClass GetDriverClass();
	virtual bool IsLoading();
	virtual float GetOffscreenTime();
	virtual float GetOnScreenTime();
	virtual void SetVehicleOnGround(const UMath::Vector3&, const UMath::Vector3&);
	virtual void ForceStopOn(bool);
	virtual void ForceStopOff(bool);
	virtual bool GetForceStop();
	virtual bool InShock();
	virtual bool IsDestroyed();
	virtual void Activate();
	virtual void Deactivate();
	virtual bool IsActive();
	virtual float GetSpeedometer();
	virtual float GetSpeed();
	virtual void SetSpeed(float speed);
	virtual float GetAbsoluteSpeed();
	virtual bool IsGlareOn(int id);
	virtual void GlareOn(int id);
	virtual void GlareOff(int id);
	virtual bool IsCollidingWithSoftBarrier();
	virtual void* GetAIVehiclePtr();
	virtual float GetSlipAngle();
	virtual UMath::Vector3& GetLocalVelocity();
	virtual void ComputeHeading(UMath::Vector3*);
	virtual bool IsAnimating();
	virtual void SetAnimating(bool animating);
	virtual bool IsOffWorld();
	virtual void* GetCustomizations();
	virtual Physics::Tunings* GetTunings();
	virtual void SetTunings(const Physics::Tunings&);
	virtual void* GetPerformance(void*);
	virtual char* GetCacheName();
};