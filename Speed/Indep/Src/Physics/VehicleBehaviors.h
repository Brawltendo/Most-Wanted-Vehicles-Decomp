#pragma once
#include "Physics/Behavior.h"


class VehicleBehavior : public Behavior
{
public:
	class IVehicle* GetVehicle() { return mVehicle; }

private:
	class IVehicle* mVehicle;
};