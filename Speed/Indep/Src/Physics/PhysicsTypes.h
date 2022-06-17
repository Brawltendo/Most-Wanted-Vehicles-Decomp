#pragma once


struct AxlePair
{
    union {
        struct {
            float Front;
            float Rear;
        };
        float Pair[2];
    };

	float At(int index) { return Pair[index]; }
	
};

enum DriverClass
{
  DRIVER_HUMAN,
  DRIVER_TRAFFIC,
  DRIVER_COP,
  DRIVER_RACER,
  DRIVER_NONE,
  DRIVER_NIS,
  DRIVER_REMOTE
};

enum DriverStyle
{
	STYLE_RACING,
	STYLE_DRAG
};

enum eInductionType
{
	INDUCTION_NONE,
	INDUCTION_TURBO_CHARGER,
	INDUCTION_SUPER_CHARGER
};

enum eTireIdx
{
	// The front left wheel
	TIRE_FL,
	// The front right wheel
	TIRE_FR,
	// The number of wheels on the front axle
	TIRE_MAX_FRONT = 2,
	// The rear right wheel
	TIRE_RR = 2,
	// The rear left wheel
	TIRE_RL,
	// The total number of wheels
	TIRE_MAX,
};

enum GearID
{
	G_REVERSE,
	G_NEUTRAL,
	G_FIRST,
	G_SECOND,
	G_THIRD,
	G_FOURTH,
	G_FIFTH,
	G_SIXTH,
	G_SEVENTH,
	G_EIGHTH,
	G_MAX
};

enum ShiftPotential
{
	SHIFT_POTENTIAL_NONE,
	SHIFT_POTENTIAL_DOWN,
	SHIFT_POTENTIAL_UP,
	SHIFT_POTENTIAL_GOOD,
	SHIFT_POTENTIAL_PERFECT,
	SHIFT_POTENTIAL_MISS
};

enum ShiftStatus
{
	SHIFT_STATUS_NONE,
	SHIFT_STATUS_NORMAL,
	SHIFT_STATUS_GOOD,
	SHIFT_STATUS_PERFECT,
	SHIFT_STATUS_MISSED
};
