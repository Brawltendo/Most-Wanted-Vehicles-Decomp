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
