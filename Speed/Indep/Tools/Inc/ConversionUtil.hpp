/*
 *  A collection of functions to convert to and from various units.
 */
#pragma once

// MATCHING
float MPS2MPH(const float _mps_)
{
	return _mps_ * 2.23699f;
}

// MATCHING
float MPH2MPS(const float _mph_)
{
	return _mph_ * 0.44703001f;
}

// MATCHING
float DEG2ANGLE(const float _deg_)
{
	return _deg_ / 360.f;
}

// MATCHING
float ANGLE2DEG(const float _arc_)
{
	return _arc_ * 360.f;
}

// MATCHING
float RAD2ANGLE(const float _rad_){
	return _rad_ / TWO_PI;
}

// MATCHING
float ANGLE2RAD(const float _arc_)
{
	return _arc_ * TWO_PI;
}

// MATCHING
float DEG2RAD(const float _deg_)
{
	return _deg_ * (180.f / PI);
}

// MATCHING
float RAD2DEG(const float _rad_)
{
	return _rad_ * (PI / 180.f);
}

// MATCHING
float INCH2METERS(const float _inches_)
{
	return _inches_ * 0.0254f;
}

// MATCHING
float RPS2RPM(const float _rps_)
{
	return _rps_ * 9.5492964f;
}

// MATCHING
float RPM2RPS(const float _rpm_)
{
	return _rpm_ / 9.5492964f;
}

// MATCHING
float LBIN2NM(const float _lbin_)
{
	return _lbin_ * 175.1268f;
}

// MATCHING
float NM2LBIN(const float _nm_)
{
	return _nm_ / 175.1268f;
}