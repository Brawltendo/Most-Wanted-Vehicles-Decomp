#include "physics/wheel.h"


bool Wheel::UpdatePosition(const UMath::Vector3& body_av, const UMath::Vector3& body_lv, const UMath::Matrix4& body_matrix, const UMath::Vector3& cog, float dT, float wheel_radius, bool usecache, const void* collider, float vehicle_height)
{
	UMath::Rotate(mLocalArm, body_matrix, mWorldArm);
	mPosition = UMath::Vector4To3(body_matrix.v3) + mWorldArm;
	UMath::Vector3 velocity;
	UMath::Cross(body_av, mWorldArm - cog, velocity);
	velocity += body_lv;
	mVelocity = velocity;
	mPosition = mWorldArm + UMath::Vector4To3(body_matrix.v3);

	float velPerTick = UMath::Max(-(velocity.y * dT), 0.f);
	float yoffset = Sqrt(velocity.z * velocity.z + velocity.x + velocity.x) * dT + velPerTick + wheel_radius;
	yoffset = UMath::Min(yoffset, vehicle_height * 0.5f);
	mWorldPos.fYOffset = yoffset;
	bool quitIfOnSameFace = mCompression > 0.f && usecache;
	return true;
}