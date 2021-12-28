#include "speedcommon.h"
#include "math/matrix.h"
#include "math/vector.h"
#include "physics/physicstunings.h"
#include "physics/physicstypes.h"

class IRigidBody
{
public:
	virtual void _PADDING();
    virtual void* GetOwner();
    virtual bool IsSimple();
    virtual int GetIndex();
    virtual int GetSimableType();
    virtual float GetRadius();
    virtual float GetMass();
    virtual float GetOOMass();
    virtual UMath::Vector3& GetPosition();
    virtual UMath::Vector3& GetLinearVelocity();
    virtual UMath::Vector3& GetAngularVelocity();
    virtual float GetSpeed();
    virtual float GetSpeedXZ();
	virtual void GetForwardVector(UMath::Vector3& dest);
	virtual void GetRightVector(UMath::Vector3& dest);
	virtual void GetUpVector(UMath::Vector3& dest);
	virtual void GetMatrix4(UMath::Matrix4& dest);
	virtual UMath::Vector4& GetOrientation();
	virtual UMath::Vector3& GetDimension();
	virtual void GetDimension(UMath::Vector3& dest);
	virtual int GetTriggerFlags();
	virtual struct WCollider& GetWCollider();
	virtual void GetPointVelocity(const UMath::Vector3& pt, UMath::Vector3& vel);
	virtual void SetPosition(const UMath::Vector3& pos);
	virtual void SetLinearVelocity(const UMath::Vector3& lv);
	virtual void SetAngularVelocity(const UMath::Vector3& av);
	virtual void SetRadius(float r);
	virtual void SetMass(float m);
	virtual void SetOrientation(const UMath::Vector4& o);
	virtual void SetOrientation(const UMath::Matrix4& o);
	virtual void ModifyXPos(float val);
	virtual void ModifyYPos(float val);
	virtual void ModifyZPos(float val);
	virtual void Resolve(const UMath::Vector3& force, const UMath::Vector3& torque);
	virtual void ResolveForceAtPoint(const UMath::Vector3& force, const UMath::Vector3& pt);
	virtual void ResolveForce(const UMath::Vector3& force);
	virtual void ResolveTorqueAtPoint(const UMath::Vector3& torque, const UMath::Vector3& pt);
	virtual void ResolveTorque(const UMath::Vector3& torque);
	virtual void PlaceObject(const UMath::Vector4&, const UMath::Vector3&);
	virtual void Accelerate(const UMath::Vector3&, float);
	virtual void ConvertLocalToWorld(UMath::Vector3&, bool);
	virtual void ConvertWorldToLocal(UMath::Vector3&, bool);
	virtual void Debug();
};