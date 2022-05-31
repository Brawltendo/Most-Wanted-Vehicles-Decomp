#pragma once
#include "speedcommon.h"
//#include "interfaces/simables/irigidbody.h"
#include "math/matrix.h"
#include "math/vector.h"

SPEED_INTERFACE ISimable
{
public:
	virtual void _PADDING();
    virtual int GetSimableType();
    virtual void Kill();
    virtual void Attach(void*);
    virtual void Detach(void*);
    virtual void* GetAttachments();
    virtual void AttachEntity(void*);
    virtual void DetachEntity(void*);
    virtual class IPlayer* GetPlayer();
    virtual bool IsPlayer();
    virtual bool IsOwnedByPlayer();
    virtual void* GetEntity();
    virtual void* DebugObject();
    virtual int GetOwnerHandle();
    virtual void* GetOwner();
    virtual bool IsOwnedBy(void*);
    virtual void SetOwnerObject(void*);
    virtual void* GetAttributes();
    virtual void* GetWPos();
    virtual void* GetWPos() const;
    virtual class IRigidBody* GetRigidBody();
    virtual class IRigidBody* GetRigidBody() const;
    virtual bool IsRigidBodySimple();
    virtual bool IsRigidBodyComplex();
    virtual UMath::Vector3& GetPosition();
	virtual void GetTransform(UMath::Matrix4& dest);
	virtual void GetLinearVelocity(UMath::Vector3&);
    virtual void GetAngularVelocity(UMath::Vector3&);
    virtual int GetWorldID();
    virtual void* GetEventSequencer();
    virtual void ProcessStimulus();
    virtual void* GetModel();
    virtual void* GetModel() const;
    virtual void SetCausality(void*, float);
    virtual int GetCausality();
    virtual float GetCausalityTime();
};