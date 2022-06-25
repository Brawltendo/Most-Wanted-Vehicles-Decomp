#pragma once
#include "speedcommon.h"
#include "math/matrix.h"
#include "math/vector.h"
#include "Speed/Indep/Libs/Support/Utility/UCOM.h"
#include "Speed/Indep/Libs/Support/Utility/UCollections.h"


struct HSIMABLE__
{
	int unused;
};

SPEED_INTERFACE ISimable : public UTL::COM::IUnknown, 
						   public UTL::Collections::Instanceable<HSIMABLE__*, ISimable>
{
public:
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
    virtual ISimable* GetOwner();
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