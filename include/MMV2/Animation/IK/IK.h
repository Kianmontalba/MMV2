#pragma once
#ifndef MMV2_IK_H
#define MMV2_IK_H

#include "Config.h"
#include "Vec3.h"
#include "Quat.h"
#include "Transform.h"
#include "Pose.h"
#include "Bone.h"
#include "Vector.h"

MMV2_NAMESPACE_BEGIN

enum class IKSolverType : uint8 {
    CCD = 0,
    FABRIK = 1,
    Jacobian = 2,
    Analytical = 3,
    TwoBone = 4,
    Limb = 5,
    Spine = 6,
    Aim = 7,
    LookAt = 8,
    Foot = 9,
    Hand = 10
};

struct IKSolverSettings {
    IKSolverType type;
    int32 maxIterations;
    float32 tolerance;
    float32 damping;
    float32 maxAngleChange;
    bool enableConstraints;
    bool enableStretch;
    float32 stretchRatio;

    IKSolverSettings()
        : type(IKSolverType::CCD), maxIterations(20), tolerance(0.001f),
          damping(0.1f), maxAngleChange(30.0f * MMV2_DEG2RAD),
          enableConstraints(true), enableStretch(false), stretchRatio(1.0f) {}
};

struct JointConstraint {
    float32 minAngleX;
    float32 maxAngleX;
    float32 minAngleY;
    float32 maxAngleY;
    float32 minAngleZ;
    float32 maxAngleZ;
    bool enabled;

    JointConstraint()
        : minAngleX(-180.0f * MMV2_DEG2RAD), maxAngleX(180.0f * MMV2_DEG2RAD),
          minAngleY(-180.0f * MMV2_DEG2RAD), maxAngleY(180.0f * MMV2_DEG2RAD),
          minAngleZ(-180.0f * MMV2_DEG2RAD), maxAngleZ(180.0f * MMV2_DEG2RAD),
          enabled(true) {}
};

struct IKChain {
    Vector<int32> boneIndices;
    Vector<float32> boneLengths;
    Vector<JointConstraint> constraints;
    Vector<Quat> initialRotations;
    float32 totalLength;
    int32 effectorIndex;
    int32 rootIndex;

    IKChain() : totalLength(0.0f), effectorIndex(-1), rootIndex(-1) {}

    void AddBone(int32 index, float32 length, const JointConstraint& constraint = JointConstraint());
    void ComputeLengths(const BoneHierarchy& hierarchy);
    bool IsValid() const;
};

class MMV2_API IKSolver {
public:
    virtual ~IKSolver() = default;
    virtual IKSolverType GetType() const = 0;
    virtual bool Solve(const Vec3& target, Pose& pose, const BoneHierarchy& hierarchy,
                       const IKChain& chain, const IKSolverSettings& settings) = 0;
    virtual bool SolveWithRotation(const Vec3& target, const Quat& targetRotation,
                                   Pose& pose, const BoneHierarchy& hierarchy,
                                   const IKChain& chain, const IKSolverSettings& settings) = 0;
};

class MMV2_API CCDSolver : public IKSolver {
public:
    IKSolverType GetType() const override { return IKSolverType::CCD; }
    bool Solve(const Vec3& target, Pose& pose, const BoneHierarchy& hierarchy,
               const IKChain& chain, const IKSolverSettings& settings) override;
    bool SolveWithRotation(const Vec3& target, const Quat& targetRotation,
                           Pose& pose, const BoneHierarchy& hierarchy,
                           const IKChain& chain, const IKSolverSettings& settings) override;

private:
    void ApplyConstraints(Quat& rotation, const JointConstraint& constraint);
};

class MMV2_API FABRIKSolver : public IKSolver {
public:
    IKSolverType GetType() const override { return IKSolverType::FABRIK; }
    bool Solve(const Vec3& target, Pose& pose, const BoneHierarchy& hierarchy,
               const IKChain& chain, const IKSolverSettings& settings) override;
    bool SolveWithRotation(const Vec3& target, const Quat& targetRotation,
                           Pose& pose, const BoneHierarchy& hierarchy,
                           const IKChain& chain, const IKSolverSettings& settings) override;

private:
    void ForwardReach(Vector<Vec3>& positions, const Vec3& target, const IKChain& chain);
    void BackwardReach(Vector<Vec3>& positions, const Vec3& rootPos, const IKChain& chain);
    void PositionsToPose(const Vector<Vec3>& positions, Pose& pose, const BoneHierarchy& hierarchy,
                         const IKChain& chain);
};

class MMV2_API TwoBoneSolver : public IKSolver {
public:
    IKSolverType GetType() const override { return IKSolverType::TwoBone; }
    bool Solve(const Vec3& target, Pose& pose, const BoneHierarchy& hierarchy,
               const IKChain& chain, const IKSolverSettings& settings) override;
    bool SolveWithRotation(const Vec3& target, const Quat& targetRotation,
                           Pose& pose, const BoneHierarchy& hierarchy,
                           const IKChain& chain, const IKSolverSettings& settings) override;
};

class MMV2_API LimbSolver : public IKSolver {
public:
    IKSolverType GetType() const override { return IKSolverType::Limb; }
    bool Solve(const Vec3& target, Pose& pose, const BoneHierarchy& hierarchy,
               const IKChain& chain, const IKSolverSettings& settings) override;
    bool SolveWithRotation(const Vec3& target, const Quat& targetRotation,
                           Pose& pose, const BoneHierarchy& hierarchy,
                           const IKChain& chain, const IKSolverSettings& settings) override;
};

class MMV2_API AimSolver : public IKSolver {
public:
    AimSolver() : m_axis(Vec3::Forward()) {}
    IKSolverType GetType() const override { return IKSolverType::Aim; }
    bool Solve(const Vec3& target, Pose& pose, const BoneHierarchy& hierarchy,
               const IKChain& chain, const IKSolverSettings& settings) override;
    bool SolveWithRotation(const Vec3& target, const Quat& targetRotation,
                           Pose& pose, const BoneHierarchy& hierarchy,
                           const IKChain& chain, const IKSolverSettings& settings) override;
    void SetAimAxis(const Vec3& axis) { m_axis = axis.Normalized(); }

private:
    Vec3 m_axis;
};

class MMV2_API FootSolver : public IKSolver {
public:
    IKSolverType GetType() const override { return IKSolverType::Foot; }
    bool Solve(const Vec3& target, Pose& pose, const BoneHierarchy& hierarchy,
               const IKChain& chain, const IKSolverSettings& settings) override;
    bool SolveWithRotation(const Vec3& target, const Quat& targetRotation,
                           Pose& pose, const BoneHierarchy& hierarchy,
                           const IKChain& chain, const IKSolverSettings& settings) override;

    void SetGroundNormal(const Vec3& normal) { m_groundNormal = normal.Normalized(); }
    void SetFootOffset(float32 offset) { m_footOffset = offset; }

private:
    Vec3 m_groundNormal;
    float32 m_footOffset;
};

class MMV2_API IKManager {
public:
    IKManager();
    ~IKManager();

    void SetSkeleton(const BoneHierarchy* hierarchy);
    void AddChain(const char* name, const IKChain& chain);
    void RemoveChain(const char* name);
    IKChain* GetChain(const char* name);

    void Solve(const char* chainName, const Vec3& target, Pose& pose,
               const IKSolverSettings& settings = IKSolverSettings());
    void Solve(const char* chainName, const Vec3& target, const Quat& targetRotation,
               Pose& pose, const IKSolverSettings& settings = IKSolverSettings());

    void SolveAll(const Vec3& leftFootTarget, const Vec3& rightFootTarget,
                  const Vec3& leftHandTarget, const Vec3& rightHandTarget,
                  const Vec3& headTarget, Pose& pose);

    void SetSolver(IKSolverType type, IKSolver* solver);
    IKSolver* GetSolver(IKSolverType type);

private:
    const BoneHierarchy* m_hierarchy;
    HashMap<String, IKChain> m_chains;
    HashMap<uint32, IKSolver*> m_solvers;
    IAllocator* m_allocator;
};

MMV2_NAMESPACE_END

#endif
