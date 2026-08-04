#pragma once
#ifndef MMV2_POSE_H
#define MMV2_POSE_H

#include "Config.h"
#include "Bone.h"
#include "Vector.h"
#include "Span.h"

MMV2_NAMESPACE_BEGIN

struct Pose {
    Vector<Vec3> localPositions;
    Vector<Quat> localRotations;
    Vector<Vec3> localScales;
    Vector<Vec3> worldPositions;
    Vector<Quat> worldRotations;
    Vector<Vec3> worldScales;
    Vector<Mat4> localMatrices;
    Vector<Mat4> worldMatrices;
    Vector<float32> boneWeights;
    int32 boneCount;
    float32 timestamp;
    float32 deltaTime;
    bool isWorldValid;

    Pose() : boneCount(0), timestamp(0.0f), deltaTime(0.0f), isWorldValid(false) {}
    explicit Pose(int32 count) : boneCount(count), timestamp(0.0f), deltaTime(0.0f), isWorldValid(false) {
        localPositions.Resize(count, Vec3::Zero());
        localRotations.Resize(count, Quat::Identity());
        localScales.Resize(count, Vec3::One());
        worldPositions.Resize(count, Vec3::Zero());
        worldRotations.Resize(count, Quat::Identity());
        worldScales.Resize(count, Vec3::One());
        localMatrices.Resize(count, Mat4::Identity());
        worldMatrices.Resize(count, Mat4::Identity());
        boneWeights.Resize(count, 1.0f);
    }

    void Resize(int32 count);
    void CopyFrom(const Pose& other);
    void SetIdentity();
    void SetFromBindPose(const BoneHierarchy& hierarchy);
    void ComputeWorldMatrices(const BoneHierarchy& hierarchy);
    void ComputeLocalFromWorld(const BoneHierarchy& hierarchy);
    void ApplyWeight(float32 weight);
    void BlendWith(const Pose& other, float32 t);
    void BlendWith(const Pose& other, float32 t, const BoneHierarchy& hierarchy, BoneFlags mask);
    void AdditiveBlend(const Pose& additive, float32 weight);
    void Interpolate(const Pose& a, const Pose& b, float32 t);
    void InterpolateWithMask(const Pose& a, const Pose& b, float32 t, const Vector<bool>& mask);
    bool IsValid() const;
    Pose ExtractPartial(const Vector<int32>& boneIndices) const;
    void SetPartial(const Pose& partial, const Vector<int32>& boneIndices);
};

struct PoseVelocity {
    Vector<Vec3> linearVelocities;
    Vector<Vec3> angularVelocities;
    int32 boneCount;

    PoseVelocity() : boneCount(0) {}
    explicit PoseVelocity(int32 count) : boneCount(count) {
        linearVelocities.Resize(count, Vec3::Zero());
        angularVelocities.Resize(count, Vec3::Zero());
    }

    void Resize(int32 count);
    void ComputeFromPoses(const Pose& current, const Pose& previous, float32 deltaTime);
    void ComputeFromPosesSmooth(const Pose& current, const Pose& previous, const Pose& beforePrevious, float32 deltaTime);
    void BlendWith(const PoseVelocity& other, float32 t);
    void Scale(float32 s);
};

struct PoseAcceleration {
    Vector<Vec3> linearAccelerations;
    Vector<Vec3> angularAccelerations;
    int32 boneCount;

    PoseAcceleration() : boneCount(0) {}
    explicit PoseAcceleration(int32 count) : boneCount(count) {
        linearAccelerations.Resize(count, Vec3::Zero());
        angularAccelerations.Resize(count, Vec3::Zero());
    }

    void Resize(int32 count);
    void ComputeFromVelocities(const PoseVelocity& current, const PoseVelocity& previous, float32 deltaTime);
};

MMV2_NAMESPACE_END

#endif
