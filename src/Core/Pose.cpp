#include "MMV2/Core/Pose.h"

MMV2_NAMESPACE_BEGIN

void Pose::Resize(int32 count) {
    boneCount = count;
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

void Pose::CopyFrom(const Pose& other) {
    Resize(other.boneCount);
    for (int32 i = 0; i < boneCount; ++i) {
        localPositions[i] = other.localPositions[i];
        localRotations[i] = other.localRotations[i];
        localScales[i] = other.localScales[i];
        worldPositions[i] = other.worldPositions[i];
        worldRotations[i] = other.worldRotations[i];
        worldScales[i] = other.worldScales[i];
        localMatrices[i] = other.localMatrices[i];
        worldMatrices[i] = other.worldMatrices[i];
        boneWeights[i] = other.boneWeights[i];
    }
    timestamp = other.timestamp;
    deltaTime = other.deltaTime;
    isWorldValid = other.isWorldValid;
}

void Pose::SetIdentity() {
    for (int32 i = 0; i < boneCount; ++i) {
        localPositions[i] = Vec3::Zero();
        localRotations[i] = Quat::Identity();
        localScales[i] = Vec3::One();
        worldPositions[i] = Vec3::Zero();
        worldRotations[i] = Quat::Identity();
        worldScales[i] = Vec3::One();
        localMatrices[i] = Mat4::Identity();
        worldMatrices[i] = Mat4::Identity();
        boneWeights[i] = 1.0f;
    }
    isWorldValid = true;
}

void Pose::SetFromBindPose(const BoneHierarchy& hierarchy) {
    Resize(static_cast<int32>(hierarchy.bones.Size()));
    for (int32 i = 0; i < boneCount; ++i) {
        localPositions[i] = hierarchy.bones[i].localBindPose.position;
        localRotations[i] = hierarchy.bones[i].localBindPose.rotation;
        localScales[i] = hierarchy.bones[i].localBindPose.scale;
    }
    ComputeWorldMatrices(hierarchy);
}

void Pose::ComputeWorldMatrices(const BoneHierarchy& hierarchy) {
    for (int32 i = 0; i < boneCount; ++i) {
        localMatrices[i] = Mat4::TRS(localPositions[i], localRotations[i], localScales[i]);
        int32 parent = hierarchy.bones[i].parentIndex;
        if (parent >= 0) {
            worldMatrices[i] = worldMatrices[parent] * localMatrices[i];
        } else {
            worldMatrices[i] = localMatrices[i];
        }
        worldMatrices[i].Decompose(worldPositions[i], worldRotations[i], worldScales[i]);
    }
    isWorldValid = true;
}

void Pose::ComputeLocalFromWorld(const BoneHierarchy& hierarchy) {
    for (int32 i = 0; i < boneCount; ++i) {
        int32 parent = hierarchy.bones[i].parentIndex;
        if (parent >= 0) {
            Mat4 parentInv = worldMatrices[parent].Inverted();
            Mat4 local = parentInv * worldMatrices[i];
            local.Decompose(localPositions[i], localRotations[i], localScales[i]);
        } else {
            localPositions[i] = worldPositions[i];
            localRotations[i] = worldRotations[i];
            localScales[i] = worldScales[i];
        }
        localMatrices[i] = Mat4::TRS(localPositions[i], localRotations[i], localScales[i]);
    }
}

void Pose::ApplyWeight(float32 weight) {
    for (int32 i = 0; i < boneCount; ++i) {
        localPositions[i] *= weight;
        Vec3 axis;
        float32 angle;
        localRotations[i].ToAxisAngle(axis, angle);
        localRotations[i] = Quat::FromAxisAngle(axis, angle * weight);
        localScales[i] = Vec3::Lerp(Vec3::One(), localScales[i], weight);
        boneWeights[i] *= weight;
    }
}

void Pose::BlendWith(const Pose& other, float32 t) {
    for (int32 i = 0; i < boneCount; ++i) {
        localPositions[i] = Vec3::Lerp(localPositions[i], other.localPositions[i], t);
        localRotations[i] = Quat::SlerpShortestPath(localRotations[i], other.localRotations[i], t);
        localScales[i] = Vec3::Lerp(localScales[i], other.localScales[i], t);
    }
}

void Pose::BlendWith(const Pose& other, float32 t, const BoneHierarchy& hierarchy, BoneFlags mask) {
    for (int32 i = 0; i < boneCount; ++i) {
        if (HasFlag(hierarchy.bones[i].flags, mask)) {
            localPositions[i] = Vec3::Lerp(localPositions[i], other.localPositions[i], t);
            localRotations[i] = Quat::SlerpShortestPath(localRotations[i], other.localRotations[i], t);
            localScales[i] = Vec3::Lerp(localScales[i], other.localScales[i], t);
        }
    }
}

void Pose::AdditiveBlend(const Pose& additive, float32 weight) {
    for (int32 i = 0; i < boneCount; ++i) {
        localPositions[i] += additive.localPositions[i] * weight;
        Quat addRot = additive.localRotations[i];
        Vec3 axis;
        float32 angle;
        addRot.ToAxisAngle(axis, angle);
        addRot = Quat::FromAxisAngle(axis, angle * weight);
        localRotations[i] = addRot * localRotations[i];
        localScales[i] += (additive.localScales[i] - Vec3::One()) * weight;
    }
}

void Pose::Interpolate(const Pose& a, const Pose& b, float32 t) {
    for (int32 i = 0; i < boneCount; ++i) {
        localPositions[i] = Vec3::Lerp(a.localPositions[i], b.localPositions[i], t);
        localRotations[i] = Quat::SlerpShortestPath(a.localRotations[i], b.localRotations[i], t);
        localScales[i] = Vec3::Lerp(a.localScales[i], b.localScales[i], t);
    }
}

void Pose::InterpolateWithMask(const Pose& a, const Pose& b, float32 t, const Vector<bool>& mask) {
    for (int32 i = 0; i < boneCount; ++i) {
        if (i < static_cast<int32>(mask.Size()) && mask[i]) {
            localPositions[i] = Vec3::Lerp(a.localPositions[i], b.localPositions[i], t);
            localRotations[i] = Quat::SlerpShortestPath(a.localRotations[i], b.localRotations[i], t);
            localScales[i] = Vec3::Lerp(a.localScales[i], b.localScales[i], t);
        }
    }
}

bool Pose::IsValid() const {
    if (boneCount <= 0) return false;
    if (localPositions.Size() != static_cast<size_type>(boneCount)) return false;
    if (localRotations.Size() != static_cast<size_type>(boneCount)) return false;
    if (localScales.Size() != static_cast<size_type>(boneCount)) return false;
    for (int32 i = 0; i < boneCount; ++i) {
        if (!localRotations[i].IsNormalized()) return false;
    }
    return true;
}

Pose Pose::ExtractPartial(const Vector<int32>& boneIndices) const {
    Pose result(static_cast<int32>(boneIndices.Size()));
    for (size_type i = 0; i < boneIndices.Size(); ++i) {
        int32 srcIdx = boneIndices[i];
        if (srcIdx >= 0 && srcIdx < boneCount) {
            result.localPositions[i] = localPositions[srcIdx];
            result.localRotations[i] = localRotations[srcIdx];
            result.localScales[i] = localScales[srcIdx];
            result.worldPositions[i] = worldPositions[srcIdx];
            result.worldRotations[i] = worldRotations[srcIdx];
            result.worldScales[i] = worldScales[srcIdx];
            result.boneWeights[i] = boneWeights[srcIdx];
        }
    }
    return result;
}

void Pose::SetPartial(const Pose& partial, const Vector<int32>& boneIndices) {
    for (size_type i = 0; i < boneIndices.Size() && i < static_cast<size_type>(partial.boneCount); ++i) {
        int32 dstIdx = boneIndices[i];
        if (dstIdx >= 0 && dstIdx < boneCount) {
            localPositions[dstIdx] = partial.localPositions[i];
            localRotations[dstIdx] = partial.localRotations[i];
            localScales[dstIdx] = partial.localScales[i];
        }
    }
}

// PoseVelocity
void PoseVelocity::Resize(int32 count) {
    boneCount = count;
    linearVelocities.Resize(count, Vec3::Zero());
    angularVelocities.Resize(count, Vec3::Zero());
}

void PoseVelocity::ComputeFromPoses(const Pose& current, const Pose& previous, float32 deltaTime) {
    if (deltaTime < MMV2_EPSILON) return;
    int32 count = std::min(current.boneCount, previous.boneCount);
    Resize(count);
    for (int32 i = 0; i < count; ++i) {
        linearVelocities[i] = (current.worldPositions[i] - previous.worldPositions[i]) / deltaTime;
        Quat delta = current.worldRotations[i] * previous.worldRotations[i].Inverse();
        Vec3 axis;
        float32 angle;
        delta.ToAxisAngle(axis, angle);
        angularVelocities[i] = axis * (angle / deltaTime);
    }
}

void PoseVelocity::ComputeFromPosesSmooth(const Pose& current, const Pose& previous, const Pose& beforePrevious, float32 deltaTime) {
    if (deltaTime < MMV2_EPSILON) return;
    int32 count = std::min({current.boneCount, previous.boneCount, beforePrevious.boneCount});
    Resize(count);
    for (int32 i = 0; i < count; ++i) {
        Vec3 v1 = (current.worldPositions[i] - previous.worldPositions[i]) / deltaTime;
        Vec3 v2 = (previous.worldPositions[i] - beforePrevious.worldPositions[i]) / deltaTime;
        linearVelocities[i] = (v1 + v2) * 0.5f;

        Quat d1 = current.worldRotations[i] * previous.worldRotations[i].Inverse();
        Quat d2 = previous.worldRotations[i] * beforePrevious.worldRotations[i].Inverse();
        Vec3 axis1, axis2;
        float32 angle1, angle2;
        d1.ToAxisAngle(axis1, angle1);
        d2.ToAxisAngle(axis2, angle2);
        angularVelocities[i] = (axis1 * angle1 + axis2 * angle2) * (0.5f / deltaTime);
    }
}

void PoseVelocity::BlendWith(const PoseVelocity& other, float32 t) {
    int32 count = std::min(boneCount, other.boneCount);
    for (int32 i = 0; i < count; ++i) {
        linearVelocities[i] = Vec3::Lerp(linearVelocities[i], other.linearVelocities[i], t);
        angularVelocities[i] = Vec3::Lerp(angularVelocities[i], other.angularVelocities[i], t);
    }
}

void PoseVelocity::Scale(float32 s) {
    for (int32 i = 0; i < boneCount; ++i) {
        linearVelocities[i] *= s;
        angularVelocities[i] *= s;
    }
}

// PoseAcceleration
void PoseAcceleration::Resize(int32 count) {
    boneCount = count;
    linearAccelerations.Resize(count, Vec3::Zero());
    angularAccelerations.Resize(count, Vec3::Zero());
}

void PoseAcceleration::ComputeFromVelocities(const PoseVelocity& current, const PoseVelocity& previous, float32 deltaTime) {
    if (deltaTime < MMV2_EPSILON) return;
    int32 count = std::min(current.boneCount, previous.boneCount);
    Resize(count);
    for (int32 i = 0; i < count; ++i) {
        linearAccelerations[i] = (current.linearVelocities[i] - previous.linearVelocities[i]) / deltaTime;
        angularAccelerations[i] = (current.angularVelocities[i] - previous.angularVelocities[i]) / deltaTime;
    }
}

MMV2_NAMESPACE_END
