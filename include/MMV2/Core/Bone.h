#pragma once
#ifndef MMV2_BONE_H
#define MMV2_BONE_H

#include "Config.h"
#include "Vec3.h"
#include "Quat.h"
#include "Transform.h"
#include "String.h"

MMV2_NAMESPACE_BEGIN

enum class BoneFlags : uint32 {
    None = 0,
    Root = 1 << 0,
    Spine = 1 << 1,
    Head = 1 << 2,
    LeftArm = 1 << 3,
    RightArm = 1 << 4,
    LeftLeg = 1 << 5,
    RightLeg = 1 << 6,
    LeftHand = 1 << 7,
    RightHand = 1 << 8,
    LeftFoot = 1 << 9,
    RightFoot = 1 << 10,
    IKTarget = 1 << 11,
    Twist = 1 << 12,
    Helper = 1 << 13,
    Facial = 1 << 14,
    Weapon = 1 << 15,
    Prop = 1 << 16,
    Dynamic = 1 << 17,
    Simulated = 1 << 18,
    Collider = 1 << 19,
    All = 0xFFFFFFFF
};

MMV2_FORCE_INLINE BoneFlags operator|(BoneFlags a, BoneFlags b) noexcept {
    return static_cast<BoneFlags>(static_cast<uint32>(a) | static_cast<uint32>(b));
}
MMV2_FORCE_INLINE BoneFlags operator&(BoneFlags a, BoneFlags b) noexcept {
    return static_cast<BoneFlags>(static_cast<uint32>(a) & static_cast<uint32>(b));
}
MMV2_FORCE_INLINE BoneFlags operator~(BoneFlags a) noexcept {
    return static_cast<BoneFlags>(~static_cast<uint32>(a));
}
MMV2_FORCE_INLINE bool HasFlag(BoneFlags flags, BoneFlags flag) noexcept {
    return (static_cast<uint32>(flags) & static_cast<uint32>(flag)) != 0;
}

struct Bone {
    String name;
    int32 parentIndex;
    uint32 hash;
    BoneFlags flags;
    Transform localBindPose;
    Transform worldBindPose;
    Vec3 localPosition;
    Quat localRotation;
    Vec3 localScale;
    Vec3 worldPosition;
    Quat worldRotation;
    Vec3 worldScale;
    Mat4 localMatrix;
    Mat4 worldMatrix;
    Mat4 inverseBindPose;
    float32 length;
    int32 depth;
    int32 childCount;
    int32 children[8];
    int32 twistIndex;
    int32 ikTargetIndex;
    float32 weight;
    bool isActive;
    bool isSimulated;

    Bone() : parentIndex(-1), hash(0), flags(BoneFlags::None),
             localPosition(Vec3::Zero()), localRotation(Quat::Identity()), localScale(Vec3::One()),
             worldPosition(Vec3::Zero()), worldRotation(Quat::Identity()), worldScale(Vec3::One()),
             length(0.0f), depth(0), childCount(0), twistIndex(-1), ikTargetIndex(-1),
             weight(1.0f), isActive(true), isSimulated(false) {
        for (int i = 0; i < 8; ++i) children[i] = -1;
    }
};

struct BoneHierarchy {
    Vector<Bone> bones;
    HashMap<uint32, int32> nameToIndex;
    int32 rootIndex;
    int32 spineBaseIndex;
    int32 headIndex;
    int32 leftFootIndex;
    int32 rightFootIndex;
    int32 leftHandIndex;
    int32 rightHandIndex;
    int32 weaponBoneIndex;

    BoneHierarchy() : rootIndex(-1), spineBaseIndex(-1), headIndex(-1),
                      leftFootIndex(-1), rightFootIndex(-1), leftHandIndex(-1),
                      rightHandIndex(-1), weaponBoneIndex(-1) {}

    int32 FindBoneIndex(const char* name) const;
    int32 FindBoneIndex(uint32 hash) const;
    const Bone* FindBone(const char* name) const;
    Bone* FindBone(const char* name);
    bool IsDescendant(int32 child, int32 ancestor) const;
    Vector<int32> GetChildren(int32 parentIndex) const;
    Vector<int32> GetChain(int32 from, int32 to) const;
    void ComputeWorldMatrices();
    void ComputeLocalFromWorld();
    void UpdateDepths();
};

MMV2_NAMESPACE_END

#endif
