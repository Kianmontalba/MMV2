// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Bone Implementation
// ============================================================================

#include "MMV2/Core/Bone.h"
#include "MMV2/Core/Math.h"

MMV2_NAMESPACE_BEGIN

Bone::Bone()
    : name(), parentIndex(-1), depth(0), flags(BoneFlags::None),
      length(0.0f), radius(0.0f), mass(1.0f), isActive(true) {}

Bone::Bone(const char* name_, int32 parent)
    : name(name_), parentIndex(parent), depth(0), flags(BoneFlags::None),
      length(0.0f), radius(0.0f), mass(1.0f), isActive(true) {}

BoneHierarchy::BoneHierarchy() : m_bones(), m_nameToIndex() {}

BoneHierarchy::~BoneHierarchy() {}

void BoneHierarchy::AddBone(const Bone& bone) {
    int32 index = static_cast<int32>(m_bones.Size());
    m_bones.PushBack(bone);
    m_nameToIndex[bone.name] = index;

    // Compute depth
    if (bone.parentIndex >= 0 && bone.parentIndex < index) {
        m_bones[index].depth = m_bones[bone.parentIndex].depth + 1;
    } else {
        m_bones[index].depth = 0;
    }
}

void BoneHierarchy::RemoveBone(int32 index) {
    if (index < 0 || index >= static_cast<int32>(m_bones.Size())) return;
    m_bones.Erase(m_bones.begin() + index);
    RebuildNameMap();
}

const Bone* BoneHierarchy::GetBone(int32 index) const {
    if (index < 0 || index >= static_cast<int32>(m_bones.Size())) return nullptr;
    return &m_bones[index];
}

const Bone* BoneHierarchy::FindBone(const char* name) const {
    auto it = m_nameToIndex.Find(name);
    if (it != m_nameToIndex.End()) {
        return &m_bones[it->value];
    }
    return nullptr;
}

int32 BoneHierarchy::GetBoneIndex(const char* name) const {
    auto it = m_nameToIndex.Find(name);
    if (it != m_nameToIndex.End()) return it->value;
    return -1;
}

int32 BoneHierarchy::GetBoneCount() const {
    return static_cast<int32>(m_bones.Size());
}

int32 BoneHierarchy::GetRootBoneCount() const {
    int32 count = 0;
    for (const auto& bone : m_bones) {
        if (bone.parentIndex < 0) ++count;
    }
    return count;
}

Vector<int32> BoneHierarchy::GetChildren(int32 parentIndex) const {
    Vector<int32> children;
    for (size_type i = 0; i < m_bones.Size(); ++i) {
        if (m_bones[i].parentIndex == parentIndex) {
            children.PushBack(static_cast<int32>(i));
        }
    }
    return children;
}

Vector<int32> BoneHierarchy::GetBoneChain(int32 fromIndex, int32 toIndex) const {
    Vector<int32> chain;
    if (fromIndex < 0 || fromIndex >= static_cast<int32>(m_bones.Size())) return chain;
    if (toIndex < 0 || toIndex >= static_cast<int32>(m_bones.Size())) return chain;

    // Build path from toIndex up to root
    Vector<int32> toPath;
    int32 current = toIndex;
    while (current >= 0) {
        toPath.PushBack(current);
        current = m_bones[current].parentIndex;
    }

    // Find common ancestor
    current = fromIndex;
    while (current >= 0) {
        for (size_type i = 0; i < toPath.Size(); ++i) {
            if (toPath[i] == current) {
                // Found common ancestor, build chain
                for (size_type j = 0; j < i; ++j) {
                    chain.PushBack(toPath[j]);
                }
                // Add from path
                Vector<int32> fromPath;
                int32 fromCurrent = fromIndex;
                while (fromCurrent != current) {
                    fromPath.PushBack(fromCurrent);
                    fromCurrent = m_bones[fromCurrent].parentIndex;
                }
                // Reverse from path and add
                for (int32 j = static_cast<int32>(fromPath.Size()) - 1; j >= 0; --j) {
                    chain.PushBack(fromPath[j]);
                }
                return chain;
            }
        }
        current = m_bones[current].parentIndex;
    }

    return chain;
}

void BoneHierarchy::RebuildNameMap() {
    m_nameToIndex.Clear();
    for (size_type i = 0; i < m_bones.Size(); ++i) {
        m_nameToIndex[m_bones[i].name] = static_cast<int32>(i);
    }
}

void BoneHierarchy::ComputeBoneLengths(const Pose& referencePose) {
    for (size_type i = 0; i < m_bones.Size(); ++i) {
        if (m_bones[i].parentIndex >= 0) {
            Vec3 parentPos = referencePose.GetBoneTransform(m_bones[i].parentIndex).position;
            Vec3 bonePos = referencePose.GetBoneTransform(static_cast<int32>(i)).position;
            m_bones[i].length = (bonePos - parentPos).Length();
        }
    }
}

bool BoneHierarchy::IsValid() const {
    if (m_bones.Empty()) return false;
    // Check for cycles
    for (size_type i = 0; i < m_bones.Size(); ++i) {
        int32 current = static_cast<int32>(i);
        int32 steps = 0;
        while (current >= 0) {
            current = m_bones[current].parentIndex;
            ++steps;
            if (steps > static_cast<int32>(m_bones.Size())) return false; // Cycle detected
        }
    }
    return true;
}

MMV2_NAMESPACE_END
