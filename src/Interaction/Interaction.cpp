// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Interaction System Implementation
// ============================================================================

#include "MMV2/Interaction/Interaction.h"
#include "MMV2/Core/Math.h"
#include "MMV2/Core/Serializer.h"
#include <algorithm>

MMV2_NAMESPACE_BEGIN

// ============================================================================
// InteractionDatabase
// ============================================================================

InteractionDatabase::InteractionDatabase() {}

InteractionDatabase::~InteractionDatabase() {}

int32 InteractionDatabase::AddDefinition(const InteractionDefinition& definition) {
    int32 index = static_cast<int32>(m_definitions.Size());
    m_definitions.PushBack(definition);
    m_nameToIndex[definition.name] = index;
    return index;
}

void InteractionDatabase::RemoveDefinition(int32 index) {
    if (index < 0 || index >= static_cast<int32>(m_definitions.Size())) return;
    m_definitions.Erase(m_definitions.begin() + index);
    m_nameToIndex.Clear();
    for (size_type i = 0; i < m_definitions.Size(); ++i) {
        m_nameToIndex[m_definitions[i].name] = static_cast<int32>(i);
    }
}

const InteractionDefinition* InteractionDatabase::GetDefinition(int32 index) const {
    if (index < 0 || index >= static_cast<int32>(m_definitions.Size())) return nullptr;
    return &m_definitions[index];
}

const InteractionDefinition* InteractionDatabase::FindDefinition(const char* name) const {
    auto it = m_nameToIndex.Find(name);
    if (it != m_nameToIndex.End()) {
        return &m_definitions[it->value];
    }
    return nullptr;
}

int32 InteractionDatabase::GetDefinitionCount() const {
    return static_cast<int32>(m_definitions.Size());
}

bool InteractionDatabase::LoadFromFile(const char* path) {
    BinarySerializer serializer;
    if (!serializer.OpenRead(path)) return false;

    uint32_t count;
    serializer.Read(count);
    m_definitions.Resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        serializer.Read(m_definitions[i].name);
        serializer.Read(m_definitions[i].type);
        uint32_t slotCount;
        serializer.Read(slotCount);
        m_definitions[i].slots.Resize(slotCount);
        for (uint32_t s = 0; s < slotCount; ++s) {
            serializer.Read(m_definitions[i].slots[s].name);
            serializer.Read(m_definitions[i].slots[s].localTransform);
            serializer.Read(m_definitions[i].slots[s].boneIndex);
        }
        serializer.Read(m_definitions[i].duration);
        serializer.Read(m_definitions[i].transitionInTime);
        serializer.Read(m_definitions[i].transitionOutTime);
        serializer.Read(m_definitions[i].requiresSync);
        serializer.Read(m_definitions[i].canInterrupt);
        serializer.Read(m_definitions[i].priority);
    }

    serializer.Close();

    // Rebuild name map
    m_nameToIndex.Clear();
    for (size_type i = 0; i < m_definitions.Size(); ++i) {
        m_nameToIndex[m_definitions[i].name] = static_cast<int32>(i);
    }

    return true;
}

bool InteractionDatabase::SaveToFile(const char* path) const {
    BinarySerializer serializer;
    if (!serializer.OpenWrite(path)) return false;

    serializer.Write(static_cast<uint32_t>(m_definitions.Size()));
    for (const auto& def : m_definitions) {
        serializer.Write(def.name);
        serializer.Write(def.type);
        serializer.Write(static_cast<uint32_t>(def.slots.Size()));
        for (const auto& slot : def.slots) {
            serializer.Write(slot.name);
            serializer.Write(slot.localTransform);
            serializer.Write(slot.boneIndex);
        }
        serializer.Write(def.duration);
        serializer.Write(def.transitionInTime);
        serializer.Write(def.transitionOutTime);
        serializer.Write(def.requiresSync);
        serializer.Write(def.canInterrupt);
        serializer.Write(def.priority);
    }

    serializer.Close();
    return true;
}

// ============================================================================
// InteractionManager
// ============================================================================

InteractionManager::InteractionManager()
    : m_database(nullptr), m_nextInstanceId(1) {}

InteractionManager::~InteractionManager() {
    Clear();
}

void InteractionManager::SetDatabase(InteractionDatabase* database) {
    m_database = database;
}

int32 InteractionManager::StartInteraction(int32 definitionIndex, const Vector<int32>& participantIds) {
    if (!m_database) return -1;

    const InteractionDefinition* def = m_database->GetDefinition(definitionIndex);
    if (!def) return -1;

    if (!ValidateParticipants(*def, participantIds)) return -1;

    // Check if participants are available
    for (int32 id : participantIds) {
        if (!CanStartInteraction(id)) return -1;
    }

    InteractionInstance instance;
    instance.definitionIndex = definitionIndex;
    instance.instanceId = m_nextInstanceId++;
    instance.participantIds = participantIds;
    instance.participantPoses.Resize(participantIds.Size());
    instance.participantOffsets.Resize(participantIds.Size());
    instance.isActive = true;
    instance.isComplete = false;

    // Initialize offsets based on slots
    for (size_type i = 0; i < participantIds.Size() && i < def->slots.Size(); ++i) {
        instance.participantOffsets[i] = def->slots[i].localTransform;
    }

    int32 index = static_cast<int32>(m_instances.Size());
    m_instances.PushBack(instance);
    m_instanceMap[instance.instanceId] = index;

    return instance.instanceId;
}

void InteractionManager::EndInteraction(int32 instanceId) {
    auto it = m_instanceMap.Find(instanceId);
    if (it == m_instanceMap.End()) return;

    int32 index = it->value;
    if (index >= 0 && index < static_cast<int32>(m_instances.Size())) {
        m_instances[index].isActive = false;
        m_instances[index].isComplete = true;
    }
}

void InteractionManager::UpdateInteraction(int32 instanceId, float32 deltaTime) {
    auto it = m_instanceMap.Find(instanceId);
    if (it == m_instanceMap.End()) return;

    int32 index = it->value;
    if (index < 0 || index >= static_cast<int32>(m_instances.Size())) return;

    InteractionInstance& instance = m_instances[index];
    if (!instance.isActive || instance.isComplete) return;

    const InteractionDefinition* def = m_database->GetDefinition(instance.definitionIndex);
    if (!def) {
        instance.isComplete = true;
        return;
    }

    instance.currentTime += deltaTime;
    instance.normalizedTime = instance.currentTime / def->duration;

    if (instance.normalizedTime >= 1.0f) {
        instance.normalizedTime = 1.0f;
        instance.isComplete = true;
        instance.isActive = false;
    }

    ComputeParticipantTransforms(instance);
}

bool InteractionManager::IsInteractionActive(int32 instanceId) const {
    auto it = m_instanceMap.Find(instanceId);
    if (it == m_instanceMap.End()) return false;
    int32 index = it->value;
    if (index < 0 || index >= static_cast<int32>(m_instances.Size())) return false;
    return m_instances[index].isActive;
}

bool InteractionManager::CanStartInteraction(int32 participantId) const {
    for (const auto& instance : m_instances) {
        if (!instance.isActive) continue;
        for (int32 id : instance.participantIds) {
            if (id == participantId) return false;
        }
    }
    return true;
}

const InteractionInstance* InteractionManager::GetInstance(int32 instanceId) const {
    auto it = m_instanceMap.Find(instanceId);
    if (it == m_instanceMap.End()) return nullptr;
    int32 index = it->value;
    if (index < 0 || index >= static_cast<int32>(m_instances.Size())) return nullptr;
    return &m_instances[index];
}

Vector<int32> InteractionManager::GetActiveInteractionsForParticipant(int32 participantId) const {
    Vector<int32> result;
    for (const auto& instance : m_instances) {
        if (!instance.isActive) continue;
        for (int32 id : instance.participantIds) {
            if (id == participantId) {
                result.PushBack(instance.instanceId);
                break;
            }
        }
    }
    return result;
}

void InteractionManager::SetParticipantPose(int32 instanceId, int32 participantIndex, const Pose& pose) {
    auto it = m_instanceMap.Find(instanceId);
    if (it == m_instanceMap.End()) return;
    int32 index = it->value;
    if (index < 0 || index >= static_cast<int32>(m_instances.Size())) return;
    if (participantIndex < 0 || participantIndex >= static_cast<int32>(m_instances[index].participantPoses.Size())) return;
    m_instances[index].participantPoses[participantIndex] = pose;
}

Pose InteractionManager::GetParticipantTargetPose(int32 instanceId, int32 participantIndex) const {
    const InteractionInstance* instance = GetInstance(instanceId);
    if (!instance) return Pose();
    if (participantIndex < 0 || participantIndex >= static_cast<int32>(instance->participantPoses.Size())) return Pose();

    const InteractionDefinition* def = m_database->GetDefinition(instance->definitionIndex);
    if (!def || participantIndex >= static_cast<int32>(def->slots.Size())) return Pose();

    // Compute target pose based on slot transform and interaction progress
    Pose result = instance->participantPoses[participantIndex];
    Transform offset = instance->participantOffsets[participantIndex];

    // Apply offset to root
    Transform root = result.GetBoneTransform(0);
    root.position += offset.position;
    root.rotation = offset.rotation * root.rotation;
    result.SetBoneTransform(0, root);

    return result;
}

void InteractionManager::UpdateAll(float32 deltaTime) {
    for (auto& instance : m_instances) {
        if (instance.isActive && !instance.isComplete) {
            UpdateInteraction(instance.instanceId, deltaTime);
        }
    }

    // Clean up completed interactions
    size_type writeIdx = 0;
    for (size_type i = 0; i < m_instances.Size(); ++i) {
        if (!m_instances[i].isComplete || m_instances[i].isActive) {
            if (writeIdx != i) m_instances[writeIdx] = std::move(m_instances[i]);
            ++writeIdx;
        }
    }
    m_instances.Resize(writeIdx);

    // Rebuild instance map
    m_instanceMap.Clear();
    for (size_type i = 0; i < m_instances.Size(); ++i) {
        m_instanceMap[m_instances[i].instanceId] = static_cast<int32>(i);
    }
}

void InteractionManager::Clear() {
    m_instances.Clear();
    m_instanceMap.Clear();
    m_nextInstanceId = 1;
}

void InteractionManager::ComputeParticipantTransforms(InteractionInstance& instance) {
    // Compute relative transforms for all participants
    // In a full implementation, this would use the reference participant's pose
    // to position all other participants
}

bool InteractionManager::ValidateParticipants(const InteractionDefinition& def, const Vector<int32>& participantIds) {
    if (participantIds.Size() != def.slots.Size()) return false;
    if (participantIds.Empty()) return false;
    return true;
}

MMV2_NAMESPACE_END
