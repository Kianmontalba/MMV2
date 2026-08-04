// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Interaction System
// ============================================================================

#pragma once
#ifndef MMV2_INTERACTION_H
#define MMV2_INTERACTION_H

#include "MMV2/Core/Config.h"
#include "MMV2/Core/Vector.h"
#include "MMV2/Core/String.h"
#include "MMV2/Core/Pose.h"
#include "MMV2/Core/Transform.h"
#include "MMV2/Database/Database.h"

MMV2_NAMESPACE_BEGIN

enum class InteractionType : uint8 {
    Handshake = 0,
    Hug = 1,
    Push = 2,
    Pull = 3,
    Carry = 4,
    Throw = 5,
    Catch = 6,
    Dance = 7,
    Combat = 8,
    Custom = 9
};

struct InteractionSlot {
    String name;
    Transform localTransform;
    int32 boneIndex;
    bool isOccupied;
    int32 occupantId;

    InteractionSlot() : boneIndex(-1), isOccupied(false), occupantId(-1) {}
};

struct InteractionDefinition {
    String name;
    InteractionType type;
    Vector<InteractionSlot> slots;
    float32 duration;
    float32 transitionInTime;
    float32 transitionOutTime;
    bool requiresSync;
    bool canInterrupt;
    int32 priority;

    InteractionDefinition()
        : type(InteractionType::Custom), duration(2.0f), transitionInTime(0.3f),
          transitionOutTime(0.3f), requiresSync(true), canInterrupt(false), priority(0) {}
};

struct InteractionInstance {
    int32 definitionIndex;
    int32 instanceId;
    float32 currentTime;
    float32 normalizedTime;
    bool isActive;
    bool isComplete;
    Vector<int32> participantIds;
    Vector<Pose> participantPoses;
    Vector<Transform> participantOffsets;

    InteractionInstance()
        : definitionIndex(-1), instanceId(-1), currentTime(0.0f), normalizedTime(0.0f),
          isActive(false), isComplete(false) {}
};

class MMV2_API InteractionDatabase {
public:
    InteractionDatabase();
    ~InteractionDatabase();

    int32 AddDefinition(const InteractionDefinition& definition);
    void RemoveDefinition(int32 index);
    const InteractionDefinition* GetDefinition(int32 index) const;
    const InteractionDefinition* FindDefinition(const char* name) const;
    int32 GetDefinitionCount() const;

    bool LoadFromFile(const char* path);
    bool SaveToFile(const char* path) const;

private:
    Vector<InteractionDefinition> m_definitions;
    HashMap<String, int32> m_nameToIndex;
};

class MMV2_API InteractionManager {
public:
    InteractionManager();
    ~InteractionManager();

    void SetDatabase(InteractionDatabase* database);

    int32 StartInteraction(int32 definitionIndex, const Vector<int32>& participantIds);
    void EndInteraction(int32 instanceId);
    void UpdateInteraction(int32 instanceId, float32 deltaTime);

    bool IsInteractionActive(int32 instanceId) const;
    bool CanStartInteraction(int32 participantId) const;

    const InteractionInstance* GetInstance(int32 instanceId) const;
    Vector<int32> GetActiveInteractionsForParticipant(int32 participantId) const;

    void SetParticipantPose(int32 instanceId, int32 participantIndex, const Pose& pose);
    Pose GetParticipantTargetPose(int32 instanceId, int32 participantIndex) const;

    void UpdateAll(float32 deltaTime);
    void Clear();

private:
    InteractionDatabase* m_database;
    Vector<InteractionInstance> m_instances;
    HashMap<int32, int32> m_instanceMap; // instanceId -> index
    int32 m_nextInstanceId;

    void ComputeParticipantTransforms(InteractionInstance& instance);
    bool ValidateParticipants(const InteractionDefinition& def, const Vector<int32>& participantIds);
};

MMV2_NAMESPACE_END

#endif
