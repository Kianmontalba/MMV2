// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Network System
// ============================================================================
// Provides network synchronization, client-side prediction, and server
// reconciliation for multiplayer motion matching.
// ============================================================================

#pragma once

#include "MMV2/Core/Config.h"
#include "MMV2/Core/Vector.h"
#include "MMV2/Core/RingBuffer.h"
#include "MMV2/Core/Pose.h"
#include "MMV2/Core/Transform.h"

MMV2_NAMESPACE_BEGIN

// ============================================================================
// Network Config
// ============================================================================

struct NetworkConfig
{
    bool isServer = false;
    bool isClient = false;
    float sendRate = 30.0f;             // Updates per second
    float interpDelay = 0.1f;           // Interpolation delay (seconds)
    float maxExtrapolationTime = 0.5f;  // Max time to extrapolate
    bool useClientPrediction = true;
    bool useServerReconciliation = true;
    bool useEntityInterpolation = true;
    uint32_t maxBufferedStates = 120;
    float positionTolerance = 0.01f;    // Meters
    float rotationTolerance = 0.01f;    // Radians
    float velocityTolerance = 0.1f;
};

// ============================================================================
// Networked Pose State
// ============================================================================

struct NetworkedPoseState
{
    uint32_t sequenceNumber;
    float timestamp;
    Pose pose;
    Vec3 velocity;
    Vec3 angularVelocity;
    uint32_t animationIndex;
    float animationTime;
    uint32_t targetPoseIndex;
    float blendWeight;

    NetworkedPoseState() : sequenceNumber(0), timestamp(0), animationIndex(0),
                           animationTime(0), targetPoseIndex(0), blendWeight(0) {}
};

// ============================================================================
// Networked Input
// ============================================================================

struct NetworkedInput
{
    uint32_t sequenceNumber;
    float timestamp;
    Vec3 desiredVelocity;
    Vec3 desiredDirection;
    float desiredSpeed;
    bool wantsJump;
    bool wantsCrouch;
    bool wantsSprint;
    uint32_t actionId;

    NetworkedInput() : sequenceNumber(0), timestamp(0), desiredSpeed(0),
                       wantsJump(false), wantsCrouch(false), wantsSprint(false),
                       actionId(0) {}
};

// ============================================================================
// Snapshot
// ============================================================================

struct NetworkSnapshot
{
    uint32_t tick;
    float timestamp;
    HashMap<uint32_t, NetworkedPoseState> entityStates;

    bool IsEmpty() const { return entityStates.Empty(); }
};

// ============================================================================
// Client-Side Prediction
// ============================================================================

class MMV2_API ClientPrediction
{
public:
    void Initialize(const NetworkConfig& config);

    void RecordInput(const NetworkedInput& input);
    void RecordState(const NetworkedPoseState& state);

    void ApplyCorrection(const NetworkedPoseState& serverState);

    NetworkedPoseState PredictCurrentState() const;

    void SetLocalEntityId(uint32_t id) { m_localEntityId = id; }

    bool HasPendingInputs() const { return !m_pendingInputs.Empty(); }
    uint32_t GetPendingInputCount() const { return static_cast<uint32_t>(m_pendingInputs.Size()); }

    void Clear();

private:
    NetworkConfig m_config;
    uint32_t m_localEntityId;

    RingBuffer<NetworkedInput> m_pendingInputs;
    RingBuffer<NetworkedPoseState> m_predictedStates;
    NetworkedPoseState m_lastServerState;

    void ReplayInputs(uint32_t fromSequence);
};

// ============================================================================
// Server Reconciliation
// ============================================================================

class MMV2_API ServerReconciliation
{
public:
    void Initialize(const NetworkConfig& config);

    void ProcessClientInput(uint32_t clientId, const NetworkedInput& input);
    void GenerateSnapshot(uint32_t tick, NetworkSnapshot& outSnapshot);

    void SetEntityState(uint32_t entityId, const NetworkedPoseState& state);
    NetworkedPoseState* GetEntityState(uint32_t entityId);

    void RemoveEntity(uint32_t entityId);
    void ClearEntities();

    uint32_t GetEntityCount() const { return static_cast<uint32_t>(m_entityStates.Size()); }

private:
    NetworkConfig m_config;
    HashMap<uint32_t, NetworkedPoseState> m_entityStates;
    HashMap<uint32_t, RingBuffer<NetworkedInput>> m_clientInputs;
    uint32_t m_currentTick;
};

// ============================================================================
// Entity Interpolation
// ============================================================================

class MMV2_API EntityInterpolator
{
public:
    void Initialize(const NetworkConfig& config);

    void AddSnapshot(const NetworkSnapshot& snapshot);
    void Update(float deltaTime);

    bool GetInterpolatedState(uint32_t entityId, NetworkedPoseState& outState) const;
    bool GetExtrapolatedState(uint32_t entityId, NetworkedPoseState& outState) const;

    void RemoveEntity(uint32_t entityId);
    void Clear();

    float GetInterpolationDelay() const { return m_config.interpDelay; }
    void SetInterpolationDelay(float delay) { m_config.interpDelay = delay; }

private:
    NetworkConfig m_config;
    RingBuffer<NetworkSnapshot> m_snapshots;
    HashMap<uint32_t, NetworkedPoseState> m_interpolatedStates;
    float m_currentTime;

    bool FindSurroundingSnapshots(float targetTime, NetworkSnapshot& outOlder,
                                   NetworkSnapshot& outNewer) const;
    void InterpolateStates(const NetworkedPoseState& older, const NetworkedPoseState& newer,
                            float t, NetworkedPoseState& outState) const;
    void ExtrapolateState(const NetworkedPoseState& lastState, float deltaTime,
                           NetworkedPoseState& outState) const;
};

// ============================================================================
// Network Serializer
// ============================================================================

class MMV2_API NetworkSerializer
{
public:
    static void SerializePoseState(const NetworkedPoseState& state, Vector<uint8_t>& outData);
    static void DeserializePoseState(const Vector<uint8_t>& data, NetworkedPoseState& outState);

    static void SerializeInput(const NetworkedInput& input, Vector<uint8_t>& outData);
    static void DeserializeInput(const Vector<uint8_t>& data, NetworkedInput& outInput);

    static void SerializeSnapshot(const NetworkSnapshot& snapshot, Vector<uint8_t>& outData);
    static void DeserializeSnapshot(const Vector<uint8_t>& data, NetworkSnapshot& outSnapshot);

    // Delta compression
    static void SerializeDeltaPoseState(const NetworkedPoseState& current,
                                         const NetworkedPoseState& baseline,
                                         Vector<uint8_t>& outData);
    static void DeserializeDeltaPoseState(const Vector<uint8_t>& data,
                                           const NetworkedPoseState& baseline,
                                           NetworkedPoseState& outState);

    // Quantized transforms for network efficiency
    static void QuantizeTransform(const Transform& transform, uint8_t* outData);
    static void DequantizeTransform(const uint8_t* data, Transform& outTransform);
};

// ============================================================================
// Network Manager
// ============================================================================

class MMV2_API NetworkManager
{
public:
    NetworkManager();
    ~NetworkManager();

    void Initialize(const NetworkConfig& config);
    void Shutdown();

    // Client functions
    void ClientSendInput(const NetworkedInput& input);
    void ClientReceiveSnapshot(const Vector<uint8_t>& data);
    void ClientUpdate(float deltaTime);

    // Server functions
    void ServerReceiveInput(uint32_t clientId, const Vector<uint8_t>& data);
    void ServerGenerateSnapshot(Vector<uint8_t>& outData);
    void ServerUpdate(float deltaTime);

    // Common
    bool IsServer() const { return m_config.isServer; }
    bool IsClient() const { return m_config.isClient; }

    void SetLocalEntityId(uint32_t id) { m_localEntityId = id; }
    uint32_t GetLocalEntityId() const { return m_localEntityId; }

    // State access
    bool GetLocalState(NetworkedPoseState& outState) const;
    bool GetEntityState(uint32_t entityId, NetworkedPoseState& outState) const;

    // Statistics
    float GetLatency() const { return m_latency; }
    float GetJitter() const { return m_jitter; }
    uint32_t GetPacketsSent() const { return m_packetsSent; }
    uint32_t GetPacketsReceived() const { return m_packetsReceived; }
    uint32_t GetBytesSent() const { return m_bytesSent; }
    uint32_t GetBytesReceived() const { return m_bytesReceived; }

    void ResetStats();

private:
    NetworkConfig m_config;
    uint32_t m_localEntityId;

    ClientPrediction m_clientPrediction;
    ServerReconciliation m_serverReconciliation;
    EntityInterpolator m_entityInterpolator;

    float m_latency;
    float m_jitter;
    uint32_t m_packetsSent;
    uint32_t m_packetsReceived;
    uint32_t m_bytesSent;
    uint32_t m_bytesReceived;

    float m_timeSinceLastSend;
    uint32_t m_sequenceNumber;
};

// ============================================================================
// Networked Motion Matching
// ============================================================================

class MMV2_API NetworkedMotionMatching
{
public:
    void Initialize(class MotionMatchingController* controller, const NetworkConfig& config);

    void Update(float deltaTime);

    // Called on client
    void OnLocalInput(const NetworkedInput& input);
    void OnServerSnapshot(const NetworkSnapshot& snapshot);

    // Called on server
    void OnClientInput(uint32_t clientId, const NetworkedInput& input);
    NetworkSnapshot GenerateServerSnapshot();

    // Visual smoothing for remote players
    void GetSmoothedPose(uint32_t entityId, Pose& outPose) const;

private:
    MotionMatchingController* m_controller;
    NetworkManager m_networkManager;
    NetworkConfig m_config;
};

MMV2_NAMESPACE_END
