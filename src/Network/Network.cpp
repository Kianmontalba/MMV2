// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Network System Implementation
// ============================================================================

#include "MMV2/Network/Network.h"
#include "MMV2/Core/Math.h"
#include "MMV2/Core/Serializer.h"
#include <cmath>
#include <cstring>

MMV2_NAMESPACE_BEGIN

// ============================================================================
// ClientPrediction
// ============================================================================

void ClientPrediction::Initialize(const NetworkConfig& config)
{
    m_config = config;
    m_pendingInputs.Resize(config.maxBufferedStates);
    m_predictedStates.Resize(config.maxBufferedStates);
    Clear();
}

void ClientPrediction::RecordInput(const NetworkedInput& input)
{
    m_pendingInputs.PushBack(input);
}

void ClientPrediction::RecordState(const NetworkedPoseState& state)
{
    m_predictedStates.PushBack(state);
}

void ClientPrediction::ApplyCorrection(const NetworkedPoseState& serverState)
{
    // Find the predicted state that matches the server state's sequence
    uint32_t serverSeq = serverState.sequenceNumber;

    // Remove acknowledged inputs
    while (!m_pendingInputs.Empty() && m_pendingInputs.Front()->sequenceNumber <= serverSeq)
    {
        m_pendingInputs.PopFront();
    }

    // Check for significant error
    float positionError = (m_lastServerState.pose.GetBoneTransform(0).position -
                            serverState.pose.GetBoneTransform(0).position).Magnitude();

    if (positionError > m_config.positionTolerance)
    {
        // Snap to server state and replay inputs
        m_lastServerState = serverState;
        ReplayInputs(serverSeq + 1);
    }
}

NetworkedPoseState ClientPrediction::PredictCurrentState() const
{
    if (m_predictedStates.Empty())
        return NetworkedPoseState();

    return *m_predictedStates.Back();
}

void ClientPrediction::Clear()
{
    m_pendingInputs.Clear();
    m_predictedStates.Clear();
    m_lastServerState = NetworkedPoseState();
}

void ClientPrediction::ReplayInputs(uint32_t fromSequence)
{
    // Re-apply all pending inputs starting from the corrected state
    NetworkedPoseState currentState = m_lastServerState;

    for (const auto& input : m_pendingInputs)
    {
        if (input.sequenceNumber >= fromSequence)
        {
            // Would apply input to state using motion matching logic
            // This is a simplified version
            currentState.pose.GetBoneTransform(0).position += input.desiredVelocity * 0.016f;
            currentState.sequenceNumber = input.sequenceNumber;

            m_predictedStates.PushBack(currentState);
        }
    }
}

// ============================================================================
// ServerReconciliation
// ============================================================================

void ServerReconciliation::Initialize(const NetworkConfig& config)
{
    m_config = config;
    m_currentTick = 0;
    ClearEntities();
}

void ServerReconciliation::ProcessClientInput(uint32_t clientId, const NetworkedInput& input)
{
    auto it = m_clientInputs.Find(clientId);
    if (it == m_clientInputs.End())
    {
        m_clientInputs[clientId] = RingBuffer<NetworkedInput>(m_config.maxBufferedStates);
        it = m_clientInputs.Find(clientId);
    }

    it->second.PushBack(input);

    // Process input for the entity
    auto stateIt = m_entityStates.Find(clientId);
    if (stateIt != m_entityStates.End())
    {
        // Apply input to entity state
        NetworkedPoseState& state = stateIt->second;
        state.pose.GetBoneTransform(0).position += input.desiredVelocity * 0.016f;
        state.velocity = input.desiredVelocity;
        state.sequenceNumber = input.sequenceNumber;
    }
}

void ServerReconciliation::GenerateSnapshot(uint32_t tick, NetworkSnapshot& outSnapshot)
{
    outSnapshot.tick = tick;
    outSnapshot.timestamp = static_cast<float>(tick) / m_config.sendRate;
    outSnapshot.entityStates = m_entityStates;
}

void ServerReconciliation::SetEntityState(uint32_t entityId, const NetworkedPoseState& state)
{
    m_entityStates[entityId] = state;
}

NetworkedPoseState* ServerReconciliation::GetEntityState(uint32_t entityId)
{
    auto it = m_entityStates.Find(entityId);
    return (it != m_entityStates.End()) ? &it->second : nullptr;
}

void ServerReconciliation::RemoveEntity(uint32_t entityId)
{
    m_entityStates.Erase(entityId);
    m_clientInputs.Erase(entityId);
}

void ServerReconciliation::ClearEntities()
{
    m_entityStates.Clear();
    m_clientInputs.Clear();
}

// ============================================================================
// EntityInterpolator
// ============================================================================

void EntityInterpolator::Initialize(const NetworkConfig& config)
{
    m_config = config;
    m_snapshots.Resize(config.maxBufferedStates);
    m_currentTime = 0.0f;
    Clear();
}

void EntityInterpolator::AddSnapshot(const NetworkSnapshot& snapshot)
{
    m_snapshots.PushBack(snapshot);

    // Remove old snapshots
    while (m_snapshots.GetCount() > m_config.maxBufferedStates)
    {
        m_snapshots.PopFront();
    }
}

void EntityInterpolator::Update(float deltaTime)
{
    m_currentTime += deltaTime;

    float renderTime = m_currentTime - m_config.interpDelay;

    // Update all entities
    for (const auto& pair : m_snapshots.Back()->entityStates)
    {
        uint32_t entityId = pair.first;
        NetworkedPoseState state;

        if (GetInterpolatedState(entityId, state))
        {
            m_interpolatedStates[entityId] = state;
        }
        else if (GetExtrapolatedState(entityId, state))
        {
            m_interpolatedStates[entityId] = state;
        }
    }
}

bool EntityInterpolator::GetInterpolatedState(uint32_t entityId, NetworkedPoseState& outState) const
{
    float renderTime = m_currentTime - m_config.interpDelay;

    NetworkSnapshot older, newer;
    if (!FindSurroundingSnapshots(renderTime, older, newer))
        return false;

    auto olderIt = older.entityStates.Find(entityId);
    auto newerIt = newer.entityStates.Find(entityId);

    if (olderIt == older.entityStates.End() || newerIt == newer.entityStates.End())
        return false;

    float t = 0.0f;
    if (newer.timestamp > older.timestamp)
    {
        t = (renderTime - older.timestamp) / (newer.timestamp - older.timestamp);
    }
    t = Math::Clamp(t, 0.0f, 1.0f);

    InterpolateStates(olderIt->second, newerIt->second, t, outState);
    return true;
}

bool EntityInterpolator::GetExtrapolatedState(uint32_t entityId, NetworkedPoseState& outState) const
{
    if (m_snapshots.Empty())
        return false;

    const NetworkSnapshot& lastSnapshot = *m_snapshots.Back();
    auto it = lastSnapshot.entityStates.Find(entityId);

    if (it == lastSnapshot.entityStates.End())
        return false;

    float extrapolationTime = m_currentTime - m_config.interpDelay - lastSnapshot.timestamp;
    extrapolationTime = Math::Min(extrapolationTime, m_config.maxExtrapolationTime);

    if (extrapolationTime <= 0.0f)
    {
        outState = it->second;
        return true;
    }

    ExtrapolateState(it->second, extrapolationTime, outState);
    return true;
}

void EntityInterpolator::RemoveEntity(uint32_t entityId)
{
    m_interpolatedStates.Erase(entityId);
}

void EntityInterpolator::Clear()
{
    m_snapshots.Clear();
    m_interpolatedStates.Clear();
    m_currentTime = 0.0f;
}

bool EntityInterpolator::FindSurroundingSnapshots(float targetTime, NetworkSnapshot& outOlder,
                                                     NetworkSnapshot& outNewer) const
{
    if (m_snapshots.GetCount() < 2)
        return false;

    for (uint32_t i = 0; i + 1 < m_snapshots.GetCount(); ++i)
    {
        if (m_snapshots[i].timestamp <= targetTime && m_snapshots[i + 1].timestamp >= targetTime)
        {
            outOlder = m_snapshots[i];
            outNewer = m_snapshots[i + 1];
            return true;
        }
    }

    return false;
}

void EntityInterpolator::InterpolateStates(const NetworkedPoseState& older,
                                              const NetworkedPoseState& newer,
                                              float t, NetworkedPoseState& outState) const
{
    outState = older;

    // Interpolate pose
    uint32_t boneCount = older.pose.GetBoneCount();
    outState.pose.Resize(boneCount);

    for (uint32_t i = 0; i < boneCount; ++i)
    {
        const Transform& a = older.pose.GetBoneTransform(i);
        const Transform& b = newer.pose.GetBoneTransform(i);

        outState.pose.GetBoneTransform(i).position = Vec3::Lerp(a.position, b.position, t);
        outState.pose.GetBoneTransform(i).rotation = Quat::Slerp(a.rotation, b.rotation, t);
        outState.pose.GetBoneTransform(i).scale = Vec3::Lerp(a.scale, b.scale, t);
    }

    // Interpolate other properties
    outState.velocity = Vec3::Lerp(older.velocity, newer.velocity, t);
    outState.angularVelocity = Vec3::Lerp(older.angularVelocity, newer.angularVelocity, t);
    outState.animationTime = Math::Lerp(older.animationTime, newer.animationTime, t);
    outState.blendWeight = Math::Lerp(older.blendWeight, newer.blendWeight, t);
}

void EntityInterpolator::ExtrapolateState(const NetworkedPoseState& lastState, float deltaTime,
                                           NetworkedPoseState& outState) const
{
    outState = lastState;

    // Simple velocity-based extrapolation
    outState.pose.GetBoneTransform(0).position += lastState.velocity * deltaTime;

    // Angular extrapolation
    float angle = lastState.angularVelocity.Magnitude() * deltaTime;
    if (angle > 1e-6f)
    {
        Vec3 axis = lastState.angularVelocity.Normalized();
        Quat deltaRot(axis, angle);
        outState.pose.GetBoneTransform(0).rotation = (deltaRot * lastState.pose.GetBoneTransform(0).rotation).Normalized();
    }
}

// ============================================================================
// NetworkSerializer
// ============================================================================

void NetworkSerializer::SerializePoseState(const NetworkedPoseState& state, Vector<uint8_t>& outData)
{
    // Simplified serialization
    uint32_t size = sizeof(uint32_t) * 5 + sizeof(float) * 3 + sizeof(Vec3) * 3;
    outData.Resize(size);

    uint8_t* ptr = outData.Data();
    std::memcpy(ptr, &state.sequenceNumber, sizeof(uint32_t)); ptr += sizeof(uint32_t);
    std::memcpy(ptr, &state.timestamp, sizeof(float)); ptr += sizeof(float);
    std::memcpy(ptr, &state.animationIndex, sizeof(uint32_t)); ptr += sizeof(uint32_t);
    std::memcpy(ptr, &state.animationTime, sizeof(float)); ptr += sizeof(float);
    std::memcpy(ptr, &state.targetPoseIndex, sizeof(uint32_t)); ptr += sizeof(uint32_t);
    std::memcpy(ptr, &state.blendWeight, sizeof(float)); ptr += sizeof(float);
    std::memcpy(ptr, &state.velocity, sizeof(Vec3)); ptr += sizeof(Vec3);
    std::memcpy(ptr, &state.angularVelocity, sizeof(Vec3)); ptr += sizeof(Vec3);
}

void NetworkSerializer::DeserializePoseState(const Vector<uint8_t>& data, NetworkedPoseState& outState)
{
    if (data.Size() < sizeof(uint32_t) * 5 + sizeof(float) * 3 + sizeof(Vec3) * 3)
        return;

    const uint8_t* ptr = data.Data();
    std::memcpy(&outState.sequenceNumber, ptr, sizeof(uint32_t)); ptr += sizeof(uint32_t);
    std::memcpy(&outState.timestamp, ptr, sizeof(float)); ptr += sizeof(float);
    std::memcpy(&outState.animationIndex, ptr, sizeof(uint32_t)); ptr += sizeof(uint32_t);
    std::memcpy(&outState.animationTime, ptr, sizeof(float)); ptr += sizeof(float);
    std::memcpy(&outState.targetPoseIndex, ptr, sizeof(uint32_t)); ptr += sizeof(uint32_t);
    std::memcpy(&outState.blendWeight, ptr, sizeof(float)); ptr += sizeof(float);
    std::memcpy(&outState.velocity, ptr, sizeof(Vec3)); ptr += sizeof(Vec3);
    std::memcpy(&outState.angularVelocity, ptr, sizeof(Vec3)); ptr += sizeof(Vec3);
}

void NetworkSerializer::SerializeInput(const NetworkedInput& input, Vector<uint8_t>& outData)
{
    uint32_t size = sizeof(uint32_t) + sizeof(float) + sizeof(Vec3) * 2 + sizeof(float) + sizeof(uint32_t) + sizeof(bool) * 3;
    outData.Resize(size);

    uint8_t* ptr = outData.Data();
    std::memcpy(ptr, &input.sequenceNumber, sizeof(uint32_t)); ptr += sizeof(uint32_t);
    std::memcpy(ptr, &input.timestamp, sizeof(float)); ptr += sizeof(float);
    std::memcpy(ptr, &input.desiredVelocity, sizeof(Vec3)); ptr += sizeof(Vec3);
    std::memcpy(ptr, &input.desiredDirection, sizeof(Vec3)); ptr += sizeof(Vec3);
    std::memcpy(ptr, &input.desiredSpeed, sizeof(float)); ptr += sizeof(float);
    std::memcpy(ptr, &input.wantsJump, sizeof(bool)); ptr += sizeof(bool);
    std::memcpy(ptr, &input.wantsCrouch, sizeof(bool)); ptr += sizeof(bool);
    std::memcpy(ptr, &input.wantsSprint, sizeof(bool)); ptr += sizeof(bool);
    std::memcpy(ptr, &input.actionId, sizeof(uint32_t)); ptr += sizeof(uint32_t);
}

void NetworkSerializer::DeserializeInput(const Vector<uint8_t>& data, NetworkedInput& outInput)
{
    if (data.Size() < sizeof(uint32_t) + sizeof(float) + sizeof(Vec3) * 2 + sizeof(float) + sizeof(uint32_t) + sizeof(bool) * 3)
        return;

    const uint8_t* ptr = data.Data();
    std::memcpy(&outInput.sequenceNumber, ptr, sizeof(uint32_t)); ptr += sizeof(uint32_t);
    std::memcpy(&outInput.timestamp, ptr, sizeof(float)); ptr += sizeof(float);
    std::memcpy(&outInput.desiredVelocity, ptr, sizeof(Vec3)); ptr += sizeof(Vec3);
    std::memcpy(&outInput.desiredDirection, ptr, sizeof(Vec3)); ptr += sizeof(Vec3);
    std::memcpy(&outInput.desiredSpeed, ptr, sizeof(float)); ptr += sizeof(float);
    std::memcpy(&outInput.wantsJump, ptr, sizeof(bool)); ptr += sizeof(bool);
    std::memcpy(&outInput.wantsCrouch, ptr, sizeof(bool)); ptr += sizeof(bool);
    std::memcpy(&outInput.wantsSprint, ptr, sizeof(bool)); ptr += sizeof(bool);
    std::memcpy(&outInput.actionId, ptr, sizeof(uint32_t)); ptr += sizeof(uint32_t);
}

void NetworkSerializer::SerializeSnapshot(const NetworkSnapshot& snapshot, Vector<uint8_t>& outData)
{
    // Simplified: just serialize entity count and states
    uint32_t entityCount = static_cast<uint32_t>(snapshot.entityStates.Size());
    outData.Clear();

    // Write header
    outData.Resize(sizeof(uint32_t) * 2 + sizeof(float));
    uint8_t* ptr = outData.Data();
    std::memcpy(ptr, &snapshot.tick, sizeof(uint32_t)); ptr += sizeof(uint32_t);
    std::memcpy(ptr, &snapshot.timestamp, sizeof(float)); ptr += sizeof(float);
    std::memcpy(ptr, &entityCount, sizeof(uint32_t)); ptr += sizeof(uint32_t);

    // Write entity states
    for (const auto& pair : snapshot.entityStates)
    {
        Vector<uint8_t> stateData;
        SerializePoseState(pair.second, stateData);

        uint32_t stateSize = static_cast<uint32_t>(stateData.Size());
        uint32_t oldSize = static_cast<uint32_t>(outData.Size());
        outData.Resize(oldSize + sizeof(uint32_t) + stateSize);

        ptr = outData.Data() + oldSize;
        std::memcpy(ptr, &pair.first, sizeof(uint32_t)); ptr += sizeof(uint32_t);
        std::memcpy(ptr, stateData.Data(), stateSize);
    }
}

void NetworkSerializer::DeserializeSnapshot(const Vector<uint8_t>& data, NetworkSnapshot& outSnapshot)
{
    if (data.Size() < sizeof(uint32_t) * 2 + sizeof(float))
        return;

    const uint8_t* ptr = data.Data();
    std::memcpy(&outSnapshot.tick, ptr, sizeof(uint32_t)); ptr += sizeof(uint32_t);
    std::memcpy(&outSnapshot.timestamp, ptr, sizeof(float)); ptr += sizeof(float);

    uint32_t entityCount;
    std::memcpy(&entityCount, ptr, sizeof(uint32_t)); ptr += sizeof(uint32_t);

    outSnapshot.entityStates.Clear();

    for (uint32_t i = 0; i < entityCount; ++i)
    {
        uint32_t entityId;
        std::memcpy(&entityId, ptr, sizeof(uint32_t)); ptr += sizeof(uint32_t);

        // Simplified: assume fixed state size
        uint32_t stateSize = sizeof(uint32_t) * 5 + sizeof(float) * 3 + sizeof(Vec3) * 3;
        Vector<uint8_t> stateData;
        stateData.Resize(stateSize);
        std::memcpy(stateData.Data(), ptr, stateSize);
        ptr += stateSize;

        NetworkedPoseState state;
        DeserializePoseState(stateData, state);
        outSnapshot.entityStates[entityId] = state;
    }
}

void NetworkSerializer::SerializeDeltaPoseState(const NetworkedPoseState& current,
                                                 const NetworkedPoseState& baseline,
                                                 Vector<uint8_t>& outData)
{
    // Delta compression: only send changed values
    outData.Clear();

    uint8_t flags = 0;
    if ((current.pose.GetBoneTransform(0).position - baseline.pose.GetBoneTransform(0).position).Magnitude() > 0.001f)
        flags |= 1 << 0;
    if (current.animationIndex != baseline.animationIndex)
        flags |= 1 << 1;
    if (std::abs(current.animationTime - baseline.animationTime) > 0.01f)
        flags |= 1 << 2;

    outData.PushBack(flags);

    if (flags & (1 << 0))
    {
        // Send position delta
        Vec3 delta = current.pose.GetBoneTransform(0).position - baseline.pose.GetBoneTransform(0).position;
        outData.Resize(outData.Size() + sizeof(Vec3));
        std::memcpy(outData.Data() + outData.Size() - sizeof(Vec3), &delta, sizeof(Vec3));
    }

    if (flags & (1 << 1))
    {
        outData.Resize(outData.Size() + sizeof(uint32_t));
        std::memcpy(outData.Data() + outData.Size() - sizeof(uint32_t), &current.animationIndex, sizeof(uint32_t));
    }

    if (flags & (1 << 2))
    {
        outData.Resize(outData.Size() + sizeof(float));
        std::memcpy(outData.Data() + outData.Size() - sizeof(float), &current.animationTime, sizeof(float));
    }
}

void NetworkSerializer::DeserializeDeltaPoseState(const Vector<uint8_t>& data,
                                                   const NetworkedPoseState& baseline,
                                                   NetworkedPoseState& outState)
{
    outState = baseline;

    if (data.Empty())
        return;

    uint8_t flags = data[0];
    const uint8_t* ptr = data.Data() + 1;

    if (flags & (1 << 0))
    {
        Vec3 delta;
        std::memcpy(&delta, ptr, sizeof(Vec3));
        ptr += sizeof(Vec3);
        outState.pose.GetBoneTransform(0).position = baseline.pose.GetBoneTransform(0).position + delta;
    }

    if (flags & (1 << 1))
    {
        std::memcpy(&outState.animationIndex, ptr, sizeof(uint32_t));
        ptr += sizeof(uint32_t);
    }

    if (flags & (1 << 2))
    {
        std::memcpy(&outState.animationTime, ptr, sizeof(float));
        ptr += sizeof(float);
    }
}

void NetworkSerializer::QuantizeTransform(const Transform& transform, uint8_t* outData)
{
    // 6 bytes: 2 bytes per axis for position (quantized to 16-bit)
    int16_t px = static_cast<int16_t>(Math::Clamp(transform.position.x * 100.0f, -32767.0f, 32767.0f));
    int16_t py = static_cast<int16_t>(Math::Clamp(transform.position.y * 100.0f, -32767.0f, 32767.0f));
    int16_t pz = static_cast<int16_t>(Math::Clamp(transform.position.z * 100.0f, -32767.0f, 32767.0f));

    std::memcpy(outData, &px, sizeof(int16_t));
    std::memcpy(outData + 2, &py, sizeof(int16_t));
    std::memcpy(outData + 4, &pz, sizeof(int16_t));
}

void NetworkSerializer::DequantizeTransform(const uint8_t* data, Transform& outTransform)
{
    int16_t px, py, pz;
    std::memcpy(&px, data, sizeof(int16_t));
    std::memcpy(&py, data + 2, sizeof(int16_t));
    std::memcpy(&pz, data + 4, sizeof(int16_t));

    outTransform.position.x = px / 100.0f;
    outTransform.position.y = py / 100.0f;
    outTransform.position.z = pz / 100.0f;
}

// ============================================================================
// NetworkManager
// ============================================================================

NetworkManager::NetworkManager()
    : m_localEntityId(0), m_latency(0), m_jitter(0),
      m_packetsSent(0), m_packetsReceived(0), m_bytesSent(0), m_bytesReceived(0),
      m_timeSinceLastSend(0), m_sequenceNumber(0)
{
}

NetworkManager::~NetworkManager()
{
    Shutdown();
}

void NetworkManager::Initialize(const NetworkConfig& config)
{
    m_config = config;

    if (config.isClient)
    {
        m_clientPrediction.Initialize(config);
    }

    if (config.isServer)
    {
        m_serverReconciliation.Initialize(config);
    }

    m_entityInterpolator.Initialize(config);
    ResetStats();
}

void NetworkManager::Shutdown()
{
    m_clientPrediction.Clear();
    m_serverReconciliation.ClearEntities();
    m_entityInterpolator.Clear();
}

void NetworkManager::ClientSendInput(const NetworkedInput& input)
{
    if (!m_config.isClient)
        return;

    Vector<uint8_t> data;
    NetworkSerializer::SerializeInput(input, data);

    ++m_packetsSent;
    m_bytesSent += static_cast<uint32_t>(data.Size());

    m_clientPrediction.RecordInput(input);
}

void NetworkManager::ClientReceiveSnapshot(const Vector<uint8_t>& data)
{
    if (!m_config.isClient)
        return;

    ++m_packetsReceived;
    m_bytesReceived += static_cast<uint32_t>(data.Size());

    NetworkSnapshot snapshot;
    NetworkSerializer::DeserializeSnapshot(data, snapshot);

    m_entityInterpolator.AddSnapshot(snapshot);

    // Apply server reconciliation for local entity
    auto it = snapshot.entityStates.Find(m_localEntityId);
    if (it != snapshot.entityStates.End())
    {
        m_clientPrediction.ApplyCorrection(it->second);
    }
}

void NetworkManager::ClientUpdate(float deltaTime)
{
    if (!m_config.isClient)
        return;

    m_timeSinceLastSend += deltaTime;
    m_entityInterpolator.Update(deltaTime);

    // Send input at configured rate
    if (m_timeSinceLastSend >= 1.0f / m_config.sendRate)
    {
        m_timeSinceLastSend = 0.0f;

        // Would send accumulated input here
    }
}

void NetworkManager::ServerReceiveInput(uint32_t clientId, const Vector<uint8_t>& data)
{
    if (!m_config.isServer)
        return;

    ++m_packetsReceived;
    m_bytesReceived += static_cast<uint32_t>(data.Size());

    NetworkedInput input;
    NetworkSerializer::DeserializeInput(data, input);

    m_serverReconciliation.ProcessClientInput(clientId, input);
}

void NetworkManager::ServerGenerateSnapshot(Vector<uint8_t>& outData)
{
    if (!m_config.isServer)
        return;

    NetworkSnapshot snapshot;
    m_serverReconciliation.GenerateSnapshot(m_sequenceNumber++, snapshot);

    NetworkSerializer::SerializeSnapshot(snapshot, outData);

    ++m_packetsSent;
    m_bytesSent += static_cast<uint32_t>(outData.Size());
}

void NetworkManager::ServerUpdate(float deltaTime)
{
    if (!m_config.isServer)
        return;

    // Server processes all inputs and updates entity states
}

bool NetworkManager::GetLocalState(NetworkedPoseState& outState) const
{
    if (!m_config.isClient)
        return false;

    outState = m_clientPrediction.PredictCurrentState();
    return true;
}

bool NetworkManager::GetEntityState(uint32_t entityId, NetworkedPoseState& outState) const
{
    auto it = m_entityInterpolator.m_interpolatedStates.Find(entityId);
    if (it != m_entityInterpolator.m_interpolatedStates.End())
    {
        outState = it->second;
        return true;
    }
    return false;
}

void NetworkManager::ResetStats()
{
    m_packetsSent = 0;
    m_packetsReceived = 0;
    m_bytesSent = 0;
    m_bytesReceived = 0;
    m_latency = 0;
    m_jitter = 0;
}

MMV2_NAMESPACE_END
