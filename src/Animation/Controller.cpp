// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Motion Matching Controller Implementation
// ============================================================================
// Main runtime controller that ties together database, search, blending,
// and playback. This is the heart of the motion matching system.
// ============================================================================

#include "MMV2/Animation/Controller.h"
#include "MMV2/Database/Database.h"
#include "MMV2/Search/KDTree.h"
#include "MMV2/Search/VPTree.h"
#include "MMV2/Search/ANN.h"
#include "MMV2/Animation/Blend/Blend.h"
#include "MMV2/Animation/Warp/Warp.h"
#include "MMV2/Animation/IK/IK.h"
#include "MMV2/History/PoseHistory.h"
#include "MMV2/Features/Feature.h"
#include "MMV2/Core/Math.h"
#include "MMV2/Profiler/Profiler.h"
#include <cmath>
#include <algorithm>
#include <thread>
#include <future>
#include <chrono>

MMV2_NAMESPACE_BEGIN

// ============================================================================
// Construction / Destruction
// ============================================================================

MotionMatchingController::MotionMatchingController()
    : m_database(nullptr)
    , m_lastSearchTime(0.0f)
    , m_updatePending(false)
    , m_isRunning(false)
    , m_stateCallback(nullptr)
    , m_stateCallbackUserData(nullptr)
    , m_transitionCallback(nullptr)
    , m_transitionCallbackUserData(nullptr)
{
    m_state.state = MotionMatchingState::Idle;
    m_state.currentEntry = -1;
    m_state.nextEntry = -1;
    m_state.currentTime = 0.0f;
    m_state.normalizedTime = 0.0f;
    m_state.phase = 0.0f;
    m_state.playbackSpeed = 1.0f;
    m_state.lastSearchTime = 0.0f;
    m_state.deltaTime = 0.0f;
    m_state.isMirrored = false;
    m_state.frameCounter = 0;
    m_state.avgSearchTime = 0.0f;
    m_state.totalSearchTime = 0.0f;
    m_state.searchCount = 0;
    m_state.transition = TransitionState();
}

MotionMatchingController::~MotionMatchingController() {
    Shutdown();
}

// ============================================================================
// Initialization
// ============================================================================

bool MotionMatchingController::Initialize(MotionDatabase* database, const MotionMatchingSettings& settings) {
    if (!database || !database->GetEntryCount() > 0) return false;
    if (m_isRunning) return false;

    m_database = database;
    m_settings = settings;

    // Initialize pose history
    m_poseHistory.Resize(settings.poseHistorySize);
    m_velocityHistory.Resize(settings.poseHistorySize);

    // Initialize feature extractor
    m_featureExtractor.Initialize(database->GetFeatureSchema());

    // Set initial state
    m_state.state = MotionMatchingState::Idle;
    m_state.currentEntry = 0;
    m_state.currentTime = 0.0f;

    // Get initial pose from database
    const DatabaseEntry* entry = database->GetEntry(0);
    if (entry) {
        m_state.currentPose = entry->pose;
        m_state.currentVelocity = entry->velocity;
        m_state.currentTrajectory = entry->trajectory;
        m_state.currentFeatures = entry->feature;
    }

    m_isRunning = true;
    SetState(MotionMatchingState::Playing);

    return true;
}

void MotionMatchingController::Shutdown() {
    m_isRunning = false;
    m_database = nullptr;
    m_state.state = MotionMatchingState::Idle;
}

// ============================================================================
// Main Update Loop
// ============================================================================

void MotionMatchingController::Update(float32 deltaTime) {
    if (!m_isRunning || !m_database) return;

    m_state.deltaTime = deltaTime;
    m_state.frameCounter++;

    // Update based on current state
    switch (m_state.state) {
        case MotionMatchingState::Playing:
            UpdatePlayback(deltaTime);
            break;
        case MotionMatchingState::Transitioning:
            UpdateTransition(deltaTime);
            break;
        case MotionMatchingState::Searching:
            PerformSearch();
            break;
        case MotionMatchingState::Blending:
            UpdatePlayback(deltaTime); // Blend during playback
            break;
        case MotionMatchingState::Paused:
            // Do nothing
            break;
        case MotionMatchingState::Error:
            // Error recovery
            break;
        default:
            break;
    }

    // Check if we should search for a new pose
    if (ShouldSearch()) {
        SetState(MotionMatchingState::Searching);
    }

    // Update pose history
    UpdatePoseHistory();

    // Apply post-processing
    ApplyMotionWarping();
    ApplyFootLocking();
    ApplyInertialization();
}

void MotionMatchingController::UpdateAsync(float32 deltaTime) {
    if (!m_isRunning || !m_database) return;

    // Launch search in background
    if (ShouldSearch() && !m_updatePending) {
        m_updatePending = true;
        m_searchFuture = std::async(std::launch::async, [this]() {
            PerformSearch();
            m_updatePending = false;
        });
    }

    // Continue playback on main thread
    UpdatePlayback(deltaTime);
    UpdatePoseHistory();
}

void MotionMatchingController::WaitForUpdate() {
    if (m_searchFuture.valid()) {
        m_searchFuture.wait();
    }
}

// ============================================================================
// Search
// ============================================================================

void MotionMatchingController::PerformSearch() {
    if (!m_database || m_database->GetEntryCount() == 0) return;

    auto startTime = std::chrono::high_resolution_clock::now();

    // Build query features from current state + desired trajectory
    FeatureVector queryFeatures;
    BuildQueryFeatures(queryFeatures);

    // Perform search
    Vector<SearchResult> results;
    if (m_settings.search.useKDTree) {
        PerformSearchKDTree(results);
    } else if (m_settings.search.useVPTree) {
        PerformSearchVPTree(results);
    } else if (m_settings.search.useANN) {
        PerformSearchANN(results);
    } else {
        PerformSearchBruteForce(results);
    }

    // Evaluate and score candidates
    EvaluateCandidates(results);

    // Select best result
    int32 bestIndex = SelectBestResult(results);

    if (bestIndex >= 0 && bestIndex != m_state.currentEntry) {
        // Start transition to new pose
        StartTransition(bestIndex);
    }

    // Update timing stats
    auto endTime = std::chrono::high_resolution_clock::now();
    float32 searchTime = std::chrono::duration<float32, std::milli>(endTime - startTime).count();
    m_lastSearchTime = searchTime;
    m_state.totalSearchTime += searchTime;
    m_state.searchCount++;
    m_state.avgSearchTime = m_state.totalSearchTime / m_state.searchCount;
    m_state.lastSearchTime = m_state.deltaTime;

    m_state.lastSearchResults = results;

    SetState(MotionMatchingState::Playing);
}

void MotionMatchingController::PerformSearchKDTree(Vector<SearchResult>& results) {
    if (!m_database) return;

    FeatureVector query;
    BuildQueryFeatures(query);

    Vector<int32> indices;
    Vector<float32> distances;
    m_database->FindKNearest(query, m_settings.search.kNearest, indices, &distances);

    results.Resize(indices.Size());
    for (size_type i = 0; i < indices.Size(); ++i) {
        results[i].entryIndex = indices[i];
        results[i].distance = distances[i];
        results[i].score = ComputeScore(indices[i], query);
        results[i].isValid = true;
    }
}

void MotionMatchingController::PerformSearchVPTree(Vector<SearchResult>& results) {
    PerformSearchKDTree(results); // Fallback for now
}

void MotionMatchingController::PerformSearchANN(Vector<SearchResult>& results) {
    PerformSearchKDTree(results); // Fallback for now
}

void MotionMatchingController::PerformSearchBruteForce(Vector<SearchResult>& results) {
    if (!m_database) return;

    FeatureVector query;
    BuildQueryFeatures(query);

    const int32 count = m_database->GetEntryCount();
    results.Reserve(count);

    for (int32 i = 0; i < count; ++i) {
        const DatabaseEntry* entry = m_database->GetEntry(i);
        if (!entry || !entry->isValid) continue;

        float32 dist = query.DistanceTo(entry->feature);
        SearchResult result;
        result.entryIndex = i;
        result.distance = dist;
        result.score = ComputeScore(i, query);
        result.isValid = true;
        results.PushBack(result);
    }

    // Sort by score (higher is better)
    std::partial_sort(results.begin(),
                      results.begin() + std::min(static_cast<size_type>(m_settings.search.kNearest), results.Size()),
                      results.end(),
                      [](const SearchResult& a, const SearchResult& b) {
        return a.score > b.score;
    });

    if (results.Size() > static_cast<size_type>(m_settings.search.kNearest)) {
        results.Resize(m_settings.search.kNearest);
    }
}

void MotionMatchingController::EvaluateCandidates(Vector<SearchResult>& results) {
    for (auto& result : results) {
        if (!result.isValid) continue;

        const DatabaseEntry* entry = m_database->GetEntry(result.entryIndex);
        if (!entry) {
            result.isValid = false;
            continue;
        }

        // Check transition validity
        if (m_state.currentEntry >= 0) {
            if (!m_database->IsValidTransition(m_state.currentEntry, result.entryIndex)) {
                result.score *= 0.5f; // Penalize but don't invalidate
            }
        }

        // Check pose history to avoid immediate repetition
        for (const auto& histEntry : m_poseHistory) {
            if (histEntry.GetBoneCount() > 0) {
                float32 similarity = entry->pose.CompareTo(histEntry);
                if (similarity > 0.95f) {
                    result.score *= 0.3f; // Heavy penalty for recent repeats
                }
            }
        }

        // Apply trajectory projection if enabled
        if (m_settings.search.enableTrajectoryProjection) {
            float32 trajError = ComputeTrajectoryError(entry->trajectory, m_state.currentTrajectory);
            result.score *= 1.0f - Math::Clamp(trajError * m_settings.search.trajectoryProjectionTime, 0.0f, 1.0f);
        }
    }

    // Remove invalid results
    size_type writeIdx = 0;
    for (size_type i = 0; i < results.Size(); ++i) {
        if (results[i].isValid) {
            if (writeIdx != i) results[writeIdx] = std::move(results[i]);
            ++writeIdx;
        }
    }
    results.Resize(writeIdx);
}

float32 MotionMatchingController::ComputeScore(int32 entryIndex, const FeatureVector& queryFeatures) const {
    if (!m_database || entryIndex < 0 || entryIndex >= m_database->GetEntryCount()) return 0.0f;

    const DatabaseEntry* entry = m_database->GetEntry(entryIndex);
    if (!entry) return 0.0f;

    float32 score = 0.0f;

    // Feature distance score (inverse)
    float32 featureDist = queryFeatures.DistanceTo(entry->feature);
    score += (1.0f / (1.0f + featureDist)) * m_settings.search.poseWeight;

    // Trajectory match
    float32 trajDist = m_state.currentTrajectory.DistanceTo(entry->trajectory);
    score += (1.0f / (1.0f + trajDist)) * m_settings.search.trajectoryWeight;

    // Velocity match
    float32 velDist = m_state.currentVelocity.DistanceTo(entry->velocity);
    score += (1.0f / (1.0f + velDist)) * m_settings.search.velocityWeight;

    // Contact match
    float32 contactScore = 1.0f - std::abs(entry->contactLeft - m_state.currentEntry >= 0 ?
        m_database->GetEntry(m_state.currentEntry)->contactLeft : 0.0f);
    contactScore += 1.0f - std::abs(entry->contactRight - m_state.currentEntry >= 0 ?
        m_database->GetEntry(m_state.currentEntry)->contactRight : 0.0f);
    score += (contactScore * 0.5f) * m_settings.search.contactWeight;

    // Phase continuity
    float32 phaseDiff = std::abs(entry->phase - m_state.phase);
    if (phaseDiff > 0.5f) phaseDiff = 1.0f - phaseDiff; // Wrap around
    score += (1.0f - phaseDiff * 2.0f) * m_settings.search.phaseWeight;

    return score;
}

float32 MotionMatchingController::ComputeTrajectoryError(const Trajectory& candidate, const Trajectory& desired) const {
    if (candidate.sampleCount != desired.sampleCount) return 1.0f;

    float32 error = 0.0f;
    for (int32 i = 0; i < candidate.sampleCount; ++i) {
        error += (candidate.points[i].position - desired.points[i].position).Length();
        error += (candidate.points[i].velocity - desired.points[i].velocity).Length() * 0.5f;
    }
    return error / candidate.sampleCount;
}

int32 MotionMatchingController::SelectBestResult(const Vector<SearchResult>& results) const {
    if (results.Empty()) return -1;

    // Pick highest scoring result
    int32 bestIdx = 0;
    float32 bestScore = results[0].score;

    for (size_type i = 1; i < results.Size(); ++i) {
        if (results[i].score > bestScore) {
            bestScore = results[i].score;
            bestIdx = static_cast<int32>(i);
        }
    }

    // Add some randomness for variety (configurable)
    if (results.Size() > 1 && m_settings.search.responsiveness < 1.0f) {
        float32 randomFactor = static_cast<float32>(rand()) / RAND_MAX;
        if (randomFactor > m_settings.search.responsiveness) {
            int32 randomIdx = rand() % std::min(static_cast<int32>(results.Size()), 3);
            bestIdx = randomIdx;
        }
    }

    return results[bestIdx].entryIndex;
}

// ============================================================================
// Transition
// ============================================================================

void MotionMatchingController::StartTransition(int32 toEntry) {
    if (toEntry < 0 || toEntry >= m_database->GetEntryCount()) return;

    const DatabaseEntry* fromEntry = m_database->GetEntry(m_state.currentEntry);
    const DatabaseEntry* targetEntry = m_database->GetEntry(toEntry);
    if (!fromEntry || !targetEntry) return;

    m_state.transition.type = TransitionType::Search;
    m_state.transition.fromEntry = m_state.currentEntry;
    m_state.transition.toEntry = toEntry;
    m_state.transition.fromPose = m_state.currentPose;
    m_state.transition.toPose = targetEntry->pose;
    m_state.transition.progress = 0.0f;
    m_state.transition.duration = m_settings.transition.blendTime;
    m_state.transition.isActive = true;

    m_state.nextEntry = toEntry;

    SetState(MotionMatchingState::Transitioning);

    if (m_transitionCallback) {
        m_transitionCallback(m_state.currentEntry, toEntry, TransitionType::Search, m_transitionCallbackUserData);
    }
}

void MotionMatchingController::UpdateTransition(float32 deltaTime) {
    if (!m_state.transition.isActive) {
        SetState(MotionMatchingState::Playing);
        return;
    }

    m_state.transition.progress += deltaTime / m_state.transition.duration;

    if (m_state.transition.progress >= 1.0f) {
        // Transition complete
        m_state.transition.progress = 1.0f;
        m_state.transition.isActive = false;
        m_state.currentEntry = m_state.transition.toEntry;
        m_state.currentTime = m_database->GetEntry(m_state.currentEntry)->time;
        m_state.currentPose = m_state.transition.toPose;

        SetState(MotionMatchingState::Playing);
        return;
    }

    // Blend between poses
    BlendRequest request;
    request.sourcePose = &m_state.transition.fromPose;
    request.targetPose = &m_state.transition.toPose;
    request.blendWeight = m_state.transition.progress;
    request.blendType = static_cast<BlendType>(m_settings.transition.blendCurve);
    request.deltaTime = deltaTime;
    request.useInertialBlending = m_settings.playback.enableInertialization;

    BlendResult result;
    PoseBlender::Blend(request, result);

    m_state.currentPose = result.pose;
}

bool MotionMatchingController::ForceTransition(int32 entryIndex, const TransitionSettings& settings) {
    if (!m_database || entryIndex < 0 || entryIndex >= m_database->GetEntryCount()) return false;

    m_settings.transition = settings;
    StartTransition(entryIndex);
    return true;
}

bool MotionMatchingController::ForceTransition(const char* clipName, float32 normalizedTime, const TransitionSettings& settings) {
    if (!m_database) return false;

    // Find entry matching clip name and time
    for (int32 i = 0; i < m_database->GetEntryCount(); ++i) {
        const DatabaseEntry* entry = m_database->GetEntry(i);
        if (!entry) continue;

        const AnimationClip* clip = m_database->GetClip(entry->clipIndex);
        if (clip && clip->name == clipName) {
            float32 entryNormTime = entry->time / clip->duration;
            if (std::abs(entryNormTime - normalizedTime) < 0.01f) {
                m_settings.transition = settings;
                StartTransition(i);
                return true;
            }
        }
    }

    return false;
}

// ============================================================================
// Playback
// ============================================================================

void MotionMatchingController::UpdatePlayback(float32 deltaTime) {
    if (m_state.currentEntry < 0 || !m_database) return;

    const DatabaseEntry* entry = m_database->GetEntry(m_state.currentEntry);
    if (!entry) return;

    // Advance time
    m_state.currentTime += deltaTime * m_state.playbackSpeed * m_settings.playback.playbackSpeed;

    const AnimationClip* clip = m_database->GetClip(entry->clipIndex);
    if (!clip) return;

    // Loop or clamp
    if (m_state.currentTime > clip->duration) {
        if (clip->isLooping) {
            m_state.currentTime = std::fmod(m_state.currentTime, clip->duration);
        } else {
            m_state.currentTime = clip->duration;
        }
    }

    // Update normalized time
    m_state.normalizedTime = clip->duration > 0.0f ? m_state.currentTime / clip->duration : 0.0f;

    // Update phase
    m_state.phase = entry->phase + m_state.normalizedTime;
    if (m_state.phase > 1.0f) m_state.phase -= 1.0f;

    // Get pose at current time
    const Pose* pose = clip->GetPoseAtTime(m_state.currentTime);
    if (pose) {
        m_state.currentPose = *pose;
    }

    // Get velocity
    const PoseVelocity* vel = clip->GetVelocityAtTime(m_state.currentTime);
    if (vel) {
        m_state.currentVelocity = *vel;
    }

    // Get trajectory
    int32 frame = clip->GetFrameAtTime(m_state.currentTime);
    if (frame >= 0 && frame < static_cast<int32>(clip->trajectories.Size())) {
        m_state.currentTrajectory = clip->trajectories[frame];
    }
}

void MotionMatchingController::UpdatePose(float32 deltaTime) {
    // Additional pose updates (IK, constraints, etc.)
    if (m_settings.playback.enableFootLocking) {
        ApplyFootLocking();
    }
}

// ============================================================================
// Trajectory
// ============================================================================

void MotionMatchingController::SetTrajectory(const Trajectory& trajectory) {
    m_state.currentTrajectory = trajectory;
}

void MotionMatchingController::SetTrajectoryFromInput(const Vec3& moveInput) {
    TrajectoryGenerator generator = m_settings.trajectoryGenerator;
    m_state.currentTrajectory = generator.GenerateFromInput(moveInput, 5.0f, 180.0f * MMV2_DEG2RAD);
}

void MotionMatchingController::UpdateTrajectory() {
    // Project future trajectory based on current velocity
    if (m_state.currentVelocity.linear.Size() > 0) {
        Vec3 rootVel = m_state.currentVelocity.linear[0];
        Vec3 rootPos = m_state.currentPose.GetBoneTransform(0).position;
        Quat rootRot = m_state.currentPose.GetBoneTransform(0).rotation;

        TrajectoryGenerator generator = m_settings.trajectoryGenerator;
        m_state.currentTrajectory = generator.GenerateFromVelocity(rootVel, rootPos, rootRot);
    }
}

// ============================================================================
// Query Features
// ============================================================================

void MotionMatchingController::BuildQueryFeatures(FeatureVector& outFeatures) {
    outFeatures = m_database->ExtractFeatures(m_state.currentPose, m_state.currentVelocity, m_state.currentTrajectory);

    // Add history influence
    if (!m_poseHistory.Empty() && m_settings.search.poseResponsiveness > 0.0f) {
        FeatureVector historyFeatures = m_database->ExtractFeatures(
            m_poseHistory.Back(), m_velocityHistory.Back(), m_state.currentTrajectory);

        for (int32 i = 0; i < outFeatures.Size(); ++i) {
            outFeatures[i] = Math::Lerp(historyFeatures[i], outFeatures[i], m_settings.search.poseResponsiveness);
        }
    }
}

// ============================================================================
// Post-Processing
// ============================================================================

void MotionMatchingController::ApplyMotionWarping() {
    if (!m_settings.playback.enableMotionWarping) return;

    // Apply motion warping to align with environment
    // This is a simplified version - full implementation would use the Warp system
    const DatabaseEntry* entry = m_database->GetEntry(m_state.currentEntry);
    if (!entry) return;

    // Adjust root position based on trajectory
    Vec3 targetPos = m_state.currentTrajectory.points[0].position;
    Vec3 currentPos = m_state.currentPose.GetBoneTransform(0).position;
    Vec3 delta = (targetPos - currentPos) * m_settings.playback.motionWarpStrength;

    Transform rootTransform = m_state.currentPose.GetBoneTransform(0);
    rootTransform.position += delta;
    m_state.currentPose.SetBoneTransform(0, rootTransform);
}

void MotionMatchingController::ApplyFootLocking() {
    if (!m_settings.playback.enableFootLocking) return;

    const BoneHierarchy& hierarchy = m_state.currentPose.GetBoneHierarchy();
    for (int32 i = 0; i < hierarchy.GetBoneCount(); ++i) {
        const Bone& bone = hierarchy.GetBone(i);
        if (bone.name.Contains("Foot") || bone.name.Contains("foot")) {
            float32 height = m_state.currentPose.GetBoneTransform(i).position.y;
            if (height < m_settings.playback.footLockThreshold) {
                // Lock foot to ground
                Transform t = m_state.currentPose.GetBoneTransform(i);
                t.position.y = 0.0f;
                m_state.currentPose.SetBoneTransform(i, t);
            }
        }
    }
}

void MotionMatchingController::ApplyInertialization() {
    if (!m_settings.playback.enableInertialization) return;

    // Preserve momentum during transitions
    // This would use the inertial blending system from Blend module
}

// ============================================================================
// Pose History
// ============================================================================

void MotionMatchingController::UpdatePoseHistory() {
    if (m_poseHistory.Size() >= static_cast<size_type>(m_settings.poseHistorySize)) {
        // Shift history (ring buffer would be more efficient)
        for (size_type i = 1; i < m_poseHistory.Size(); ++i) {
            m_poseHistory[i - 1] = m_poseHistory[i];
            m_velocityHistory[i - 1] = m_velocityHistory[i];
        }
        m_poseHistory.Back() = m_state.currentPose;
        m_velocityHistory.Back() = m_state.currentVelocity;
    } else {
        m_poseHistory.PushBack(m_state.currentPose);
        m_velocityHistory.PushBack(m_state.currentVelocity);
    }
}

// ============================================================================
// Search Decision
// ============================================================================

bool MotionMatchingController::ShouldSearch() const {
    if (m_state.state != MotionMatchingState::Playing) return false;
    if (!m_database) return false;

    // Minimum interval between searches
    float32 timeSinceLastSearch = m_state.deltaTime - m_state.lastSearchTime;
    if (timeSinceLastSearch < m_settings.searchInterval) return false;

    // Don't search during active transition
    if (m_state.transition.isActive) return false;

    // Check if current pose is near end of clip
    const DatabaseEntry* entry = m_database->GetEntry(m_state.currentEntry);
    if (entry) {
        const AnimationClip* clip = m_database->GetClip(entry->clipIndex);
        if (clip && !clip->isLooping) {
            float32 timeRemaining = clip->duration - entry->time;
            if (timeRemaining < m_settings.transition.blendTime * 2.0f) {
                return true; // Search before clip ends
            }
        }
    }

    // Check trajectory deviation
    if (m_settings.search.enableTrajectoryProjection) {
        // If desired trajectory differs significantly from current, search
        // Simplified check
        return true;
    }

    return false;
}

// ============================================================================
// State Management
// ============================================================================

void MotionMatchingController::SetState(MotionMatchingState newState) {
    if (m_state.state == newState) return;

    MotionMatchingState oldState = m_state.state;
    m_state.state = newState;

    if (m_stateCallback) {
        m_stateCallback(oldState, newState, m_stateCallbackUserData);
    }
}

void MotionMatchingController::SetPlaybackSpeed(float32 speed) {
    m_state.playbackSpeed = Math::Clamp(speed, 0.0f, 5.0f);
}

void MotionMatchingController::SetMirrored(bool mirrored) {
    m_state.isMirrored = mirrored;
    if (mirrored) {
        m_state.currentPose.Mirror();
    }
}

void MotionMatchingController::Pause() {
    if (m_state.state == MotionMatchingState::Playing) {
        SetState(MotionMatchingState::Paused);
    }
}

void MotionMatchingController::Resume() {
    if (m_state.state == MotionMatchingState::Paused) {
        SetState(MotionMatchingState::Playing);
    }
}

void MotionMatchingController::Reset() {
    m_state.currentEntry = 0;
    m_state.currentTime = 0.0f;
    m_state.normalizedTime = 0.0f;
    m_state.phase = 0.0f;
    m_state.transition.isActive = false;
    m_poseHistory.Clear();
    m_velocityHistory.Clear();

    if (m_database && m_database->GetEntryCount() > 0) {
        const DatabaseEntry* entry = m_database->GetEntry(0);
        if (entry) {
            m_state.currentPose = entry->pose;
            m_state.currentVelocity = entry->velocity;
            m_state.currentTrajectory = entry->trajectory;
        }
    }

    SetState(MotionMatchingState::Idle);
}

// ============================================================================
// Settings
// ============================================================================

void MotionMatchingController::SetSettings(const MotionMatchingSettings& settings) {
    m_settings = settings;
}

// ============================================================================
// Callbacks
// ============================================================================

void MotionMatchingController::SetStateChangedCallback(StateChangedCallback callback, void* userData) {
    m_stateCallback = callback;
    m_stateCallbackUserData = userData;
}

void MotionMatchingController::SetTransitionCallback(TransitionCallback callback, void* userData) {
    m_transitionCallback = callback;
    m_transitionCallbackUserData = userData;
}

// ============================================================================
// Debug
// ============================================================================

String MotionMatchingController::GetDebugInfo() const {
    String info;
    info += String::Format("State: %d\\n", static_cast<int32>(m_state.state));
    info += String::Format("Current Entry: %d\\n", m_state.currentEntry);
    info += String::Format("Next Entry: %d\\n", m_state.nextEntry);
    info += String::Format("Time: %.3f / %.3f\\n", m_state.currentTime, m_state.normalizedTime);
    info += String::Format("Phase: %.3f\\n", m_state.phase);
    info += String::Format("Playback Speed: %.2f\\n", m_state.playbackSpeed);
    info += String::Format("Search Count: %d\\n", m_state.searchCount);
    info += String::Format("Avg Search Time: %.3f ms\\n", m_state.avgSearchTime);
    info += String::Format("Last Search Time: %.3f ms\\n", m_lastSearchTime);
    info += String::Format("Pose History: %zu/%d\\n", m_poseHistory.Size(), m_settings.poseHistorySize);
    info += String::Format("Transition Active: %s\\n", m_state.transition.isActive ? "Yes" : "No");
    info += String::Format("Transition Progress: %.2f\\n", m_state.transition.progress);
    info += String::Format("Is Mirrored: %s\\n", m_state.isMirrored ? "Yes" : "No");
    return info;
}

MMV2_NAMESPACE_END
