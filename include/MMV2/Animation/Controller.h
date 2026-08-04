#pragma once
#ifndef MMV2_CONTROLLER_H
#define MMV2_CONTROLLER_H

#include "Config.h"
#include "Pose.h"
#include "Feature.h"
#include "Trajectory.h"
#include "Database.h"
#include "Bone.h"
#include "Vector.h"
#include "HashMap.h"
#include <atomic>

MMV2_NAMESPACE_BEGIN

enum class MotionMatchingState : uint8 {
    Idle = 0,
    Searching = 1,
    Transitioning = 2,
    Playing = 3,
    Blending = 4,
    Paused = 5,
    Error = 6
};

enum class TransitionType : uint8 {
    None = 0,
    Search = 1,
    Scheduled = 2,
    Event = 3,
    Interrupt = 4,
    Blend = 5
};

struct TransitionSettings {
    float32 blendTime;
    float32 blendInTime;
    float32 blendOutTime;
    int32 blendCurve;
    bool syncPhase;
    bool preserveMomentum;
    float32 minTransitionTime;
    float32 maxTransitionTime;

    TransitionSettings()
        : blendTime(0.2f), blendInTime(0.1f), blendOutTime(0.1f), blendCurve(0),
          syncPhase(false), preserveMomentum(true),
          minTransitionTime(0.1f), maxTransitionTime(1.0f) {}
};

struct SearchSettings {
    int32 kNearest;
    float32 searchRadius;
    float32 trajectoryWeight;
    float32 poseWeight;
    float32 velocityWeight;
    float32 contactWeight;
    float32 phaseWeight;
    bool useKDTree;
    bool useVPTree;
    bool useANN;
    bool enableMirroring;
    bool enableTrajectoryProjection;
    float32 trajectoryProjectionTime;
    float32 responsiveness;
    float32 poseResponsiveness;

    SearchSettings()
        : kNearest(10), searchRadius(1000.0f), trajectoryWeight(1.0f),
          poseWeight(0.5f), velocityWeight(0.3f), contactWeight(0.2f),
          phaseWeight(0.1f), useKDTree(true), useVPTree(false), useANN(false),
          enableMirroring(false), enableTrajectoryProjection(true),
          trajectoryProjectionTime(1.0f), responsiveness(0.5f), poseResponsiveness(0.3f) {}
};

struct PlaybackSettings {
    float32 playbackSpeed;
    bool enableTimeWarping;
    bool enableMotionWarping;
    bool enableFootLocking;
    bool enableInertialization;
    float32 timeWarpStrength;
    float32 motionWarpStrength;
    float32 footLockThreshold;

    PlaybackSettings()
        : playbackSpeed(1.0f), enableTimeWarping(true), enableMotionWarping(true),
          enableFootLocking(true), enableInertialization(true),
          timeWarpStrength(0.5f), motionWarpStrength(0.7f), footLockThreshold(0.05f) {}
};

struct MotionMatchingSettings {
    SearchSettings search;
    TransitionSettings transition;
    PlaybackSettings playback;
    TrajectoryGenerator trajectoryGenerator;
    FeatureSchema featureSchema;
    float32 updateRate;
    float32 searchInterval;
    float32 minSearchInterval;
    int32 poseHistorySize;
    bool enableMultiThreading;
    bool enableProfiling;
    bool enableDebugDraw;

    MotionMatchingSettings()
        : updateRate(60.0f), searchInterval(0.1f), minSearchInterval(0.05f),
          poseHistorySize(60), enableMultiThreading(true),
          enableProfiling(false), enableDebugDraw(false) {}
};

struct SearchResult {
    int32 entryIndex;
    float32 distance;
    float32 score;
    bool isValid;

    SearchResult() : entryIndex(-1), distance(0.0f), score(0.0f), isValid(false) {}
};

struct TransitionState {
    TransitionType type;
    float32 progress;
    float32 duration;
    int32 fromEntry;
    int32 toEntry;
    Pose fromPose;
    Pose toPose;
    bool isActive;

    TransitionState() : type(TransitionType::None), progress(0.0f), duration(0.0f),
                        fromEntry(-1), toEntry(-1), isActive(false) {}
};

struct ControllerState {
    MotionMatchingState state;
    int32 currentEntry;
    int32 nextEntry;
    float32 currentTime;
    float32 normalizedTime;
    float32 phase;
    float32 playbackSpeed;
    float32 lastSearchTime;
    float32 deltaTime;
    Pose currentPose;
    PoseVelocity currentVelocity;
    Trajectory currentTrajectory;
    FeatureVector currentFeatures;
    Vector<SearchResult> lastSearchResults;
    TransitionState transition;
    bool isMirrored;
    int32 frameCounter;
    float32 avgSearchTime;
    float32 totalSearchTime;
    int32 searchCount;

    ControllerState() : state(MotionMatchingState::Idle), currentEntry(-1), nextEntry(-1),
                        currentTime(0.0f), normalizedTime(0.0f), phase(0.0f),
                        playbackSpeed(1.0f), lastSearchTime(0.0f), deltaTime(0.0f),
                        isMirrored(false), frameCounter(0), avgSearchTime(0.0f),
                        totalSearchTime(0.0f), searchCount(0) {}
};

class MMV2_API MotionMatchingController {
public:
    MotionMatchingController();
    ~MotionMatchingController();

    bool Initialize(MotionDatabase* database, const MotionMatchingSettings& settings);
    void Shutdown();
    bool IsInitialized() const { return m_database != nullptr; }

    void Update(float32 deltaTime);
    void UpdateAsync(float32 deltaTime);
    void WaitForUpdate();

    const Pose& GetCurrentPose() const { return m_state.currentPose; }
    const PoseVelocity& GetCurrentVelocity() const { return m_state.currentVelocity; }
    const Trajectory& GetCurrentTrajectory() const { return m_state.currentTrajectory; }
    const ControllerState& GetState() const { return m_state; }

    void SetTrajectory(const Trajectory& trajectory);
    void SetTrajectoryFromInput(const Vec3& moveInput);
    void SetPlaybackSpeed(float32 speed);
    void SetMirrored(bool mirrored);

    bool ForceTransition(int32 entryIndex, const TransitionSettings& settings);
    bool ForceTransition(const char* clipName, float32 normalizedTime, const TransitionSettings& settings);
    void Pause();
    void Resume();
    void Reset();

    void SetSettings(const MotionMatchingSettings& settings);
    const MotionMatchingSettings& GetSettings() const { return m_settings; }

    // Debug
    void SetDebugDrawEnabled(bool enabled) { m_settings.enableDebugDraw = enabled; }
    bool IsDebugDrawEnabled() const { return m_settings.enableDebugDraw; }
    const Vector<SearchResult>& GetLastSearchResults() const { return m_state.lastSearchResults; }

    // Events
    using StateChangedCallback = void(*)(MotionMatchingState oldState, MotionMatchingState newState, void* userData);
    using TransitionCallback = void(*)(int32 fromEntry, int32 toEntry, TransitionType type, void* userData);
    void SetStateChangedCallback(StateChangedCallback callback, void* userData);
    void SetTransitionCallback(TransitionCallback callback, void* userData);

    // Profiling
    float32 GetAvgSearchTime() const { return m_state.avgSearchTime; }
    float32 GetLastSearchTime() const { return m_lastSearchTime; }
    int32 GetSearchCount() const { return m_state.searchCount; }

private:
    MotionDatabase* m_database;
    MotionMatchingSettings m_settings;
    ControllerState m_state;
    Vector<Pose> m_poseHistory;
    Vector<PoseVelocity> m_velocityHistory;
    FeatureExtractorManager m_featureExtractor;
    float32 m_lastSearchTime;
    bool m_updatePending;
    std::atomic<bool> m_isRunning;

    StateChangedCallback m_stateCallback;
    void* m_stateCallbackUserData;
    TransitionCallback m_transitionCallback;
    void* m_transitionCallbackUserData;

    void SetState(MotionMatchingState newState);
    void PerformSearch();
    void PerformSearchKDTree(Vector<SearchResult>& results);
    void PerformSearchVPTree(Vector<SearchResult>& results);
    void PerformSearchANN(Vector<SearchResult>& results);
    void PerformSearchBruteForce(Vector<SearchResult>& results);
    void EvaluateCandidates(const Vector<int32>& candidates, Vector<SearchResult>& results);
    float32 ComputeScore(int32 entryIndex, const FeatureVector& queryFeatures);
    void StartTransition(int32 toEntry);
    void UpdateTransition(float32 deltaTime);
    void UpdatePlayback(float32 deltaTime);
    void UpdatePose(float32 deltaTime);
    void ApplyMotionWarping();
    void ApplyFootLocking();
    void ApplyInertialization();
    void BuildQueryFeatures(FeatureVector& outFeatures);
    void UpdateTrajectory();
    bool ShouldSearch() const;
    int32 SelectBestResult(const Vector<SearchResult>& results);
};

MMV2_NAMESPACE_END

#endif
