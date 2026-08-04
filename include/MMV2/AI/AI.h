// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// AI Integration System
// ============================================================================
// Provides AI-driven animation selection, behavior tree integration, and
// autonomous character animation control.
// ============================================================================

#pragma once

#include "MMV2/Core/Config.h"
#include "MMV2/Core/Vector.h"
#include "MMV2/Core/HashMap.h"
#include "MMV2/Core/String.h"
#include "MMV2/Animation/Chooser/Chooser.h"

MMV2_NAMESPACE_BEGIN

// ============================================================================
// AI Animation Request
// ============================================================================

struct AIAnimationRequest
{
    String actionName;
    Vec3 desiredVelocity;
    Vec3 desiredDirection;
    float desiredSpeed;
    bool wantsMove;
    bool wantsStop;
    bool wantsTurn;
    float urgency;              // 0-1, how quickly to transition
    Vector<String> preferredTags;
    Vector<String> excludedTags;

    AIAnimationRequest() : desiredSpeed(0), wantsMove(false), wantsStop(false),
                           wantsTurn(false), urgency(0.5f) {}
};

// ============================================================================
// AI Animation State
// ============================================================================

enum class AIAnimationState : uint32_t
{
    Idle = 0,
    Moving,
    Turning,
    Starting,
    Stopping,
    Reacting,
    Interacting,
    Custom
};

struct AIStateInfo
{
    AIAnimationState state;
    float enterTime;
    float duration;
    bool canInterrupt;
    float interruptPriority;
    String debugReason;

    AIStateInfo() : state(AIAnimationState::Idle), enterTime(0), duration(0),
                    canInterrupt(true), interruptPriority(0) {}
};

// ============================================================================
// Behavior Context
// ============================================================================

struct BehaviorContext
{
    Vec3 characterPosition;
    Vec3 characterVelocity;
    Quat characterRotation;
    Vec3 targetPosition;
    Vec3 targetVelocity;
    float distanceToTarget;
    float timeToTarget;
    bool hasLineOfSight;
    bool isInCombat;
    bool isAlerted;
    float health;
    float stamina;
    Vector<String> activeTags;

    BehaviorContext() : distanceToTarget(0), timeToTarget(0), hasLineOfSight(false),
                        isInCombat(false), isAlerted(false), health(1.0f), stamina(1.0f) {}
};

// ============================================================================
// AI Animation Controller
// ============================================================================

class MMV2_API AIAnimationController
{
public:
    AIAnimationController();

    void Initialize(AnimationChooser* chooser);
    void Shutdown();

    // Main update
    void Update(float deltaTime, const BehaviorContext& context);

    // Request animation
    void RequestAnimation(const AIAnimationRequest& request);
    void RequestAnimation(const String& actionName, float urgency = 0.5f);

    // Force animation
    void ForceAnimation(uint32_t animationIndex, float transitionTime = 0.2f);
    void ForceAction(const String& actionName, float transitionTime = 0.2f);

    // State queries
    AIAnimationState GetCurrentState() const { return m_currentState.state; }
    const AIStateInfo& GetCurrentStateInfo() const { return m_currentState; }
    bool IsTransitioning() const { return m_transitionProgress < 1.0f; }
    float GetTransitionProgress() const { return m_transitionProgress; }

    // Current selection
    uint32_t GetSelectedAnimation() const { return m_selectedAnimation; }
    float GetAnimationTime() const { return m_animationTime; }

    // Configuration
    void SetReactionTime(float time) { m_reactionTime = time; }
    void SetMinTransitionTime(float time) { m_minTransitionTime = time; }
    void SetMaxTransitionTime(float time) { m_maxTransitionTime = time; }
    void SetIdleTimeout(float timeout) { m_idleTimeout = timeout; }

    // Tag preferences
    void SetPreferredTags(const Vector<String>& tags) { m_preferredTags = tags; }
    void SetExcludedTags(const Vector<String>& tags) { m_excludedTags = tags; }

    // Debug
    String GetDebugInfo() const;
    void SetDebugEnabled(bool enabled) { m_debugEnabled = enabled; }

private:
    AnimationChooser* m_chooser;
    AIStateInfo m_currentState;
    AIStateInfo m_pendingState;

    uint32_t m_selectedAnimation;
    float m_animationTime;
    float m_transitionProgress;
    float m_stateTimer;

    float m_reactionTime;
    float m_minTransitionTime;
    float m_maxTransitionTime;
    float m_idleTimeout;

    Vector<String> m_preferredTags;
    Vector<String> m_excludedTags;

    bool m_debugEnabled;

    void EvaluateState(const BehaviorContext& context);
    void SelectAnimation(const BehaviorContext& context);
    void StartTransition(const AIStateInfo& newState);
    void UpdateTransition(float deltaTime);
    bool CanInterruptCurrentState(float priority) const;
    float ComputeUrgency(const BehaviorContext& context) const;
};

// ============================================================================
// Behavior Tree Integration
// ============================================================================

enum class BTNodeStatus : uint32_t
{
    Invalid = 0,
    Success,
    Failure,
    Running
};

class MMV2_API IBTNode
{
public:
    virtual ~IBTNode() = default;
    virtual BTNodeStatus Tick(float deltaTime, const BehaviorContext& context) = 0;
    virtual void OnEnter(const BehaviorContext& context) {}
    virtual void OnExit(const BehaviorContext& context) {}
    virtual String GetName() const = 0;
};

class MMV2_API BTAnimationNode : public IBTNode
{
public:
    BTAnimationNode(AIAnimationController* controller, const String& actionName);

    BTNodeStatus Tick(float deltaTime, const BehaviorContext& context) override;
    void OnEnter(const BehaviorContext& context) override;
    void OnExit(const BehaviorContext& context) override;
    String GetName() const override { return "Animation_" + m_actionName; }

    void SetWaitForCompletion(bool wait) { m_waitForCompletion = wait; }
    void SetLoop(bool loop) { m_loop = loop; }

private:
    AIAnimationController* m_controller;
    String m_actionName;
    bool m_waitForCompletion;
    bool m_loop;
    float m_elapsedTime;
};

class MMV2_API BTSelectorNode : public IBTNode
{
public:
    BTSelectorNode(const String& name);

    void AddChild(UniquePtr<IBTNode> child);
    void RemoveChild(uint32_t index);

    BTNodeStatus Tick(float deltaTime, const BehaviorContext& context) override;
    String GetName() const override { return m_name; }

private:
    String m_name;
    Vector<UniquePtr<IBTNode>> m_children;
    uint32_t m_currentChild;
};

class MMV2_API BTSequenceNode : public IBTNode
{
public:
    BTSequenceNode(const String& name);

    void AddChild(UniquePtr<IBTNode> child);
    void RemoveChild(uint32_t index);

    BTNodeStatus Tick(float deltaTime, const BehaviorContext& context) override;
    String GetName() const override { return m_name; }

private:
    String m_name;
    Vector<UniquePtr<IBTNode>> m_children;
    uint32_t m_currentChild;
};

class MMV2_API BTConditionNode : public IBTNode
{
public:
    using ConditionFunc = bool(*)(const BehaviorContext& context);

    BTConditionNode(const String& name, ConditionFunc condition);

    BTNodeStatus Tick(float deltaTime, const BehaviorContext& context) override;
    String GetName() const override { return m_name; }

private:
    String m_name;
    ConditionFunc m_condition;
};

// ============================================================================
// Behavior Tree
// ============================================================================

class MMV2_API BehaviorTree
{
public:
    BehaviorTree();

    void SetRoot(UniquePtr<IBTNode> root);
    void Update(float deltaTime, const BehaviorContext& context);

    BTNodeStatus GetStatus() const { return m_status; }
    String GetActiveNodeName() const;

    void Reset();

private:
    UniquePtr<IBTNode> m_root;
    BTNodeStatus m_status;
    IBTNode* m_activeNode;
};

// ============================================================================
// AI Director
// ============================================================================

struct AIDirectorConfig
{
    float updateInterval;
    uint32_t maxActiveAgents;
    bool enableLOD;
    float lodDistances[3];
    bool enableCulling;
    float cullDistance;
};

class MMV2_API AIDirector
{
public:
    void Initialize(const AIDirectorConfig& config);
    void Shutdown();

    void RegisterAgent(AIAnimationController* agent);
    void UnregisterAgent(AIAnimationController* agent);

    void Update(float deltaTime);
    void UpdateAgentLOD(AIAnimationController* agent, float distance);

    void SetGlobalState(const String& state);
    String GetGlobalState() const { return m_globalState; }

    void SetDifficulty(float difficulty) { m_difficulty = difficulty; }
    float GetDifficulty() const { return m_difficulty; }

private:
    AIDirectorConfig m_config;
    Vector<AIAnimationController*> m_agents;
    String m_globalState;
    float m_difficulty;
    float m_updateTimer;
};

MMV2_NAMESPACE_END
