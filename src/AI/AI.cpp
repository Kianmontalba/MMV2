// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// AI Integration System Implementation
// ============================================================================
// Provides AI-driven animation selection, behavior tree integration, and
// autonomous character animation control. Bridges game AI with motion matching.
// ============================================================================

#include "MMV2/AI/AI.h"
#include "MMV2/Core/Math.h"
#include "MMV2/Core/Serializer.h"
#include "MMV2/Database/Database.h"
#include "MMV2/Animation/Controller.h"
#include <cmath>

MMV2_NAMESPACE_BEGIN

// ============================================================================
// AIAnimationController
// ============================================================================

AIAnimationController::AIAnimationController()
    : m_currentState(AIAnimationState::Idle)
    , m_previousState(AIAnimationState::Idle)
    , m_stateEnterTime(0.0f)
    , m_stateDuration(0.0f)
    , m_canInterrupt(true)
    , m_interruptPriority(0.0f)
    , m_motionMatchingController(nullptr)
    , m_database(nullptr)
    , m_chooserTable(nullptr)
    , m_staminaSystem(nullptr)
    , m_debugEnabled(false)
    , m_blendOutTime(0.2f)
    , m_blendInTime(0.2f)
    , m_minStateTime(0.1f)
    , m_reactionCooldown(0.0f)
    , m_reactionCooldownDuration(0.5f)
{
}

AIAnimationController::~AIAnimationController()
{
    for (auto& pair : m_behaviorTrees)
    {
        delete pair.value;
    }
    m_behaviorTrees.Clear();
}

void AIAnimationController::Initialize(MotionMatchingController* mmController,
                                        PoseSearchDatabase* database,
                                        ChooserTable* chooserTable)
{
    m_motionMatchingController = mmController;
    m_database = database;
    m_chooserTable = chooserTable;
}

void AIAnimationController::SetStaminaSystem(StaminaSystem* stamina)
{
    m_staminaSystem = stamina;
}

// ============================================================================
// State Management
// ============================================================================

void AIAnimationController::ChangeState(AIAnimationState newState, const String& reason)
{
    if (newState == m_currentState)
        return;

    if (!m_canInterrupt && m_interruptPriority < GetStatePriority(newState))
    {
        if (m_debugEnabled)
        {
            Log::Debug("AIAnimation: State change blocked - current state not interruptible");
        }
        return;
    }

    float currentTime = GetCurrentTime();
    float timeInState = currentTime - m_stateEnterTime;

    if (timeInState < m_minStateTime)
    {
        if (m_debugEnabled)
        {
            Log::Debug("AIAnimation: State change blocked - minimum state time not reached");
        }
        return;
    }

    // Record previous state
    m_previousState = m_currentState;
    m_currentState = newState;
    m_stateEnterTime = currentTime;
    m_stateDuration = 0.0f;

    // Update state info
    m_stateInfo.state = newState;
    m_stateInfo.enterTime = currentTime;
    m_stateInfo.duration = 0.0f;
    m_stateInfo.canInterrupt = true;
    m_stateInfo.interruptPriority = GetStatePriority(newState);
    m_stateInfo.debugReason = reason;

    // Set interruptibility based on state
    switch (newState)
    {
        case AIAnimationState::Reacting:
        case AIAnimationState::Interacting:
            m_canInterrupt = false;
            m_interruptPriority = 10.0f;
            break;
        case AIAnimationState::Starting:
        case AIAnimationState::Stopping:
            m_canInterrupt = false;
            m_interruptPriority = 5.0f;
            break;
        default:
            m_canInterrupt = true;
            m_interruptPriority = 0.0f;
            break;
    }

    if (m_debugEnabled)
    {
        Log::Debug("AIAnimation: State changed from %s to %s (reason: %s)",
                   StateToString(m_previousState).CStr(),
                   StateToString(newState).CStr(),
                   reason.CStr());
    }
}

float AIAnimationController::GetStatePriority(AIAnimationState state) const
{
    switch (state)
    {
        case AIAnimationState::Reacting:    return 10.0f;
        case AIAnimationState::Interacting: return 9.0f;
        case AIAnimationState::Starting:    return 7.0f;
        case AIAnimationState::Stopping:    return 7.0f;
        case AIAnimationState::Turning:     return 5.0f;
        case AIAnimationState::Moving:      return 4.0f;
        case AIAnimationState::Idle:        return 1.0f;
        case AIAnimationState::Custom:      return 3.0f;
        default:                            return 0.0f;
    }
}

String AIAnimationController::StateToString(AIAnimationState state) const
{
    switch (state)
    {
        case AIAnimationState::Idle:        return "Idle";
        case AIAnimationState::Moving:      return "Moving";
        case AIAnimationState::Turning:     return "Turning";
        case AIAnimationState::Starting:    return "Starting";
        case AIAnimationState::Stopping:    return "Stopping";
        case AIAnimationState::Reacting:    return "Reacting";
        case AIAnimationState::Interacting: return "Interacting";
        case AIAnimationState::Custom:      return "Custom";
        default:                            return "Unknown";
    }
}

// ============================================================================
// Request Processing
// ============================================================================

void AIAnimationController::ProcessRequest(const AIAnimationRequest& request)
{
    if (m_reactionCooldown > 0.0f)
    {
        m_reactionCooldown -= GetDeltaTime();
        if (m_reactionCooldown < 0.0f)
            m_reactionCooldown = 0.0f;
    }

    // Determine desired state from request
    AIAnimationState desiredState = DetermineDesiredState(request);

    // Check stamina constraints
    if (m_staminaSystem != nullptr)
    {
        desiredState = ApplyStaminaConstraints(request, desiredState);
    }

    // State transition logic
    if (desiredState != m_currentState)
    {
        String reason = BuildTransitionReason(request, desiredState);
        ChangeState(desiredState, reason);
    }

    // Build and execute animation query
    ExecuteAnimationQuery(request);
}

AIAnimationState AIAnimationController::DetermineDesiredState(const AIAnimationRequest& request) const
{
    if (request.wantsStop && !request.wantsMove)
        return AIAnimationState::Stopping;

    if (request.wantsTurn && !request.wantsMove)
        return AIAnimationState::Turning;

    if (request.wantsMove)
    {
        if (m_currentState == AIAnimationState::Idle ||
            m_currentState == AIAnimationState::Stopping)
        {
            return AIAnimationState::Starting;
        }
        return AIAnimationState::Moving;
    }

    if (m_currentState == AIAnimationState::Moving ||
        m_currentState == AIAnimationState::Starting)
    {
        if (request.desiredSpeed < 0.1f)
            return AIAnimationState::Stopping;
    }

    return AIAnimationState::Idle;
}

AIAnimationState AIAnimationController::ApplyStaminaConstraints(
    const AIAnimationRequest& request, AIAnimationState desired) const
{
    if (m_staminaSystem == nullptr)
        return desired;

    float stamina = m_staminaSystem->GetCurrentStamina();
    float maxStamina = m_staminaSystem->GetMaxStamina();
    float staminaRatio = stamina / maxStamina;

    // Force walk if stamina is critically low
    if (staminaRatio < 0.15f && desired == AIAnimationState::Moving)
    {
        // Reduce to walking speed
        return AIAnimationState::Moving; // Still moving but slower
    }

    // Prevent sprint if stamina is low
    if (staminaRatio < 0.3f && request.desiredSpeed > 7.0f)
    {
        // Cap at run speed
        return AIAnimationState::Moving;
    }

    return desired;
}

String AIAnimationController::BuildTransitionReason(
    const AIAnimationRequest& request, AIAnimationState newState) const
{
    switch (newState)
    {
        case AIAnimationState::Idle:
            return "No movement input";
        case AIAnimationState::Moving:
            return String::Format("Moving at speed %.2f", request.desiredSpeed);
        case AIAnimationState::Turning:
            return "Turning in place";
        case AIAnimationState::Starting:
            return "Starting to move";
        case AIAnimationState::Stopping:
            return "Coming to stop";
        case AIAnimationState::Reacting:
            return request.actionName.IsEmpty() ? "Reacting" : request.actionName;
        case AIAnimationState::Interacting:
            return "Interacting with object";
        default:
            return "State transition";
    }
}

// ============================================================================
// Animation Query Execution
// ============================================================================

void AIAnimationController::ExecuteAnimationQuery(const AIAnimationRequest& request)
{
    if (m_chooserTable == nullptr || m_motionMatchingController == nullptr)
        return;

    // Build chooser query based on current state and request
    ChooserQuery query = BuildChooserQuery(request);

    // Evaluate chooser to get animation category
    ChooserResult chooserResult = m_chooserTable->Evaluate(query);

    if (chooserResult.score < 0.0f)
    {
        if (m_debugEnabled)
        {
            Log::Debug("AIAnimation: No matching animation found in chooser");
        }
        return;
    }

    // Build trajectory for motion matching
    Trajectory desiredTrajectory = BuildDesiredTrajectory(request);

    // Set search settings based on state
    SearchSettings searchSettings = m_motionMatchingController->GetSearchSettings();
    searchSettings.trajectoryWeight = ComputeTrajectoryWeight(request);
    searchSettings.poseWeight = ComputePoseWeight(request);
    searchSettings.responsiveness = request.urgency;

    m_motionMatchingController->SetSearchSettings(searchSettings);

    // Perform motion matching search
    if (m_database != nullptr)
    {
        SearchResult mmResult = m_motionMatchingController->Search(
            desiredTrajectory, chooserResult.animationIndex);

        if (mmResult.IsValid())
        {
            m_motionMatchingController->TransitionToPose(
                mmResult.poseIndex, m_blendInTime, m_blendOutTime);
        }
    }
}

ChooserQuery AIAnimationController::BuildChooserQuery(const AIAnimationRequest& request) const
{
    ChooserQuery query;

    // Set boolean parameters
    query.SetBool(0, request.wantsMove);           // IsMoving
    query.SetBool(1, request.desiredSpeed > 7.0f); // IsSprinting
    query.SetBool(2, request.wantsStop);           // WantsStop
    query.SetBool(3, request.wantsTurn);           // WantsTurn

    // Set float parameters
    query.SetFloat(4, request.desiredSpeed);       // DesiredSpeed
    query.SetFloat(5, request.urgency);            // Urgency

    // Set stamina info if available
    if (m_staminaSystem != nullptr)
    {
        float staminaRatio = m_staminaSystem->GetCurrentStamina() /
                             m_staminaSystem->GetMaxStamina();
        query.SetBool(6, staminaRatio < 0.2f);     // StaminaLow
    }
    else
    {
        query.SetBool(6, false);
    }

    // Set tags
    for (const auto& tag : request.preferredTags)
    {
        query.AddPreferredTag(tag);
    }
    for (const auto& tag : request.excludedTags)
    {
        query.AddExcludedTag(tag);
    }

    return query;
}

Trajectory AIAnimationController::BuildDesiredTrajectory(const AIAnimationRequest& request) const
{
    Trajectory trajectory(10, 0.1f); // 10 points, 0.1s interval

    Vec3 currentPos = Vec3::Zero();
    Vec3 currentVel = request.desiredVelocity;
    Quat currentOrient = Quat::FromDirection(request.desiredDirection);

    // Predict future positions based on desired velocity
    for (int32 i = 0; i < trajectory.sampleCount; ++i)
    {
        float t = i * trajectory.sampleInterval;

        // Simple prediction: current velocity + slight damping
        Vec3 predictedVel = currentVel * Math::Exp(-0.5f * t);
        Vec3 predictedPos = currentPos + predictedVel * t;

        trajectory.points[i].position = predictedPos;
        trajectory.points[i].velocity = predictedVel;
        trajectory.points[i].direction = predictedVel.IsNearZero()
            ? currentOrient.Rotate(Vec3::Forward())
            : predictedVel.Normalized();
        trajectory.points[i].orientation = currentOrient;
        trajectory.points[i].speed = predictedVel.Length();
        trajectory.points[i].isValid = true;
    }

    return trajectory;
}

float AIAnimationController::ComputeTrajectoryWeight(const AIAnimationRequest& request) const
{
    // Higher weight for movement requests
    if (request.wantsMove)
        return 1.5f;
    if (request.wantsStop)
        return 0.8f;
    return 1.0f;
}

float AIAnimationController::ComputePoseWeight(const AIAnimationRequest& request) const
{
    // Higher pose weight for precise actions
    if (request.wantsTurn)
        return 1.2f;
    if (request.urgency > 0.7f)
        return 0.8f; // Less pose fidelity for urgent actions
    return 1.0f;
}

// ============================================================================
// Behavior Tree Integration
// ============================================================================

void AIAnimationController::RegisterBehaviorTree(const String& name,
                                                  BehaviorTree* tree)
{
    if (tree == nullptr)
        return;

    auto it = m_behaviorTrees.Find(name);
    if (it != m_behaviorTrees.End())
    {
        delete it->value;
    }
    m_behaviorTrees[name] = tree;
}

BehaviorTree* AIAnimationController::GetBehaviorTree(const String& name) const
{
    auto it = m_behaviorTrees.Find(name);
    if (it != m_behaviorTrees.End())
        return it->value;
    return nullptr;
}

void AIAnimationController::UpdateBehaviorTrees(float deltaTime)
{
    for (auto& pair : m_behaviorTrees)
    {
        if (pair.value != nullptr && pair.value->IsActive())
        {
            pair.value->Tick(deltaTime);

            // Process any animation requests from the behavior tree
            AIAnimationRequest request = pair.value->GetAnimationRequest();
            if (request.wantsMove || request.wantsStop || request.wantsTurn ||
                !request.actionName.IsEmpty())
            {
                ProcessRequest(request);
            }
        }
    }
}

// ============================================================================
// Reaction System
// ============================================================================

void AIAnimationController::TriggerReaction(const String& reactionName,
                                             float priority,
                                             float duration)
{
    if (m_reactionCooldown > 0.0f)
        return;

    if (priority < m_interruptPriority)
        return;

    AIAnimationRequest request;
    request.actionName = reactionName;
    request.urgency = priority;

    ChangeState(AIAnimationState::Reacting,
                String::Format("Reaction: %s", reactionName.CStr()));

    m_reactionCooldown = m_reactionCooldownDuration;
    m_stateDuration = duration;

    // Execute the reaction animation
    ExecuteAnimationQuery(request);
}

void AIAnimationController::CancelReaction()
{
    if (m_currentState == AIAnimationState::Reacting)
    {
        ChangeState(AIAnimationState::Idle, "Reaction cancelled");
    }
}

// ============================================================================
// Update
// ============================================================================

void AIAnimationController::Update(float deltaTime)
{
    // Update state duration
    m_stateDuration += deltaTime;
    m_stateInfo.duration = m_stateDuration;

    // Update behavior trees
    UpdateBehaviorTrees(deltaTime);

    // Auto-transition from temporary states
    AutoTransitionStates(deltaTime);

    // Update stamina if available
    if (m_staminaSystem != nullptr)
    {
        UpdateStaminaConsumption(deltaTime);
    }
}

void AIAnimationController::AutoTransitionStates(float deltaTime)
{
    switch (m_currentState)
    {
        case AIAnimationState::Starting:
            // Auto-transition to Moving after short time
            if (m_stateDuration > 0.3f)
            {
                ChangeState(AIAnimationState::Moving, "Start complete");
            }
            break;

        case AIAnimationState::Stopping:
            // Auto-transition to Idle after stop animation
            if (m_stateDuration > 0.5f)
            {
                ChangeState(AIAnimationState::Idle, "Stop complete");
            }
            break;

        case AIAnimationState::Reacting:
            // Return to previous state after reaction
            if (m_stateDuration > m_stateInfo.duration && m_stateInfo.duration > 0.0f)
            {
                ChangeState(m_previousState, "Reaction complete");
            }
            break;

        default:
            break;
    }
}

void AIAnimationController::UpdateStaminaConsumption(float deltaTime)
{
    if (m_staminaSystem == nullptr)
        return;

    float consumptionRate = 0.0f;

    switch (m_currentState)
    {
        case AIAnimationState::Moving:
            consumptionRate = 5.0f;  // Base movement cost
            break;
        case AIAnimationState::Sprinting:
            consumptionRate = 20.0f; // High sprint cost
            break;
        case AIAnimationState::Reacting:
            consumptionRate = 10.0f; // Reaction cost
            break;
        default:
            consumptionRate = -2.0f; // Recovery
            break;
    }

    m_staminaSystem->ConsumeStamina(consumptionRate * deltaTime);
}

// ============================================================================
// Debug
// ============================================================================

void AIAnimationController::SetDebugEnabled(bool enabled)
{
    m_debugEnabled = enabled;
}

String AIAnimationController::GetDebugInfo() const
{
    String info;
    info += String::Format("State: %s\n", StateToString(m_currentState).CStr());
    info += String::Format("Duration: %.2f\n", m_stateDuration);
    info += String::Format("Can Interrupt: %s\n", m_canInterrupt ? "Yes" : "No");
    info += String::Format("Priority: %.1f\n", m_interruptPriority);
    info += String::Format("Reason: %s\n", m_stateInfo.debugReason.CStr());

    if (m_staminaSystem != nullptr)
    {
        info += String::Format("Stamina: %.1f / %.1f\n",
                               m_staminaSystem->GetCurrentStamina(),
                               m_staminaSystem->GetMaxStamina());
    }

    return info;
}

// ============================================================================
// StaminaSystem
// ============================================================================

StaminaSystem::StaminaSystem()
    : m_currentStamina(100.0f)
    , m_maxStamina(100.0f)
    , m_regenRate(10.0f)
    , m_regenDelay(1.0f)
    , m_timeSinceConsumption(0.0f)
    , m_exhausted(false)
{
}

void StaminaSystem::SetMaxStamina(float maxStamina)
{
    m_maxStamina = maxStamina;
    m_currentStamina = Math::Min(m_currentStamina, m_maxStamina);
}

void StaminaSystem::ConsumeStamina(float amount)
{
    if (amount > 0.0f)
    {
        m_currentStamina = Math::Max(0.0f, m_currentStamina - amount);
        m_timeSinceConsumption = 0.0f;

        if (m_currentStamina <= 0.0f)
        {
            m_exhausted = true;
        }
    }
    else if (amount < 0.0f)
    {
        // Regeneration
        if (m_timeSinceConsumption >= m_regenDelay)
        {
            m_currentStamina = Math::Min(m_maxStamina, m_currentStamina - amount);

            if (m_currentStamina > m_maxStamina * 0.2f)
            {
                m_exhausted = false;
            }
        }
    }
}

void StaminaSystem::Update(float deltaTime)
{
    m_timeSinceConsumption += deltaTime;

    // Passive regeneration
    if (m_timeSinceConsumption >= m_regenDelay && m_currentStamina < m_maxStamina)
    {
        float regen = m_regenRate * deltaTime;
        m_currentStamina = Math::Min(m_maxStamina, m_currentStamina + regen);

        if (m_currentStamina > m_maxStamina * 0.2f)
        {
            m_exhausted = false;
        }
    }
}

// ============================================================================
// BehaviorTree (Base)
// ============================================================================

BehaviorTree::BehaviorTree()
    : m_rootNode(nullptr)
    , m_active(false)
    , m_blackboard(nullptr)
{
}

BehaviorTree::~BehaviorTree()
{
    delete m_rootNode;
    delete m_blackboard;
}

void BehaviorTree::Initialize()
{
    m_blackboard = new Blackboard();
    BuildTree();
    m_active = true;
}

void BehaviorTree::Tick(float deltaTime)
{
    if (!m_active || m_rootNode == nullptr)
        return;

    BTStatus status = m_rootNode->Tick(deltaTime, m_blackboard);

    if (status == BTStatus::Success || status == BTStatus::Failure)
    {
        // Tree completed, can reset or keep running
        m_rootNode->Reset();
    }
}

AIAnimationRequest BehaviorTree::GetAnimationRequest() const
{
    AIAnimationRequest request;

    if (m_blackboard != nullptr)
    {
        request.wantsMove = m_blackboard->GetBool("wantsMove", false);
        request.wantsStop = m_blackboard->GetBool("wantsStop", false);
        request.wantsTurn = m_blackboard->GetBool("wantsTurn", false);
        request.desiredSpeed = m_blackboard->GetFloat("desiredSpeed", 0.0f);
        request.desiredDirection = m_blackboard->GetVec3("desiredDirection", Vec3::Forward());
        request.desiredVelocity = m_blackboard->GetVec3("desiredVelocity", Vec3::Zero());
        request.urgency = m_blackboard->GetFloat("urgency", 0.5f);
    }

    return request;
}

// ============================================================================
// BTNode Base
// ============================================================================

BTNode::BTNode(const String& name)
    : m_name(name)
    , m_status(BTStatus::Invalid)
{
}

BTNode::~BTNode()
{
    for (auto child : m_children)
    {
        delete child;
    }
}

void BTNode::AddChild(BTNode* child)
{
    if (child != nullptr)
    {
        m_children.PushBack(child);
    }
}

BTStatus BTNode::Tick(float deltaTime, Blackboard* blackboard)
{
    if (m_status != BTStatus::Running)
    {
        OnEnter(blackboard);
    }

    m_status = OnTick(deltaTime, blackboard);

    if (m_status != BTStatus::Running)
    {
        OnExit(blackboard);
    }

    return m_status;
}

void BTNode::Reset()
{
    m_status = BTStatus::Invalid;
    for (auto child : m_children)
    {
        child->Reset();
    }
}

// ============================================================================
// Blackboard
// ============================================================================

Blackboard::Blackboard()
{
}

void Blackboard::SetBool(const String& key, bool value)
{
    m_bools[key] = value;
}

bool Blackboard::GetBool(const String& key, bool defaultValue) const
{
    auto it = m_bools.Find(key);
    if (it != m_bools.End())
        return it->value;
    return defaultValue;
}

void Blackboard::SetFloat(const String& key, float value)
{
    m_floats[key] = value;
}

float Blackboard::GetFloat(const String& key, float defaultValue) const
{
    auto it = m_floats.Find(key);
    if (it != m_floats.End())
        return it->value;
    return defaultValue;
}

void Blackboard::SetVec3(const String& key, const Vec3& value)
{
    m_vec3s[key] = value;
}

Vec3 Blackboard::GetVec3(const String& key, const Vec3& defaultValue) const
{
    auto it = m_vec3s.Find(key);
    if (it != m_vec3s.End())
        return it->value;
    return defaultValue;
}

void Blackboard::SetString(const String& key, const String& value)
{
    m_strings[key] = value;
}

String Blackboard::GetString(const String& key, const String& defaultValue) const
{
    auto it = m_strings.Find(key);
    if (it != m_strings.End())
        return it->value;
    return defaultValue;
}

void Blackboard::Clear()
{
    m_bools.Clear();
    m_floats.Clear();
    m_vec3s.Clear();
    m_strings.Clear();
}

MMV2_NAMESPACE_END
