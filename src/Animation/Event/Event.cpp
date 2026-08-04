// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Animation Event System Implementation
// ============================================================================

#include "MMV2/Animation/Event/Event.h"
#include "MMV2/Core/Serializer.h"
#include "MMV2/Core/Math.h"
#include <algorithm>

MMV2_NAMESPACE_BEGIN

// ============================================================================
// AnimationEventTrack
// ============================================================================

AnimationEventTrack::AnimationEventTrack()
    : m_animationIndex(0), m_callback(nullptr), m_callbackUserData(nullptr)
{
}

uint32_t AnimationEventTrack::AddEvent(const AnimationEvent& event)
{
    uint32_t index = static_cast<uint32_t>(m_events.Size());
    m_events.PushBack(event);
    return index;
}

void AnimationEventTrack::RemoveEvent(uint32_t index)
{
    if (index < m_events.Size())
        m_events.Erase(index);
}

void AnimationEventTrack::ClearEvents()
{
    m_events.Clear();
}

void AnimationEventTrack::SortEvents()
{
    std::sort(m_events.Begin(), m_events.End(), [](const AnimationEvent& a, const AnimationEvent& b)
    {
        return a.triggerTime < b.triggerTime;
    });
}

Vector<const AnimationEvent*> AnimationEventTrack::GetEventsAtTime(float time, float tolerance) const
{
    Vector<const AnimationEvent*> result;
    for (const auto& event : m_events)
    {
        if (std::abs(event.triggerTime - time) <= tolerance)
            result.PushBack(&event);
    }
    return result;
}

Vector<const AnimationEvent*> AnimationEventTrack::GetEventsInRange(float startTime, float endTime) const
{
    Vector<const AnimationEvent*> result;
    for (const auto& event : m_events)
    {
        if (event.triggerTime >= startTime && event.triggerTime <= endTime)
            result.PushBack(&event);
    }
    return result;
}

void AnimationEventTrack::ResetTriggers()
{
    for (auto& event : m_events)
        event.triggered = false;
}

void AnimationEventTrack::Update(float currentTime, float previousTime)
{
    for (auto& event : m_events)
    {
        if (!event.enabled || event.triggered)
            continue;

        bool shouldTrigger = false;

        if (event.useNormalizedTime)
        {
            // For normalized time (0-1), we trigger when crossing the threshold
            shouldTrigger = (previousTime < event.triggerTime && currentTime >= event.triggerTime);
        }
        else
        {
            // For absolute time
            shouldTrigger = (previousTime < event.triggerTime && currentTime >= event.triggerTime);
        }

        if (shouldTrigger)
        {
            event.triggered = true;
            FireEvent(event);
        }
    }
}

bool AnimationEventTrack::CheckCondition(const MotionMatchingContext& context) const
{
    switch (m_condition.type)
    {
        case EventTriggerCondition::Always:
            return !m_condition.invert;
        case EventTriggerCondition::OnVelocityChange:
            // Would check velocity against threshold
            return true;
        case EventTriggerCondition::OnFootContact:
            // Would check foot contact
            return true;
        default:
            return true;
    }
}

void AnimationEventTrack::SetCallback(EventCallback callback, void* userData)
{
    m_callback = callback;
    m_callbackUserData = userData;
}

void AnimationEventTrack::AddListener(IEventListener* listener)
{
    if (listener)
        m_listeners.PushBack(listener);
}

void AnimationEventTrack::RemoveListener(IEventListener* listener)
{
    for (uint32_t i = 0; i < m_listeners.Size(); ++i)
    {
        if (m_listeners[i] == listener)
        {
            m_listeners.Erase(i);
            break;
        }
    }
}

void AnimationEventTrack::FireEvent(const AnimationEvent& event)
{
    if (m_callback)
        m_callback(event, m_callbackUserData);

    for (auto* listener : m_listeners)
    {
        if (listener && listener->WantsEvent(event.type))
            listener->OnEvent(event);
    }
}

void AnimationEventTrack::Serialize(BinarySerializer& serializer) const
{
    serializer.Write(static_cast<uint32_t>(m_events.Size()));
    for (const auto& event : m_events)
    {
        serializer.Write(event.name);
        serializer.Write(static_cast<uint32_t>(event.type));
        serializer.Write(event.triggerTime);
        serializer.Write(event.useNormalizedTime);
        serializer.Write(event.duration);
        serializer.Write(event.payload);
        serializer.Write(event.boneIndex);
        serializer.Write(event.enabled);
        serializer.Write(event.priority);
    }
}

void AnimationEventTrack::Deserialize(BinarySerializer& serializer)
{
    uint32_t count;
    serializer.Read(count);
    m_events.Resize(count);

    for (auto& event : m_events)
    {
        serializer.Read(event.name);
        uint32_t type;
        serializer.Read(type);
        event.type = static_cast<AnimationEventType>(type);
        serializer.Read(event.triggerTime);
        serializer.Read(event.useNormalizedTime);
        serializer.Read(event.duration);
        serializer.Read(event.payload);
        serializer.Read(event.boneIndex);
        serializer.Read(event.enabled);
        serializer.Read(event.priority);
        event.triggered = false;
    }
}

// ============================================================================
// AnimationEventSystem
// ============================================================================

AnimationEventSystem::AnimationEventSystem() : m_debugEnabled(false)
{
}

AnimationEventSystem::~AnimationEventSystem()
{
    Shutdown();
}

void AnimationEventSystem::Initialize()
{
    m_tracks.Clear();
    m_trackMap.Clear();
    m_globalListeners.Clear();
}

void AnimationEventSystem::Shutdown()
{
    m_tracks.Clear();
    m_trackMap.Clear();
    m_globalListeners.Clear();
}

AnimationEventTrack* AnimationEventSystem::CreateTrack(uint32_t animationIndex)
{
    auto track = MakeUnique<AnimationEventTrack>();
    track->SetAnimationIndex(animationIndex);
    AnimationEventTrack* ptr = track.Get();
    m_tracks.PushBack(std::move(track));
    m_trackMap[animationIndex] = ptr;
    return ptr;
}

void AnimationEventSystem::DestroyTrack(AnimationEventTrack* track)
{
    if (!track) return;

    for (uint32_t i = 0; i < m_tracks.Size(); ++i)
    {
        if (m_tracks[i].Get() == track)
        {
            m_trackMap.Erase(track->GetAnimationIndex());
            m_tracks.Erase(i);
            break;
        }
    }
}

AnimationEventTrack* AnimationEventSystem::GetTrack(uint32_t animationIndex)
{
    auto it = m_trackMap.Find(animationIndex);
    return (it != m_trackMap.End()) ? it->second : nullptr;
}

const AnimationEventTrack* AnimationEventSystem::GetTrack(uint32_t animationIndex) const
{
    auto it = m_trackMap.Find(animationIndex);
    return (it != m_trackMap.End()) ? it->second : nullptr;
}

void AnimationEventSystem::AddGlobalListener(IEventListener* listener)
{
    if (listener)
        m_globalListeners.PushBack(listener);
}

void AnimationEventSystem::RemoveGlobalListener(IEventListener* listener)
{
    for (uint32_t i = 0; i < m_globalListeners.Size(); ++i)
    {
        if (m_globalListeners[i] == listener)
        {
            m_globalListeners.Erase(i);
            break;
        }
    }
}

void AnimationEventSystem::Update(float currentTime, float previousTime, uint32_t currentAnimation)
{
    auto* track = GetTrack(currentAnimation);
    if (track)
        track->Update(currentTime, previousTime);
}

void AnimationEventSystem::UpdateAllTracks(float currentTime, float previousTime)
{
    for (auto& track : m_tracks)
        track->Update(currentTime, previousTime);
}

Vector<const AnimationEvent*> AnimationEventSystem::QueryEvents(AnimationEventType type) const
{
    Vector<const AnimationEvent*> result;
    for (const auto& track : m_tracks)
    {
        for (uint32_t i = 0; i < track->GetEventCount(); ++i)
        {
            const auto& event = track->GetEvent(i);
            if (event.type == type)
                result.PushBack(&event);
        }
    }
    return result;
}

Vector<const AnimationEvent*> AnimationEventSystem::QueryEventsByName(const String& name) const
{
    Vector<const AnimationEvent*> result;
    for (const auto& track : m_tracks)
    {
        for (uint32_t i = 0; i < track->GetEventCount(); ++i)
        {
            const auto& event = track->GetEvent(i);
            if (event.name == name)
                result.PushBack(&event);
        }
    }
    return result;
}

void AnimationEventSystem::AutoGenerateFootsteps(uint32_t animationIndex, uint32_t leftFootBone, uint32_t rightFootBone)
{
    auto* track = GetTrack(animationIndex);
    if (!track)
        track = CreateTrack(animationIndex);

    // Would analyze animation to find foot contact times
    // Placeholder: add generic footstep events
    AnimationEvent leftFoot;
    leftFoot.name = "LeftFootstep";
    leftFoot.type = AnimationEventType::Footstep;
    leftFoot.triggerTime = 0.0f;
    leftFoot.useNormalizedTime = true;
    leftFoot.boneIndex = leftFootBone;
    track->AddEvent(leftFoot);

    AnimationEvent rightFoot;
    rightFoot.name = "RightFootstep";
    rightFoot.type = AnimationEventType::Footstep;
    rightFoot.triggerTime = 0.5f;
    rightFoot.useNormalizedTime = true;
    rightFoot.boneIndex = rightFootBone;
    track->AddEvent(rightFoot);
}

void AnimationEventSystem::AutoGenerateFootstepsFromHeight(uint32_t animationIndex, uint32_t leftFootBone,
                                                            uint32_t rightFootBone, float groundHeight)
{
    AutoGenerateFootsteps(animationIndex, leftFootBone, rightFootBone);
}

void AnimationEventSystem::AutoGenerateSoundEvents(uint32_t animationIndex, AnimationEventType soundType,
                                                    float interval, float randomOffset)
{
    auto* track = GetTrack(animationIndex);
    if (!track)
        track = CreateTrack(animationIndex);

    float time = 0.0f;
    while (time < 1.0f)
    {
        AnimationEvent event;
        event.name = "AutoSound";
        event.type = soundType;
        event.triggerTime = time;
        event.useNormalizedTime = true;
        track->AddEvent(event);

        time += interval + (Math::RandomFloat() * randomOffset);
    }
}

void AnimationEventSystem::AddStateChangeEvent(uint32_t animationIndex, float triggerTime,
                                                const String& fromState, const String& toState)
{
    auto* track = GetTrack(animationIndex);
    if (!track)
        track = CreateTrack(animationIndex);

    AnimationEvent event;
    event.name = "StateChange_" + fromState + "_to_" + toState;
    event.type = AnimationEventType::StateChange;
    event.triggerTime = triggerTime;
    event.useNormalizedTime = true;
    event.payload = "{"from":"" + fromState + "","to":"" + toState + ""}";
    track->AddEvent(event);
}

void AnimationEventSystem::CopyEvents(uint32_t fromAnimation, uint32_t toAnimation)
{
    auto* fromTrack = GetTrack(fromAnimation);
    auto* toTrack = GetTrack(toAnimation);

    if (!fromTrack)
        return;
    if (!toTrack)
        toTrack = CreateTrack(toAnimation);

    for (uint32_t i = 0; i < fromTrack->GetEventCount(); ++i)
    {
        toTrack->AddEvent(fromTrack->GetEvent(i));
    }
}

void AnimationEventSystem::MirrorEvents(uint32_t animationIndex, uint32_t leftBone, uint32_t rightBone)
{
    auto* track = GetTrack(animationIndex);
    if (!track) return;

    // Swap left/right bone indices in events
    for (uint32_t i = 0; i < track->GetEventCount(); ++i)
    {
        auto& event = track->GetEvent(i);
        if (event.boneIndex == leftBone)
            event.boneIndex = rightBone;
        else if (event.boneIndex == rightBone)
            event.boneIndex = leftBone;
    }
}

uint32_t AnimationEventSystem::GetTotalEventCount() const
{
    uint32_t count = 0;
    for (const auto& track : m_tracks)
        count += track->GetEventCount();
    return count;
}

String AnimationEventSystem::GetDebugInfo() const
{
    String info;
    info += "Animation Event System:
";
    info += "  Tracks: " + String::FromInt(static_cast<int32_t>(m_tracks.Size())) + "
";
    info += "  Total Events: " + String::FromInt(static_cast<int32_t>(GetTotalEventCount())) + "
";
    return info;
}

void AnimationEventSystem::NotifyGlobalListeners(const AnimationEvent& event)
{
    for (auto* listener : m_globalListeners)
    {
        if (listener && listener->WantsEvent(event.type))
            listener->OnEvent(event);
    }
}

// ============================================================================
// FootstepEventListener
// ============================================================================

void FootstepEventListener::OnEvent(const AnimationEvent& event)
{
    if (event.type == AnimationEventType::Footstep)
    {
        // Would play footstep sound based on bone
        // Would spawn footstep particle
    }
    else if (event.type == AnimationEventType::FootLift)
    {
        // Would stop footstep sound
    }
}

void FootstepEventListener::SetFootstepSound(const String& leftFootSound, const String& rightFootSound)
{
    m_leftFootSound = leftFootSound;
    m_rightFootSound = rightFootSound;
}

void FootstepEventListener::SetFootstepParticle(const String& particleEffect)
{
    m_particleEffect = particleEffect;
}

void FootstepEventListener::SetGroundType(const String& groundType)
{
    m_groundType = groundType;
}

// ============================================================================
// SoundEventListener
// ============================================================================

void SoundEventListener::OnEvent(const AnimationEvent& event)
{
    auto it = m_soundMap.Find(static_cast<uint32_t>(event.type));
    if (it != m_soundMap.End())
    {
        // Would play sound: it->second
    }
}

void SoundEventListener::RegisterSound(AnimationEventType eventType, const String& soundPath)
{
    m_soundMap[static_cast<uint32_t>(eventType)] = soundPath;
}

MMV2_NAMESPACE_END
