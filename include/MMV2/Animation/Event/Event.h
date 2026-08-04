// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Animation Event System
// ============================================================================

#pragma once

#include "MMV2/Core/Config.h"
#include "MMV2/Core/Vector.h"
#include "MMV2/Core/HashMap.h"
#include "MMV2/Core/String.h"
#include "MMV2/Core/Transform.h"

MMV2_NAMESPACE_BEGIN

enum class AnimationEventType : uint32_t
{
    None = 0, Footstep, FootLift, Sound, Particle, CameraShake,
    HitboxActivate, HitboxDeactivate, StateChange, NotifyBegin, NotifyEnd, Custom
};

struct AnimationEvent
{
    String name;
    AnimationEventType type;
    float triggerTime;
    bool useNormalizedTime;
    float duration;
    String payload;
    uint32_t boneIndex;
    Transform offset;
    bool triggered;
    bool enabled;
    int32_t priority;
    AnimationEvent() : type(AnimationEventType::None), triggerTime(0), useNormalizedTime(true),
                       duration(0), boneIndex(0), triggered(false), enabled(true), priority(0) {}
};

enum class EventTriggerCondition : uint32_t
{
    Always, OnStateEntry, OnStateExit, OnFootContact, OnFootLift,
    OnVelocityChange, OnDirectionChange, OnCustomCondition
};

struct EventCondition
{
    EventTriggerCondition type;
    float threshold;
    String customCondition;
    bool invert;
    EventCondition() : type(EventTriggerCondition::Always), threshold(0), invert(false) {}
};

using EventCallback = void(*)(const AnimationEvent& event, void* userData);

class MMV2_API IEventListener
{
public:
    virtual ~IEventListener() = default;
    virtual void OnEvent(const AnimationEvent& event) = 0;
    virtual bool WantsEvent(AnimationEventType type) const { return true; }
};

class MMV2_API AnimationEventTrack
{
public:
    AnimationEventTrack();
    void SetAnimationIndex(uint32_t index) { m_animationIndex = index; }
    uint32_t GetAnimationIndex() const { return m_animationIndex; }
    uint32_t AddEvent(const AnimationEvent& event);
    void RemoveEvent(uint32_t index);
    void ClearEvents();
    uint32_t GetEventCount() const { return static_cast<uint32_t>(m_events.Size()); }
    const AnimationEvent& GetEvent(uint32_t index) const { return m_events[index]; }
    AnimationEvent& GetEvent(uint32_t index) { return m_events[index]; }
    void SortEvents();
    Vector<const AnimationEvent*> GetEventsAtTime(float time, float tolerance = 0.01f) const;
    Vector<const AnimationEvent*> GetEventsInRange(float startTime, float endTime) const;
    void ResetTriggers();
    void Update(float currentTime, float previousTime);
    void SetCondition(const EventCondition& condition) { m_condition = condition; }
    const EventCondition& GetCondition() const { return m_condition; }
    bool CheckCondition(const class MotionMatchingContext& context) const;
    void SetCallback(EventCallback callback, void* userData = nullptr);
    void AddListener(IEventListener* listener);
    void RemoveListener(IEventListener* listener);
    void Serialize(class BinarySerializer& serializer) const;
    void Deserialize(class BinarySerializer& serializer);
private:
    uint32_t m_animationIndex;
    Vector<AnimationEvent> m_events;
    EventCondition m_condition;
    EventCallback m_callback;
    void* m_callbackUserData;
    Vector<IEventListener*> m_listeners;
    void FireEvent(const AnimationEvent& event);
};

class MMV2_API AnimationEventSystem
{
public:
    AnimationEventSystem();
    ~AnimationEventSystem();
    void Initialize();
    void Shutdown();
    AnimationEventTrack* CreateTrack(uint32_t animationIndex);
    void DestroyTrack(AnimationEventTrack* track);
    AnimationEventTrack* GetTrack(uint32_t animationIndex);
    const AnimationEventTrack* GetTrack(uint32_t animationIndex) const;
    void AddGlobalListener(IEventListener* listener);
    void RemoveGlobalListener(IEventListener* listener);
    void Update(float currentTime, float previousTime, uint32_t currentAnimation);
    void UpdateAllTracks(float currentTime, float previousTime);
    Vector<const AnimationEvent*> QueryEvents(AnimationEventType type) const;
    Vector<const AnimationEvent*> QueryEventsByName(const String& name) const;
    void AutoGenerateFootsteps(uint32_t animationIndex, uint32_t leftFootBone, uint32_t rightFootBone);
    void AutoGenerateFootstepsFromHeight(uint32_t animationIndex, uint32_t leftFootBone, uint32_t rightFootBone, float groundHeight);
    void AutoGenerateSoundEvents(uint32_t animationIndex, AnimationEventType soundType, float interval, float randomOffset = 0.0f);
    void AddStateChangeEvent(uint32_t animationIndex, float triggerTime, const String& fromState, const String& toState);
    void CopyEvents(uint32_t fromAnimation, uint32_t toAnimation);
    void MirrorEvents(uint32_t animationIndex, uint32_t leftBone, uint32_t rightBone);
    uint32_t GetTotalEventCount() const;
    uint32_t GetTrackCount() const { return static_cast<uint32_t>(m_tracks.Size()); }
    void SetDebugEnabled(bool enabled) { m_debugEnabled = enabled; }
    String GetDebugInfo() const;
private:
    Vector<UniquePtr<AnimationEventTrack>> m_tracks;
    HashMap<uint32_t, AnimationEventTrack*> m_trackMap;
    Vector<IEventListener*> m_globalListeners;
    bool m_debugEnabled;
    void NotifyGlobalListeners(const AnimationEvent& event);
};

class MMV2_API EventBuilder
{
public:
    EventBuilder& Name(const String& name) { m_event.name = name; return *this; }
    EventBuilder& Type(AnimationEventType type) { m_event.type = type; return *this; }
    EventBuilder& AtTime(float time, bool normalized = true) { m_event.triggerTime = time; m_event.useNormalizedTime = normalized; return *this; }
    EventBuilder& WithDuration(float duration) { m_event.duration = duration; return *this; }
    EventBuilder& OnBone(uint32_t boneIndex) { m_event.boneIndex = boneIndex; return *this; }
    EventBuilder& WithOffset(const Transform& offset) { m_event.offset = offset; return *this; }
    EventBuilder& WithPayload(const String& payload) { m_event.payload = payload; return *this; }
    EventBuilder& WithPriority(int32_t priority) { m_event.priority = priority; return *this; }
    AnimationEvent Build() const { return m_event; }
private:
    AnimationEvent m_event;
};

class MMV2_API FootstepEventListener : public IEventListener
{
public:
    void OnEvent(const AnimationEvent& event) override;
    bool WantsEvent(AnimationEventType type) const override { return type == AnimationEventType::Footstep || type == AnimationEventType::FootLift; }
    void SetFootstepSound(const String& leftFootSound, const String& rightFootSound);
    void SetFootstepParticle(const String& particleEffect);
    void SetGroundType(const String& groundType);
private:
    String m_leftFootSound;
    String m_rightFootSound;
    String m_particleEffect;
    String m_groundType;
};

class MMV2_API SoundEventListener : public IEventListener
{
public:
    void OnEvent(const AnimationEvent& event) override;
    bool WantsEvent(AnimationEventType type) const override { return type == AnimationEventType::Sound; }
    void RegisterSound(AnimationEventType eventType, const String& soundPath);
    void SetGlobalVolume(float volume) { m_globalVolume = volume; }
private:
    HashMap<uint32_t, String> m_soundMap;
    float m_globalVolume = 1.0f;
};

MMV2_NAMESPACE_END
