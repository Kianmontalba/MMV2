// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Event System
// ============================================================================

#pragma once
#ifndef MMV2_EVENTS_H
#define MMV2_EVENTS_H

#include "MMV2/Core/Config.h"
#include "MMV2/Core/Vector.h"
#include "MMV2/Core/String.h"
#include "MMV2/Core/Transform.h"

MMV2_NAMESPACE_BEGIN

enum class EventType : uint8 {
    None = 0,
    Footstep = 1,
    Sound = 2,
    Particle = 3,
    Damage = 4,
    StateChange = 5,
    Trigger = 6,
    Custom = 7
};

struct AnimationEvent {
    String name;
    EventType type;
    float32 normalizedTime;
    float32 duration;
    Transform transform;
    int32 boneIndex;
    BoneFlags flags;
    String payload;
    bool enabled;

    AnimationEvent()
        : type(EventType::Custom), normalizedTime(0.0f), duration(0.0f),
          boneIndex(-1), flags(BoneFlags::None), enabled(true) {}

    bool IsActiveAtTime(float32 normTime) const {
        return normTime >= normalizedTime && normTime <= normalizedTime + duration;
    }
};

struct EventTrack {
    String name;
    Vector<AnimationEvent> events;
    bool enabled;

    EventTrack() : enabled(true) {}

    void AddEvent(const AnimationEvent& event) { events.PushBack(event); }
    void RemoveEvent(int32 index) { if (index >= 0 && index < static_cast<int32>(events.Size())) events.Erase(events.begin() + index); }
    Vector<AnimationEvent> GetActiveEvents(float32 normalizedTime) const;
};

using EventCallback = void(*)(const AnimationEvent& event, void* userData);

class MMV2_API EventSystem {
public:
    EventSystem();
    ~EventSystem();

    void AddTrack(const EventTrack& track);
    void RemoveTrack(int32 index);
    void ClearTracks();

    void SetCallback(EventCallback callback, void* userData);

    void Update(float32 normalizedTime);
    void Update(float32 normalizedTime, const char* trackName);

    Vector<AnimationEvent> GetActiveEvents(float32 normalizedTime) const;
    Vector<AnimationEvent> GetActiveEvents(float32 normalizedTime, const char* trackName) const;

    bool HasEventAtTime(float32 normalizedTime, EventType type) const;
    AnimationEvent GetNextEvent(float32 normalizedTime) const;
    AnimationEvent GetPreviousEvent(float32 normalizedTime) const;

    void TriggerEvent(const AnimationEvent& event);
    void TriggerEventByName(const char* name, float32 normalizedTime);

private:
    Vector<EventTrack> m_tracks;
    EventCallback m_callback;
    void* m_userData;
    float32 m_lastNormalizedTime;
};

MMV2_NAMESPACE_END

#endif
