// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Event System Implementation
// ============================================================================

#include "MMV2/Events/Events.h"
#include "MMV2/Core/Math.h"
#include <algorithm>

MMV2_NAMESPACE_BEGIN

Vector<AnimationEvent> EventTrack::GetActiveEvents(float32 normalizedTime) const {
    Vector<AnimationEvent> result;
    if (!enabled) return result;
    for (const auto& event : events) {
        if (event.enabled && event.IsActiveAtTime(normalizedTime)) {
            result.PushBack(event);
        }
    }
    return result;
}

EventSystem::EventSystem() : m_callback(nullptr), m_userData(nullptr), m_lastNormalizedTime(0.0f) {}

EventSystem::~EventSystem() {}

void EventSystem::AddTrack(const EventTrack& track) {
    m_tracks.PushBack(track);
}

void EventSystem::RemoveTrack(int32 index) {
    if (index < 0 || index >= static_cast<int32>(m_tracks.Size())) return;
    m_tracks.Erase(m_tracks.begin() + index);
}

void EventSystem::ClearTracks() {
    m_tracks.Clear();
}

void EventSystem::SetCallback(EventCallback callback, void* userData) {
    m_callback = callback;
    m_userData = userData;
}

void EventSystem::Update(float32 normalizedTime) {
    for (const auto& track : m_tracks) {
        if (!track.enabled) continue;
        for (const auto& event : track.events) {
            if (!event.enabled) continue;
            // Check if we crossed this event's time
            if (m_lastNormalizedTime < event.normalizedTime && normalizedTime >= event.normalizedTime) {
                if (m_callback) {
                    m_callback(event, m_userData);
                }
            }
        }
    }
    m_lastNormalizedTime = normalizedTime;
}

void EventSystem::Update(float32 normalizedTime, const char* trackName) {
    for (const auto& track : m_tracks) {
        if (!track.enabled || track.name != trackName) continue;
        for (const auto& event : track.events) {
            if (!event.enabled) continue;
            if (m_lastNormalizedTime < event.normalizedTime && normalizedTime >= event.normalizedTime) {
                if (m_callback) {
                    m_callback(event, m_userData);
                }
            }
        }
    }
}

Vector<AnimationEvent> EventSystem::GetActiveEvents(float32 normalizedTime) const {
    Vector<AnimationEvent> result;
    for (const auto& track : m_tracks) {
        if (!track.enabled) continue;
        for (const auto& event : track.events) {
            if (event.enabled && event.IsActiveAtTime(normalizedTime)) {
                result.PushBack(event);
            }
        }
    }
    return result;
}

Vector<AnimationEvent> EventSystem::GetActiveEvents(float32 normalizedTime, const char* trackName) const {
    Vector<AnimationEvent> result;
    for (const auto& track : m_tracks) {
        if (!track.enabled || track.name != trackName) continue;
        for (const auto& event : track.events) {
            if (event.enabled && event.IsActiveAtTime(normalizedTime)) {
                result.PushBack(event);
            }
        }
    }
    return result;
}

bool EventSystem::HasEventAtTime(float32 normalizedTime, EventType type) const {
    for (const auto& track : m_tracks) {
        if (!track.enabled) continue;
        for (const auto& event : track.events) {
            if (event.enabled && event.type == type && event.IsActiveAtTime(normalizedTime)) {
                return true;
            }
        }
    }
    return false;
}

AnimationEvent EventSystem::GetNextEvent(float32 normalizedTime) const {
    AnimationEvent next;
    next.normalizedTime = 2.0f; // Beyond end
    for (const auto& track : m_tracks) {
        if (!track.enabled) continue;
        for (const auto& event : track.events) {
            if (event.enabled && event.normalizedTime > normalizedTime && event.normalizedTime < next.normalizedTime) {
                next = event;
            }
        }
    }
    return next;
}

AnimationEvent EventSystem::GetPreviousEvent(float32 normalizedTime) const {
    AnimationEvent prev;
    prev.normalizedTime = -1.0f;
    for (const auto& track : m_tracks) {
        if (!track.enabled) continue;
        for (const auto& event : track.events) {
            if (event.enabled && event.normalizedTime < normalizedTime && event.normalizedTime > prev.normalizedTime) {
                prev = event;
            }
        }
    }
    return prev;
}

void EventSystem::TriggerEvent(const AnimationEvent& event) {
    if (m_callback) {
        m_callback(event, m_userData);
    }
}

void EventSystem::TriggerEventByName(const char* name, float32 normalizedTime) {
    for (const auto& track : m_tracks) {
        if (!track.enabled) continue;
        for (const auto& event : track.events) {
            if (event.enabled && event.name == name) {
                AnimationEvent triggered = event;
                triggered.normalizedTime = normalizedTime;
                if (m_callback) {
                    m_callback(triggered, m_userData);
                }
            }
        }
    }
}

MMV2_NAMESPACE_END
