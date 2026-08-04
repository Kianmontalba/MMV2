#pragma once
#ifndef MMV2_HASH_H
#define MMV2_HASH_H

#include "Config.h"
#include <cstdint>

MMV2_NAMESPACE_BEGIN

// FNV-1a hash
MMV2_FORCE_INLINE constexpr uint64_t HashFNV1a(const char* str, uint64_t hash = 14695981039346656037ULL) noexcept {
    return *str == '\0' ? hash : HashFNV1a(str + 1, (hash ^ static_cast<uint64_t>(*str)) * 1099511628211ULL);
}

MMV2_FORCE_INLINE constexpr uint64_t HashFNV1a(const void* data, size_type len, uint64_t hash = 14695981039346656037ULL) noexcept {
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    for (size_type i = 0; i < len; ++i) {
        hash ^= static_cast<uint64_t>(bytes[i]);
        hash *= 1099511628211ULL;
    }
    return hash;
}

// MurmurHash3 64-bit
uint64_t HashMurmur3_64(const void* key, size_type len, uint64_t seed = 0) noexcept;

// Combine hashes
MMV2_FORCE_INLINE constexpr uint64_t HashCombine(uint64_t h1, uint64_t h2) noexcept {
    return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
}

template<typename T>
struct Hash {
    uint64_t operator()(const T& value) const noexcept {
        return HashFNV1a(&value, sizeof(T));
    }
};

template<>
struct Hash<int32> {
    uint64_t operator()(int32 value) const noexcept {
        uint64_t x = static_cast<uint64_t>(value);
        x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
        x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
        x = x ^ (x >> 31);
        return x;
    }
};

template<>
struct Hash<uint32> {
    uint64_t operator()(uint32 value) const noexcept {
        uint64_t x = static_cast<uint64_t>(value);
        x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
        x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
        x = x ^ (x >> 31);
        return x;
    }
};

template<>
struct Hash<int64> {
    uint64_t operator()(int64 value) const noexcept {
        uint64_t x = static_cast<uint64_t>(value);
        x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
        x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
        x = x ^ (x >> 31);
        return x;
    }
};

template<>
struct Hash<uint64> {
    uint64_t operator()(uint64 value) const noexcept {
        uint64_t x = value;
        x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
        x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
        x = x ^ (x >> 31);
        return x;
    }
};

template<>
struct Hash<float32> {
    uint64_t operator()(float32 value) const noexcept {
        union { float32 f; uint32 i; } u;
        u.f = value;
        return Hash<uint32>{}(u.i);
    }
};

template<>
struct Hash<float64> {
    uint64_t operator()(float64 value) const noexcept {
        union { float64 f; uint64 i; } u;
        u.f = value;
        return Hash<uint64>{}(u.i);
    }
};

template<>
struct Hash<String> {
    uint64_t operator()(const String& str) const noexcept {
        return HashFNV1a(str.CStr(), str.Size());
    }
};

MMV2_NAMESPACE_END

#endif
