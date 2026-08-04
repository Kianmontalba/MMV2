#pragma once
// =============================================================================
// Motion Matching V2 - Core Configuration
// Version: 2.0.0-Alpha
// C++ Standard: C++20/23
// =============================================================================

#ifndef MMV2_CONFIG_H
#define MMV2_CONFIG_H

// =============================================================================
// Version Information
// =============================================================================
#define MMV2_VERSION_MAJOR 2
#define MMV2_VERSION_MINOR 0
#define MMV2_VERSION_PATCH 0
#define MMV2_VERSION_STRING "2.0.0-Alpha"

// =============================================================================
// Platform Detection
// =============================================================================
#if defined(_WIN32) || defined(_WIN64)
    #define MMV2_PLATFORM_WINDOWS 1
    #if defined(_WIN64)
        #define MMV2_PLATFORM_64BIT 1
    #else
        #define MMV2_PLATFORM_32BIT 1
    #endif
#elif defined(__APPLE__) && defined(__MACH__)
    #define MMV2_PLATFORM_MACOS 1
    #define MMV2_PLATFORM_64BIT 1
#elif defined(__linux__)
    #define MMV2_PLATFORM_LINUX 1
    #if defined(__x86_64__) || defined(__aarch64__)
        #define MMV2_PLATFORM_64BIT 1
    #else
        #define MMV2_PLATFORM_32BIT 1
    #endif
#else
    #define MMV2_PLATFORM_UNKNOWN 1
#endif

// =============================================================================
// Compiler Detection
// =============================================================================
#if defined(__clang__)
    #define MMV2_COMPILER_CLANG 1
#elif defined(__GNUC__) || defined(__GNUG__)
    #define MMV2_COMPILER_GCC 1
#elif defined(_MSC_VER)
    #define MMV2_COMPILER_MSVC 1
#endif

// =============================================================================
// SIMD Detection
// =============================================================================
#if defined(__AVX512F__)
    #define MMV2_SIMD_AVX512 1
    #define MMV2_SIMD_LEVEL 512
#elif defined(__AVX2__)
    #define MMV2_SIMD_AVX2 1
    #define MMV2_SIMD_LEVEL 256
#elif defined(__AVX__)
    #define MMV2_SIMD_AVX 1
    #define MMV2_SIMD_LEVEL 256
#elif defined(__SSE4_2__)
    #define MMV2_SIMD_SSE42 1
    #define MMV2_SIMD_LEVEL 128
#elif defined(__SSE4_1__)
    #define MMV2_SIMD_SSE41 1
    #define MMV2_SIMD_LEVEL 128
#elif defined(__SSE3__)
    #define MMV2_SIMD_SSE3 1
    #define MMV2_SIMD_LEVEL 128
#elif defined(__SSE2__)
    #define MMV2_SIMD_SSE2 1
    #define MMV2_SIMD_LEVEL 128
#else
    #define MMV2_SIMD_NONE 1
    #define MMV2_SIMD_LEVEL 0
#endif

// =============================================================================
// Architecture Detection
// =============================================================================
#if defined(__x86_64__) || defined(_M_X64)
    #define MMV2_ARCH_X64 1
#elif defined(__i386__) || defined(_M_IX86)
    #define MMV2_ARCH_X86 1
#elif defined(__aarch64__)
    #define MMV2_ARCH_ARM64 1
#elif defined(__arm__)
    #define MMV2_ARCH_ARM32 1
#endif

// =============================================================================
// Build Configuration
// =============================================================================
#ifdef NDEBUG
    #define MMV2_BUILD_RELEASE 1
    #define MMV2_BUILD_DEBUG 0
#else
    #define MMV2_BUILD_RELEASE 0
    #define MMV2_BUILD_DEBUG 1
#endif

// =============================================================================
// Feature Toggles
// =============================================================================
#define MMV2_ENABLE_SIMD 1
#define MMV2_ENABLE_MULTITHREADING 1
#define MMV2_ENABLE_PROFILING 1
#define MMV2_ENABLE_NETWORKING 1
#define MMV2_ENABLE_AI_INTEGRATION 1
#define MMV2_ENABLE_EDITOR 1
#define MMV2_ENABLE_LIVE_DEBUGGER 1
#define MMV2_ENABLE_RECORDING 1
#define MMV2_ENABLE_VALIDATION 1
#define MMV2_ENABLE_AUTO_OPTIMIZATION 1

// =============================================================================
// Memory Configuration
// =============================================================================
#define MMV2_DEFAULT_ALIGNMENT 64
#define MMV2_CACHE_LINE_SIZE 64
#define MMV2_MAX_BONE_COUNT 256
#define MMV2_MAX_FEATURE_CHANNELS 64
#define MMV2_MAX_SEARCH_RESULTS 32
#define MMV2_MAX_TRAJECTORY_POINTS 16
#define MMV2_MAX_BLEND_CHANNELS 16
#define MMV2_MAX_EVENT_TYPES 256
#define MMV2_MAX_DATABASES 8

// =============================================================================
// Performance Configuration
// =============================================================================
#define MMV2_DEFAULT_SEARCH_BATCH_SIZE 256
#define MMV2_DEFAULT_POSE_HISTORY_SIZE 120
#define MMV2_DEFAULT_FEATURE_VECTOR_SIZE 512
#define MMV2_DEFAULT_KDTREE_LEAF_SIZE 16
#define MMV2_DEFAULT_VPTREE_BRANCH_FACTOR 4
#define MMV2_DEFAULT_ANN_EF_CONSTRUCTION 200
#define MMV2_DEFAULT_ANN_M 16

// =============================================================================
// Epsilon and Precision
// =============================================================================
#define MMV2_EPSILON 1e-6f
#define MMV2_EPSILON_SQ (MMV2_EPSILON * MMV2_EPSILON)
#define MMV2_PI 3.14159265358979323846f
#define MMV2_PI_2 1.57079632679489661923f
#define MMV2_PI_4 0.78539816339744830962f
#define MMV2_TWO_PI 6.28318530717958647692f
#define MMV2_INV_PI 0.31830988618379067154f
#define MMV2_DEG2RAD (MMV2_PI / 180.0f)
#define MMV2_RAD2DEG (180.0f / MMV2_PI)

// =============================================================================
// Export Macros
// =============================================================================
#ifdef MMV2_PLATFORM_WINDOWS
    #ifdef MMV2_BUILD_DLL
        #define MMV2_API __declspec(dllexport)
    #else
        #define MMV2_API __declspec(dllimport)
    #endif
#else
    #define MMV2_API __attribute__((visibility("default")))
#endif

// =============================================================================
// Inline and Force Inline
// =============================================================================
#if defined(MMV2_COMPILER_MSVC)
    #define MMV2_FORCE_INLINE __forceinline
    #define MMV2_NO_INLINE __declspec(noinline)
#elif defined(MMV2_COMPILER_GCC) || defined(MMV2_COMPILER_CLANG)
    #define MMV2_FORCE_INLINE inline __attribute__((always_inline))
    #define MMV2_NO_INLINE __attribute__((noinline))
#else
    #define MMV2_FORCE_INLINE inline
    #define MMV2_NO_INLINE
#endif

// =============================================================================
// Likely/Unlikely Branch Hints
// =============================================================================
#if defined(MMV2_COMPILER_GCC) || defined(MMV2_COMPILER_CLANG)
    #define MMV2_LIKELY(x) __builtin_expect(!!(x), 1)
    #define MMV2_UNLIKELY(x) __builtin_expect(!!(x), 0)
#else
    #define MMV2_LIKELY(x) (x)
    #define MMV2_UNLIKELY(x) (x)
#endif

// =============================================================================
// Alignment Macros
// =============================================================================
#define MMV2_ALIGN(x) alignas(x)
#define MMV2_ALIGN_CACHE MMV2_ALIGN(MMV2_CACHE_LINE_SIZE)
#define MMV2_ALIGN_SIMD MMV2_ALIGN(MMV2_SIMD_LEVEL / 8)

// =============================================================================
// Noexcept Specifiers
// =============================================================================
#define MMV2_NOEXCEPT noexcept
#define MMV2_NOEXCEPT_IF(x) noexcept(x)

// =============================================================================
// Constexpr Specifiers
// =============================================================================
#if __cplusplus >= 202002L
    #define MMV2_CONSTEXPR20 constexpr
#else
    #define MMV2_CONSTEXPR20
#endif

#if __cplusplus >= 202300L
    #define MMV2_CONSTEXPR23 constexpr
#else
    #define MMV2_CONSTEXPR23
#endif

// =============================================================================
// Concept Helpers (C++20)
// =============================================================================
#if __cplusplus >= 202002L
    #include <concepts>
    #define MMV2_CONCEPTS_ENABLED 1
#else
    #define MMV2_CONCEPTS_ENABLED 0
#endif

// =============================================================================
// Module Support (C++20)
// =============================================================================
#if __cplusplus >= 202002L && defined(MMV2_ENABLE_MODULES)
    #define MMV2_MODULES_ENABLED 1
#else
    #define MMV2_MODULES_ENABLED 0
#endif

// =============================================================================
// Assert Macros
// =============================================================================
#include <cassert>

#if MMV2_BUILD_DEBUG
    #define MMV2_ASSERT(x) assert(x)
    #define MMV2_ASSERT_MSG(x, msg) assert((x) && (msg))
#else
    #define MMV2_ASSERT(x) ((void)0)
    #define MMV2_ASSERT_MSG(x, msg) ((void)0)
#endif

// =============================================================================
// Static Assert
// =============================================================================
#define MMV2_STATIC_ASSERT(x) static_assert(x)
#define MMV2_STATIC_ASSERT_MSG(x, msg) static_assert(x, msg)

// =============================================================================
// Deprecation Macro
// =============================================================================
#if defined(MMV2_COMPILER_MSVC)
    #define MMV2_DEPRECATED(msg) __declspec(deprecated(msg))
#elif defined(MMV2_COMPILER_GCC) || defined(MMV2_COMPILER_CLANG)
    #define MMV2_DEPRECATED(msg) __attribute__((deprecated(msg)))
#else
    #define MMV2_DEPRECATED(msg)
#endif

// =============================================================================
// Namespace Macros
// =============================================================================
#define MMV2_NAMESPACE_BEGIN namespace MMV2 {
#define MMV2_NAMESPACE_END }
#define MMV2_USING_NAMESPACE using namespace MMV2;

// =============================================================================
// Type Aliases
// =============================================================================
#include <cstdint>
#include <cstddef>

MMV2_NAMESPACE_BEGIN

using int8 = int8_t;
using int16 = int16_t;
using int32 = int32_t;
using int64 = int64_t;
using uint8 = uint8_t;
using uint16 = uint16_t;
using uint32 = uint32_t;
using uint64 = uint64_t;
using float32 = float;
using float64 = double;
using size_type = std::size_t;
using index_type = std::ptrdiff_t;

MMV2_NAMESPACE_END

#endif // MMV2_CONFIG_H
