#pragma once
#ifndef MMV2_ARRAY_H
#define MMV2_ARRAY_H

#include "Config.h"
#include "Allocator.h"
#include <type_traits>
#include <utility>
#include <initializer_list>
#include <algorithm>

MMV2_NAMESPACE_BEGIN

template<typename T, size_type N>
class Array {
public:
    using value_type = T;
    using size_type = MMV2::size_type;
    using iterator = T*;
    using const_iterator = const T*;

    MMV2_FORCE_INLINE constexpr Array() noexcept = default;
    MMV2_FORCE_INLINE constexpr Array(std::initializer_list<T> init) noexcept {
        size_type i = 0;
        for (auto& v : init) m_data[i++] = v;
    }

    MMV2_FORCE_INLINE T& operator[](size_type index) noexcept { return m_data[index]; }
    MMV2_FORCE_INLINE const T& operator[](size_type index) const noexcept { return m_data[index]; }
    MMV2_FORCE_INLINE T& At(size_type index) noexcept { return m_data[index]; }
    MMV2_FORCE_INLINE const T& At(size_type index) const noexcept { return m_data[index]; }
    MMV2_FORCE_INLINE T* Data() noexcept { return m_data; }
    MMV2_FORCE_INLINE const T* Data() const noexcept { return m_data; }
    MMV2_FORCE_INLINE static constexpr size_type Size() noexcept { return N; }
    MMV2_FORCE_INLINE static constexpr size_type Capacity() noexcept { return N; }
    MMV2_FORCE_INLINE bool IsEmpty() const noexcept { return N == 0; }
    MMV2_FORCE_INLINE bool IsFull() const noexcept { return true; }
    MMV2_FORCE_INLINE T* Begin() noexcept { return m_data; }
    MMV2_FORCE_INLINE const T* Begin() const noexcept { return m_data; }
    MMV2_FORCE_INLINE T* End() noexcept { return m_data + N; }
    MMV2_FORCE_INLINE const T* End() const noexcept { return m_data + N; }
    MMV2_FORCE_INLINE iterator begin() noexcept { return Begin(); }
    MMV2_FORCE_INLINE const_iterator begin() const noexcept { return Begin(); }
    MMV2_FORCE_INLINE iterator end() noexcept { return End(); }
    MMV2_FORCE_INLINE const_iterator end() const noexcept { return End(); }
    MMV2_FORCE_INLINE T& Front() noexcept { return m_data[0]; }
    MMV2_FORCE_INLINE const T& Front() const noexcept { return m_data[0]; }
    MMV2_FORCE_INLINE T& Back() noexcept { return m_data[N - 1]; }
    MMV2_FORCE_INLINE const T& Back() const noexcept { return m_data[N - 1]; }
    MMV2_FORCE_INLINE void Fill(const T& value) noexcept { for (size_type i = 0; i < N; ++i) m_data[i] = value; }
    MMV2_FORCE_INLINE void Swap(Array& other) noexcept { for (size_type i = 0; i < N; ++i) std::swap(m_data[i], other.m_data[i]); }
    MMV2_FORCE_INLINE bool Contains(const T& value) const noexcept { for (size_type i = 0; i < N; ++i) if (m_data[i] == value) return true; return false; }
    MMV2_FORCE_INLINE size_type IndexOf(const T& value) const noexcept { for (size_type i = 0; i < N; ++i) if (m_data[i] == value) return i; return N; }
    MMV2_FORCE_INLINE T* Find(const T& value) noexcept { for (size_type i = 0; i < N; ++i) if (m_data[i] == value) return &m_data[i]; return nullptr; }
    MMV2_FORCE_INLINE const T* Find(const T& value) const noexcept { for (size_type i = 0; i < N; ++i) if (m_data[i] == value) return &m_data[i]; return nullptr; }

    template<typename Compare>
    MMV2_FORCE_INLINE void Sort(Compare comp) noexcept { std::sort(Begin(), End(), comp); }
    MMV2_FORCE_INLINE void Sort() noexcept { std::sort(Begin(), End()); }

    MMV2_FORCE_INLINE bool operator==(const Array& other) const noexcept {
        for (size_type i = 0; i < N; ++i) if (m_data[i] != other.m_data[i]) return false;
        return true;
    }
    MMV2_FORCE_INLINE bool operator!=(const Array& other) const noexcept { return !(*this == other); }

private:
    T m_data[N];
};

MMV2_NAMESPACE_END

#endif
