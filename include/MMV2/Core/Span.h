#pragma once
#ifndef MMV2_SPAN_H
#define MMV2_SPAN_H

#include "Config.h"

MMV2_NAMESPACE_BEGIN

template<typename T>
class Span {
public:
    using value_type = T;
    using size_type = MMV2::size_type;
    using iterator = T*;
    using const_iterator = const T*;

    MMV2_FORCE_INLINE constexpr Span() noexcept : m_data(nullptr), m_size(0) {}
    MMV2_FORCE_INLINE constexpr Span(T* data, size_type size) noexcept : m_data(data), m_size(size) {}
    MMV2_FORCE_INLINE constexpr Span(T* first, T* last) noexcept : m_data(first), m_size(last - first) {}
    template<size_type N>
    MMV2_FORCE_INLINE constexpr Span(T (&arr)[N]) noexcept : m_data(arr), m_size(N) {}
    template<typename Container>
    MMV2_FORCE_INLINE constexpr Span(Container& container) noexcept : m_data(container.Data()), m_size(container.Size()) {}

    MMV2_FORCE_INLINE T& operator[](size_type index) noexcept { return m_data[index]; }
    MMV2_FORCE_INLINE const T& operator[](size_type index) const noexcept { return m_data[index]; }
    MMV2_FORCE_INLINE T* Data() noexcept { return m_data; }
    MMV2_FORCE_INLINE const T* Data() const noexcept { return m_data; }
    MMV2_FORCE_INLINE size_type Size() const noexcept { return m_size; }
    MMV2_FORCE_INLINE bool IsEmpty() const noexcept { return m_size == 0; }
    MMV2_FORCE_INLINE iterator Begin() noexcept { return m_data; }
    MMV2_FORCE_INLINE const_iterator Begin() const noexcept { return m_data; }
    MMV2_FORCE_INLINE iterator End() noexcept { return m_data + m_size; }
    MMV2_FORCE_INLINE const_iterator End() const noexcept { return m_data + m_size; }
    MMV2_FORCE_INLINE iterator begin() noexcept { return Begin(); }
    MMV2_FORCE_INLINE const_iterator begin() const noexcept { return Begin(); }
    MMV2_FORCE_INLINE iterator end() noexcept { return End(); }
    MMV2_FORCE_INLINE const_iterator end() const noexcept { return End(); }
    MMV2_FORCE_INLINE T& Front() noexcept { return m_data[0]; }
    MMV2_FORCE_INLINE const T& Front() const noexcept { return m_data[0]; }
    MMV2_FORCE_INLINE T& Back() noexcept { return m_data[m_size - 1]; }
    MMV2_FORCE_INLINE const T& Back() const noexcept { return m_data[m_size - 1]; }

    MMV2_FORCE_INLINE Span Subspan(size_type offset, size_type count = ~size_type(0)) const noexcept {
        if (offset >= m_size) return Span();
        if (count > m_size - offset) count = m_size - offset;
        return Span(m_data + offset, count);
    }

    MMV2_FORCE_INLINE T* Find(const T& value) noexcept {
        for (size_type i = 0; i < m_size; ++i) if (m_data[i] == value) return &m_data[i];
        return nullptr;
    }
    MMV2_FORCE_INLINE const T* Find(const T& value) const noexcept {
        for (size_type i = 0; i < m_size; ++i) if (m_data[i] == value) return &m_data[i];
        return nullptr;
    }
    MMV2_FORCE_INLINE bool Contains(const T& value) const noexcept { return Find(value) != nullptr; }

private:
    T* m_data;
    size_type m_size;
};

template<typename T>
class ConstSpan {
public:
    using value_type = T;
    using size_type = MMV2::size_type;
    using const_iterator = const T*;

    MMV2_FORCE_INLINE constexpr ConstSpan() noexcept : m_data(nullptr), m_size(0) {}
    MMV2_FORCE_INLINE constexpr ConstSpan(const T* data, size_type size) noexcept : m_data(data), m_size(size) {}
    MMV2_FORCE_INLINE constexpr ConstSpan(const T* first, const T* last) noexcept : m_data(first), m_size(last - first) {}
    template<size_type N>
    MMV2_FORCE_INLINE constexpr ConstSpan(const T (&arr)[N]) noexcept : m_data(arr), m_size(N) {}
    template<typename Container>
    MMV2_FORCE_INLINE constexpr ConstSpan(const Container& container) noexcept : m_data(container.Data()), m_size(container.Size()) {}
    MMV2_FORCE_INLINE constexpr ConstSpan(const Span<T>& span) noexcept : m_data(span.Data()), m_size(span.Size()) {}

    MMV2_FORCE_INLINE const T& operator[](size_type index) const noexcept { return m_data[index]; }
    MMV2_FORCE_INLINE const T* Data() const noexcept { return m_data; }
    MMV2_FORCE_INLINE size_type Size() const noexcept { return m_size; }
    MMV2_FORCE_INLINE bool IsEmpty() const noexcept { return m_size == 0; }
    MMV2_FORCE_INLINE const_iterator Begin() const noexcept { return m_data; }
    MMV2_FORCE_INLINE const_iterator End() const noexcept { return m_data + m_size; }
    MMV2_FORCE_INLINE const_iterator begin() const noexcept { return Begin(); }
    MMV2_FORCE_INLINE const_iterator end() const noexcept { return End(); }
    MMV2_FORCE_INLINE const T& Front() const noexcept { return m_data[0]; }
    MMV2_FORCE_INLINE const T& Back() const noexcept { return m_data[m_size - 1]; }

    MMV2_FORCE_INLINE ConstSpan Subspan(size_type offset, size_type count = ~size_type(0)) const noexcept {
        if (offset >= m_size) return ConstSpan();
        if (count > m_size - offset) count = m_size - offset;
        return ConstSpan(m_data + offset, count);
    }

    MMV2_FORCE_INLINE const T* Find(const T& value) const noexcept {
        for (size_type i = 0; i < m_size; ++i) if (m_data[i] == value) return &m_data[i];
        return nullptr;
    }
    MMV2_FORCE_INLINE bool Contains(const T& value) const noexcept { return Find(value) != nullptr; }

private:
    const T* m_data;
    size_type m_size;
};

MMV2_NAMESPACE_END

#endif
