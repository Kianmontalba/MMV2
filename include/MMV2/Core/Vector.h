#pragma once
#ifndef MMV2_VECTOR_H
#define MMV2_VECTOR_H

#include "Config.h"
#include "Allocator.h"
#include <type_traits>
#include <utility>
#include <initializer_list>
#include <algorithm>
#include <cstring>

MMV2_NAMESPACE_BEGIN

template<typename T>
class Vector {
public:
    using value_type = T;
    using size_type = MMV2::size_type;
    using iterator = T*;
    using const_iterator = const T*;
    using reference = T&;
    using const_reference = const T&;

    Vector(IAllocator* allocator = GetDefaultAllocator())
        : m_allocator(allocator), m_data(nullptr), m_size(0), m_capacity(0) {}

    Vector(size_type count, IAllocator* allocator = GetDefaultAllocator())
        : m_allocator(allocator), m_data(nullptr), m_size(0), m_capacity(0) {
        Reserve(count);
        for (size_type i = 0; i < count; ++i) new (&m_data[i]) T();
        m_size = count;
    }

    Vector(size_type count, const T& value, IAllocator* allocator = GetDefaultAllocator())
        : m_allocator(allocator), m_data(nullptr), m_size(0), m_capacity(0) {
        Reserve(count);
        for (size_type i = 0; i < count; ++i) new (&m_data[i]) T(value);
        m_size = count;
    }

    Vector(std::initializer_list<T> init, IAllocator* allocator = GetDefaultAllocator())
        : m_allocator(allocator), m_data(nullptr), m_size(0), m_capacity(0) {
        Reserve(init.size());
        size_type i = 0;
        for (const auto& v : init) new (&m_data[i++]) T(v);
        m_size = init.size();
    }

    Vector(const Vector& other)
        : m_allocator(other.m_allocator), m_data(nullptr), m_size(0), m_capacity(0) {
        Reserve(other.m_size);
        for (size_type i = 0; i < other.m_size; ++i) new (&m_data[i]) T(other.m_data[i]);
        m_size = other.m_size;
    }

    Vector(Vector&& other) noexcept
        : m_allocator(other.m_allocator), m_data(other.m_data), m_size(other.m_size), m_capacity(other.m_capacity) {
        other.m_data = nullptr;
        other.m_size = 0;
        other.m_capacity = 0;
    }

    ~Vector() {
        Clear();
        if (m_data) m_allocator->Deallocate(m_data, m_capacity * sizeof(T));
    }

    Vector& operator=(const Vector& other) {
        if (this != &other) {
            Clear();
            Reserve(other.m_size);
            for (size_type i = 0; i < other.m_size; ++i) new (&m_data[i]) T(other.m_data[i]);
            m_size = other.m_size;
        }
        return *this;
    }

    Vector& operator=(Vector&& other) noexcept {
        if (this != &other) {
            Clear();
            if (m_data) m_allocator->Deallocate(m_data, m_capacity * sizeof(T));
            m_allocator = other.m_allocator;
            m_data = other.m_data;
            m_size = other.m_size;
            m_capacity = other.m_capacity;
            other.m_data = nullptr;
            other.m_size = 0;
            other.m_capacity = 0;
        }
        return *this;
    }

    // Element access
    MMV2_FORCE_INLINE T& operator[](size_type index) noexcept { return m_data[index]; }
    MMV2_FORCE_INLINE const T& operator[](size_type index) const noexcept { return m_data[index]; }
    MMV2_FORCE_INLINE T& At(size_type index) noexcept { return m_data[index]; }
    MMV2_FORCE_INLINE const T& At(size_type index) const noexcept { return m_data[index]; }
    MMV2_FORCE_INLINE T& Front() noexcept { return m_data[0]; }
    MMV2_FORCE_INLINE const T& Front() const noexcept { return m_data[0]; }
    MMV2_FORCE_INLINE T& Back() noexcept { return m_data[m_size - 1]; }
    MMV2_FORCE_INLINE const T& Back() const noexcept { return m_data[m_size - 1]; }
    MMV2_FORCE_INLINE T* Data() noexcept { return m_data; }
    MMV2_FORCE_INLINE const T* Data() const noexcept { return m_data; }

    // Iterators
    MMV2_FORCE_INLINE iterator Begin() noexcept { return m_data; }
    MMV2_FORCE_INLINE const_iterator Begin() const noexcept { return m_data; }
    MMV2_FORCE_INLINE iterator End() noexcept { return m_data + m_size; }
    MMV2_FORCE_INLINE const_iterator End() const noexcept { return m_data + m_size; }
    MMV2_FORCE_INLINE iterator begin() noexcept { return Begin(); }
    MMV2_FORCE_INLINE const_iterator begin() const noexcept { return Begin(); }
    MMV2_FORCE_INLINE iterator end() noexcept { return End(); }
    MMV2_FORCE_INLINE const_iterator end() const noexcept { return End(); }

    // Capacity
    MMV2_FORCE_INLINE bool IsEmpty() const noexcept { return m_size == 0; }
    MMV2_FORCE_INLINE size_type Size() const noexcept { return m_size; }
    MMV2_FORCE_INLINE size_type Capacity() const noexcept { return m_capacity; }

    void Reserve(size_type newCapacity) {
        if (newCapacity <= m_capacity) return;
        size_type allocSize = std::max(newCapacity, m_capacity * 2);
        if (allocSize < 4) allocSize = 4;
        T* newData = static_cast<T*>(m_allocator->Allocate(allocSize * sizeof(T), alignof(T)));
        for (size_type i = 0; i < m_size; ++i) {
            new (&newData[i]) T(std::move(m_data[i]));
            m_data[i].~T();
        }
        if (m_data) m_allocator->Deallocate(m_data, m_capacity * sizeof(T));
        m_data = newData;
        m_capacity = allocSize;
    }

    void Resize(size_type newSize) {
        if (newSize < m_size) {
            for (size_type i = newSize; i < m_size; ++i) m_data[i].~T();
        } else if (newSize > m_size) {
            Reserve(newSize);
            for (size_type i = m_size; i < newSize; ++i) new (&m_data[i]) T();
        }
        m_size = newSize;
    }

    void Resize(size_type newSize, const T& value) {
        if (newSize < m_size) {
            for (size_type i = newSize; i < m_size; ++i) m_data[i].~T();
        } else if (newSize > m_size) {
            Reserve(newSize);
            for (size_type i = m_size; i < newSize; ++i) new (&m_data[i]) T(value);
        }
        m_size = newSize;
    }

    void ShrinkToFit() {
        if (m_size == m_capacity) return;
        if (m_size == 0) {
            if (m_data) m_allocator->Deallocate(m_data, m_capacity * sizeof(T));
            m_data = nullptr;
            m_capacity = 0;
            return;
        }
        T* newData = static_cast<T*>(m_allocator->Allocate(m_size * sizeof(T), alignof(T)));
        for (size_type i = 0; i < m_size; ++i) {
            new (&newData[i]) T(std::move(m_data[i]));
            m_data[i].~T();
        }
        m_allocator->Deallocate(m_data, m_capacity * sizeof(T));
        m_data = newData;
        m_capacity = m_size;
    }

    // Modifiers
    void PushBack(const T& value) {
        Reserve(m_size + 1);
        new (&m_data[m_size]) T(value);
        ++m_size;
    }

    void PushBack(T&& value) {
        Reserve(m_size + 1);
        new (&m_data[m_size]) T(std::move(value));
        ++m_size;
    }

    template<typename... Args>
    T& EmplaceBack(Args&&... args) {
        Reserve(m_size + 1);
        new (&m_data[m_size]) T(std::forward<Args>(args)...);
        return m_data[m_size++];
    }

    void PopBack() {
        if (m_size > 0) {
            m_data[--m_size].~T();
        }
    }

    iterator Insert(const_iterator pos, const T& value) {
        size_type index = pos - m_data;
        Reserve(m_size + 1);
        for (size_type i = m_size; i > index; --i) {
            new (&m_data[i]) T(std::move(m_data[i - 1]));
            m_data[i - 1].~T();
        }
        new (&m_data[index]) T(value);
        ++m_size;
        return m_data + index;
    }

    iterator Erase(const_iterator pos) {
        size_type index = pos - m_data;
        m_data[index].~T();
        for (size_type i = index; i < m_size - 1; ++i) {
            new (&m_data[i]) T(std::move(m_data[i + 1]));
            m_data[i + 1].~T();
        }
        --m_size;
        return m_data + index;
    }

    iterator Erase(const_iterator first, const_iterator last) {
        size_type start = first - m_data;
        size_type count = last - first;
        for (size_type i = start; i < start + count; ++i) m_data[i].~T();
        for (size_type i = start; i < m_size - count; ++i) {
            new (&m_data[i]) T(std::move(m_data[i + count]));
            m_data[i + count].~T();
        }
        m_size -= count;
        return m_data + start;
    }

    void Clear() {
        for (size_type i = 0; i < m_size; ++i) m_data[i].~T();
        m_size = 0;
    }

    template<typename... Args>
    iterator Emplace(const_iterator pos, Args&&... args) {
        size_type index = pos - m_data;
        Reserve(m_size + 1);
        for (size_type i = m_size; i > index; --i) {
            new (&m_data[i]) T(std::move(m_data[i - 1]));
            m_data[i - 1].~T();
        }
        new (&m_data[index]) T(std::forward<Args>(args)...);
        ++m_size;
        return m_data + index;
    }

    // Search
    MMV2_FORCE_INLINE iterator Find(const T& value) noexcept {
        for (size_type i = 0; i < m_size; ++i) if (m_data[i] == value) return m_data + i;
        return End();
    }
    MMV2_FORCE_INLINE const_iterator Find(const T& value) const noexcept {
        for (size_type i = 0; i < m_size; ++i) if (m_data[i] == value) return m_data + i;
        return End();
    }
    MMV2_FORCE_INLINE bool Contains(const T& value) const noexcept { return Find(value) != End(); }
    MMV2_FORCE_INLINE size_type IndexOf(const T& value) const noexcept {
        for (size_type i = 0; i < m_size; ++i) if (m_data[i] == value) return i;
        return m_size;
    }

    // Sort
    template<typename Compare>
    void Sort(Compare comp) { std::sort(Begin(), End(), comp); }
    void Sort() { std::sort(Begin(), End()); }

    void Swap(Vector& other) noexcept {
        std::swap(m_allocator, other.m_allocator);
        std::swap(m_data, other.m_data);
        std::swap(m_size, other.m_size);
        std::swap(m_capacity, other.m_capacity);
    }

    bool operator==(const Vector& other) const noexcept {
        if (m_size != other.m_size) return false;
        for (size_type i = 0; i < m_size; ++i) if (m_data[i] != other.m_data[i]) return false;
        return true;
    }
    bool operator!=(const Vector& other) const noexcept { return !(*this == other); }

private:
    IAllocator* m_allocator;
    T* m_data;
    size_type m_size;
    size_type m_capacity;
};

MMV2_NAMESPACE_END

#endif
