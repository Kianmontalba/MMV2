#pragma once
#ifndef MMV2_STRING_H
#define MMV2_STRING_H

#include "Config.h"
#include "Allocator.h"
#include "Span.h"
#include <cstring>
#include <cstdarg>

MMV2_NAMESPACE_BEGIN

class String {
public:
    using size_type = MMV2::size_type;

    String(IAllocator* allocator = GetDefaultAllocator())
        : m_allocator(allocator), m_data(nullptr), m_size(0), m_capacity(0) {}

    String(const char* str, IAllocator* allocator = GetDefaultAllocator())
        : m_allocator(allocator), m_data(nullptr), m_size(0), m_capacity(0) {
        if (str) {
            size_type len = std::strlen(str);
            Reserve(len + 1);
            std::memcpy(m_data, str, len + 1);
            m_size = len;
        }
    }

    String(const char* str, size_type len, IAllocator* allocator = GetDefaultAllocator())
        : m_allocator(allocator), m_data(nullptr), m_size(0), m_capacity(0) {
        Reserve(len + 1);
        std::memcpy(m_data, str, len);
        m_data[len] = '\0';
        m_size = len;
    }

    String(const String& other)
        : m_allocator(other.m_allocator), m_data(nullptr), m_size(0), m_capacity(0) {
        Reserve(other.m_size + 1);
        std::memcpy(m_data, other.m_data, other.m_size + 1);
        m_size = other.m_size;
    }

    String(String&& other) noexcept
        : m_allocator(other.m_allocator), m_data(other.m_data), m_size(other.m_size), m_capacity(other.m_capacity) {
        other.m_data = nullptr;
        other.m_size = 0;
        other.m_capacity = 0;
    }

    ~String() {
        if (m_data) m_allocator->Deallocate(m_data, m_capacity);
    }

    String& operator=(const String& other) {
        if (this != &other) {
            Clear();
            Reserve(other.m_size + 1);
            std::memcpy(m_data, other.m_data, other.m_size + 1);
            m_size = other.m_size;
        }
        return *this;
    }

    String& operator=(String&& other) noexcept {
        if (this != &other) {
            if (m_data) m_allocator->Deallocate(m_data, m_capacity);
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

    String& operator=(const char* str) {
        Clear();
        if (str) {
            size_type len = std::strlen(str);
            Reserve(len + 1);
            std::memcpy(m_data, str, len + 1);
            m_size = len;
        }
        return *this;
    }

    char& operator[](size_type index) noexcept { return m_data[index]; }
    const char& operator[](size_type index) const noexcept { return m_data[index]; }

    const char* CStr() const noexcept { return m_data ? m_data : ""; }
    char* Data() noexcept { return m_data; }
    const char* Data() const noexcept { return m_data; }
    size_type Size() const noexcept { return m_size; }
    size_type Length() const noexcept { return m_size; }
    size_type Capacity() const noexcept { return m_capacity; }
    bool IsEmpty() const noexcept { return m_size == 0; }

    void Reserve(size_type newCapacity) {
        if (newCapacity <= m_capacity) return;
        size_type allocSize = std::max(newCapacity, m_capacity * 2);
        if (allocSize < 16) allocSize = 16;
        char* newData = static_cast<char*>(m_allocator->Allocate(allocSize, 1));
        if (m_data) {
            std::memcpy(newData, m_data, m_size + 1);
            m_allocator->Deallocate(m_data, m_capacity);
        }
        m_data = newData;
        m_capacity = allocSize;
    }

    void Resize(size_type newSize) {
        Reserve(newSize + 1);
        if (newSize > m_size) {
            for (size_type i = m_size; i < newSize; ++i) m_data[i] = '\0';
        }
        m_data[newSize] = '\0';
        m_size = newSize;
    }

    void Clear() {
        if (m_data) m_data[0] = '\0';
        m_size = 0;
    }

    void Append(char c) {
        Reserve(m_size + 2);
        m_data[m_size++] = c;
        m_data[m_size] = '\0';
    }

    void Append(const char* str) {
        if (!str) return;
        size_type len = std::strlen(str);
        Append(str, len);
    }

    void Append(const char* str, size_type len) {
        Reserve(m_size + len + 1);
        std::memcpy(m_data + m_size, str, len);
        m_size += len;
        m_data[m_size] = '\0';
    }

    void Append(const String& other) { Append(other.m_data, other.m_size); }

    String& operator+=(char c) { Append(c); return *this; }
    String& operator+=(const char* str) { Append(str); return *this; }
    String& operator+=(const String& other) { Append(other); return *this; }

    String operator+(const String& other) const {
        String result(*this);
        result += other;
        return result;
    }

    bool operator==(const String& other) const noexcept {
        if (m_size != other.m_size) return false;
        return std::memcmp(m_data, other.m_data, m_size) == 0;
    }
    bool operator!=(const String& other) const noexcept { return !(*this == other); }
    bool operator==(const char* str) const noexcept {
        if (!str) return m_size == 0;
        return std::strcmp(CStr(), str) == 0;
    }
    bool operator!=(const char* str) const noexcept { return !(*this == str); }
    bool operator<(const String& other) const noexcept { return std::strcmp(CStr(), other.CStr()) < 0; }

    bool StartsWith(const char* prefix) const noexcept {
        if (!prefix) return true;
        size_type prefixLen = std::strlen(prefix);
        if (prefixLen > m_size) return false;
        return std::strncmp(m_data, prefix, prefixLen) == 0;
    }
    bool EndsWith(const char* suffix) const noexcept {
        if (!suffix) return true;
        size_type suffixLen = std::strlen(suffix);
        if (suffixLen > m_size) return false;
        return std::strncmp(m_data + m_size - suffixLen, suffix, suffixLen) == 0;
    }
    bool Contains(const char* substr) const noexcept {
        if (!substr) return true;
        return std::strstr(m_data, substr) != nullptr;
    }

    size_type Find(const char* substr, size_type start = 0) const noexcept {
        if (!substr || start >= m_size) return npos;
        const char* pos = std::strstr(m_data + start, substr);
        if (!pos) return npos;
        return pos - m_data;
    }
    size_type Find(char c, size_type start = 0) const noexcept {
        for (size_type i = start; i < m_size; ++i) if (m_data[i] == c) return i;
        return npos;
    }
    size_type RFind(char c) const noexcept {
        for (size_type i = m_size; i > 0; --i) if (m_data[i - 1] == c) return i - 1;
        return npos;
    }

    String Substring(size_type start, size_type len = npos) const {
        if (start >= m_size) return String(m_allocator);
        if (len == npos || start + len > m_size) len = m_size - start;
        return String(m_data + start, len, m_allocator);
    }

    void Replace(char oldChar, char newChar) {
        for (size_type i = 0; i < m_size; ++i) if (m_data[i] == oldChar) m_data[i] = newChar;
    }

    static String Format(const char* fmt, ...);
    static String FormatV(const char* fmt, va_list args);

    static constexpr size_type npos = ~size_type(0);

private:
    IAllocator* m_allocator;
    char* m_data;
    size_type m_size;
    size_type m_capacity;
};

MMV2_NAMESPACE_END

#endif
