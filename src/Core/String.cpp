// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// String Implementation
// ============================================================================

#include "MMV2/Core/String.h"
#include <cstring>
#include <cctype>
#include <cstdarg>

MMV2_NAMESPACE_BEGIN

String::String() : m_data(nullptr), m_length(0), m_capacity(0) {
    m_data = new char[1];
    m_data[0] = '\0';
    m_capacity = 1;
}

String::String(const char* str) : m_data(nullptr), m_length(0), m_capacity(0) {
    if (str) {
        m_length = static_cast<int32>(std::strlen(str));
        m_capacity = m_length + 1;
        m_data = new char[m_capacity];
        std::strcpy(m_data, str);
    } else {
        m_data = new char[1];
        m_data[0] = '\0';
        m_capacity = 1;
    }
}

String::String(const String& other) : m_data(nullptr), m_length(other.m_length), m_capacity(other.m_capacity) {
    m_data = new char[m_capacity];
    std::strcpy(m_data, other.m_data);
}

String::String(String&& other) noexcept : m_data(other.m_data), m_length(other.m_length), m_capacity(other.m_capacity) {
    other.m_data = nullptr;
    other.m_length = 0;
    other.m_capacity = 0;
}

String::~String() { delete[] m_data; }

String& String::operator=(const String& other) {
    if (this != &other) {
        delete[] m_data;
        m_length = other.m_length;
        m_capacity = other.m_capacity;
        m_data = new char[m_capacity];
        std::strcpy(m_data, other.m_data);
    }
    return *this;
}

String& String::operator=(String&& other) noexcept {
    if (this != &other) {
        delete[] m_data;
        m_data = other.m_data;
        m_length = other.m_length;
        m_capacity = other.m_capacity;
        other.m_data = nullptr;
        other.m_length = 0;
        other.m_capacity = 0;
    }
    return *this;
}

String& String::operator=(const char* str) {
    delete[] m_data;
    if (str) {
        m_length = static_cast<int32>(std::strlen(str));
        m_capacity = m_length + 1;
        m_data = new char[m_capacity];
        std::strcpy(m_data, str);
    } else {
        m_data = new char[1];
        m_data[0] = '\0';
        m_length = 0;
        m_capacity = 1;
    }
    return *this;
}

String String::operator+(const String& other) const {
    String result;
    result.m_length = m_length + other.m_length;
    result.m_capacity = result.m_length + 1;
    delete[] result.m_data;
    result.m_data = new char[result.m_capacity];
    std::strcpy(result.m_data, m_data);
    std::strcat(result.m_data, other.m_data);
    return result;
}

String& String::operator+=(const String& other) {
    int32 newLength = m_length + other.m_length;
    if (newLength + 1 > m_capacity) {
        m_capacity = newLength + 1;
        char* newData = new char[m_capacity];
        std::strcpy(newData, m_data);
        delete[] m_data;
        m_data = newData;
    }
    std::strcat(m_data, other.m_data);
    m_length = newLength;
    return *this;
}

bool String::operator==(const String& other) const { return std::strcmp(m_data, other.m_data) == 0; }
bool String::operator!=(const String& other) const { return std::strcmp(m_data, other.m_data) != 0; }
bool String::operator<(const String& other) const { return std::strcmp(m_data, other.m_data) < 0; }

char& String::operator[](int32 index) { return m_data[index]; }
const char& String::operator[](int32 index) const { return m_data[index]; }

const char* String::CStr() const { return m_data; }
int32 String::Length() const { return m_length; }
bool String::Empty() const { return m_length == 0; }

void String::Clear() {
    m_length = 0;
    m_data[0] = '\0';
}

void String::Resize(int32 newLength) {
    if (newLength + 1 > m_capacity) {
        m_capacity = newLength + 1;
        char* newData = new char[m_capacity];
        std::strcpy(newData, m_data);
        delete[] m_data;
        m_data = newData;
    }
    m_length = newLength;
    m_data[m_length] = '\0';
}

void String::ToLower() {
    for (int32 i = 0; i < m_length; ++i) {
        m_data[i] = static_cast<char>(std::tolower(static_cast<unsigned char>(m_data[i])));
    }
}

void String::ToUpper() {
    for (int32 i = 0; i < m_length; ++i) {
        m_data[i] = static_cast<char>(std::toupper(static_cast<unsigned char>(m_data[i])));
    }
}

bool String::Contains(const char* substr) const {
    return std::strstr(m_data, substr) != nullptr;
}

bool String::StartsWith(const char* prefix) const {
    int32 prefixLen = static_cast<int32>(std::strlen(prefix));
    if (prefixLen > m_length) return false;
    return std::strncmp(m_data, prefix, prefixLen) == 0;
}

bool String::EndsWith(const char* suffix) const {
    int32 suffixLen = static_cast<int32>(std::strlen(suffix));
    if (suffixLen > m_length) return false;
    return std::strncmp(m_data + m_length - suffixLen, suffix, suffixLen) == 0;
}

int32 String::Find(const char* substr, int32 startIndex) const {
    if (startIndex < 0 || startIndex >= m_length) return -1;
    const char* found = std::strstr(m_data + startIndex, substr);
    if (!found) return -1;
    return static_cast<int32>(found - m_data);
}

int32 String::FindLast(const char* substr) const {
    int32 lastFound = -1;
    int32 currentPos = 0;
    while (true) {
        const char* found = std::strstr(m_data + currentPos, substr);
        if (!found) break;
        lastFound = static_cast<int32>(found - m_data);
        currentPos = lastFound + 1;
    }
    return lastFound;
}

String String::Substring(int32 start, int32 length) const {
    if (start < 0) start = 0;
    if (start >= m_length) return String();
    if (length < 0 || start + length > m_length) length = m_length - start;
    String result;
    result.Resize(length);
    std::strncpy(result.m_data, m_data + start, length);
    result.m_data[length] = '\0';
    return result;
}

void String::Replace(const char* oldStr, const char* newStr) {
    int32 pos = 0;
    int32 oldLen = static_cast<int32>(std::strlen(oldStr));
    int32 newLen = static_cast<int32>(std::strlen(newStr));
    while ((pos = Find(oldStr, pos)) != -1) {
        String before = Substring(0, pos);
        String after = Substring(pos + oldLen, m_length - pos - oldLen);
        *this = before + newStr + after;
        pos += newLen;
    }
}

String String::Format(const char* fmt, ...) {
    char buffer[4096];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    return String(buffer);
}

MMV2_NAMESPACE_END
