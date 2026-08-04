#pragma once
#ifndef MMV2_HASHMAP_H
#define MMV2_HASHMAP_H

#include "Config.h"
#include "Allocator.h"
#include "Vector.h"
#include "Hash.h"
#include <utility>

MMV2_NAMESPACE_BEGIN

template<typename K, typename V, typename HashFunc = Hash<K>>
class HashMap {
public:
    using KeyType = K;
    using ValueType = V;
    using size_type = MMV2::size_type;

    struct Entry {
        K key;
        V value;
        uint64_t hash;
        bool occupied;
        bool deleted;

        Entry() : hash(0), occupied(false), deleted(false) {}
    };

    struct Iterator {
        Entry* entries;
        size_type capacity;
        size_type index;

        Iterator(Entry* e, size_type cap, size_type idx) : entries(e), capacity(cap), index(idx) {
            while (index < capacity && !entries[index].occupied) ++index;
        }

        std::pair<K&, V&> operator*() { return { entries[index].key, entries[index].value }; }
        std::pair<const K&, V&> operator*() const { return { entries[index].key, entries[index].value }; }

        Iterator& operator++() {
            ++index;
            while (index < capacity && !entries[index].occupied) ++index;
            return *this;
        }
        bool operator!=(const Iterator& other) const { return index != other.index; }
        bool operator==(const Iterator& other) const { return index == other.index; }
    };

    HashMap(IAllocator* allocator = GetDefaultAllocator())
        : m_allocator(allocator), m_entries(nullptr), m_capacity(0), m_size(0), m_deletedCount(0) {}

    ~HashMap() {
        Clear();
        if (m_entries) m_allocator->Deallocate(m_entries, m_capacity * sizeof(Entry));
    }

    HashMap(const HashMap& other)
        : m_allocator(other.m_allocator), m_entries(nullptr), m_capacity(0), m_size(0), m_deletedCount(0) {
        Reserve(other.m_size);
        for (size_type i = 0; i < other.m_capacity; ++i) {
            if (other.m_entries[i].occupied) {
                Insert(other.m_entries[i].key, other.m_entries[i].value);
            }
        }
    }

    HashMap(HashMap&& other) noexcept
        : m_allocator(other.m_allocator), m_entries(other.m_entries), m_capacity(other.m_capacity),
          m_size(other.m_size), m_deletedCount(other.m_deletedCount) {
        other.m_entries = nullptr;
        other.m_capacity = 0;
        other.m_size = 0;
        other.m_deletedCount = 0;
    }

    HashMap& operator=(const HashMap& other) {
        if (this != &other) {
            Clear();
            if (m_entries) m_allocator->Deallocate(m_entries, m_capacity * sizeof(Entry));
            m_entries = nullptr;
            m_capacity = 0;
            m_size = 0;
            m_deletedCount = 0;
            Reserve(other.m_size);
            for (size_type i = 0; i < other.m_capacity; ++i) {
                if (other.m_entries[i].occupied) {
                    Insert(other.m_entries[i].key, other.m_entries[i].value);
                }
            }
        }
        return *this;
    }

    V& operator[](const K& key) {
        if (m_size * 2 >= m_capacity) Grow();
        size_type index = FindEntry(key);
        if (!m_entries[index].occupied) {
            new (&m_entries[index].key) K(key);
            new (&m_entries[index].value) V();
            m_entries[index].hash = HashFunc{}(key);
            m_entries[index].occupied = true;
            m_entries[index].deleted = false;
            ++m_size;
        }
        return m_entries[index].value;
    }

    const V* Find(const K& key) const {
        if (m_size == 0) return nullptr;
        size_type index = FindEntry(key);
        if (m_entries[index].occupied) return &m_entries[index].value;
        return nullptr;
    }

    V* Find(const K& key) {
        if (m_size == 0) return nullptr;
        size_type index = FindEntry(key);
        if (m_entries[index].occupied) return &m_entries[index].value;
        return nullptr;
    }

    bool Contains(const K& key) const { return Find(key) != nullptr; }

    bool Insert(const K& key, const V& value) {
        if (m_size * 2 >= m_capacity) Grow();
        size_type index = FindEntry(key);
        if (m_entries[index].occupied) return false;
        new (&m_entries[index].key) K(key);
        new (&m_entries[index].value) V(value);
        m_entries[index].hash = HashFunc{}(key);
        m_entries[index].occupied = true;
        m_entries[index].deleted = false;
        ++m_size;
        return true;
    }

    bool Insert(const K& key, V&& value) {
        if (m_size * 2 >= m_capacity) Grow();
        size_type index = FindEntry(key);
        if (m_entries[index].occupied) return false;
        new (&m_entries[index].key) K(key);
        new (&m_entries[index].value) V(std::move(value));
        m_entries[index].hash = HashFunc{}(key);
        m_entries[index].occupied = true;
        m_entries[index].deleted = false;
        ++m_size;
        return true;
    }

    template<typename... Args>
    bool Emplace(const K& key, Args&&... args) {
        if (m_size * 2 >= m_capacity) Grow();
        size_type index = FindEntry(key);
        if (m_entries[index].occupied) return false;
        new (&m_entries[index].key) K(key);
        new (&m_entries[index].value) V(std::forward<Args>(args)...);
        m_entries[index].hash = HashFunc{}(key);
        m_entries[index].occupied = true;
        m_entries[index].deleted = false;
        ++m_size;
        return true;
    }

    bool Remove(const K& key) {
        if (m_size == 0) return false;
        size_type index = FindEntry(key);
        if (!m_entries[index].occupied) return false;
        m_entries[index].key.~K();
        m_entries[index].value.~V();
        m_entries[index].occupied = false;
        m_entries[index].deleted = true;
        --m_size;
        ++m_deletedCount;
        return true;
    }

    void Clear() {
        for (size_type i = 0; i < m_capacity; ++i) {
            if (m_entries[i].occupied) {
                m_entries[i].key.~K();
                m_entries[i].value.~V();
                m_entries[i].occupied = false;
            }
        }
        m_size = 0;
        m_deletedCount = 0;
    }

    void Reserve(size_type count) {
        size_type newCapacity = count * 2;
        if (newCapacity < 16) newCapacity = 16;
        if (newCapacity <= m_capacity) return;
        Rehash(newCapacity);
    }

    size_type Size() const noexcept { return m_size; }
    size_type Capacity() const noexcept { return m_capacity; }
    bool IsEmpty() const noexcept { return m_size == 0; }

    Iterator Begin() { return Iterator(m_entries, m_capacity, 0); }
    Iterator End() { return Iterator(m_entries, m_capacity, m_capacity); }
    Iterator begin() { return Begin(); }
    Iterator end() { return End(); }

    // Key/Value iteration helpers
    Vector<K> GetKeys() const {
        Vector<K> keys(m_allocator);
        keys.Reserve(m_size);
        for (size_type i = 0; i < m_capacity; ++i) {
            if (m_entries[i].occupied) keys.PushBack(m_entries[i].key);
        }
        return keys;
    }

    Vector<V> GetValues() const {
        Vector<V> values(m_allocator);
        values.Reserve(m_size);
        for (size_type i = 0; i < m_capacity; ++i) {
            if (m_entries[i].occupied) values.PushBack(m_entries[i].value);
        }
        return values;
    }

private:
    IAllocator* m_allocator;
    Entry* m_entries;
    size_type m_capacity;
    size_type m_size;
    size_type m_deletedCount;

    size_type FindEntry(const K& key) const {
        uint64_t hash = HashFunc{}(key);
        size_type index = hash & (m_capacity - 1);
        size_type firstDeleted = ~size_type(0);

        while (m_entries[index].occupied || m_entries[index].deleted) {
            if (m_entries[index].occupied && m_entries[index].hash == hash && m_entries[index].key == key) {
                return index;
            }
            if (m_entries[index].deleted && firstDeleted == ~size_type(0)) {
                firstDeleted = index;
            }
            index = (index + 1) & (m_capacity - 1);
        }
        return firstDeleted != ~size_type(0) ? firstDeleted : index;
    }

    void Grow() {
        size_type newCapacity = m_capacity == 0 ? 16 : m_capacity * 2;
        Rehash(newCapacity);
    }

    void Rehash(size_type newCapacity) {
        Entry* oldEntries = m_entries;
        size_type oldCapacity = m_capacity;

        m_entries = static_cast<Entry*>(m_allocator->Allocate(newCapacity * sizeof(Entry), alignof(Entry)));
        for (size_type i = 0; i < newCapacity; ++i) {
            m_entries[i].occupied = false;
            m_entries[i].deleted = false;
        }
        m_capacity = newCapacity;
        m_size = 0;
        m_deletedCount = 0;

        if (oldEntries) {
            for (size_type i = 0; i < oldCapacity; ++i) {
                if (oldEntries[i].occupied) {
                    size_type index = FindEntry(oldEntries[i].key);
                    new (&m_entries[index].key) K(std::move(oldEntries[i].key));
                    new (&m_entries[index].value) V(std::move(oldEntries[i].value));
                    m_entries[index].hash = oldEntries[i].hash;
                    m_entries[index].occupied = true;
                    m_entries[index].deleted = false;
                    ++m_size;
                    oldEntries[i].key.~K();
                    oldEntries[i].value.~V();
                }
            }
            m_allocator->Deallocate(oldEntries, oldCapacity * sizeof(Entry));
        }
    }
};

MMV2_NAMESPACE_END

#endif
