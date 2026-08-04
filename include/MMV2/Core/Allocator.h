#pragma once
#ifndef MMV2_ALLOCATOR_H
#define MMV2_ALLOCATOR_H

#include "Config.h"
#include <cstdlib>
#include <new>

MMV2_NAMESPACE_BEGIN

class IAllocator {
public:
    virtual ~IAllocator() = default;
    virtual void* Allocate(size_type size, size_type alignment = MMV2_DEFAULT_ALIGNMENT) = 0;
    virtual void* Reallocate(void* ptr, size_type oldSize, size_type newSize, size_type alignment = MMV2_DEFAULT_ALIGNMENT) = 0;
    virtual void Deallocate(void* ptr, size_type size = 0) = 0;
    virtual size_type GetAllocatedSize() const = 0;
    virtual size_type GetTotalSize() const = 0;
    virtual const char* GetName() const = 0;
};

class MMV2_API DefaultAllocator : public IAllocator {
public:
    DefaultAllocator(const char* name = "Default") : m_name(name), m_allocatedSize(0) {}
    void* Allocate(size_type size, size_type alignment = MMV2_DEFAULT_ALIGNMENT) override;
    void* Reallocate(void* ptr, size_type oldSize, size_type newSize, size_type alignment = MMV2_DEFAULT_ALIGNMENT) override;
    void Deallocate(void* ptr, size_type size = 0) override;
    size_type GetAllocatedSize() const override { return m_allocatedSize; }
    size_type GetTotalSize() const override { return m_allocatedSize; }
    const char* GetName() const override { return m_name; }
private:
    const char* m_name;
    size_type m_allocatedSize;
};

class MMV2_API LinearAllocator : public IAllocator {
public:
    LinearAllocator(size_type capacity, const char* name = "Linear");
    ~LinearAllocator();
    void* Allocate(size_type size, size_type alignment = MMV2_DEFAULT_ALIGNMENT) override;
    void* Reallocate(void* ptr, size_type oldSize, size_type newSize, size_type alignment = MMV2_DEFAULT_ALIGNMENT) override;
    void Deallocate(void* ptr, size_type size = 0) override;
    size_type GetAllocatedSize() const override { return m_offset; }
    size_type GetTotalSize() const override { return m_capacity; }
    const char* GetName() const override { return m_name; }
    void Reset();
    void* GetBase() const { return m_buffer; }
private:
    void* m_buffer;
    size_type m_capacity;
    size_type m_offset;
    const char* m_name;
};

class MMV2_API StackAllocator : public IAllocator {
public:
    struct AllocationHeader {
        size_type size;
        size_type padding;
    };

    StackAllocator(size_type capacity, const char* name = "Stack");
    ~StackAllocator();
    void* Allocate(size_type size, size_type alignment = MMV2_DEFAULT_ALIGNMENT) override;
    void* Reallocate(void* ptr, size_type oldSize, size_type newSize, size_type alignment = MMV2_DEFAULT_ALIGNMENT) override;
    void Deallocate(void* ptr, size_type size = 0) override;
    size_type GetAllocatedSize() const override { return m_offset; }
    size_type GetTotalSize() const override { return m_capacity; }
    const char* GetName() const override { return m_name; }
    void Reset();
    size_type GetMarker() const { return m_offset; }
    void FreeToMarker(size_type marker);
private:
    void* m_buffer;
    size_type m_capacity;
    size_type m_offset;
    const char* m_name;
};

class MMV2_API PoolAllocator : public IAllocator {
public:
    PoolAllocator(size_type elementSize, size_type elementCount, size_type alignment = MMV2_DEFAULT_ALIGNMENT, const char* name = "Pool");
    ~PoolAllocator();
    void* Allocate(size_type size, size_type alignment = MMV2_DEFAULT_ALIGNMENT) override;
    void* Reallocate(void* ptr, size_type oldSize, size_type newSize, size_type alignment = MMV2_DEFAULT_ALIGNMENT) override;
    void Deallocate(void* ptr, size_type size = 0) override;
    size_type GetAllocatedSize() const override { return m_allocatedCount * m_elementSize; }
    size_type GetTotalSize() const override { return m_elementCount * m_elementSize; }
    const char* GetName() const override { return m_name; }
    size_type GetFreeCount() const { return m_freeCount; }
    size_type GetAllocatedCount() const { return m_allocatedCount; }
private:
    struct FreeNode { FreeNode* next; };
    void* m_buffer;
    size_type m_elementSize;
    size_type m_elementCount;
    size_type m_alignment;
    size_type m_allocatedCount;
    size_type m_freeCount;
    FreeNode* m_freeList;
    const char* m_name;
};

class MMV2_API ArenaAllocator : public IAllocator {
public:
    ArenaAllocator(size_type blockSize, const char* name = "Arena");
    ~ArenaAllocator();
    void* Allocate(size_type size, size_type alignment = MMV2_DEFAULT_ALIGNMENT) override;
    void* Reallocate(void* ptr, size_type oldSize, size_type newSize, size_type alignment = MMV2_DEFAULT_ALIGNMENT) override;
    void Deallocate(void* ptr, size_type size = 0) override;
    size_type GetAllocatedSize() const override;
    size_type GetTotalSize() const override;
    const char* GetName() const override { return m_name; }
    void Reset();
private:
    struct Block {
        void* memory;
        size_type size;
        size_type used;
        Block* next;
    };
    Block* m_head;
    size_type m_blockSize;
    size_type m_totalAllocated;
    const char* m_name;
    Block* AllocateBlock(size_type minSize);
};

MMV2_API IAllocator* GetDefaultAllocator();
MMV2_API void SetDefaultAllocator(IAllocator* allocator);

template<typename T, typename... Args>
T* New(IAllocator* allocator, Args&&... args) {
    void* mem = allocator->Allocate(sizeof(T), alignof(T));
    return new (mem) T(std::forward<Args>(args)...);
}

template<typename T>
void Delete(IAllocator* allocator, T* ptr) {
    if (ptr) {
        ptr->~T();
        allocator->Deallocate(ptr, sizeof(T));
    }
}

MMV2_NAMESPACE_END

#endif
