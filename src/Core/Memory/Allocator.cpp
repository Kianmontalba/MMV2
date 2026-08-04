#include "MMV2/Core/Allocator.h"
#include <cstring>
#include <algorithm>

MMV2_NAMESPACE_BEGIN

static DefaultAllocator s_defaultAllocator("GlobalDefault");
static IAllocator* s_currentAllocator = &s_defaultAllocator;

IAllocator* GetDefaultAllocator() { return s_currentAllocator; }
void SetDefaultAllocator(IAllocator* allocator) { s_currentAllocator = allocator; }

// DefaultAllocator
void* DefaultAllocator::Allocate(size_type size, size_type alignment) {
    void* ptr = std::aligned_alloc(alignment, size);
    if (!ptr) throw std::bad_alloc();
    m_allocatedSize += size;
    return ptr;
}

void* DefaultAllocator::Reallocate(void* ptr, size_type oldSize, size_type newSize, size_type alignment) {
    void* newPtr = std::aligned_alloc(alignment, newSize);
    if (!newPtr) throw std::bad_alloc();
    if (ptr) {
        std::memcpy(newPtr, ptr, std::min(oldSize, newSize));
        std::free(ptr);
    }
    m_allocatedSize += newSize - oldSize;
    return newPtr;
}

void DefaultAllocator::Deallocate(void* ptr, size_type size) {
    if (ptr) {
        std::free(ptr);
        m_allocatedSize -= size;
    }
}

// LinearAllocator
LinearAllocator::LinearAllocator(size_type capacity, const char* name)
    : m_capacity(capacity), m_offset(0), m_name(name) {
    m_buffer = std::aligned_alloc(MMV2_DEFAULT_ALIGNMENT, capacity);
    if (!m_buffer) throw std::bad_alloc();
}

LinearAllocator::~LinearAllocator() {
    std::free(m_buffer);
}

void* LinearAllocator::Allocate(size_type size, size_type alignment) {
    size_type alignedOffset = (m_offset + alignment - 1) & ~(alignment - 1);
    if (alignedOffset + size > m_capacity) return nullptr;
    void* ptr = static_cast<char*>(m_buffer) + alignedOffset;
    m_offset = alignedOffset + size;
    return ptr;
}

void* LinearAllocator::Reallocate(void* ptr, size_type oldSize, size_type newSize, size_type alignment) {
    void* newPtr = Allocate(newSize, alignment);
    if (newPtr && ptr) std::memcpy(newPtr, ptr, std::min(oldSize, newSize));
    return newPtr;
}

void LinearAllocator::Deallocate(void* ptr, size_type size) {
    // Linear allocator doesn't support individual deallocation
}

void LinearAllocator::Reset() {
    m_offset = 0;
}

// StackAllocator
StackAllocator::StackAllocator(size_type capacity, const char* name)
    : m_capacity(capacity), m_offset(0), m_name(name) {
    m_buffer = std::aligned_alloc(MMV2_DEFAULT_ALIGNMENT, capacity);
    if (!m_buffer) throw std::bad_alloc();
}

StackAllocator::~StackAllocator() {
    std::free(m_buffer);
}

void* StackAllocator::Allocate(size_type size, size_type alignment) {
    size_type currentAddr = reinterpret_cast<size_type>(static_cast<char*>(m_buffer) + m_offset);
    size_type padding = (alignment - (currentAddr % alignment)) % alignment;
    size_type totalSize = size + padding + sizeof(AllocationHeader);
    if (m_offset + totalSize > m_capacity) return nullptr;

    size_type alignedAddr = currentAddr + padding;
    AllocationHeader* header = reinterpret_cast<AllocationHeader*>(alignedAddr - sizeof(AllocationHeader));
    header->size = size;
    header->padding = padding;
    m_offset += totalSize;
    return reinterpret_cast<void*>(alignedAddr);
}

void* StackAllocator::Reallocate(void* ptr, size_type oldSize, size_type newSize, size_type alignment) {
    void* newPtr = Allocate(newSize, alignment);
    if (newPtr && ptr) std::memcpy(newPtr, ptr, std::min(oldSize, newSize));
    return newPtr;
}

void StackAllocator::Deallocate(void* ptr, size_type size) {
    if (!ptr) return;
    size_type currentAddr = reinterpret_cast<size_type>(ptr);
    AllocationHeader* header = reinterpret_cast<AllocationHeader*>(currentAddr - sizeof(AllocationHeader));
    size_type headerAddr = currentAddr - sizeof(AllocationHeader) - header->padding;
    size_type totalSize = header->size + header->padding + sizeof(AllocationHeader);
    if (headerAddr + totalSize == reinterpret_cast<size_type>(static_cast<char*>(m_buffer) + m_offset)) {
        m_offset -= totalSize;
    }
}

void StackAllocator::Reset() {
    m_offset = 0;
}

void StackAllocator::FreeToMarker(size_type marker) {
    m_offset = marker;
}

// PoolAllocator
PoolAllocator::PoolAllocator(size_type elementSize, size_type elementCount, size_type alignment, const char* name)
    : m_elementSize(elementSize), m_elementCount(elementCount), m_alignment(alignment),
      m_allocatedCount(0), m_freeCount(elementCount), m_name(name) {
    m_buffer = std::aligned_alloc(alignment, elementSize * elementCount);
    if (!m_buffer) throw std::bad_alloc();
    m_freeList = nullptr;
    for (size_type i = 0; i < elementCount; ++i) {
        FreeNode* node = reinterpret_cast<FreeNode*>(static_cast<char*>(m_buffer) + i * elementSize);
        node->next = m_freeList;
        m_freeList = node;
    }
}

PoolAllocator::~PoolAllocator() {
    std::free(m_buffer);
}

void* PoolAllocator::Allocate(size_type size, size_type alignment) {
    if (size > m_elementSize || !m_freeList) return nullptr;
    FreeNode* node = m_freeList;
    m_freeList = node->next;
    --m_freeCount;
    ++m_allocatedCount;
    return node;
}

void* PoolAllocator::Reallocate(void* ptr, size_type oldSize, size_type newSize, size_type alignment) {
    return nullptr; // Pool allocator doesn't support reallocation
}

void PoolAllocator::Deallocate(void* ptr, size_type size) {
    if (!ptr) return;
    FreeNode* node = static_cast<FreeNode*>(ptr);
    node->next = m_freeList;
    m_freeList = node;
    ++m_freeCount;
    --m_allocatedCount;
}

// ArenaAllocator
ArenaAllocator::ArenaAllocator(size_type blockSize, const char* name)
    : m_head(nullptr), m_blockSize(blockSize), m_totalAllocated(0), m_name(name) {}

ArenaAllocator::~ArenaAllocator() {
    Block* current = m_head;
    while (current) {
        Block* next = current->next;
        std::free(current->memory);
        delete current;
        current = next;
    }
}

ArenaAllocator::Block* ArenaAllocator::AllocateBlock(size_type minSize) {
    size_type size = std::max(m_blockSize, minSize);
    Block* block = new Block();
    block->memory = std::aligned_alloc(MMV2_DEFAULT_ALIGNMENT, size);
    block->size = size;
    block->used = 0;
    block->next = m_head;
    m_head = block;
    return block;
}

void* ArenaAllocator::Allocate(size_type size, size_type alignment) {
    if (!m_head || m_head->used + size > m_head->size) {
        AllocateBlock(size + alignment);
    }
    size_type currentAddr = reinterpret_cast<size_type>(static_cast<char*>(m_head->memory) + m_head->used);
    size_type padding = (alignment - (currentAddr % alignment)) % alignment;
    void* ptr = static_cast<char*>(m_head->memory) + m_head->used + padding;
    m_head->used += size + padding;
    m_totalAllocated += size;
    return ptr;
}

void* ArenaAllocator::Reallocate(void* ptr, size_type oldSize, size_type newSize, size_type alignment) {
    void* newPtr = Allocate(newSize, alignment);
    if (newPtr && ptr) std::memcpy(newPtr, ptr, std::min(oldSize, newSize));
    return newPtr;
}

void ArenaAllocator::Deallocate(void* ptr, size_type size) {
    // Arena allocator doesn't support individual deallocation
}

size_type ArenaAllocator::GetAllocatedSize() const {
    return m_totalAllocated;
}

size_type ArenaAllocator::GetTotalSize() const {
    size_type total = 0;
    Block* current = m_head;
    while (current) {
        total += current->size;
        current = current->next;
    }
    return total;
}

void ArenaAllocator::Reset() {
    Block* current = m_head;
    while (current) {
        current->used = 0;
        current = current->next;
    }
    m_totalAllocated = 0;
}

MMV2_NAMESPACE_END
