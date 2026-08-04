// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// KD-Tree Implementation
// ============================================================================

#include "MMV2/Search/KDTree.h"
#include "MMV2/Core/Math.h"
#include "MMV2/Features/Feature.h"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <queue>

MMV2_NAMESPACE_BEGIN

KDTree::KDTree() : m_pointCount(0), m_dimensions(0), m_leafSize(16), m_isBuilt(false) {}
KDTree::~KDTree() { Clear(); }

bool KDTree::Build(Span<const float32> data, int32 pointCount, int32 dimensions, int32 leafSize) {
    Clear();
    if (pointCount <= 0 || dimensions <= 0 || data.Size() < static_cast<size_type>(pointCount * dimensions)) return false;
    m_pointCount = pointCount; m_dimensions = dimensions; m_leafSize = leafSize;
    m_data.Resize(static_cast<size_type>(pointCount * dimensions));
    std::copy(data.Data(), data.Data() + pointCount * dimensions, m_data.Data());
    m_pointIndices.Resize(pointCount);
    std::iota(m_pointIndices.begin(), m_pointIndices.end(), 0);
    int32 root = BuildRecursive(m_pointIndices.Data(), pointCount, 0);
    if (root < 0) { Clear(); return false; }
    m_isBuilt = true; return true;
}

bool KDTree::Build(const Vector<FeatureVector>& features, int32 leafSize) {
    if (features.Empty()) return false;
    int32 pc = static_cast<int32>(features.Size()), dims = features[0].Size();
    Vector<float32> flat; flat.Reserve(static_cast<size_type>(pc * dims));
    for (const auto& fv : features) for (int32 d = 0; d < dims; ++d) flat.PushBack(fv[d]);
    return Build(flat.Data(), pc, dims, leafSize);
}

void KDTree::Clear() {
    m_nodes.Clear(); m_pointIndices.Clear(); m_data.Clear();
    m_pointCount = 0; m_dimensions = 0; m_isBuilt = false;
}

int32 KDTree::BuildRecursive(int32* indices, int32 count, int32 depth) {
    if (count <= 0) return -1;
    int32 nodeIdx = static_cast<int32>(m_nodes.Size());
    m_nodes.Resize(m_nodes.Size() + 1);
    KDTreeNode& node = m_nodes.Back();
    node.axis = depth % m_dimensions;
    if (count <= m_leafSize) {
        node.isLeaf = true; node.pointIndex = indices[0]; node.leftChild = -1; node.rightChild = -1; node.splitValue = 0.0f;
        return nodeIdx;
    }
    int32 mid = count / 2;
    std::nth_element(indices, indices + mid, indices + count,
        [this, depth](int32 a, int32 b) {
            return m_data[static_cast<size_type>(a) * m_dimensions + (depth % m_dimensions)] <
                   m_data[static_cast<size_type>(b) * m_dimensions + (depth % m_dimensions)];
        });
    node.pointIndex = indices[mid];
    node.splitValue = m_data[static_cast<size_type>(indices[mid]) * m_dimensions + node.axis];
    node.isLeaf = false;
    node.leftChild = BuildRecursive(indices, mid, depth + 1);
    node.rightChild = BuildRecursive(indices + mid + 1, count - mid - 1, depth + 1);
    return nodeIdx;
}

int32 KDTree::FindNearest(const float32* query, float32* outDistanceSq) const {
    if (!m_isBuilt || !query) return -1;
    int32 bestIdx = -1; float32 bestDistSq = std::numeric_limits<float32>::max();
    SearchNearestRecursive(0, query, bestIdx, bestDistSq);
    if (outDistanceSq) *outDistanceSq = bestDistSq;
    return bestIdx;
}

void KDTree::SearchNearestRecursive(int32 nodeIdx, const float32* query, int32& bestIdx, float32& bestDistSq) const {
    if (nodeIdx < 0 || nodeIdx >= static_cast<int32>(m_nodes.Size())) return;
    const KDTreeNode& node = m_nodes[nodeIdx];
    if (node.isLeaf) {
        float32 distSq = DistanceSq(query, &m_data[static_cast<size_type>(node.pointIndex) * m_dimensions]);
        if (distSq < bestDistSq) { bestDistSq = distSq; bestIdx = node.pointIndex; }
        return;
    }
    float32 diff = query[node.axis] - node.splitValue;
    int32 first = diff < 0.0f ? node.leftChild : node.rightChild;
    int32 second = diff < 0.0f ? node.rightChild : node.leftChild;
    SearchNearestRecursive(first, query, bestIdx, bestDistSq);
    if (diff * diff < bestDistSq) SearchNearestRecursive(second, query, bestIdx, bestDistSq);
}

void KDTree::FindKNearest(const float32* query, int32 k, Vector<int32>& outIndices, Vector<float32>* outDistancesSq) const {
    outIndices.Clear(); if (outDistancesSq) outDistancesSq->Clear();
    if (!m_isBuilt || !query || k <= 0) return;
    auto cmp = [](const KDTreeSearchResult& a, const KDTreeSearchResult& b) { return a.distanceSq < b.distanceSq; };
    std::priority_queue<KDTreeSearchResult, Vector<KDTreeSearchResult>, decltype(cmp)> heap(cmp);
    SearchKNearestRecursive(0, query, k, heap);
    Vector<KDTreeSearchResult> results; results.Reserve(heap.size());
    while (!heap.empty()) { results.PushBack(heap.top()); heap.pop(); }
    std::sort(results.begin(), results.end(), [](const KDTreeSearchResult& a, const KDTreeSearchResult& b) { return a.distanceSq < b.distanceSq; });
    outIndices.Resize(results.Size()); if (outDistancesSq) outDistancesSq->Resize(results.Size());
    for (size_type i = 0; i < results.Size(); ++i) { outIndices[i] = results[i].index; if (outDistancesSq) (*outDistancesSq)[i] = results[i].distanceSq; }
}

void KDTree::SearchKNearestRecursive(int32 nodeIdx, const float32* query, int32 k,
    std::priority_queue<KDTreeSearchResult, Vector<KDTreeSearchResult>, bool(*)(const KDTreeSearchResult&, const KDTreeSearchResult&)>& results) const {
    if (nodeIdx < 0 || nodeIdx >= static_cast<int32>(m_nodes.Size())) return;
    const KDTreeNode& node = m_nodes[nodeIdx];
    if (node.isLeaf) {
        float32 distSq = DistanceSq(query, &m_data[static_cast<size_type>(node.pointIndex) * m_dimensions]);
        KDTreeSearchResult r; r.index = node.pointIndex; r.distanceSq = distSq;
        if (static_cast<int32>(results.size()) < k) results.push(r);
        else if (distSq < results.top().distanceSq) { results.pop(); results.push(r); }
        return;
    }
    float32 diff = query[node.axis] - node.splitValue;
    int32 first = diff < 0.0f ? node.leftChild : node.rightChild;
    int32 second = diff < 0.0f ? node.rightChild : node.leftChild;
    SearchKNearestRecursive(first, query, k, results);
    float32 best = results.empty() ? std::numeric_limits<float32>::max() : results.top().distanceSq;
    if (diff * diff < best || static_cast<int32>(results.size()) < k) SearchKNearestRecursive(second, query, k, results);
}

void KDTree::FindInRadius(const float32* query, float32 radiusSq, Vector<int32>& outIndices) const {
    outIndices.Clear(); if (!m_isBuilt || !query || radiusSq <= 0.0f) return;
    SearchRadiusRecursive(0, query, radiusSq, outIndices);
}

void KDTree::SearchRadiusRecursive(int32 nodeIdx, const float32* query, float32 radiusSq, Vector<int32>& results) const {
    if (nodeIdx < 0 || nodeIdx >= static_cast<int32>(m_nodes.Size())) return;
    const KDTreeNode& node = m_nodes[nodeIdx];
    if (node.isLeaf) {
        float32 distSq = DistanceSq(query, &m_data[static_cast<size_type>(node.pointIndex) * m_dimensions]);
        if (distSq <= radiusSq) results.PushBack(node.pointIndex);
        return;
    }
    float32 diff = query[node.axis] - node.splitValue;
    int32 first = diff < 0.0f ? node.leftChild : node.rightChild;
    int32 second = diff < 0.0f ? node.rightChild : node.leftChild;
    SearchRadiusRecursive(first, query, radiusSq, results);
    if (diff * diff <= radiusSq) SearchRadiusRecursive(second, query, radiusSq, results);
}

float32 KDTree::DistanceSq(const float32* a, const float32* b) const {
    float32 sum = 0.0f;
    #if MMV2_SIMD_AVX512
    __m512 s = _mm512_setzero_ps(); int32 i = 0;
    for (; i <= m_dimensions - 16; i += 16) { __m512 va = _mm512_loadu_ps(a+i); __m512 vb = _mm512_loadu_ps(b+i); __m512 d = _mm512_sub_ps(va,vb); s = _mm512_fmadd_ps(d,d,s); }
    sum = _mm512_reduce_add_ps(s); for (; i < m_dimensions; ++i) { float32 d = a[i]-b[i]; sum += d*d; }
    #elif MMV2_SIMD_AVX
    __m256 s = _mm256_setzero_ps(); int32 i = 0;
    for (; i <= m_dimensions - 8; i += 8) { __m256 va = _mm256_loadu_ps(a+i); __m256 vb = _mm256_loadu_ps(b+i); __m256 d = _mm256_sub_ps(va,vb); s = _mm256_fmadd_ps(d,d,s); }
    float32 t[8]; _mm256_storeu_ps(t,s); sum = t[0]+t[1]+t[2]+t[3]+t[4]+t[5]+t[6]+t[7]; for (; i < m_dimensions; ++i) { float32 d = a[i]-b[i]; sum += d*d; }
    #elif MMV2_SIMD_SSE42
    __m128 s = _mm_setzero_ps(); int32 i = 0;
    for (; i <= m_dimensions - 4; i += 4) { __m128 va = _mm_loadu_ps(a+i); __m128 vb = _mm_loadu_ps(b+i); __m128 d = _mm_sub_ps(va,vb); s = _mm_add_ps(s, _mm_mul_ps(d,d)); }
    float32 t[4]; _mm_storeu_ps(t,s); sum = t[0]+t[1]+t[2]+t[3]; for (; i < m_dimensions; ++i) { float32 d = a[i]-b[i]; sum += d*d; }
    #else
    for (int32 i = 0; i < m_dimensions; ++i) { float32 d = a[i]-b[i]; sum += d*d; }
    #endif
    return sum;
}

MMV2_NAMESPACE_END
