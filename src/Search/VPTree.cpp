// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// VP-Tree Implementation
// ============================================================================

#include "MMV2/Search/VPTree.h"
#include "MMV2/Core/Math.h"
#include "MMV2/Features/Feature.h"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <queue>

MMV2_NAMESPACE_BEGIN

VPTree::VPTree() : m_pointCount(0), m_dimensions(0), m_branchFactor(4), m_isBuilt(false) {}
VPTree::~VPTree() { Clear(); }

bool VPTree::Build(Span<const float32> data, int32 pointCount, int32 dimensions, int32 branchFactor) {
    Clear();
    if (pointCount <= 0 || dimensions <= 0 || data.Size() < static_cast<size_type>(pointCount * dimensions)) return false;
    m_pointCount = pointCount; m_dimensions = dimensions; m_branchFactor = branchFactor;
    m_data.Resize(static_cast<size_type>(pointCount * dimensions));
    std::copy(data.Data(), data.Data() + pointCount * dimensions, m_data.Data());
    m_pointIndices.Resize(pointCount);
    std::iota(m_pointIndices.begin(), m_pointIndices.end(), 0);
    int32 root = BuildRecursive(m_pointIndices.Data(), pointCount);
    if (root < 0) { Clear(); return false; }
    m_isBuilt = true; return true;
}

bool VPTree::Build(const Vector<FeatureVector>& features, int32 branchFactor) {
    if (features.Empty()) return false;
    int32 pc = static_cast<int32>(features.Size()), dims = features[0].Size();
    Vector<float32> flat; flat.Reserve(static_cast<size_type>(pc * dims));
    for (const auto& fv : features) for (int32 d = 0; d < dims; ++d) flat.PushBack(fv[d]);
    return Build(flat.Data(), pc, dims, branchFactor);
}

void VPTree::Clear() {
    m_nodes.Clear(); m_pointIndices.Clear(); m_data.Clear();
    m_pointCount = 0; m_dimensions = 0; m_isBuilt = false;
}

int32 VPTree::BuildRecursive(int32* indices, int32 count) {
    if (count <= 0) return -1;
    int32 nodeIdx = static_cast<int32>(m_nodes.Size());
    m_nodes.Resize(m_nodes.Size() + 1);
    VPTreeNode& node = m_nodes.Back();
    if (count <= m_branchFactor) {
        node.isLeaf = true; node.vantagePoint = indices[0]; node.leftChild = -1; node.rightChild = -1; node.medianDistance = 0.0f;
        return nodeIdx;
    }
    node.vantagePoint = indices[0];
    Vector<float32> distances; distances.Reserve(count - 1);
    for (int32 i = 1; i < count; ++i) {
        distances.PushBack(std::sqrt(DistanceSq(&m_data[static_cast<size_type>(indices[0]) * m_dimensions],
                                                 &m_data[static_cast<size_type>(indices[i]) * m_dimensions])));
    }
    Vector<float32> sorted = distances;
    std::nth_element(sorted.begin(), sorted.begin() + sorted.Size() / 2, sorted.end());
    node.medianDistance = sorted[sorted.Size() / 2];
    node.isLeaf = false;
    int32* inside = indices + 1;
    int32* outside = indices + count - 1;
    int32 insideCount = 0;
    for (int32 i = 1; i < count; ++i) {
        float32 d = std::sqrt(DistanceSq(&m_data[static_cast<size_type>(node.vantagePoint) * m_dimensions],
                                          &m_data[static_cast<size_type>(indices[i]) * m_dimensions]));
        if (d <= node.medianDistance) { std::swap(*inside, indices[i]); ++inside; ++insideCount; }
        else { std::swap(*outside, indices[i]); --outside; }
    }
    node.leftChild = BuildRecursive(indices + 1, insideCount);
    node.rightChild = BuildRecursive(inside, count - 1 - insideCount);
    return nodeIdx;
}

int32 VPTree::FindNearest(const float32* query, float32* outDistanceSq) const {
    if (!m_isBuilt || !query) return -1;
    int32 bestIdx = -1; float32 bestDistSq = std::numeric_limits<float32>::max();
    SearchNearestRecursive(0, query, bestIdx, bestDistSq);
    if (outDistanceSq) *outDistanceSq = bestDistSq;
    return bestIdx;
}

void VPTree::SearchNearestRecursive(int32 nodeIdx, const float32* query, int32& bestIdx, float32& bestDistSq) const {
    if (nodeIdx < 0 || nodeIdx >= static_cast<int32>(m_nodes.Size())) return;
    const VPTreeNode& node = m_nodes[nodeIdx];
    float32 distSq = DistanceSq(query, &m_data[static_cast<size_type>(node.vantagePoint) * m_dimensions]);
    if (distSq < bestDistSq) { bestDistSq = distSq; bestIdx = node.vantagePoint; }
    if (node.isLeaf) return;
    float32 dist = std::sqrt(distSq);
    if (dist < node.medianDistance) {
        SearchNearestRecursive(node.leftChild, query, bestIdx, bestDistSq);
        if (dist + std::sqrt(bestDistSq) >= node.medianDistance) SearchNearestRecursive(node.rightChild, query, bestIdx, bestDistSq);
    } else {
        SearchNearestRecursive(node.rightChild, query, bestIdx, bestDistSq);
        if (dist - std::sqrt(bestDistSq) <= node.medianDistance) SearchNearestRecursive(node.leftChild, query, bestIdx, bestDistSq);
    }
}

void VPTree::FindKNearest(const float32* query, int32 k, Vector<int32>& outIndices, Vector<float32>* outDistancesSq) const {
    outIndices.Clear(); if (outDistancesSq) outDistancesSq->Clear();
    if (!m_isBuilt || !query || k <= 0) return;
    auto cmp = [](const std::pair<float32,int32>& a, const std::pair<float32,int32>& b) { return a.first < b.first; };
    std::priority_queue<std::pair<float32,int32>, Vector<std::pair<float32,int32>>, decltype(cmp)> heap(cmp);
    SearchKNearestRecursive(0, query, k, heap);
    Vector<std::pair<float32,int32>> results; results.Reserve(heap.size());
    while (!heap.empty()) { results.PushBack(heap.top()); heap.pop(); }
    std::sort(results.begin(), results.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    outIndices.Resize(results.Size()); if (outDistancesSq) outDistancesSq->Resize(results.Size());
    for (size_type i = 0; i < results.Size(); ++i) { outIndices[i] = results[i].second; if (outDistancesSq) (*outDistancesSq)[i] = results[i].first; }
}

void VPTree::SearchKNearestRecursive(int32 nodeIdx, const float32* query, int32 k,
    std::priority_queue<std::pair<float32,int32>, Vector<std::pair<float32,int32>>, bool(*)(const std::pair<float32,int32>&, const std::pair<float32,int32>&)>& results) const {
    if (nodeIdx < 0 || nodeIdx >= static_cast<int32>(m_nodes.Size())) return;
    const VPTreeNode& node = m_nodes[nodeIdx];
    float32 distSq = DistanceSq(query, &m_data[static_cast<size_type>(node.vantagePoint) * m_dimensions]);
    if (static_cast<int32>(results.size()) < k) results.push({distSq, node.vantagePoint});
    else if (distSq < results.top().first) { results.pop(); results.push({distSq, node.vantagePoint}); }
    if (node.isLeaf) return;
    float32 dist = std::sqrt(distSq);
    float32 best = results.empty() ? std::numeric_limits<float32>::max() : results.top().first;
    if (dist < node.medianDistance) {
        SearchKNearestRecursive(node.leftChild, query, k, results);
        if (dist + std::sqrt(best) >= node.medianDistance || static_cast<int32>(results.size()) < k)
            SearchKNearestRecursive(node.rightChild, query, k, results);
    } else {
        SearchKNearestRecursive(node.rightChild, query, k, results);
        if (dist - std::sqrt(best) <= node.medianDistance || static_cast<int32>(results.size()) < k)
            SearchKNearestRecursive(node.leftChild, query, k, results);
    }
}

void VPTree::FindInRadius(const float32* query, float32 radiusSq, Vector<int32>& outIndices) const {
    outIndices.Clear(); if (!m_isBuilt || !query || radiusSq <= 0.0f) return;
    SearchRadiusRecursive(0, query, radiusSq, outIndices);
}

void VPTree::SearchRadiusRecursive(int32 nodeIdx, const float32* query, float32 radiusSq, Vector<int32>& results) const {
    if (nodeIdx < 0 || nodeIdx >= static_cast<int32>(m_nodes.Size())) return;
    const VPTreeNode& node = m_nodes[nodeIdx];
    float32 distSq = DistanceSq(query, &m_data[static_cast<size_type>(node.vantagePoint) * m_dimensions]);
    if (distSq <= radiusSq) results.PushBack(node.vantagePoint);
    if (node.isLeaf) return;
    float32 dist = std::sqrt(distSq);
    if (dist < node.medianDistance) {
        SearchRadiusRecursive(node.leftChild, query, radiusSq, results);
        if (dist + std::sqrt(radiusSq) >= node.medianDistance) SearchRadiusRecursive(node.rightChild, query, radiusSq, results);
    } else {
        SearchRadiusRecursive(node.rightChild, query, radiusSq, results);
        if (dist - std::sqrt(radiusSq) <= node.medianDistance) SearchRadiusRecursive(node.leftChild, query, radiusSq, results);
    }
}

float32 VPTree::DistanceSq(const float32* a, const float32* b) const {
    float32 sum = 0.0f;
    for (int32 i = 0; i < m_dimensions; ++i) { float32 d = a[i] - b[i]; sum += d * d; }
    return sum;
}

MMV2_NAMESPACE_END
