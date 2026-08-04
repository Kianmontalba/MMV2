// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// ANN (Approximate Nearest Neighbor) Implementation
// ============================================================================

#include "MMV2/Search/ANN.h"
#include "MMV2/Core/Math.h"
#include "MMV2/Features/Feature.h"
#include <cmath>
#include <algorithm>
#include <random>
#include <queue>
#include <set>

MMV2_NAMESPACE_BEGIN

ANNIndex::ANNIndex() : m_pointCount(0), m_dimensions(0), m_isBuilt(false), m_m(16), m_efConstruction(200), m_maxLevel(0) {}
ANNIndex::~ANNIndex() { Clear(); }

bool ANNIndex::Build(Span<const float32> data, int32 pointCount, int32 dimensions, int32 m, int32 efConstruction) {
    Clear();
    if (pointCount <= 0 || dimensions <= 0) return false;
    m_pointCount = pointCount; m_dimensions = dimensions; m_m = m; m_efConstruction = efConstruction;
    m_data.Resize(static_cast<size_type>(pointCount * dimensions));
    std::copy(data.Data(), data.Data() + pointCount * dimensions, m_data.Data());
    m_nodes.Resize(pointCount);
    for (int32 i = 0; i < pointCount; ++i) m_nodes[i].pointIndex = i;
    std::mt19937 rng(42);
    std::uniform_real_distribution<float32> dist(0.0f, 1.0f);
    float32 levelMult = 1.0f / std::log(static_cast<float32>(m));
    for (int32 i = 0; i < pointCount; ++i) {
        float32 r = dist(rng);
        int32 level = static_cast<int32>(-std::log(r) * levelMult);
        m_nodes[i].level = level;
        m_maxLevel = std::max(m_maxLevel, level);
    }
    for (int32 level = m_maxLevel; level >= 0; --level) {
        for (int32 i = 0; i < pointCount; ++i) {
            if (m_nodes[i].level < level) continue;
            Vector<int32> neighbors = SearchLayer(i, level, m);
            for (int32 n : neighbors) {
                if (n != i) {
                    m_nodes[i].neighbors.PushBack(n);
                    m_nodes[n].neighbors.PushBack(i);
                }
            }
        }
    }
    m_isBuilt = true; return true;
}

bool ANNIndex::Build(const Vector<FeatureVector>& features, int32 m, int32 efConstruction) {
    if (features.Empty()) return false;
    int32 pc = static_cast<int32>(features.Size()), dims = features[0].Size();
    Vector<float32> flat; flat.Reserve(static_cast<size_type>(pc * dims));
    for (const auto& fv : features) for (int32 d = 0; d < dims; ++d) flat.PushBack(fv[d]);
    return Build(flat.Data(), pc, dims, m, efConstruction);
}

void ANNIndex::Clear() {
    m_nodes.Clear(); m_data.Clear();
    m_pointCount = 0; m_dimensions = 0; m_isBuilt = false; m_maxLevel = 0;
}

Vector<int32> ANNIndex::SearchLayer(int32 pointIdx, int32 level, int32 ef) const {
    Vector<int32> result;
    if (pointIdx < 0 || pointIdx >= m_pointCount) return result;
    std::set<int32> visited;
    std::priority_queue<std::pair<float32,int32>> candidates;
    const float32* query = &m_data[static_cast<size_type>(pointIdx) * m_dimensions];
    candidates.push({0.0f, pointIdx});
    visited.insert(pointIdx);
    while (!candidates.empty() && static_cast<int32>(result.Size()) < ef) {
        auto current = candidates.top(); candidates.pop();
        int32 idx = current.second;
        result.PushBack(idx);
        for (int32 neighbor : m_nodes[idx].neighbors) {
            if (visited.count(neighbor)) continue;
            if (m_nodes[neighbor].level < level) continue;
            visited.insert(neighbor);
            float32 d = DistanceSq(query, &m_data[static_cast<size_type>(neighbor) * m_dimensions]);
            candidates.push({-d, neighbor});
        }
    }
    return result;
}

int32 ANNIndex::FindNearest(const float32* query, float32* outDistanceSq) const {
    if (!m_isBuilt || !query) return -1;
    int32 bestIdx = -1; float32 bestDistSq = std::numeric_limits<float32>::max();
    for (int32 i = 0; i < m_pointCount; ++i) {
        float32 distSq = DistanceSq(query, &m_data[static_cast<size_type>(i) * m_dimensions]);
        if (distSq < bestDistSq) { bestDistSq = distSq; bestIdx = i; }
    }
    if (outDistanceSq) *outDistanceSq = bestDistSq;
    return bestIdx;
}

void ANNIndex::FindKNearest(const float32* query, int32 k, Vector<int32>& outIndices, Vector<float32>* outDistancesSq) const {
    outIndices.Clear(); if (outDistancesSq) outDistancesSq->Clear();
    if (!m_isBuilt || !query || k <= 0) return;
    Vector<std::pair<float32,int32>> results;
    results.Reserve(m_pointCount);
    for (int32 i = 0; i < m_pointCount; ++i) {
        float32 distSq = DistanceSq(query, &m_data[static_cast<size_type>(i) * m_dimensions]);
        results.PushBack({distSq, i});
    }
    std::partial_sort(results.begin(), results.begin() + std::min(static_cast<size_type>(k), results.Size()), results.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; });
    if (results.Size() > static_cast<size_type>(k)) results.Resize(k);
    outIndices.Resize(results.Size()); if (outDistancesSq) outDistancesSq->Resize(results.Size());
    for (size_type i = 0; i < results.Size(); ++i) { outIndices[i] = results[i].second; if (outDistancesSq) (*outDistancesSq)[i] = results[i].first; }
}

void ANNIndex::FindInRadius(const float32* query, float32 radiusSq, Vector<int32>& outIndices) const {
    outIndices.Clear(); if (!m_isBuilt || !query || radiusSq <= 0.0f) return;
    for (int32 i = 0; i < m_pointCount; ++i) {
        float32 distSq = DistanceSq(query, &m_data[static_cast<size_type>(i) * m_dimensions]);
        if (distSq <= radiusSq) outIndices.PushBack(i);
    }
}

float32 ANNIndex::DistanceSq(const float32* a, const float32* b) const {
    float32 sum = 0.0f;
    for (int32 i = 0; i < m_dimensions; ++i) { float32 d = a[i] - b[i]; sum += d * d; }
    return sum;
}

MMV2_NAMESPACE_END
