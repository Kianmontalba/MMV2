#pragma once
#ifndef MMV2_KDTREE_H
#define MMV2_KDTREE_H

#include "Config.h"
#include "Vector.h"
#include "Span.h"

MMV2_NAMESPACE_BEGIN

struct KDTreeNode {
    int32 leftChild;
    int32 rightChild;
    int32 pointIndex;
    int32 axis;
    float32 splitValue;
    bool isLeaf;

    KDTreeNode() : leftChild(-1), rightChild(-1), pointIndex(-1), axis(0), splitValue(0.0f), isLeaf(false) {}
};

struct KDTreeSearchResult {
    int32 index;
    float32 distanceSq;

    bool operator>(const KDTreeSearchResult& other) const { return distanceSq > other.distanceSq; }
    bool operator<(const KDTreeSearchResult& other) const { return distanceSq < other.distanceSq; }
};

class MMV2_API KDTree {
public:
    KDTree();
    ~KDTree();

    bool Build(Span<const float32> data, int32 pointCount, int32 dimensions, int32 leafSize = 16);
    bool Build(const Vector<FeatureVector>& features, int32 leafSize = 16);
    void Clear();

    int32 FindNearest(const float32* query, float32* outDistanceSq = nullptr) const;
    void FindKNearest(const float32* query, int32 k, Vector<int32>& outIndices,
                      Vector<float32>* outDistancesSq = nullptr) const;
    void FindInRadius(const float32* query, float32 radiusSq, Vector<int32>& outIndices) const;

    int32 GetPointCount() const { return m_pointCount; }
    int32 GetDimensions() const { return m_dimensions; }
    int32 GetNodeCount() const { return static_cast<int32>(m_nodes.Size()); }
    bool IsBuilt() const { return m_isBuilt; }

private:
    Vector<KDTreeNode> m_nodes;
    Vector<int32> m_pointIndices;
    Vector<float32> m_data;
    int32 m_pointCount;
    int32 m_dimensions;
    int32 m_leafSize;
    bool m_isBuilt;

    int32 BuildRecursive(int32* indices, int32 count, int32 depth);
    void SearchNearestRecursive(int32 nodeIndex, const float32* query, int32& bestIndex,
                                float32& bestDistSq) const;
    void SearchKNearestRecursive(int32 nodeIndex, const float32* query, int32 k,
                                   Vector<KDTreeSearchResult>& results) const;
    void SearchRadiusRecursive(int32 nodeIndex, const float32* query, float32 radiusSq,
                                 Vector<int32>& results) const;
    float32 DistanceSq(const float32* a, const float32* b) const;
};

MMV2_NAMESPACE_END

#endif
