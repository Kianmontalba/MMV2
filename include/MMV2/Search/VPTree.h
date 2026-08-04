#pragma once
#ifndef MMV2_VPTREE_H
#define MMV2_VPTREE_H

#include "Config.h"
#include "Vector.h"
#include "Span.h"

MMV2_NAMESPACE_BEGIN

struct VPTreeNode {
    int32 vantagePoint;
    float32 medianDistance;
    int32 leftChild;
    int32 rightChild;
    bool isLeaf;

    VPTreeNode() : vantagePoint(-1), medianDistance(0.0f), leftChild(-1), rightChild(-1), isLeaf(false) {}
};

class MMV2_API VPTree {
public:
    VPTree();
    ~VPTree();

    bool Build(Span<const float32> data, int32 pointCount, int32 dimensions, int32 branchFactor = 4);
    bool Build(const Vector<FeatureVector>& features, int32 branchFactor = 4);
    void Clear();

    int32 FindNearest(const float32* query, float32* outDistanceSq = nullptr) const;
    void FindKNearest(const float32* query, int32 k, Vector<int32>& outIndices,
                      Vector<float32>* outDistancesSq = nullptr) const;
    void FindInRadius(const float32* query, float32 radiusSq, Vector<int32>& outIndices) const;

    int32 GetPointCount() const { return m_pointCount; }
    int32 GetDimensions() const { return m_dimensions; }
    bool IsBuilt() const { return m_isBuilt; }

private:
    Vector<VPTreeNode> m_nodes;
    Vector<int32> m_pointIndices;
    Vector<float32> m_data;
    int32 m_pointCount;
    int32 m_dimensions;
    int32 m_branchFactor;
    bool m_isBuilt;

    int32 BuildRecursive(int32* indices, int32 count);
    float32 DistanceSq(const float32* a, const float32* b) const;
    void SearchNearestRecursive(int32 nodeIndex, const float32* query, int32& bestIndex,
                                float32& bestDistSq) const;
    void SearchKNearestRecursive(int32 nodeIndex, const float32* query, int32 k,
                                 Vector<std::pair<float32, int32>>& results) const;
    void SearchRadiusRecursive(int32 nodeIndex, const float32* query, float32 radiusSq,
                               Vector<int32>& results) const;
};

MMV2_NAMESPACE_END

#endif
