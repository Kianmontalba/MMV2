#pragma once
#ifndef MMV2_ANN_H
#define MMV2_ANN_H

#include "Config.h"
#include "Vector.h"
#include "Span.h"

MMV2_NAMESPACE_BEGIN

struct ANNNode {
    int32 pointIndex;
    Vector<int32> neighbors;
    Vector<float32> distances;

    ANNNode() : pointIndex(-1) {}
};

class MMV2_API ANNIndex {
public:
    ANNIndex();
    ~ANNIndex();

    bool Build(Span<const float32> data, int32 pointCount, int32 dimensions,
               int32 m = 16, int32 efConstruction = 200);
    bool Build(const Vector<FeatureVector>& features, int32 m = 16, int32 efConstruction = 200);
    void Clear();

    int32 FindNearest(const float32* query, float32* outDistanceSq = nullptr) const;
    void FindKNearest(const float32* query, int32 k, Vector<int32>& outIndices,
                      Vector<float32>* outDistancesSq = nullptr) const;
    void FindInRadius(const float32* query, float32 radiusSq, Vector<int32>& outIndices) const;

    int32 GetPointCount() const { return m_pointCount; }
    int32 GetDimensions() const { return m_dimensions; }
    bool IsBuilt() const { return m_isBuilt; }

private:
    Vector<ANNNode> m_nodes;
    Vector<float32> m_data;
    int32 m_pointCount;
    int32 m_dimensions;
    int32 m_m;
    int32 m_efConstruction;
    bool m_isBuilt;

    float32 DistanceSq(const float32* a, const float32* b) const;
    void AddEdge(int32 from, int32 to, float32 distSq);
    void SearchKNearest(const float32* query, int32 k, Vector<std::pair<float32, int32>>& results) const;
};

MMV2_NAMESPACE_END

#endif
