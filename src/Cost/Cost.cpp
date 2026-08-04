// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Cost Functions Implementation
// ============================================================================

#include "MMV2/Cost/Cost.h"
#include "MMV2/Core/Math.h"
#include <cmath>

MMV2_NAMESPACE_BEGIN

// ============================================================================
// CostWeights
// ============================================================================

void CostWeights::Normalize()
{
    float total = GetTotalWeight();
    if (total > 0.0f)
    {
        float invTotal = 1.0f / total;
        poseWeight *= invTotal;
        trajectoryWeight *= invTotal;
        velocityWeight *= invTotal;
        headingWeight *= invTotal;
        phaseWeight *= invTotal;
        distanceWeight *= invTotal;
        curveWeight *= invTotal;
        interactionWeight *= invTotal;
        historyWeight *= invTotal;
        continuityWeight *= invTotal;
    }
}

float CostWeights::GetTotalWeight() const
{
    return poseWeight + trajectoryWeight + velocityWeight + headingWeight +
           phaseWeight + distanceWeight + curveWeight + interactionWeight +
           historyWeight + continuityWeight;
}

// ============================================================================
// CostFunction Base
// ============================================================================

void CostFunction::SetNormalizationFactors(const float* factors, uint32_t count)
{
    m_normalizationFactors.Resize(count);
    for (uint32_t i = 0; i < count; ++i)
        m_normalizationFactors[i] = factors[i];
}

void CostFunction::ComputeNormalizationFactors(const FeatureDatabase& database)
{
    // Compute standard deviation per dimension for normalization
    const uint32_t dims = database.GetFeatureDimensions();
    m_normalizationFactors.Resize(dims);

    Vector<float> means(dims, 0.0f);
    Vector<float> variances(dims, 0.0f);

    const uint32_t poseCount = database.GetPoseCount();

    // Compute means
    for (uint32_t p = 0; p < poseCount; ++p)
    {
        const FeatureVector& fv = database.GetFeatureVector(p);
        for (uint32_t d = 0; d < dims; ++d)
            means[d] += fv.data[d];
    }

    const float invCount = 1.0f / static_cast<float>(poseCount);
    for (uint32_t d = 0; d < dims; ++d)
        means[d] *= invCount;

    // Compute variances
    for (uint32_t p = 0; p < poseCount; ++p)
    {
        const FeatureVector& fv = database.GetFeatureVector(p);
        for (uint32_t d = 0; d < dims; ++d)
        {
            float diff = fv.data[d] - means[d];
            variances[d] += diff * diff;
        }
    }

    for (uint32_t d = 0; d < dims; ++d)
    {
        float stddev = std::sqrt(variances[d] * invCount);
        m_normalizationFactors[d] = (stddev > 1e-6f) ? (1.0f / stddev) : 1.0f;
    }
}

// ============================================================================
// EuclideanCost
// ============================================================================

float EuclideanCost::Compute(const FeatureVector& query, const FeatureVector& candidate) const
{
    MMV2_ASSERT(query.dimensions == candidate.dimensions);

    float sum = 0.0f;
    const uint32_t dims = query.dimensions;

    for (uint32_t i = 0; i < dims; ++i)
    {
        float diff = query.data[i] - candidate.data[i];
        float normDiff = diff * m_normalizationFactors[i];
        sum += normDiff * normDiff;
    }

    return std::sqrt(sum);
}

float EuclideanCost::ComputePartial(const FeatureVector& query, const FeatureVector& candidate,
                                     uint32_t startDim, uint32_t endDim) const
{
    MMV2_ASSERT(startDim < endDim);
    MMV2_ASSERT(endDim <= query.dimensions);
    MMV2_ASSERT(query.dimensions == candidate.dimensions);

    float sum = 0.0f;

    for (uint32_t i = startDim; i < endDim; ++i)
    {
        float diff = query.data[i] - candidate.data[i];
        float normDiff = diff * m_normalizationFactors[i];
        sum += normDiff * normDiff;
    }

    return std::sqrt(sum);
}

// ============================================================================
// WeightedEuclideanCost
// ============================================================================

float WeightedEuclideanCost::Compute(const FeatureVector& query, const FeatureVector& candidate) const
{
    MMV2_ASSERT(query.dimensions == candidate.dimensions);
    MMV2_ASSERT(m_dimensionWeights.Size() == query.dimensions);

    float sum = 0.0f;
    const uint32_t dims = query.dimensions;

    for (uint32_t i = 0; i < dims; ++i)
    {
        float diff = query.data[i] - candidate.data[i];
        float normDiff = diff * m_normalizationFactors[i];
        float weighted = normDiff * m_dimensionWeights[i];
        sum += weighted * weighted;
    }

    return std::sqrt(sum);
}

float WeightedEuclideanCost::ComputePartial(const FeatureVector& query, const FeatureVector& candidate,
                                             uint32_t startDim, uint32_t endDim) const
{
    MMV2_ASSERT(startDim < endDim);
    MMV2_ASSERT(endDim <= query.dimensions);

    float sum = 0.0f;

    for (uint32_t i = startDim; i < endDim; ++i)
    {
        float diff = query.data[i] - candidate.data[i];
        float normDiff = diff * m_normalizationFactors[i];
        float weighted = normDiff * m_dimensionWeights[i];
        sum += weighted * weighted;
    }

    return std::sqrt(sum);
}

void WeightedEuclideanCost::SetDimensionWeights(const float* weights, uint32_t count)
{
    m_dimensionWeights.Resize(count);
    for (uint32_t i = 0; i < count; ++i)
        m_dimensionWeights[i] = weights[i];
}

// ============================================================================
// MahalanobisCost
// ============================================================================

float MahalanobisCost::Compute(const FeatureVector& query, const FeatureVector& candidate) const
{
    MMV2_ASSERT(query.dimensions == candidate.dimensions);
    MMV2_ASSERT(m_dimensions == query.dimensions);

    const uint32_t dims = query.dimensions;
    Vector<float> diff(dims);

    for (uint32_t i = 0; i < dims; ++i)
        diff[i] = (query.data[i] - candidate.data[i]) * m_normalizationFactors[i];

    float result = 0.0f;
    for (uint32_t i = 0; i < dims; ++i)
    {
        float rowSum = 0.0f;
        for (uint32_t j = 0; j < dims; ++j)
            rowSum += diff[j] * m_inverseCovariance[i * dims + j];
        result += diff[i] * rowSum;
    }

    return std::sqrt(result);
}

float MahalanobisCost::ComputePartial(const FeatureVector& query, const FeatureVector& candidate,
                                       uint32_t startDim, uint32_t endDim) const
{
    // For partial computation, fall back to weighted Euclidean on the subspace
    float sum = 0.0f;

    for (uint32_t i = startDim; i < endDim; ++i)
    {
        float diff = query.data[i] - candidate.data[i];
        float normDiff = diff * m_normalizationFactors[i];
        sum += normDiff * normDiff;
    }

    return std::sqrt(sum);
}

void MahalanobisCost::SetCovarianceMatrix(const float* matrix, uint32_t dims)
{
    m_dimensions = dims;
    m_inverseCovariance.Resize(dims * dims);

    // Copy and compute inverse (simplified - in production use proper matrix inversion)
    for (uint32_t i = 0; i < dims * dims; ++i)
        m_inverseCovariance[i] = matrix[i];
}

void MahalanobisCost::ComputeCovarianceFromDatabase(const FeatureDatabase& database)
{
    const uint32_t dims = database.GetFeatureDimensions();
    const uint32_t poseCount = database.GetPoseCount();

    m_dimensions = dims;

    // Compute covariance matrix
    Vector<float> means(dims, 0.0f);
    Vector<float> covariance(dims * dims, 0.0f);

    for (uint32_t p = 0; p < poseCount; ++p)
    {
        const FeatureVector& fv = database.GetFeatureVector(p);
        for (uint32_t d = 0; d < dims; ++d)
            means[d] += fv.data[d];
    }

    const float invCount = 1.0f / static_cast<float>(poseCount);
    for (uint32_t d = 0; d < dims; ++d)
        means[d] *= invCount;

    for (uint32_t p = 0; p < poseCount; ++p)
    {
        const FeatureVector& fv = database.GetFeatureVector(p);
        for (uint32_t i = 0; i < dims; ++i)
        {
            for (uint32_t j = 0; j < dims; ++j)
            {
                float diffI = fv.data[i] - means[i];
                float diffJ = fv.data[j] - means[j];
                covariance[i * dims + j] += diffI * diffJ;
            }
        }
    }

    for (uint32_t i = 0; i < dims * dims; ++i)
        covariance[i] *= invCount;

    // Simplified pseudo-inverse (add regularization for numerical stability)
    m_inverseCovariance.Resize(dims * dims);
    const float regularization = 1e-4f;

    for (uint32_t i = 0; i < dims; ++i)
    {
        for (uint32_t j = 0; j < dims; ++j)
        {
            if (i == j)
                m_inverseCovariance[i * dims + j] = 1.0f / (covariance[i * dims + j] + regularization);
            else
                m_inverseCovariance[i * dims + j] = 0.0f; // Diagonal approximation for performance
        }
    }
}

// ============================================================================
// CosineCost
// ============================================================================

float CosineCost::Compute(const FeatureVector& query, const FeatureVector& candidate) const
{
    MMV2_ASSERT(query.dimensions == candidate.dimensions);

    float dot = 0.0f;
    float queryNorm = 0.0f;
    float candidateNorm = 0.0f;

    const uint32_t dims = query.dimensions;

    for (uint32_t i = 0; i < dims; ++i)
    {
        float q = query.data[i] * m_normalizationFactors[i];
        float c = candidate.data[i] * m_normalizationFactors[i];
        dot += q * c;
        queryNorm += q * q;
        candidateNorm += c * c;
    }

    queryNorm = std::sqrt(queryNorm);
    candidateNorm = std::sqrt(candidateNorm);

    if (queryNorm < 1e-6f || candidateNorm < 1e-6f)
        return 1.0f;

    float cosine = dot / (queryNorm * candidateNorm);
    cosine = Math::Clamp(cosine, -1.0f, 1.0f);

    // Convert cosine similarity to cost (0 = identical, 2 = opposite)
    return 1.0f - cosine;
}

float CosineCost::ComputePartial(const FeatureVector& query, const FeatureVector& candidate,
                                  uint32_t startDim, uint32_t endDim) const
{
    float dot = 0.0f;
    float queryNorm = 0.0f;
    float candidateNorm = 0.0f;

    for (uint32_t i = startDim; i < endDim; ++i)
    {
        float q = query.data[i] * m_normalizationFactors[i];
        float c = candidate.data[i] * m_normalizationFactors[i];
        dot += q * c;
        queryNorm += q * q;
        candidateNorm += c * c;
    }

    queryNorm = std::sqrt(queryNorm);
    candidateNorm = std::sqrt(candidateNorm);

    if (queryNorm < 1e-6f || candidateNorm < 1e-6f)
        return 1.0f;

    float cosine = dot / (queryNorm * candidateNorm);
    cosine = Math::Clamp(cosine, -1.0f, 1.0f);

    return 1.0f - cosine;
}

// ============================================================================
// MultiObjectiveCost
// ============================================================================

float MultiObjectiveCost::Compute(const FeatureVector& query, const FeatureVector& candidate) const
{
    float totalCost = 0.0f;
    float totalWeight = 0.0f;

    for (const auto& obj : m_objectives)
    {
        if (!obj.enabled) continue;

        float value = GetObjectiveValue(obj.name, query, candidate);
        value = Math::Clamp((value - obj.minValue) / (obj.maxValue - obj.minValue + 1e-6f), 0.0f, 1.0f);
        totalCost += obj.weight * (value + obj.bias);
        totalWeight += obj.weight;
    }

    return (totalWeight > 0.0f) ? (totalCost / totalWeight) : FLT_MAX;
}

float MultiObjectiveCost::ComputePartial(const FeatureVector& query, const FeatureVector& candidate,
                                          uint32_t startDim, uint32_t endDim) const
{
    // For partial evaluation, use a simplified single-objective cost
    float sum = 0.0f;
    for (uint32_t i = startDim; i < endDim; ++i)
    {
        float diff = query.data[i] - candidate.data[i];
        sum += diff * diff;
    }
    return std::sqrt(sum);
}

void MultiObjectiveCost::AddObjective(const CostObjective& objective)
{
    uint32_t index = static_cast<uint32_t>(m_objectives.Size());
    m_objectives.PushBack(objective);
    m_objectiveIndices[objective.name] = index;
}

void MultiObjectiveCost::RemoveObjective(const String& name)
{
    auto it = m_objectiveIndices.Find(name);
    if (it != m_objectiveIndices.End())
    {
        uint32_t index = it->second;
        m_objectives.Erase(index);
        m_objectiveIndices.Erase(it);

        // Rebuild indices
        for (uint32_t i = 0; i < m_objectives.Size(); ++i)
            m_objectiveIndices[m_objectives[i].name] = i;
    }
}

void MultiObjectiveCost::SetObjectiveWeight(const String& name, float weight)
{
    auto it = m_objectiveIndices.Find(name);
    if (it != m_objectiveIndices.End())
        m_objectives[it->second].weight = weight;
}

float MultiObjectiveCost::GetObjectiveValue(const String& name, const FeatureVector& query,
                                             const FeatureVector& candidate) const
{
    auto it = m_objectiveIndices.Find(name);
    if (it == m_objectiveIndices.End())
        return 0.0f;

    // Default implementation: use Euclidean distance on full vector
    // Subclasses can override for specific objective calculations
    float sum = 0.0f;
    for (uint32_t i = 0; i < query.dimensions; ++i)
    {
        float diff = query.data[i] - candidate.data[i];
        sum += diff * diff;
    }
    return std::sqrt(sum);
}

// ============================================================================
// CostFactory
// ============================================================================

HashMap<String, CostFactory::CustomCostCreator> CostFactory::s_customCreators;

UniquePtr<CostFunction> CostFactory::Create(CostType type)
{
    switch (type)
    {
        case CostType::Euclidean:           return MakeUnique<EuclideanCost>();
        case CostType::Manhattan:           return MakeUnique<EuclideanCost>(); // TODO: Implement Manhattan
        case CostType::Cosine:              return MakeUnique<CosineCost>();
        case CostType::WeightedEuclidean:   return MakeUnique<WeightedEuclideanCost>();
        case CostType::Mahalanobis:         return MakeUnique<MahalanobisCost>();
        case CostType::Custom:              return MakeUnique<MultiObjectiveCost>();
        default:                            return MakeUnique<EuclideanCost>();
    }
}

UniquePtr<CostFunction> CostFactory::CreateCustom(const String& name)
{
    auto it = s_customCreators.Find(name);
    if (it != s_customCreators.End())
        return it->second();
    return nullptr;
}

void CostFactory::RegisterCustomCost(const String& name, CustomCostCreator creator)
{
    s_customCreators[name] = creator;
}

// ============================================================================
// CostEvaluator
// ============================================================================

CostResult CostEvaluator::Evaluate(const FeatureVector& query, const FeatureVector& candidate,
                                    uint32_t candidateIndex) const
{
    CostResult result;
    result.candidateIndex = candidateIndex;

    if (!m_costFunction)
    {
        result.totalCost = FLT_MAX;
        return result;
    }

    // Compute per-channel costs if ranges are available
    if (!m_channelRanges.Empty())
    {
        for (const auto& range : m_channelRanges)
        {
            float channelCost = m_costFunction->ComputePartial(query, candidate, range.start, range.end);

            switch (range.channelType)
            {
                case FeatureChannelType::Pose:        result.poseCost = channelCost; break;
                case FeatureChannelType::Trajectory:  result.trajectoryCost = channelCost; break;
                case FeatureChannelType::Velocity:    result.velocityCost = channelCost; break;
                case FeatureChannelType::Heading:     result.headingCost = channelCost; break;
                case FeatureChannelType::Phase:       result.phaseCost = channelCost; break;
                case FeatureChannelType::Distance:    result.distanceCost = channelCost; break;
                case FeatureChannelType::Curve:       result.curveCost = channelCost; break;
                case FeatureChannelType::Interaction: result.interactionCost = channelCost; break;
                case FeatureChannelType::History:     result.historyCost = channelCost; break;
                default: break;
            }
        }
    }

    result.totalCost = m_costFunction->Compute(query, candidate);

    // Compute confidence (inverse of normalized cost)
    const float maxExpectedCost = 100.0f; // Configurable
    result.confidence = Math::Clamp(1.0f - (result.totalCost / maxExpectedCost), 0.0f, 1.0f);

    return result;
}

Vector<CostResult> CostEvaluator::EvaluateBatch(const FeatureVector& query,
                                                 const FeatureDatabase& database) const
{
    const uint32_t poseCount = database.GetPoseCount();
    Vector<CostResult> results;
    results.Reserve(poseCount);

    for (uint32_t i = 0; i < poseCount; ++i)
    {
        results.PushBack(Evaluate(query, database.GetFeatureVector(i), i));
    }

    return results;
}

void CostEvaluator::SetFeatureChannelRanges(const FeatureChannelRange* ranges, uint32_t count)
{
    m_channelRanges.Resize(count);
    for (uint32_t i = 0; i < count; ++i)
        m_channelRanges[i] = ranges[i];
}

MMV2_NAMESPACE_END
