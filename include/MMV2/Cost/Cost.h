// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Cost Functions & Scoring System
// ============================================================================
// Provides weighted cost calculation for pose comparison during search.
// Supports custom cost functions, normalization, and multi-objective scoring.
// ============================================================================

#pragma once

#include "MMV2/Core/Config.h"
#include "MMV2/Core/Vector.h"
#include "MMV2/Features/Feature.h"
#include "MMV2/Core/HashMap.h"

MMV2_NAMESPACE_BEGIN

// ============================================================================
// Cost Types
// ============================================================================

enum class CostType : uint32_t
{
    Euclidean = 0,      // Standard L2 distance
    Manhattan,          // L1 distance
    Cosine,             // Cosine similarity cost
    WeightedEuclidean,  // Per-dimension weighted L2
    Mahalanobis,        // Covariance-aware distance
    Custom              // User-defined cost function
};

// ============================================================================
// Cost Weights Configuration
// ============================================================================

struct CostWeights
{
    float poseWeight        = 1.0f;
    float trajectoryWeight  = 1.0f;
    float velocityWeight    = 1.0f;
    float headingWeight     = 1.0f;
    float phaseWeight       = 1.0f;
    float distanceWeight    = 1.0f;
    float curveWeight       = 1.0f;
    float interactionWeight = 1.0f;
    float historyWeight     = 1.0f;
    float continuityWeight  = 1.0f;

    void Normalize();
    float GetTotalWeight() const;
};

// ============================================================================
// Cost Function Base
// ============================================================================

class MMV2_API CostFunction
{
public:
    virtual ~CostFunction() = default;

    virtual CostType GetType() const = 0;
    virtual float Compute(const FeatureVector& query, const FeatureVector& candidate) const = 0;
    virtual float ComputePartial(const FeatureVector& query, const FeatureVector& candidate,
                                  uint32_t startDim, uint32_t endDim) const = 0;

    virtual void SetWeights(const CostWeights& weights) { m_weights = weights; }
    virtual const CostWeights& GetWeights() const { return m_weights; }

    virtual void SetNormalizationFactors(const float* factors, uint32_t count);
    virtual void ComputeNormalizationFactors(const FeatureDatabase& database);

protected:
    CostWeights m_weights;
    Vector<float> m_normalizationFactors;
};

// ============================================================================
// Euclidean Cost
// ============================================================================

class MMV2_API EuclideanCost : public CostFunction
{
public:
    CostType GetType() const override { return CostType::Euclidean; }
    float Compute(const FeatureVector& query, const FeatureVector& candidate) const override;
    float ComputePartial(const FeatureVector& query, const FeatureVector& candidate,
                          uint32_t startDim, uint32_t endDim) const override;
};

// ============================================================================
// Weighted Euclidean Cost
// ============================================================================

class MMV2_API WeightedEuclideanCost : public CostFunction
{
public:
    CostType GetType() const override { return CostType::WeightedEuclidean; }
    float Compute(const FeatureVector& query, const FeatureVector& candidate) const override;
    float ComputePartial(const FeatureVector& query, const FeatureVector& candidate,
                          uint32_t startDim, uint32_t endDim) const override;

    void SetDimensionWeights(const float* weights, uint32_t count);

private:
    Vector<float> m_dimensionWeights;
};

// ============================================================================
// Mahalanobis Cost
// ============================================================================

class MMV2_API MahalanobisCost : public CostFunction
{
public:
    CostType GetType() const override { return CostType::Mahalanobis; }
    float Compute(const FeatureVector& query, const FeatureVector& candidate) const override;
    float ComputePartial(const FeatureVector& query, const FeatureVector& candidate,
                          uint32_t startDim, uint32_t endDim) const override;

    void SetCovarianceMatrix(const float* matrix, uint32_t dims);
    void ComputeCovarianceFromDatabase(const FeatureDatabase& database);

private:
    Vector<float> m_inverseCovariance; // Flattened inverse covariance matrix
    uint32_t m_dimensions = 0;
};

// ============================================================================
// Cosine Cost
// ============================================================================

class MMV2_API CosineCost : public CostFunction
{
public:
    CostType GetType() const override { return CostType::Cosine; }
    float Compute(const FeatureVector& query, const FeatureVector& candidate) const override;
    float ComputePartial(const FeatureVector& query, const FeatureVector& candidate,
                          uint32_t startDim, uint32_t endDim) const override;
};

// ============================================================================
// Multi-Objective Cost
// ============================================================================

struct CostObjective
{
    String name;
    float weight;
    float bias;
    float minValue;
    float maxValue;
    bool enabled;
};

class MMV2_API MultiObjectiveCost : public CostFunction
{
public:
    CostType GetType() const override { return CostType::Custom; }
    float Compute(const FeatureVector& query, const FeatureVector& candidate) const override;
    float ComputePartial(const FeatureVector& query, const FeatureVector& candidate,
                          uint32_t startDim, uint32_t endDim) const override;

    void AddObjective(const CostObjective& objective);
    void RemoveObjective(const String& name);
    void SetObjectiveWeight(const String& name, float weight);

    float GetObjectiveValue(const String& name, const FeatureVector& query, 
                             const FeatureVector& candidate) const;

private:
    Vector<CostObjective> m_objectives;
    HashMap<String, uint32_t> m_objectiveIndices;
};

// ============================================================================
// Cost Factory
// ============================================================================

class MMV2_API CostFactory
{
public:
    static UniquePtr<CostFunction> Create(CostType type);
    static UniquePtr<CostFunction> CreateCustom(const String& name);

    using CustomCostCreator = UniquePtr<CostFunction>(*)();
    static void RegisterCustomCost(const String& name, CustomCostCreator creator);

private:
    static HashMap<String, CustomCostCreator> s_customCreators;
};

// ============================================================================
// Cost Result
// ============================================================================

struct CostResult
{
    float totalCost;
    float poseCost;
    float trajectoryCost;
    float velocityCost;
    float headingCost;
    float phaseCost;
    float distanceCost;
    float curveCost;
    float interactionCost;
    float historyCost;
    float continuityCost;
    uint32_t candidateIndex;
    float confidence;

    CostResult() : totalCost(FLT_MAX), poseCost(0), trajectoryCost(0), velocityCost(0),
                   headingCost(0), phaseCost(0), distanceCost(0), curveCost(0),
                   interactionCost(0), historyCost(0), continuityCost(0),
                   candidateIndex(0), confidence(0) {}
};

// ============================================================================
// Cost Evaluator
// ============================================================================

class MMV2_API CostEvaluator
{
public:
    void SetCostFunction(CostFunction* costFunc) { m_costFunction = costFunc; }
    CostFunction* GetCostFunction() const { return m_costFunction; }

    CostResult Evaluate(const FeatureVector& query, const FeatureVector& candidate, 
                         uint32_t candidateIndex) const;

    Vector<CostResult> EvaluateBatch(const FeatureVector& query, 
                                      const FeatureDatabase& database) const;

    void SetFeatureChannelRanges(const FeatureChannelRange* ranges, uint32_t count);

private:
    CostFunction* m_costFunction = nullptr;
    Vector<FeatureChannelRange> m_channelRanges;
};

MMV2_NAMESPACE_END
