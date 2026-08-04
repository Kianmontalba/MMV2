// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Compression System
// ============================================================================
// Provides pose data compression, quantization, and decompression for
// memory-efficient storage and fast loading. Supports multiple compression
// algorithms with configurable quality/performance tradeoffs.
// ============================================================================

#pragma once

#include "MMV2/Core/Config.h"
#include "MMV2/Core/Vector.h"
#include "MMV2/Core/Pose.h"
#include "MMV2/Features/Feature.h"

MMV2_NAMESPACE_BEGIN

// ============================================================================
// Compression Types
// ============================================================================

enum class CompressionAlgorithm : uint32_t
{
    None = 0,           // No compression (raw floats)
    Quantize,           // Uniform quantization
    AdaptiveQuantize,   // Per-dimension adaptive quantization
    PCA,                // Principal Component Analysis
    Wavelet,            // Wavelet transform compression
    CurveFit,           // Curve fitting (keyframe reduction)
    Delta,              // Delta encoding
    RLE,                // Run-length encoding
    LZ4,                // LZ4 fast compression
    Zstd,               // Zstd compression
    Custom              // User-defined algorithm
};

// ============================================================================
// Compression Settings
// ============================================================================

struct CompressionSettings
{
    CompressionAlgorithm algorithm = CompressionAlgorithm::AdaptiveQuantize;
    float positionTolerance = 0.001f;       // meters
    float rotationTolerance = 0.01f;        // radians
    float scaleTolerance = 0.001f;
    float velocityTolerance = 0.01f;
    float featureTolerance = 0.001f;
    uint32_t maxKeyframeReduction = 4;      // Keep every Nth frame max
    bool preserveRootMotion = true;
    bool preserveEndEffectors = true;
    bool useTemporalCoherence = true;
    float temporalCoherenceThreshold = 0.95f;

    // PCA-specific
    uint32_t pcaComponents = 16;
    float pcaVarianceThreshold = 0.99f;

    // Quantization bits
    uint32_t positionBits = 16;
    uint32_t rotationBits = 16;
    uint32_t scaleBits = 8;
    uint32_t velocityBits = 16;
};

// ============================================================================
// Compression Statistics
// ============================================================================

struct CompressionStats
{
    uint64_t originalSize;
    uint64_t compressedSize;
    float compressionRatio;
    float maxPositionError;
    float maxRotationError;
    float rmsPositionError;
    float rmsRotationError;
    float compressionTimeMs;
    float decompressionTimeMs;
    uint32_t keyframeCount;
    uint32_t originalFrameCount;

    CompressionStats() : originalSize(0), compressedSize(0), compressionRatio(0),
                         maxPositionError(0), maxRotationError(0),
                         rmsPositionError(0), rmsRotationError(0),
                         compressionTimeMs(0), decompressionTimeMs(0),
                         keyframeCount(0), originalFrameCount(0) {}
};

// ============================================================================
// Quantized Transform
// ============================================================================

struct QuantizedTransform
{
    int16_t posX, posY, posZ;
    int16_t rotX, rotY, rotZ, rotW;
    int8_t scaleX, scaleY, scaleZ;

    static constexpr float POSITION_SCALE = 1000.0f;  // 1mm precision
    static constexpr float ROTATION_SCALE = 32767.0f; // ~0.0001 rad precision
    static constexpr float SCALE_SCALE = 127.0f;
};

// ============================================================================
// Compressed Pose Data
// ============================================================================

struct CompressedPoseData
{
    uint32_t frameIndex;
    Vector<QuantizedTransform> boneTransforms;
    Vector<int16_t> boneVelocities; // Optional
    bool isKeyframe;
    float timestamp;
};

// ============================================================================
// Compressed Feature Vector
// ============================================================================

struct CompressedFeatureVector
{
    uint32_t dimensions;
    Vector<uint8_t> quantizedData;  // Per-dimension quantized values
    Vector<float> scales;           // Per-dimension scale factors
    Vector<float> offsets;          // Per-dimension offsets
};

// ============================================================================
// Keyframe Reduction
// ============================================================================

struct KeyframeReductionResult
{
    Vector<uint32_t> keyframeIndices;
    Vector<float> keyframeTimestamps;
    CompressionStats stats;
};

// ============================================================================
// Compression Interface
// ============================================================================

class MMV2_API ICompressor
{
public:
    virtual ~ICompressor() = default;

    virtual CompressionAlgorithm GetAlgorithm() const = 0;
    virtual void SetSettings(const CompressionSettings& settings) = 0;

    // Pose compression
    virtual bool CompressPoses(const Vector<Pose>& poses, Vector<uint8_t>& outData) = 0;
    virtual bool DecompressPoses(const Vector<uint8_t>& data, Vector<Pose>& outPoses) = 0;

    // Feature vector compression
    virtual bool CompressFeatures(const Vector<FeatureVector>& features, Vector<uint8_t>& outData) = 0;
    virtual bool DecompressFeatures(const Vector<uint8_t>& data, Vector<FeatureVector>& outFeatures) = 0;

    // Single pose operations
    virtual bool CompressPose(const Pose& pose, CompressedPoseData& outData) = 0;
    virtual bool DecompressPose(const CompressedPoseData& data, Pose& outPose) = 0;

    // Statistics
    virtual const CompressionStats& GetStats() const = 0;
    virtual void ResetStats() = 0;
};

// ============================================================================
// Quantization Compressor
// ============================================================================

class MMV2_API QuantizationCompressor : public ICompressor
{
public:
    QuantizationCompressor();

    CompressionAlgorithm GetAlgorithm() const override { return CompressionAlgorithm::Quantize; }
    void SetSettings(const CompressionSettings& settings) override;

    bool CompressPoses(const Vector<Pose>& poses, Vector<uint8_t>& outData) override;
    bool DecompressPoses(const Vector<uint8_t>& data, Vector<Pose>& outPoses) override;

    bool CompressFeatures(const Vector<FeatureVector>& features, Vector<uint8_t>& outData) override;
    bool DecompressFeatures(const Vector<uint8_t>& data, Vector<FeatureVector>& outFeatures) override;

    bool CompressPose(const Pose& pose, CompressedPoseData& outData) override;
    bool DecompressPose(const CompressedPoseData& data, Pose& outPose) override;

    const CompressionStats& GetStats() const override { return m_stats; }
    void ResetStats() override { m_stats = CompressionStats(); }

private:
    CompressionSettings m_settings;
    CompressionStats m_stats;

    void QuantizeTransform(const Transform& transform, QuantizedTransform& out);
    void DequantizeTransform(const QuantizedTransform& quantized, Transform& out);
};

// ============================================================================
// Adaptive Quantization Compressor
// ============================================================================

class MMV2_API AdaptiveQuantizationCompressor : public ICompressor
{
public:
    CompressionAlgorithm GetAlgorithm() const override { return CompressionAlgorithm::AdaptiveQuantize; }
    void SetSettings(const CompressionSettings& settings) override;

    bool CompressPoses(const Vector<Pose>& poses, Vector<uint8_t>& outData) override;
    bool DecompressPoses(const Vector<uint8_t>& data, Vector<Pose>& outPoses) override;

    bool CompressFeatures(const Vector<FeatureVector>& features, Vector<uint8_t>& outData) override;
    bool DecompressFeatures(const Vector<uint8_t>& data, Vector<FeatureVector>& outFeatures) override;

    bool CompressPose(const Pose& pose, CompressedPoseData& outData) override;
    bool DecompressPose(const CompressedPoseData& data, Pose& outPose) override;

    const CompressionStats& GetStats() const override { return m_stats; }
    void ResetStats() override { m_stats = CompressionStats(); }

private:
    CompressionSettings m_settings;
    CompressionStats m_stats;

    struct AdaptiveRange
    {
        float min, max;
        float scale;
        uint32_t bits;
    };

    Vector<AdaptiveRange> m_positionRanges;
    Vector<AdaptiveRange> m_rotationRanges;

    void ComputeAdaptiveRanges(const Vector<Pose>& poses);
    void ComputeAdaptiveRanges(const Vector<FeatureVector>& features);
};

// ============================================================================
// PCA Compressor
// ============================================================================

class MMV2_API PCACompressor : public ICompressor
{
public:
    CompressionAlgorithm GetAlgorithm() const override { return CompressionAlgorithm::PCA; }
    void SetSettings(const CompressionSettings& settings) override;

    bool CompressPoses(const Vector<Pose>& poses, Vector<uint8_t>& outData) override;
    bool DecompressPoses(const Vector<uint8_t>& data, Vector<Pose>& outPoses) override;

    bool CompressFeatures(const Vector<FeatureVector>& features, Vector<uint8_t>& outData) override;
    bool DecompressFeatures(const Vector<uint8_t>& data, Vector<FeatureVector>& outFeatures) override;

    bool CompressPose(const Pose& pose, CompressedPoseData& outData) override;
    bool DecompressPose(const CompressedPoseData& data, Pose& outPose) override;

    const CompressionStats& GetStats() const override { return m_stats; }
    void ResetStats() override { m_stats = CompressionStats(); }

private:
    CompressionSettings m_settings;
    CompressionStats m_stats;

    // PCA data
    Vector<float> m_meanVector;
    Vector<float> m_eigenVectors; // Flattened matrix
    Vector<float> m_eigenValues;
    uint32_t m_originalDimensions;
    uint32_t m_componentCount;

    void ComputePCA(const Vector<FeatureVector>& features);
    void ProjectToPCA(const FeatureVector& input, Vector<float>& outComponents);
    void ReconstructFromPCA(const Vector<float>& components, FeatureVector& out);
};

// ============================================================================
// Keyframe Reduction
// ============================================================================

class MMV2_API KeyframeReducer
{
public:
    void SetSettings(const CompressionSettings& settings);

    KeyframeReductionResult ReduceKeyframes(const Vector<Pose>& poses,
                                             const Vector<float>& timestamps);

    KeyframeReductionResult ReduceKeyframesAdaptive(const Vector<Pose>& poses,
                                                     const Vector<float>& timestamps,
                                                     float maxError);

    Vector<Pose> ReconstructFromKeyframes(const KeyframeReductionResult& keyframes,
                                           const Vector<float>& targetTimestamps);

private:
    CompressionSettings m_settings;

    float ComputePoseError(const Pose& a, const Pose& b);
    bool IsKeyframeNeeded(const Pose& prev, const Pose& current, const Pose& next, float tolerance);
};

// ============================================================================
// Compression Factory
// ============================================================================

class MMV2_API CompressionFactory
{
public:
    static UniquePtr<ICompressor> CreateCompressor(CompressionAlgorithm algorithm);
    static UniquePtr<ICompressor> CreateDefaultCompressor();

    static CompressionSettings GetRecommendedSettings(CompressionAlgorithm algorithm);
    static CompressionSettings GetFastSettings();
    static CompressionSettings GetHighQualitySettings();
    static CompressionSettings GetMaximumCompressionSettings();
};

// ============================================================================
// Database Compression
// ============================================================================

class MMV2_API DatabaseCompressor
{
public:
    void SetCompressor(ICompressor* compressor) { m_compressor = compressor; }

    bool CompressDatabase(class PoseDatabase& database);
    bool DecompressDatabase(class PoseDatabase& database);

    bool CompressFeatureDatabase(class FeatureDatabase& database);
    bool DecompressFeatureDatabase(class FeatureDatabase& database);

    const CompressionStats& GetStats() const { return m_stats; }

private:
    ICompressor* m_compressor;
    CompressionStats m_stats;
};

MMV2_NAMESPACE_END
