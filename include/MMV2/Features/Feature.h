#pragma once
#ifndef MMV2_FEATURE_H
#define MMV2_FEATURE_H

#include "Config.h"
#include "Vec3.h"
#include "Quat.h"
#include "Vector.h"
#include "Span.h"
#include "String.h"
#include "HashMap.h"

MMV2_NAMESPACE_BEGIN

enum class FeatureChannelType : uint8 {
    Invalid = 0,
    BonePosition = 1,
    BoneRotation = 2,
    BoneVelocity = 3,
    BoneAngularVelocity = 4,
    TrajectoryPosition = 5,
    TrajectoryVelocity = 6,
    TrajectoryDirection = 7,
    TrajectoryOrientation = 8,
    RootVelocity = 9,
    RootAngularVelocity = 10,
    RootHeight = 11,
    FootContact = 12,
    Phase = 13,
    UserData = 14,
    Custom = 15
};

enum class FeatureDataType : uint8 {
    Float = 0,
    Vec2 = 1,
    Vec3 = 2,
    Quat = 3,
    Bool = 4,
    Int = 5,
    Count = 6
};

struct FeatureChannelDesc {
    String name;
    FeatureChannelType type;
    FeatureDataType dataType;
    int32 boneIndex;
    int32 trajectoryIndex;
    int32 offset;
    int32 dimension;
    float32 weight;
    float32 minValue;
    float32 maxValue;
    bool normalize;
    bool enabled;

    FeatureChannelDesc()
        : type(FeatureChannelType::Invalid), dataType(FeatureDataType::Float),
          boneIndex(-1), trajectoryIndex(-1), offset(0), dimension(1),
          weight(1.0f), minValue(0.0f), maxValue(1.0f), normalize(false), enabled(true) {}
};

struct FeatureSchema {
    Vector<FeatureChannelDesc> channels;
    int32 totalDimension;
    int32 enabledDimension;
    String name;

    FeatureSchema() : totalDimension(0), enabledDimension(0) {}

    int32 AddChannel(const FeatureChannelDesc& desc);
    int32 FindChannel(const char* name) const;
    int32 FindChannel(FeatureChannelType type, int32 boneIndex = -1) const;
    void RemoveChannel(int32 index);
    void SetChannelWeight(int32 index, float32 weight);
    void SetChannelEnabled(int32 index, bool enabled);
    void RebuildOffsets();
    int32 GetDimension() const { return enabledDimension; }
    bool IsValid() const;
    FeatureSchema Clone() const;
};

struct FeatureVector {
    Vector<float32> data;
    int32 dimension;

    FeatureVector() : dimension(0) {}
    explicit FeatureVector(int32 dim) : dimension(dim) { data.Resize(dim, 0.0f); }

    MMV2_FORCE_INLINE float32& operator[](int32 index) noexcept { return data[index]; }
    MMV2_FORCE_INLINE const float32& operator[](int32 index) const noexcept { return data[index]; }
    MMV2_FORCE_INLINE float32* Data() noexcept { return data.Data(); }
    MMV2_FORCE_INLINE const float32* Data() const noexcept { return data.Data(); }
    MMV2_FORCE_INLINE int32 Size() const noexcept { return dimension; }
    MMV2_FORCE_INLINE bool IsEmpty() const noexcept { return dimension == 0; }

    void Resize(int32 newDim);
    void SetZero();
    void SetFrom(const float32* src, int32 count);
    void CopyFrom(const FeatureVector& other);
    void Normalize(const FeatureSchema& schema);
    void Denormalize(const FeatureSchema& schema);
    float32 DistanceTo(const FeatureVector& other, const FeatureSchema& schema) const;
    float32 DistanceToSq(const FeatureVector& other, const FeatureSchema& schema) const;
    float32 DistanceToWeighted(const FeatureVector& other, const FeatureSchema& schema) const;
    float32 Dot(const FeatureVector& other) const;
    float32 Length() const;
    float32 LengthSq() const;
    void Scale(float32 s);
    void Add(const FeatureVector& other);
    void Subtract(const FeatureVector& other);
    void Lerp(const FeatureVector& a, const FeatureVector& b, float32 t);
    bool IsValid() const;
    bool IsNearZero() const;
};

class IFeatureExtractor {
public:
    virtual ~IFeatureExtractor() = default;
    virtual FeatureChannelType GetType() const = 0;
    virtual int32 GetDimension() const = 0;
    virtual void Extract(const class Pose& pose, const class PoseVelocity& velocity,
                         const class Trajectory& trajectory, float32* outData, int32 offset) const = 0;
    virtual void ExtractNormalized(const class Pose& pose, const class PoseVelocity& velocity,
                                   const class Trajectory& trajectory, float32* outData, int32 offset,
                                   const FeatureSchema& schema) const = 0;
    virtual bool Validate(const FeatureSchema& schema, int32 channelIndex) const = 0;
};

class BonePositionExtractor : public IFeatureExtractor {
public:
    BonePositionExtractor(int32 boneIdx, bool localSpace = false)
        : m_boneIndex(boneIdx), m_localSpace(localSpace) {}
    FeatureChannelType GetType() const override { return FeatureChannelType::BonePosition; }
    int32 GetDimension() const override { return 3; }
    void Extract(const class Pose& pose, const class PoseVelocity& velocity,
                 const class Trajectory& trajectory, float32* outData, int32 offset) const override;
    void ExtractNormalized(const class Pose& pose, const class PoseVelocity& velocity,
                           const class Trajectory& trajectory, float32* outData, int32 offset,
                           const FeatureSchema& schema) const override;
    bool Validate(const FeatureSchema& schema, int32 channelIndex) const override;
private:
    int32 m_boneIndex;
    bool m_localSpace;
};

class BoneRotationExtractor : public IFeatureExtractor {
public:
    BoneRotationExtractor(int32 boneIdx, bool localSpace = false)
        : m_boneIndex(boneIdx), m_localSpace(localSpace) {}
    FeatureChannelType GetType() const override { return FeatureChannelType::BoneRotation; }
    int32 GetDimension() const override { return 4; }
    void Extract(const class Pose& pose, const class PoseVelocity& velocity,
                 const class Trajectory& trajectory, float32* outData, int32 offset) const override;
    void ExtractNormalized(const class Pose& pose, const class PoseVelocity& velocity,
                           const class Trajectory& trajectory, float32* outData, int32 offset,
                           const FeatureSchema& schema) const override;
    bool Validate(const FeatureSchema& schema, int32 channelIndex) const override;
private:
    int32 m_boneIndex;
    bool m_localSpace;
};

class BoneVelocityExtractor : public IFeatureExtractor {
public:
    explicit BoneVelocityExtractor(int32 boneIdx) : m_boneIndex(boneIdx) {}
    FeatureChannelType GetType() const override { return FeatureChannelType::BoneVelocity; }
    int32 GetDimension() const override { return 3; }
    void Extract(const class Pose& pose, const class PoseVelocity& velocity,
                 const class Trajectory& trajectory, float32* outData, int32 offset) const override;
    void ExtractNormalized(const class Pose& pose, const class PoseVelocity& velocity,
                           const class Trajectory& trajectory, float32* outData, int32 offset,
                           const FeatureSchema& schema) const override;
    bool Validate(const FeatureSchema& schema, int32 channelIndex) const override;
private:
    int32 m_boneIndex;
};

class TrajectoryPositionExtractor : public IFeatureExtractor {
public:
    explicit TrajectoryPositionExtractor(int32 trajIdx) : m_trajectoryIndex(trajIdx) {}
    FeatureChannelType GetType() const override { return FeatureChannelType::TrajectoryPosition; }
    int32 GetDimension() const override { return 3; }
    void Extract(const class Pose& pose, const class PoseVelocity& velocity,
                 const class Trajectory& trajectory, float32* outData, int32 offset) const override;
    void ExtractNormalized(const class Pose& pose, const class PoseVelocity& velocity,
                           const class Trajectory& trajectory, float32* outData, int32 offset,
                           const FeatureSchema& schema) const override;
    bool Validate(const FeatureSchema& schema, int32 channelIndex) const override;
private:
    int32 m_trajectoryIndex;
};

class TrajectoryVelocityExtractor : public IFeatureExtractor {
public:
    explicit TrajectoryVelocityExtractor(int32 trajIdx) : m_trajectoryIndex(trajIdx) {}
    FeatureChannelType GetType() const override { return FeatureChannelType::TrajectoryVelocity; }
    int32 GetDimension() const override { return 3; }
    void Extract(const class Pose& pose, const class PoseVelocity& velocity,
                 const class Trajectory& trajectory, float32* outData, int32 offset) const override;
    void ExtractNormalized(const class Pose& pose, const class PoseVelocity& velocity,
                           const class Trajectory& trajectory, float32* outData, int32 offset,
                           const FeatureSchema& schema) const override;
    bool Validate(const FeatureSchema& schema, int32 channelIndex) const override;
private:
    int32 m_trajectoryIndex;
};

class RootVelocityExtractor : public IFeatureExtractor {
public:
    FeatureChannelType GetType() const override { return FeatureChannelType::RootVelocity; }
    int32 GetDimension() const override { return 3; }
    void Extract(const class Pose& pose, const class PoseVelocity& velocity,
                 const class Trajectory& trajectory, float32* outData, int32 offset) const override;
    void ExtractNormalized(const class Pose& pose, const class PoseVelocity& velocity,
                           const class Trajectory& trajectory, float32* outData, int32 offset,
                           const FeatureSchema& schema) const override;
    bool Validate(const FeatureSchema& schema, int32 channelIndex) const override;
};

class FootContactExtractor : public IFeatureExtractor {
public:
    FootContactExtractor(int32 leftFootIdx, int32 rightFootIdx, float32 threshold = 0.05f)
        : m_leftFootIndex(leftFootIdx), m_rightFootIndex(rightFootIdx), m_threshold(threshold) {}
    FeatureChannelType GetType() const override { return FeatureChannelType::FootContact; }
    int32 GetDimension() const override { return 2; }
    void Extract(const class Pose& pose, const class PoseVelocity& velocity,
                 const class Trajectory& trajectory, float32* outData, int32 offset) const override;
    void ExtractNormalized(const class Pose& pose, const class PoseVelocity& velocity,
                           const class Trajectory& trajectory, float32* outData, int32 offset,
                           const FeatureSchema& schema) const override;
    bool Validate(const FeatureSchema& schema, int32 channelIndex) const override;
private:
    int32 m_leftFootIndex;
    int32 m_rightFootIndex;
    float32 m_threshold;
};

class FeatureExtractorRegistry {
public:
    static FeatureExtractorRegistry& Instance();
    void RegisterExtractor(FeatureChannelType type, IFeatureExtractor* extractor);
    IFeatureExtractor* GetExtractor(FeatureChannelType type);
    const IFeatureExtractor* GetExtractor(FeatureChannelType type) const;
    void Clear();
private:
    FeatureExtractorRegistry() = default;
    HashMap<uint32, IFeatureExtractor*> m_extractors;
};

class FeatureExtractorManager {
public:
    FeatureExtractorManager();
    ~FeatureExtractorManager();

    void SetSchema(const FeatureSchema& schema);
    void ExtractFeatures(const class Pose& pose, const class PoseVelocity& velocity,
                         const class Trajectory& trajectory, FeatureVector& outFeatures) const;
    void ExtractFeaturesNormalized(const class Pose& pose, const class PoseVelocity& velocity,
                                   const class Trajectory& trajectory, FeatureVector& outFeatures) const;
    void BuildExtractors();
    bool ValidateSchema() const;
    const FeatureSchema& GetSchema() const { return m_schema; }
    int32 GetDimension() const { return m_schema.GetDimension(); }

private:
    FeatureSchema m_schema;
    Vector<IFeatureExtractor*> m_extractors;
    IAllocator* m_allocator;
};

MMV2_NAMESPACE_END

#endif
