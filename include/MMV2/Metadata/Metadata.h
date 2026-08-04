// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Metadata & Tagging System
// ============================================================================
// Provides annotation, categorization, and semantic labeling for animations,
// poses, and motion clips. Supports hierarchical tags, search by metadata,
// and runtime tag-based filtering.
// ============================================================================

#pragma once

#include "MMV2/Core/Config.h"
#include "MMV2/Core/Vector.h"
#include "MMV2/Core/HashMap.h"
#include "MMV2/Core/String.h"
#include "MMV2/Core/Span.h"

MMV2_NAMESPACE_BEGIN

// ============================================================================
// Tag Definition
// ============================================================================

struct Tag
{
    String name;
    String category;
    String description;
    uint32_t color; // ARGB color for editor visualization
    float weight;   // Search weight/importance
    bool isSystemTag;

    Tag() : color(0xFFFFFFFF), weight(1.0f), isSystemTag(false) {}

    bool operator==(const Tag& other) const { return name == other.name; }
    bool operator!=(const Tag& other) const { return name != other.name; }
};

// ============================================================================
// Tag Category
// ============================================================================

enum class TagCategory : uint32_t
{
    None        = 0,
    Action      = 1 << 0,   // run, walk, jump, attack
    State       = 1 << 1,   // idle, moving, airborne, grounded
    Direction   = 1 << 2,   // forward, backward, left, right
    Speed       = 1 << 3,   // slow, medium, fast, sprint
    Style       = 1 << 4,   // casual, combat, stealth, injured
    Terrain     = 1 << 5,   // flat, slope, stairs, uneven
    Interaction = 1 << 6,   // solo, group, object, enemy
    Weapon      = 1 << 7,   // unarmed, sword, gun, bow
    Emotion     = 1 << 8,   // neutral, angry, scared, happy
    Quality     = 1 << 9,   // rough, polished, mocap, handKeyed
    Loop        = 1 << 10,  // looping, oneShot, transition
    Custom1     = 1 << 11,
    Custom2     = 1 << 12,
    Custom3     = 1 << 13,
    Custom4     = 1 << 14,
    All         = 0xFFFFFFFF
};

MMV2_ENUM_CLASS_FLAGS(TagCategory)

// ============================================================================
// Animation Metadata
// ============================================================================

struct AnimationMetadata
{
    String name;
    String sourceFile;
    String description;
    String author;
    String dateCreated;
    String dateModified;
    uint32_t version;

    // Timing
    float duration;
    float frameRate;
    uint32_t frameCount;

    // Classification
    Vector<Tag> tags;
    TagCategory categories;

    // Technical
    uint32_t boneCount;
    uint32_t trackCount;
    bool hasRootMotion;
    bool isLooping;
    bool isMirrored;

    // Quality metrics
    float averageVelocity;
    float maxVelocity;
    float groundContactRatio;
    float symmetryScore;

    // User data
    HashMap<String, String> userData;

    AnimationMetadata() : version(1), duration(0), frameRate(30.0f), frameCount(0),
                          categories(TagCategory::None), boneCount(0), trackCount(0),
                          hasRootMotion(false), isLooping(false), isMirrored(false),
                          averageVelocity(0), maxVelocity(0), groundContactRatio(0),
                          symmetryScore(0) {}
};

// ============================================================================
// Pose Metadata
// ============================================================================

struct PoseMetadata
{
    uint32_t animationIndex;
    float animationTime;
    uint32_t frameNumber;

    // Derived tags at this specific pose
    Vector<Tag> tags;

    // Pose-specific metrics
    float velocity;
    float height;
    float groundDistance;
    bool isGrounded;
    bool isLeftFootContact;
    bool isRightFootContact;

    // Phase information
    float gaitPhase;        // 0-1 cycle position
    float stancePhase;      // Left/right stance ratio

    // Interaction data
    uint32_t interactionId;
    uint32_t interactionRole;

    PoseMetadata() : animationIndex(0), animationTime(0), frameNumber(0),
                     velocity(0), height(0), groundDistance(0),
                     isGrounded(true), isLeftFootContact(true), isRightFootContact(true),
                     gaitPhase(0), stancePhase(0.5f), interactionId(0), interactionRole(0) {}
};

// ============================================================================
// Tag Database
// ============================================================================

class MMV2_API TagDatabase
{
public:
    TagDatabase();

    // Tag management
    void RegisterTag(const Tag& tag);
    void UnregisterTag(const String& name);
    const Tag* FindTag(const String& name) const;
    bool HasTag(const String& name) const;

    // Category management
    void RegisterCategory(const String& name, TagCategory category);
    TagCategory GetCategory(const String& name) const;
    Vector<String> GetTagsInCategory(TagCategory category) const;

    // Query
    Vector<const Tag*> GetAllTags() const;
    Vector<const Tag*> SearchTags(const String& query) const;
    Vector<const Tag*> GetTagsByCategory(TagCategory category) const;

    // System tags
    void RegisterSystemTags();

    // Serialization
    void Serialize(class BinarySerializer& serializer) const;
    void Deserialize(class BinarySerializer& serializer);

private:
    HashMap<String, Tag> m_tags;
    HashMap<String, TagCategory> m_categories;
};

// ============================================================================
// Metadata Manager
// ============================================================================

class MMV2_API MetadataManager
{
public:
    MetadataManager();

    // Animation metadata
    void SetAnimationMetadata(uint32_t animIndex, const AnimationMetadata& metadata);
    const AnimationMetadata* GetAnimationMetadata(uint32_t animIndex) const;
    AnimationMetadata* GetAnimationMetadata(uint32_t animIndex);
    bool HasAnimationMetadata(uint32_t animIndex) const;
    void RemoveAnimationMetadata(uint32_t animIndex);

    // Pose metadata
    void SetPoseMetadata(uint32_t poseIndex, const PoseMetadata& metadata);
    const PoseMetadata* GetPoseMetadata(uint32_t poseIndex) const;
    bool HasPoseMetadata(uint32_t poseIndex) const;

    // Batch operations
    void TagAnimation(uint32_t animIndex, const String& tagName);
    void UntagAnimation(uint32_t animIndex, const String& tagName);
    bool AnimationHasTag(uint32_t animIndex, const String& tagName) const;

    void TagPose(uint32_t poseIndex, const String& tagName);
    void UntagPose(uint32_t poseIndex, const String& tagName);
    bool PoseHasTag(uint32_t poseIndex, const String& tagName) const;

    // Search
    Vector<uint32_t> FindAnimationsByTag(const String& tagName) const;
    Vector<uint32_t> FindAnimationsByCategory(TagCategory category) const;
    Vector<uint32_t> FindAnimationsByQuery(const String& query) const;

    Vector<uint32_t> FindPosesByTag(const String& tagName) const;
    Vector<uint32_t> FindPosesByCategory(TagCategory category) const;

    // Filtering
    Vector<uint32_t> FilterByTags(const Vector<uint32_t>& candidates,
                                   const Vector<String>& requiredTags,
                                   const Vector<String>& excludedTags) const;

    Vector<uint32_t> FilterByCategory(const Vector<uint32_t>& candidates,
                                       TagCategory requiredCategory) const;

    // Analytics
    void ComputeAnimationMetrics(uint32_t animIndex, class PoseDatabase& database);
    void ComputePoseMetrics(uint32_t poseIndex, class PoseDatabase& database);

    // Auto-tagging
    void AutoTagByVelocity(uint32_t animIndex, float slowThreshold, float fastThreshold);
    void AutoTagByGroundContact(uint32_t animIndex);
    void AutoTagBySymmetry(uint32_t animIndex);
    void AutoTagAll(class PoseDatabase& database);

    // Import/Export
    bool ImportFromJSON(const String& filepath);
    bool ExportToJSON(const String& filepath) const;
    bool ImportFromCSV(const String& filepath);
    bool ExportToCSV(const String& filepath) const;

    // Tag database access
    TagDatabase& GetTagDatabase() { return m_tagDatabase; }
    const TagDatabase& GetTagDatabase() const { return m_tagDatabase; }

    // Statistics
    uint32_t GetAnimationCount() const { return static_cast<uint32_t>(m_animationMetadata.Size()); }
    uint32_t GetPoseCount() const { return static_cast<uint32_t>(m_poseMetadata.Size()); }
    uint32_t GetTotalTagCount() const;

private:
    TagDatabase m_tagDatabase;
    HashMap<uint32_t, AnimationMetadata> m_animationMetadata;
    HashMap<uint32_t, PoseMetadata> m_poseMetadata;
};

// ============================================================================
// Metadata Query Language
// ============================================================================

enum class MetadataQueryOperator
{
    Equals,
    NotEquals,
    Contains,
    StartsWith,
    EndsWith,
    GreaterThan,
    LessThan,
    InRange,
    HasTag,
    HasCategory
};

struct MetadataQueryCondition
{
    String field;
    MetadataQueryOperator op;
    String value;
    float numericValue;
    float numericValue2; // For range queries
    bool isNumeric;
};

class MMV2_API MetadataQuery
{
public:
    void AddCondition(const MetadataQueryCondition& condition);
    void ClearConditions();

    bool Evaluate(const AnimationMetadata& metadata) const;
    bool Evaluate(const PoseMetadata& metadata) const;

    Vector<uint32_t> Execute(const MetadataManager& manager) const;

private:
    Vector<MetadataQueryCondition> m_conditions;
};

// ============================================================================
// Tag-based Search Filter
// ============================================================================

class MMV2_API TagSearchFilter
{
public:
    void RequireTag(const String& tag);
    void ExcludeTag(const String& tag);
    void RequireCategory(TagCategory category);
    void ExcludeCategory(TagCategory category);

    void SetMinVelocity(float velocity) { m_minVelocity = velocity; }
    void SetMaxVelocity(float velocity) { m_maxVelocity = velocity; }
    void SetGroundedOnly(bool grounded) { m_groundedOnly = grounded; }
    void SetLoopingOnly(bool looping) { m_loopingOnly = looping; }

    bool PassesFilter(const AnimationMetadata& metadata) const;
    bool PassesFilter(const PoseMetadata& metadata) const;

    Vector<uint32_t> Apply(const Vector<uint32_t>& candidates,
                            const MetadataManager& manager) const;

private:
    Vector<String> m_requiredTags;
    Vector<String> m_excludedTags;
    Vector<TagCategory> m_requiredCategories;
    Vector<TagCategory> m_excludedCategories;
    float m_minVelocity = 0.0f;
    float m_maxVelocity = FLT_MAX;
    bool m_groundedOnly = false;
    bool m_loopingOnly = false;
};

MMV2_NAMESPACE_END
