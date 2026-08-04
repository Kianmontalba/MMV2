// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Metadata & Tagging System Implementation
// ============================================================================

#include "MMV2/Metadata/Metadata.h"
#include "MMV2/Core/Serializer.h"
#include "MMV2/Core/Math.h"
#include "MMV2/Database/Database.h"
#include <algorithm>

MMV2_NAMESPACE_BEGIN

// ============================================================================
// TagDatabase
// ============================================================================

TagDatabase::TagDatabase()
{
    RegisterSystemTags();
}

void TagDatabase::RegisterTag(const Tag& tag)
{
    m_tags[tag.name] = tag;

    if (!tag.category.Empty())
        m_categories[tag.category] = TagCategory::Custom1; // Default to custom
}

void TagDatabase::UnregisterTag(const String& name)
{
    m_tags.Erase(name);
}

const Tag* TagDatabase::FindTag(const String& name) const
{
    auto it = m_tags.Find(name);
    return (it != m_tags.End()) ? &it->second : nullptr;
}

bool TagDatabase::HasTag(const String& name) const
{
    return m_tags.Contains(name);
}

void TagDatabase::RegisterCategory(const String& name, TagCategory category)
{
    m_categories[name] = category;
}

TagCategory TagDatabase::GetCategory(const String& name) const
{
    auto it = m_categories.Find(name);
    return (it != m_categories.End()) ? it->second : TagCategory::None;
}

Vector<String> TagDatabase::GetTagsInCategory(TagCategory category) const
{
    Vector<String> result;
    for (const auto& pair : m_tags)
    {
        TagCategory tagCat = GetCategory(pair.second.category);
        if ((static_cast<uint32_t>(tagCat) & static_cast<uint32_t>(category)) != 0)
            result.PushBack(pair.first);
    }
    return result;
}

Vector<const Tag*> TagDatabase::GetAllTags() const
{
    Vector<const Tag*> result;
    result.Reserve(m_tags.Size());
    for (const auto& pair : m_tags)
        result.PushBack(&pair.second);
    return result;
}

Vector<const Tag*> TagDatabase::SearchTags(const String& query) const
{
    Vector<const Tag*> result;
    String lowerQuery = query.ToLower();

    for (const auto& pair : m_tags)
    {
        if (pair.first.ToLower().Contains(lowerQuery) ||
            pair.second.description.ToLower().Contains(lowerQuery))
        {
            result.PushBack(&pair.second);
        }
    }
    return result;
}

Vector<const Tag*> TagDatabase::GetTagsByCategory(TagCategory category) const
{
    Vector<const Tag*> result;
    for (const auto& pair : m_tags)
    {
        if ((static_cast<uint32_t>(pair.second.categories) & static_cast<uint32_t>(category)) != 0)
            result.PushBack(&pair.second);
    }
    return result;
}

void TagDatabase::RegisterSystemTags()
{
    // Action tags
    RegisterTag({"idle", "action", "Character is stationary", 0xFF00FF00, 1.0f, true});
    RegisterTag({"walk", "action", "Character is walking", 0xFF00AA00, 1.0f, true});
    RegisterTag({"run", "action", "Character is running", 0xFF008800, 1.0f, true});
    RegisterTag({"sprint", "action", "Character is sprinting", 0xFF006600, 1.0f, true});
    RegisterTag({"jump", "action", "Character is jumping", 0xFF0000FF, 1.0f, true});
    RegisterTag({"fall", "action", "Character is falling", 0xFF4444FF, 1.0f, true});
    RegisterTag({"land", "action", "Character is landing", 0xFF6666FF, 1.0f, true});
    RegisterTag({"attack", "action", "Character is attacking", 0xFFFF0000, 1.0f, true});
    RegisterTag({"hit", "action", "Character is hit/reacting", 0xFFFF4444, 1.0f, true});
    RegisterTag({"death", "action", "Character death animation", 0xFF880000, 1.0f, true});

    // State tags
    RegisterTag({"grounded", "state", "Character is on ground", 0xFFAAAAAA, 1.0f, true});
    RegisterTag({"airborne", "state", "Character is in air", 0xFF8888FF, 1.0f, true});
    RegisterTag({"moving", "state", "Character has velocity", 0xFFAAFFAA, 1.0f, true});
    RegisterTag({"stationary", "state", "Character has no velocity", 0xFFFFFFFF, 1.0f, true});
    RegisterTag({"turning", "state", "Character is turning", 0xFFFFFF00, 1.0f, true});

    // Direction tags
    RegisterTag({"forward", "direction", "Moving forward", 0xFF00FFFF, 1.0f, true});
    RegisterTag({"backward", "direction", "Moving backward", 0xFF0088FF, 1.0f, true});
    RegisterTag({"left", "direction", "Moving/strafing left", 0xFFFF00FF, 1.0f, true});
    RegisterTag({"right", "direction", "Moving/strafing right", 0xFFFF0088, 1.0f, true});

    // Speed tags
    RegisterTag({"slow", "speed", "Low speed movement", 0xFFFFFF88, 1.0f, true});
    RegisterTag({"medium", "speed", "Medium speed movement", 0xFFFFFF44, 1.0f, true});
    RegisterTag({"fast", "speed", "High speed movement", 0xFFFFAA00, 1.0f, true});

    // Style tags
    RegisterTag({"casual", "style", "Relaxed/casual movement", 0xFF88FF88, 1.0f, true});
    RegisterTag({"combat", "style", "Combat-ready movement", 0xFFFF4444, 1.0f, true});
    RegisterTag({"stealth", "style", "Stealthy/careful movement", 0xFF4444FF, 1.0f, true});
    RegisterTag({"injured", "style", "Injured/limp movement", 0xFFFF8800, 1.0f, true});

    // Terrain tags
    RegisterTag({"flat", "terrain", "Flat ground", 0xFFCCCCCC, 1.0f, true});
    RegisterTag({"slope", "terrain", "Sloped ground", 0xFFAAAA88, 1.0f, true});
    RegisterTag({"stairs", "terrain", "Stair climbing", 0xFF8888AA, 1.0f, true});
    RegisterTag({"uneven", "terrain", "Uneven/rough terrain", 0xFFAA8888, 1.0f, true});

    // Loop tags
    RegisterTag({"looping", "loop", "Animation loops seamlessly", 0xFF00FF88, 1.0f, true});
    RegisterTag({"oneshot", "loop", "Single-shot animation", 0xFFFF8800, 1.0f, true});
    RegisterTag({"transition", "loop", "Transition/blend animation", 0xFF8888FF, 1.0f, true});
}

void TagDatabase::Serialize(BinarySerializer& serializer) const
{
    serializer.Write(static_cast<uint32_t>(m_tags.Size()));
    for (const auto& pair : m_tags)
    {
        serializer.Write(pair.second.name);
        serializer.Write(pair.second.category);
        serializer.Write(pair.second.description);
        serializer.Write(pair.second.color);
        serializer.Write(pair.second.weight);
        serializer.Write(pair.second.isSystemTag);
    }
}

void TagDatabase::Deserialize(BinarySerializer& serializer)
{
    uint32_t count;
    serializer.Read(count);
    m_tags.Clear();

    for (uint32_t i = 0; i < count; ++i)
    {
        Tag tag;
        serializer.Read(tag.name);
        serializer.Read(tag.category);
        serializer.Read(tag.description);
        serializer.Read(tag.color);
        serializer.Read(tag.weight);
        serializer.Read(tag.isSystemTag);
        m_tags[tag.name] = tag;
    }
}

// ============================================================================
// MetadataManager
// ============================================================================

MetadataManager::MetadataManager()
{
}

void MetadataManager::SetAnimationMetadata(uint32_t animIndex, const AnimationMetadata& metadata)
{
    m_animationMetadata[animIndex] = metadata;
}

const AnimationMetadata* MetadataManager::GetAnimationMetadata(uint32_t animIndex) const
{
    auto it = m_animationMetadata.Find(animIndex);
    return (it != m_animationMetadata.End()) ? &it->second : nullptr;
}

AnimationMetadata* MetadataManager::GetAnimationMetadata(uint32_t animIndex)
{
    auto it = m_animationMetadata.Find(animIndex);
    return (it != m_animationMetadata.End()) ? &it->second : nullptr;
}

bool MetadataManager::HasAnimationMetadata(uint32_t animIndex) const
{
    return m_animationMetadata.Contains(animIndex);
}

void MetadataManager::RemoveAnimationMetadata(uint32_t animIndex)
{
    m_animationMetadata.Erase(animIndex);
}

void MetadataManager::SetPoseMetadata(uint32_t poseIndex, const PoseMetadata& metadata)
{
    m_poseMetadata[poseIndex] = metadata;
}

const PoseMetadata* MetadataManager::GetPoseMetadata(uint32_t poseIndex) const
{
    auto it = m_poseMetadata.Find(poseIndex);
    return (it != m_poseMetadata.End()) ? &it->second : nullptr;
}

bool MetadataManager::HasPoseMetadata(uint32_t poseIndex) const
{
    return m_poseMetadata.Contains(poseIndex);
}

void MetadataManager::TagAnimation(uint32_t animIndex, const String& tagName)
{
    auto it = m_animationMetadata.Find(animIndex);
    if (it == m_animationMetadata.End())
    {
        AnimationMetadata meta;
        meta.tags.PushBack({tagName, "", "", 0xFFFFFFFF, 1.0f, false});
        m_animationMetadata[animIndex] = meta;
    }
    else
    {
        // Check if tag already exists
        for (const auto& tag : it->second.tags)
        {
            if (tag.name == tagName)
                return;
        }
        it->second.tags.PushBack({tagName, "", "", 0xFFFFFFFF, 1.0f, false});
    }
}

void MetadataManager::UntagAnimation(uint32_t animIndex, const String& tagName)
{
    auto it = m_animationMetadata.Find(animIndex);
    if (it != m_animationMetadata.End())
    {
        auto& tags = it->second.tags;
        for (uint32_t i = 0; i < tags.Size(); ++i)
        {
            if (tags[i].name == tagName)
            {
                tags.Erase(i);
                break;
            }
        }
    }
}

bool MetadataManager::AnimationHasTag(uint32_t animIndex, const String& tagName) const
{
    auto it = m_animationMetadata.Find(animIndex);
    if (it == m_animationMetadata.End())
        return false;

    for (const auto& tag : it->second.tags)
    {
        if (tag.name == tagName)
            return true;
    }
    return false;
}

void MetadataManager::TagPose(uint32_t poseIndex, const String& tagName)
{
    auto it = m_poseMetadata.Find(poseIndex);
    if (it == m_poseMetadata.End())
    {
        PoseMetadata meta;
        meta.tags.PushBack({tagName, "", "", 0xFFFFFFFF, 1.0f, false});
        m_poseMetadata[poseIndex] = meta;
    }
    else
    {
        for (const auto& tag : it->second.tags)
        {
            if (tag.name == tagName)
                return;
        }
        it->second.tags.PushBack({tagName, "", "", 0xFFFFFFFF, 1.0f, false});
    }
}

void MetadataManager::UntagPose(uint32_t poseIndex, const String& tagName)
{
    auto it = m_poseMetadata.Find(poseIndex);
    if (it != m_poseMetadata.End())
    {
        auto& tags = it->second.tags;
        for (uint32_t i = 0; i < tags.Size(); ++i)
        {
            if (tags[i].name == tagName)
            {
                tags.Erase(i);
                break;
            }
        }
    }
}

bool MetadataManager::PoseHasTag(uint32_t poseIndex, const String& tagName) const
{
    auto it = m_poseMetadata.Find(poseIndex);
    if (it == m_poseMetadata.End())
        return false;

    for (const auto& tag : it->second.tags)
    {
        if (tag.name == tagName)
            return true;
    }
    return false;
}

Vector<uint32_t> MetadataManager::FindAnimationsByTag(const String& tagName) const
{
    Vector<uint32_t> result;
    for (const auto& pair : m_animationMetadata)
    {
        for (const auto& tag : pair.second.tags)
        {
            if (tag.name == tagName)
            {
                result.PushBack(pair.first);
                break;
            }
        }
    }
    return result;
}

Vector<uint32_t> MetadataManager::FindAnimationsByCategory(TagCategory category) const
{
    Vector<uint32_t> result;
    for (const auto& pair : m_animationMetadata)
    {
        if ((static_cast<uint32_t>(pair.second.categories) & static_cast<uint32_t>(category)) != 0)
            result.PushBack(pair.first);
    }
    return result;
}

Vector<uint32_t> MetadataManager::FindAnimationsByQuery(const String& query) const
{
    Vector<uint32_t> result;
    String lowerQuery = query.ToLower();

    for (const auto& pair : m_animationMetadata)
    {
        if (pair.second.name.ToLower().Contains(lowerQuery) ||
            pair.second.description.ToLower().Contains(lowerQuery))
        {
            result.PushBack(pair.first);
        }
    }
    return result;
}

Vector<uint32_t> MetadataManager::FindPosesByTag(const String& tagName) const
{
    Vector<uint32_t> result;
    for (const auto& pair : m_poseMetadata)
    {
        for (const auto& tag : pair.second.tags)
        {
            if (tag.name == tagName)
            {
                result.PushBack(pair.first);
                break;
            }
        }
    }
    return result;
}

Vector<uint32_t> MetadataManager::FindPosesByCategory(TagCategory category) const
{
    Vector<uint32_t> result;
    for (const auto& pair : m_poseMetadata)
    {
        // Pose metadata doesn't have categories directly, check tags
        for (const auto& tag : pair.second.tags)
        {
            const Tag* tagDef = m_tagDatabase.FindTag(tag.name);
            if (tagDef && (static_cast<uint32_t>(tagDef->categories) & static_cast<uint32_t>(category)) != 0)
            {
                result.PushBack(pair.first);
                break;
            }
        }
    }
    return result;
}

Vector<uint32_t> MetadataManager::FilterByTags(const Vector<uint32_t>& candidates,
                                                const Vector<String>& requiredTags,
                                                const Vector<String>& excludedTags) const
{
    Vector<uint32_t> result;
    result.Reserve(candidates.Size());

    for (uint32_t animIndex : candidates)
    {
        bool passes = true;

        // Check required tags
        for (const auto& tag : requiredTags)
        {
            if (!AnimationHasTag(animIndex, tag))
            {
                passes = false;
                break;
            }
        }

        if (!passes) continue;

        // Check excluded tags
        for (const auto& tag : excludedTags)
        {
            if (AnimationHasTag(animIndex, tag))
            {
                passes = false;
                break;
            }
        }

        if (passes)
            result.PushBack(animIndex);
    }

    return result;
}

Vector<uint32_t> MetadataManager::FilterByCategory(const Vector<uint32_t>& candidates,
                                                    TagCategory requiredCategory) const
{
    Vector<uint32_t> result;
    result.Reserve(candidates.Size());

    for (uint32_t animIndex : candidates)
    {
        auto it = m_animationMetadata.Find(animIndex);
        if (it != m_animationMetadata.End())
        {
            if ((static_cast<uint32_t>(it->second.categories) & static_cast<uint32_t>(requiredCategory)) != 0)
                result.PushBack(animIndex);
        }
    }

    return result;
}

void MetadataManager::AutoTagByVelocity(uint32_t animIndex, float slowThreshold, float fastThreshold)
{
    auto it = m_animationMetadata.Find(animIndex);
    if (it == m_animationMetadata.End())
        return;

    float avgVel = it->second.averageVelocity;

    if (avgVel < slowThreshold)
    {
        TagAnimation(animIndex, "slow");
        if (avgVel < 0.1f)
            TagAnimation(animIndex, "idle");
        else
            TagAnimation(animIndex, "walk");
    }
    else if (avgVel < fastThreshold)
    {
        TagAnimation(animIndex, "medium");
        TagAnimation(animIndex, "run");
    }
    else
    {
        TagAnimation(animIndex, "fast");
        TagAnimation(animIndex, "sprint");
    }

    if (avgVel > 0.1f)
        TagAnimation(animIndex, "moving");
    else
        TagAnimation(animIndex, "stationary");
}

void MetadataManager::AutoTagByGroundContact(uint32_t animIndex)
{
    auto it = m_animationMetadata.Find(animIndex);
    if (it == m_animationMetadata.End())
        return;

    if (it->second.groundContactRatio > 0.9f)
        TagAnimation(animIndex, "grounded");
    else if (it->second.groundContactRatio < 0.1f)
        TagAnimation(animIndex, "airborne");
}

void MetadataManager::AutoTagBySymmetry(uint32_t animIndex)
{
    auto it = m_animationMetadata.Find(animIndex);
    if (it == m_animationMetadata.End())
        return;

    if (it->second.symmetryScore > 0.8f)
    {
        // Highly symmetric - likely idle or straight movement
        if (it->second.averageVelocity < 0.1f)
            TagAnimation(animIndex, "idle");
    }
}

void MetadataManager::AutoTagAll(PoseDatabase& database)
{
    const uint32_t animCount = database.GetAnimationCount();

    for (uint32_t i = 0; i < animCount; ++i)
    {
        ComputeAnimationMetrics(i, database);
        AutoTagByVelocity(i, 1.5f, 4.0f);
        AutoTagByGroundContact(i);
        AutoTagBySymmetry(i);
    }
}

void MetadataManager::ComputeAnimationMetrics(uint32_t animIndex, PoseDatabase& database)
{
    auto it = m_animationMetadata.Find(animIndex);
    if (it == m_animationMetadata.End())
    {
        it = m_animationMetadata.Insert(animIndex, AnimationMetadata()).first;
    }

    AnimationMetadata& meta = it->second;

    // Compute average and max velocity
    float totalVel = 0.0f;
    float maxVel = 0.0f;
    uint32_t sampleCount = 0;

    const uint32_t poseCount = database.GetPoseCount();
    for (uint32_t p = 0; p < poseCount; ++p)
    {
        const PoseMetadata* poseMeta = GetPoseMetadata(p);
        if (poseMeta && poseMeta->animationIndex == animIndex)
        {
            totalVel += poseMeta->velocity;
            maxVel = Math::Max(maxVel, poseMeta->velocity);
            ++sampleCount;
        }
    }

    if (sampleCount > 0)
    {
        meta.averageVelocity = totalVel / static_cast<float>(sampleCount);
        meta.maxVelocity = maxVel;
    }
}

void MetadataManager::ComputePoseMetrics(uint32_t poseIndex, PoseDatabase& database)
{
    // Implementation depends on pose data access
    // This is a placeholder for pose-level metric computation
}

bool MetadataManager::ImportFromJSON(const String& filepath)
{
    // TODO: Implement JSON import
    return false;
}

bool MetadataManager::ExportToJSON(const String& filepath) const
{
    // TODO: Implement JSON export
    return false;
}

bool MetadataManager::ImportFromCSV(const String& filepath)
{
    // TODO: Implement CSV import
    return false;
}

bool MetadataManager::ExportToCSV(const String& filepath) const
{
    // TODO: Implement CSV export
    return false;
}

uint32_t MetadataManager::GetTotalTagCount() const
{
    uint32_t count = 0;
    for (const auto& pair : m_animationMetadata)
        count += static_cast<uint32_t>(pair.second.tags.Size());
    for (const auto& pair : m_poseMetadata)
        count += static_cast<uint32_t>(pair.second.tags.Size());
    return count;
}

// ============================================================================
// MetadataQuery
// ============================================================================

void MetadataQuery::AddCondition(const MetadataQueryCondition& condition)
{
    m_conditions.PushBack(condition);
}

void MetadataQuery::ClearConditions()
{
    m_conditions.Clear();
}

bool MetadataQuery::Evaluate(const AnimationMetadata& metadata) const
{
    for (const auto& cond : m_conditions)
    {
        bool conditionMet = false;

        if (cond.field == "name")
        {
            switch (cond.op)
            {
                case MetadataQueryOperator::Equals: conditionMet = (metadata.name == cond.value); break;
                case MetadataQueryOperator::Contains: conditionMet = metadata.name.Contains(cond.value); break;
                case MetadataQueryOperator::StartsWith: conditionMet = metadata.name.StartsWith(cond.value); break;
                default: break;
            }
        }
        else if (cond.field == "tag")
        {
            if (cond.op == MetadataQueryOperator::HasTag)
            {
                for (const auto& tag : metadata.tags)
                {
                    if (tag.name == cond.value)
                    {
                        conditionMet = true;
                        break;
                    }
                }
            }
        }
        else if (cond.field == "duration" && cond.isNumeric)
        {
            switch (cond.op)
            {
                case MetadataQueryOperator::GreaterThan: conditionMet = metadata.duration > cond.numericValue; break;
                case MetadataQueryOperator::LessThan: conditionMet = metadata.duration < cond.numericValue; break;
                case MetadataQueryOperator::InRange: conditionMet = (metadata.duration >= cond.numericValue && 
                                                                      metadata.duration <= cond.numericValue2); break;
                default: break;
            }
        }

        if (!conditionMet)
            return false;
    }

    return true;
}

bool MetadataQuery::Evaluate(const PoseMetadata& metadata) const
{
    for (const auto& cond : m_conditions)
    {
        bool conditionMet = false;

        if (cond.field == "velocity" && cond.isNumeric)
        {
            switch (cond.op)
            {
                case MetadataQueryOperator::GreaterThan: conditionMet = metadata.velocity > cond.numericValue; break;
                case MetadataQueryOperator::LessThan: conditionMet = metadata.velocity < cond.numericValue; break;
                default: break;
            }
        }
        else if (cond.field == "tag" && cond.op == MetadataQueryOperator::HasTag)
        {
            for (const auto& tag : metadata.tags)
            {
                if (tag.name == cond.value)
                {
                    conditionMet = true;
                    break;
                }
            }
        }

        if (!conditionMet)
            return false;
    }

    return true;
}

Vector<uint32_t> MetadataQuery::Execute(const MetadataManager& manager) const
{
    Vector<uint32_t> result;

    // Search animations
    const uint32_t animCount = manager.GetAnimationCount();
    for (uint32_t i = 0; i < animCount; ++i)
    {
        const AnimationMetadata* meta = manager.GetAnimationMetadata(i);
        if (meta && Evaluate(*meta))
            result.PushBack(i);
    }

    return result;
}

// ============================================================================
// TagSearchFilter
// ============================================================================

void TagSearchFilter::RequireTag(const String& tag)
{
    m_requiredTags.PushBack(tag);
}

void TagSearchFilter::ExcludeTag(const String& tag)
{
    m_excludedTags.PushBack(tag);
}

void TagSearchFilter::RequireCategory(TagCategory category)
{
    m_requiredCategories.PushBack(category);
}

void TagSearchFilter::ExcludeCategory(TagCategory category)
{
    m_excludedCategories.PushBack(category);
}

bool TagSearchFilter::PassesFilter(const AnimationMetadata& metadata) const
{
    // Check required tags
    for (const auto& tagName : m_requiredTags)
    {
        bool hasTag = false;
        for (const auto& tag : metadata.tags)
        {
            if (tag.name == tagName)
            {
                hasTag = true;
                break;
            }
        }
        if (!hasTag) return false;
    }

    // Check excluded tags
    for (const auto& tagName : m_excludedTags)
    {
        for (const auto& tag : metadata.tags)
        {
            if (tag.name == tagName)
                return false;
        }
    }

    // Check velocity range
    if (metadata.averageVelocity < m_minVelocity || metadata.averageVelocity > m_maxVelocity)
        return false;

    // Check looping
    if (m_loopingOnly && !metadata.isLooping)
        return false;

    return true;
}

bool TagSearchFilter::PassesFilter(const PoseMetadata& metadata) const
{
    if (m_groundedOnly && !metadata.isGrounded)
        return false;

    if (metadata.velocity < m_minVelocity || metadata.velocity > m_maxVelocity)
        return false;

    // Check required tags
    for (const auto& tagName : m_requiredTags)
    {
        bool hasTag = false;
        for (const auto& tag : metadata.tags)
        {
            if (tag.name == tagName)
            {
                hasTag = true;
                break;
            }
        }
        if (!hasTag) return false;
    }

    return true;
}

Vector<uint32_t> TagSearchFilter::Apply(const Vector<uint32_t>& candidates,
                                         const MetadataManager& manager) const
{
    Vector<uint32_t> result;
    result.Reserve(candidates.Size());

    for (uint32_t animIndex : candidates)
    {
        const AnimationMetadata* meta = manager.GetAnimationMetadata(animIndex);
        if (meta && PassesFilter(*meta))
            result.PushBack(animIndex);
    }

    return result;
}

MMV2_NAMESPACE_END
