// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Asset Browser Implementation
// ============================================================================

#include "MMV2/Editor/Browser/AssetBrowser.h"
#include "MMV2/Core/Serializer.h"
#include "MMV2/Core/Math.h"
#include <algorithm>

MMV2_NAMESPACE_BEGIN

// ============================================================================
// AssetFilter
// ============================================================================

bool AssetFilter::Matches(const AssetInfo& asset) const
{
    // Search query
    if (!searchQuery.IsEmpty())
    {
        bool nameMatch = asset.name.Contains(searchQuery) ||
                         asset.displayName.Contains(searchQuery);
        bool tagMatch = false;
        for (const auto& tag : asset.tags)
        {
            if (tag.name.Contains(searchQuery))
            {
                tagMatch = true;
                break;
            }
        }
        if (!nameMatch && !tagMatch)
            return false;
    }

    // Type filter
    if (!includedTypes.Empty())
    {
        bool typeMatch = false;
        for (const auto& type : includedTypes)
        {
            if (asset.type == type)
            {
                typeMatch = true;
                break;
            }
        }
        if (!typeMatch)
            return false;
    }

    // Tag filters
    for (const auto& tag : includedTags)
    {
        bool hasTag = false;
        for (const auto& assetTag : asset.tags)
        {
            if (assetTag.name == tag)
            {
                hasTag = true;
                break;
            }
        }
        if (!hasTag)
            return false;
    }

    for (const auto& tag : excludedTags)
    {
        for (const auto& assetTag : asset.tags)
        {
            if (assetTag.name == tag)
                return false;
        }
    }

    // Size filter
    if (asset.fileSize < minSize || asset.fileSize > maxSize)
        return false;

    // Date filter
    if (asset.dateModified < minDate || asset.dateModified > maxDate)
        return false;

    // Favorites
    if (favoritesOnly && !asset.isFavorite)
        return false;

    // Unused
    if (unusedOnly && asset.usageCount > 0)
        return false;

    return true;
}

void AssetFilter::Clear()
{
    searchQuery.Clear();
    includedTypes.Clear();
    includedTags.Clear();
    excludedTags.Clear();
    minSize = 0;
    maxSize = UINT64_MAX;
    minDate = 0;
    maxDate = UINT64_MAX;
    favoritesOnly = false;
    unusedOnly = false;
}

// ============================================================================
// AssetBrowser
// ============================================================================

AssetBrowser::AssetBrowser()
    : m_sortMode(AssetSortMode::Name)
    , m_sortAscending(true)
    , m_viewMode(AssetViewMode::Icon)
    , m_iconSize(128)
    , m_isDragging(false)
    , m_onAssetSelected(nullptr)
    , m_onAssetDoubleClicked(nullptr)
    , m_onAssetContextMenu(nullptr)
    , m_callbackUserData(nullptr)
    , m_initialized(false)
    , m_debugEnabled(false)
    , m_filterDirty(true)
{
}

AssetBrowser::~AssetBrowser()
{
    Shutdown();
}

void AssetBrowser::Initialize(const String& rootPath)
{
    if (m_initialized)
        return;

    m_rootPaths.PushBack(rootPath);
    LoadFavorites();
    LoadCollections();

    ScanDirectory(rootPath);
    RebuildFilter();

    m_initialized = true;
}

void AssetBrowser::Shutdown()
{
    if (!m_initialized)
        return;

    SaveFavorites();
    SaveCollections();

    m_assets.Clear();
    m_assetIndexMap.Clear();
    m_rootPaths.Clear();
    m_filteredIndices.Clear();
    m_selectedPaths.Clear();
    m_favorites.Clear();
    m_collections.Clear();
    m_thumbnailMap.Clear();
    m_thumbnailQueue.Clear();

    m_initialized = false;
}

// ============================================================================
// Asset Discovery
// ============================================================================

void AssetBrowser::ScanDirectory(const String& path)
{
    // In a real implementation, this would scan the filesystem
    // For now, we'll simulate asset discovery

    if (m_debugEnabled)
    {
        Log::Debug("AssetBrowser: Scanning directory: %s", path.CStr());
    }

    // TODO: Implement filesystem scanning
    // This would use platform-specific APIs to list files
}

void AssetBrowser::ScanRecursive(const String& path)
{
    ScanDirectory(path);
    // TODO: Recursively scan subdirectories
}

void AssetBrowser::Refresh()
{
    m_assets.Clear();
    m_assetIndexMap.Clear();

    for (const auto& rootPath : m_rootPaths)
    {
        ScanRecursive(rootPath);
    }

    m_filterDirty = true;
    RebuildFilter();
}

void AssetBrowser::AddAssetPath(const String& path)
{
    for (const auto& existing : m_rootPaths)
    {
        if (existing == path)
            return;
    }

    m_rootPaths.PushBack(path);
    ScanRecursive(path);
    m_filterDirty = true;
}

void AssetBrowser::RemoveAssetPath(const String& path)
{
    for (size_type i = 0; i < m_rootPaths.Size(); ++i)
    {
        if (m_rootPaths[i] == path)
        {
            m_rootPaths.Erase(i);
            break;
        }
    }

    // Remove assets from this path
    for (int32 i = static_cast<int32>(m_assets.Size()) - 1; i >= 0; --i)
    {
        if (m_assets[i].path.StartsWith(path))
        {
            m_assetIndexMap.Remove(m_assets[i].path);
            m_assets.Erase(i);
        }
    }

    m_filterDirty = true;
    RebuildFilter();
}

// ============================================================================
// Asset Access
// ============================================================================

const AssetInfo* AssetBrowser::GetAsset(const String& path) const
{
    auto it = m_assetIndexMap.Find(path);
    if (it != m_assetIndexMap.End())
    {
        uint32_t index = it->value;
        if (index < m_assets.Size())
            return &m_assets[index];
    }
    return nullptr;
}

AssetInfo* AssetBrowser::GetAsset(const String& path)
{
    auto it = m_assetIndexMap.Find(path);
    if (it != m_assetIndexMap.End())
    {
        uint32_t index = it->value;
        if (index < m_assets.Size())
            return &m_assets[index];
    }
    return nullptr;
}

Vector<const AssetInfo*> AssetBrowser::GetFilteredAssets() const
{
    Vector<const AssetInfo*> result;
    result.Reserve(m_filteredIndices.Size());

    for (uint32_t index : m_filteredIndices)
    {
        if (index < m_assets.Size())
            result.PushBack(&m_assets[index]);
    }

    return result;
}

Vector<const AssetInfo*> AssetBrowser::GetSelectedAssets() const
{
    Vector<const AssetInfo*> result;

    for (const auto& path : m_selectedPaths)
    {
        const AssetInfo* asset = GetAsset(path);
        if (asset != nullptr)
            result.PushBack(asset);
    }

    return result;
}

Vector<const AssetInfo*> AssetBrowser::GetFavoriteAssets() const
{
    Vector<const AssetInfo*> result;

    for (const auto& path : m_favorites)
    {
        const AssetInfo* asset = GetAsset(path);
        if (asset != nullptr)
            result.PushBack(asset);
    }

    return result;
}

// ============================================================================
// Filtering
// ============================================================================

void AssetBrowser::SetFilter(const AssetFilter& filter)
{
    m_filter = filter;
    m_filterDirty = true;
    RebuildFilter();
}

void AssetBrowser::ClearFilter()
{
    m_filter.Clear();
    m_filterDirty = true;
    RebuildFilter();
}

void AssetBrowser::SetSearchQuery(const String& query)
{
    m_filter.searchQuery = query;
    m_filterDirty = true;
    RebuildFilter();
}

void AssetBrowser::SetTypeFilter(AssetType type, bool include)
{
    if (include)
    {
        bool found = false;
        for (const auto& t : m_filter.includedTypes)
        {
            if (t == type)
            {
                found = true;
                break;
            }
        }
        if (!found)
            m_filter.includedTypes.PushBack(type);
    }
    else
    {
        for (size_type i = 0; i < m_filter.includedTypes.Size(); ++i)
        {
            if (m_filter.includedTypes[i] == type)
            {
                m_filter.includedTypes.Erase(i);
                break;
            }
        }
    }
    m_filterDirty = true;
    RebuildFilter();
}

void AssetBrowser::SetTagFilter(const String& tag, bool include)
{
    if (include)
    {
        m_filter.includedTags.PushBack(tag);
    }
    else
    {
        m_filter.excludedTags.PushBack(tag);
    }
    m_filterDirty = true;
    RebuildFilter();
}

void AssetBrowser::SetFavoritesOnly(bool only)
{
    m_filter.favoritesOnly = only;
    m_filterDirty = true;
    RebuildFilter();
}

void AssetBrowser::RebuildFilter()
{
    if (!m_filterDirty)
        return;

    UpdateFilteredIndices();
    SortAssets();
    m_filterDirty = false;
}

void AssetBrowser::UpdateFilteredIndices()
{
    m_filteredIndices.Clear();

    for (size_type i = 0; i < m_assets.Size(); ++i)
    {
        if (m_filter.Matches(m_assets[i]))
        {
            m_filteredIndices.PushBack(static_cast<uint32_t>(i));
        }
    }
}

// ============================================================================
// Sorting
// ============================================================================

void AssetBrowser::SetSortMode(AssetSortMode mode)
{
    if (m_sortMode != mode)
    {
        m_sortMode = mode;
        SortAssets();
    }
}

void AssetBrowser::SetSortAscending(bool ascending)
{
    if (m_sortAscending != ascending)
    {
        m_sortAscending = ascending;
        SortAssets();
    }
}

void AssetBrowser::SortAssets()
{
    std::sort(m_filteredIndices.Begin(), m_filteredIndices.End(),
              [this](uint32_t a, uint32_t b)
    {
        int32_t cmp = CompareAssets(m_assets[a], m_assets[b]);
        return m_sortAscending ? (cmp < 0) : (cmp > 0);
    });
}

int32_t AssetBrowser::CompareAssets(const AssetInfo& a, const AssetInfo& b) const
{
    switch (m_sortMode)
    {
        case AssetSortMode::Name:
            return a.name.Compare(b.name);
        case AssetSortMode::Type:
            if (a.type != b.type)
                return static_cast<int32_t>(a.type) - static_cast<int32_t>(b.type);
            return a.name.Compare(b.name);
        case AssetSortMode::Size:
            if (a.fileSize != b.fileSize)
                return a.fileSize < b.fileSize ? -1 : 1;
            return a.name.Compare(b.name);
        case AssetSortMode::DateModified:
            if (a.dateModified != b.dateModified)
                return a.dateModified < b.dateModified ? -1 : 1;
            return a.name.Compare(b.name);
        case AssetSortMode::DateCreated:
            if (a.dateCreated != b.dateCreated)
                return a.dateCreated < b.dateCreated ? -1 : 1;
            return a.name.Compare(b.name);
        case AssetSortMode::UsageCount:
            if (a.usageCount != b.usageCount)
                return a.usageCount < b.usageCount ? -1 : 1;
            return a.name.Compare(b.name);
        default:
            return a.name.Compare(b.name);
    }
}

// ============================================================================
// Selection
// ============================================================================

void AssetBrowser::SelectAsset(const String& path)
{
    DeselectAll();
    m_selectedPaths.PushBack(path);

    AssetInfo* asset = GetAsset(path);
    if (asset != nullptr)
        asset->isSelected = true;

    if (m_onAssetSelected != nullptr)
        m_onAssetSelected(path, m_callbackUserData);
}

void AssetBrowser::DeselectAsset(const String& path)
{
    for (size_type i = 0; i < m_selectedPaths.Size(); ++i)
    {
        if (m_selectedPaths[i] == path)
        {
            m_selectedPaths.Erase(i);
            break;
        }
    }

    AssetInfo* asset = GetAsset(path);
    if (asset != nullptr)
        asset->isSelected = false;
}

void AssetBrowser::SelectAll()
{
    m_selectedPaths.Clear();

    for (uint32_t index : m_filteredIndices)
    {
        if (index < m_assets.Size())
        {
            m_assets[index].isSelected = true;
            m_selectedPaths.PushBack(m_assets[index].path);
        }
    }
}

void AssetBrowser::DeselectAll()
{
    for (const auto& path : m_selectedPaths)
    {
        AssetInfo* asset = GetAsset(path);
        if (asset != nullptr)
            asset->isSelected = false;
    }
    m_selectedPaths.Clear();
}

void AssetBrowser::SelectRange(uint32_t start, uint32_t end)
{
    DeselectAll();

    uint32_t s = Math::Min(start, end);
    uint32_t e = Math::Max(start, end);

    for (uint32_t i = s; i <= e && i < m_filteredIndices.Size(); ++i)
    {
        uint32_t assetIndex = m_filteredIndices[i];
        if (assetIndex < m_assets.Size())
        {
            m_assets[assetIndex].isSelected = true;
            m_selectedPaths.PushBack(m_assets[assetIndex].path);
        }
    }
}

void AssetBrowser::ToggleSelection(const String& path)
{
    if (IsSelected(path))
        DeselectAsset(path);
    else
    {
        m_selectedPaths.PushBack(path);
        AssetInfo* asset = GetAsset(path);
        if (asset != nullptr)
            asset->isSelected = true;
    }
}

bool AssetBrowser::IsSelected(const String& path) const
{
    for (const auto& selected : m_selectedPaths)
    {
        if (selected == path)
            return true;
    }
    return false;
}

// ============================================================================
// Favorites
// ============================================================================

void AssetBrowser::AddToFavorites(const String& path)
{
    for (const auto& fav : m_favorites)
    {
        if (fav == path)
            return;
    }

    m_favorites.PushBack(path);

    AssetInfo* asset = GetAsset(path);
    if (asset != nullptr)
        asset->isFavorite = true;
}

void AssetBrowser::RemoveFromFavorites(const String& path)
{
    for (size_type i = 0; i < m_favorites.Size(); ++i)
    {
        if (m_favorites[i] == path)
        {
            m_favorites.Erase(i);
            break;
        }
    }

    AssetInfo* asset = GetAsset(path);
    if (asset != nullptr)
        asset->isFavorite = false;
}

void AssetBrowser::ToggleFavorite(const String& path)
{
    if (IsFavorite(path))
        RemoveFromFavorites(path);
    else
        AddToFavorites(path);
}

bool AssetBrowser::IsFavorite(const String& path) const
{
    for (const auto& fav : m_favorites)
    {
        if (fav == path)
            return true;
    }
    return false;
}

// ============================================================================
// Collections
// ============================================================================

uint32_t AssetBrowser::CreateCollection(const String& name, const String& description)
{
    AssetCollection collection;
    collection.name = name;
    collection.description = description;
    collection.isSystem = false;
    collection.color = 0xFFFFFFFF;

    uint32_t index = static_cast<uint32_t>(m_collections.Size());
    m_collections.PushBack(collection);
    return index;
}

void AssetBrowser::DeleteCollection(uint32_t index)
{
    if (index < m_collections.Size())
    {
        m_collections.Erase(index);
    }
}

void AssetBrowser::AddToCollection(uint32_t collectionIndex, const String& assetPath)
{
    if (collectionIndex >= m_collections.Size())
        return;

    for (const auto& path : m_collections[collectionIndex].assetPaths)
    {
        if (path == assetPath)
            return;
    }

    m_collections[collectionIndex].assetPaths.PushBack(assetPath);
}

void AssetBrowser::RemoveFromCollection(uint32_t collectionIndex, const String& assetPath)
{
    if (collectionIndex >= m_collections.Size())
        return;

    auto& paths = m_collections[collectionIndex].assetPaths;
    for (size_type i = 0; i < paths.Size(); ++i)
    {
        if (paths[i] == assetPath)
        {
            paths.Erase(i);
            break;
        }
    }
}

// ============================================================================
// Thumbnails
// ============================================================================

void AssetBrowser::RequestThumbnail(const String& path)
{
    if (HasThumbnail(path))
        return;

    for (const auto& queued : m_thumbnailQueue)
    {
        if (queued == path)
            return;
    }

    m_thumbnailQueue.PushBack(path);
}

void AssetBrowser::CancelThumbnailRequest(const String& path)
{
    for (size_type i = 0; i < m_thumbnailQueue.Size(); ++i)
    {
        if (m_thumbnailQueue[i] == path)
        {
            m_thumbnailQueue.Erase(i);
            break;
        }
    }
}

bool AssetBrowser::HasThumbnail(const String& path) const
{
    return m_thumbnailMap.Contains(path);
}

uint32_t AssetBrowser::GetThumbnailId(const String& path) const
{
    auto it = m_thumbnailMap.Find(path);
    if (it != m_thumbnailMap.End())
        return it->value;
    return 0;
}

// ============================================================================
// Import/Export
// ============================================================================

bool AssetBrowser::ImportAsset(const String& sourcePath, const String& destPath)
{
    // TODO: Implement file copy/import
    if (m_debugEnabled)
    {
        Log::Debug("AssetBrowser: Import %s -> %s", sourcePath.CStr(), destPath.CStr());
    }
    return false;
}

bool AssetBrowser::ExportAsset(const String& assetPath, const String& exportPath)
{
    // TODO: Implement file export
    if (m_debugEnabled)
    {
        Log::Debug("AssetBrowser: Export %s -> %s", assetPath.CStr(), exportPath.CStr());
    }
    return false;
}

bool AssetBrowser::DuplicateAsset(const String& sourcePath, const String& destPath)
{
    // TODO: Implement file duplication
    if (m_debugEnabled)
    {
        Log::Debug("AssetBrowser: Duplicate %s -> %s", sourcePath.CStr(), destPath.CStr());
    }
    return false;
}

bool AssetBrowser::DeleteAsset(const String& path)
{
    // TODO: Implement file deletion
    if (m_debugEnabled)
    {
        Log::Debug("AssetBrowser: Delete %s", path.CStr());
    }
    return false;
}

bool AssetBrowser::RenameAsset(const String& oldPath, const String& newName)
{
    AssetInfo* asset = GetAsset(oldPath);
    if (asset == nullptr)
        return false;

    asset->name = newName;
    asset->displayName = newName;

    // Update index map
    m_assetIndexMap.Remove(oldPath);
    // TODO: Rebuild path with new name
    // m_assetIndexMap[newPath] = index;

    return true;
}

// ============================================================================
// Drag & Drop
// ============================================================================

void AssetBrowser::BeginDrag(const Vector<String>& assetPaths)
{
    m_draggedAssets = assetPaths;
    m_isDragging = true;
}

void AssetBrowser::EndDrag()
{
    m_draggedAssets.Clear();
    m_isDragging = false;
}

// ============================================================================
// View
// ============================================================================

void AssetBrowser::SetViewMode(AssetViewMode mode)
{
    m_viewMode = mode;
}

void AssetBrowser::SetIconSize(uint32_t size)
{
    m_iconSize = size;
}

// ============================================================================
// Events
// ============================================================================

void AssetBrowser::SetAssetSelectedCallback(AssetSelectedCallback callback, void* userData)
{
    m_onAssetSelected = callback;
    m_callbackUserData = userData;
}

void AssetBrowser::SetAssetDoubleClickedCallback(AssetDoubleClickedCallback callback, void* userData)
{
    m_onAssetDoubleClicked = callback;
    m_callbackUserData = userData;
}

void AssetBrowser::SetAssetContextMenuCallback(AssetContextMenuCallback callback, void* userData)
{
    m_onAssetContextMenu = callback;
    m_callbackUserData = userData;
}

// ============================================================================
// Asset Type Detection
// ============================================================================

AssetType AssetBrowser::DetectAssetType(const String& path) const
{
    String ext = path;
    int32_t dotIndex = ext.LastIndexOf('.');
    if (dotIndex >= 0)
    {
        ext = ext.SubString(dotIndex + 1);
    }

    if (ext == "anim" || ext == "fbx" || ext == "gltf" || ext == "glb")
        return AssetType::Animation;
    if (ext == "mmdb" || ext == "psd")
        return AssetType::PoseDatabase;
    if (ext == "clip" || ext == "motion")
        return AssetType::MotionClip;
    if (ext == "blend" || ext == "bs")
        return AssetType::BlendSpace;
    if (ext == "chooser" || ext == "ch")
        return AssetType::ChooserTable;
    if (ext == "traj" || ext == "trajectory")
        return AssetType::TrajectoryPreset;
    if (ext == "ik" || ext == "iks")
        return AssetType::IKSetup;
    if (ext == "evt" || ext == "events")
        return AssetType::EventTrack;
    if (ext == "cfg" || ext == "config" || ext == "json")
        return AssetType::ConfigFile;

    return AssetType::Unknown;
}

// ============================================================================
// Persistence
// ============================================================================

void AssetBrowser::LoadFavorites()
{
    // TODO: Load favorites from config file
}

void AssetBrowser::SaveFavorites()
{
    // TODO: Save favorites to config file
}

void AssetBrowser::LoadCollections()
{
    // TODO: Load collections from config file
}

void AssetBrowser::SaveCollections()
{
    // TODO: Save collections to config file
}

// ============================================================================
// Debug
// ============================================================================

void AssetBrowser::SetDebugEnabled(bool enabled)
{
    m_debugEnabled = enabled;
}

String AssetBrowser::GetDebugInfo() const
{
    String info;
    info += String::Format("Assets: %zu\n", m_assets.Size());
    info += String::Format("Filtered: %zu\n", m_filteredIndices.Size());
    info += String::Format("Selected: %zu\n", m_selectedPaths.Size());
    info += String::Format("Favorites: %zu\n", m_favorites.Size());
    info += String::Format("Collections: %zu\n", m_collections.Size());
    info += String::Format("Root Paths: %zu\n", m_rootPaths.Size());
    info += String::Format("View Mode: %s\n",
                           m_viewMode == AssetViewMode::Icon ? "Icon" :
                           m_viewMode == AssetViewMode::List ? "List" :
                           m_viewMode == AssetViewMode::Detail ? "Detail" : "Tree");
    info += String::Format("Sort: %s (%s)\n",
                           m_sortMode == AssetSortMode::Name ? "Name" :
                           m_sortMode == AssetSortMode::Type ? "Type" :
                           m_sortMode == AssetSortMode::Size ? "Size" :
                           m_sortMode == AssetSortMode::DateModified ? "Date" :
                           m_sortMode == AssetSortMode::DateCreated ? "Created" : "Usage",
                           m_sortAscending ? "Asc" : "Desc");
    return info;
}

MMV2_NAMESPACE_END
