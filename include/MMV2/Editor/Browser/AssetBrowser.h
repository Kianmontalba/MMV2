// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Asset Browser - Editor Tool
// ============================================================================
// Provides a visual asset browser for browsing, filtering, and selecting
// animation assets, pose databases, and motion matching resources.
// Supports thumbnail previews, tag filtering, search, and drag-drop.
// ============================================================================

#pragma once
#include "MMV2/Core/Config.h"
#include "MMV2/Core/Vector.h"
#include "MMV2/Core/HashMap.h"
#include "MMV2/Core/String.h"
#include "MMV2/Metadata/Metadata.h"

MMV2_NAMESPACE_BEGIN

// ============================================================================
// Asset Browser Types
// ============================================================================

enum class AssetType : uint32_t
{
    Unknown = 0,
    Animation,
    PoseDatabase,
    MotionClip,
    BlendSpace,
    ChooserTable,
    TrajectoryPreset,
    IKSetup,
    EventTrack,
    ConfigFile,
    Folder
};

enum class AssetViewMode : uint32_t
{
    Icon = 0,
    List,
    Detail,
    Tree
};

enum class AssetSortMode : uint32_t
{
    Name = 0,
    Type,
    Size,
    DateModified,
    DateCreated,
    UsageCount
};

// ============================================================================
// Asset Info
// ============================================================================

struct AssetInfo
{
    String path;
    String name;
    String displayName;
    AssetType type;
    uint64_t fileSize;
    uint64_t dateModified;
    uint64_t dateCreated;
    uint32_t usageCount;
    uint32_t thumbnailId;
    bool isFavorite;
    bool isSelected;
    bool isVisible;
    Vector<Tag> tags;
    Metadata metadata;

    AssetInfo()
        : type(AssetType::Unknown), fileSize(0), dateModified(0),
          dateCreated(0), usageCount(0), thumbnailId(0),
          isFavorite(false), isSelected(false), isVisible(true) {}
};

// ============================================================================
// Asset Filter
// ============================================================================

struct AssetFilter
{
    String searchQuery;
    Vector<AssetType> includedTypes;
    Vector<String> includedTags;
    Vector<String> excludedTags;
    uint64_t minSize;
    uint64_t maxSize;
    uint64_t minDate;
    uint64_t maxDate;
    bool favoritesOnly;
    bool unusedOnly;

    AssetFilter()
        : minSize(0), maxSize(UINT64_MAX),
          minDate(0), maxDate(UINT64_MAX),
          favoritesOnly(false), unusedOnly(false) {}

    bool Matches(const AssetInfo& asset) const;
    void Clear();
};

// ============================================================================
// Asset Collection
// ============================================================================

struct AssetCollection
{
    String name;
    String description;
    Vector<String> assetPaths;
    bool isSystem;
    uint32_t color;

    AssetCollection() : isSystem(false), color(0xFFFFFFFF) {}
};

// ============================================================================
// Asset Browser
// ============================================================================

class MMV2_API AssetBrowser
{
public:
    AssetBrowser();
    ~AssetBrowser();

    // === Initialization ===
    void Initialize(const String& rootPath);
    void Shutdown();

    // === Asset Discovery ===
    void ScanDirectory(const String& path);
    void ScanRecursive(const String& path);
    void Refresh();
    void AddAssetPath(const String& path);
    void RemoveAssetPath(const String& path);

    // === Asset Access ===
    const AssetInfo* GetAsset(const String& path) const;
    AssetInfo* GetAsset(const String& path);
    const Vector<AssetInfo>& GetAllAssets() const { return m_assets; }
    Vector<const AssetInfo*> GetFilteredAssets() const;
    Vector<const AssetInfo*> GetSelectedAssets() const;
    Vector<const AssetInfo*> GetFavoriteAssets() const;

    // === Filtering ===
    void SetFilter(const AssetFilter& filter);
    void ClearFilter();
    void SetSearchQuery(const String& query);
    void SetTypeFilter(AssetType type, bool include);
    void SetTagFilter(const String& tag, bool include);
    void SetFavoritesOnly(bool only);

    // === Sorting ===
    void SetSortMode(AssetSortMode mode);
    void SetSortAscending(bool ascending);
    void SortAssets();

    // === Selection ===
    void SelectAsset(const String& path);
    void DeselectAsset(const String& path);
    void SelectAll();
    void DeselectAll();
    void SelectRange(uint32_t start, uint32_t end);
    void ToggleSelection(const String& path);
    bool IsSelected(const String& path) const;

    // === Favorites ===
    void AddToFavorites(const String& path);
    void RemoveFromFavorites(const String& path);
    void ToggleFavorite(const String& path);
    bool IsFavorite(const String& path) const;

    // === Collections ===
    uint32_t CreateCollection(const String& name, const String& description);
    void DeleteCollection(uint32_t index);
    void AddToCollection(uint32_t collectionIndex, const String& assetPath);
    void RemoveFromCollection(uint32_t collectionIndex, const String& assetPath);
    const Vector<AssetCollection>& GetCollections() const { return m_collections; }

    // === Thumbnails ===
    void RequestThumbnail(const String& path);
    void CancelThumbnailRequest(const String& path);
    bool HasThumbnail(const String& path) const;
    uint32_t GetThumbnailId(const String& path) const;

    // === Import/Export ===
    bool ImportAsset(const String& sourcePath, const String& destPath);
    bool ExportAsset(const String& assetPath, const String& exportPath);
    bool DuplicateAsset(const String& sourcePath, const String& destPath);
    bool DeleteAsset(const String& path);
    bool RenameAsset(const String& oldPath, const String& newName);

    // === Drag & Drop ===
    void BeginDrag(const Vector<String>& assetPaths);
    void EndDrag();
    bool IsDragging() const { return m_isDragging; }
    const Vector<String>& GetDraggedAssets() const { return m_draggedAssets; }

    // === View ===
    void SetViewMode(AssetViewMode mode);
    AssetViewMode GetViewMode() const { return m_viewMode; }
    void SetIconSize(uint32_t size);
    uint32_t GetIconSize() const { return m_iconSize; }

    // === Events ===
    using AssetSelectedCallback = void(*)(const String& path, void* userData);
    using AssetDoubleClickedCallback = void(*)(const String& path, void* userData);
    using AssetContextMenuCallback = void(*)(const String& path, void* userData);

    void SetAssetSelectedCallback(AssetSelectedCallback callback, void* userData);
    void SetAssetDoubleClickedCallback(AssetDoubleClickedCallback callback, void* userData);
    void SetAssetContextMenuCallback(AssetContextMenuCallback callback, void* userData);

    // === Debug ===
    void SetDebugEnabled(bool enabled);
    String GetDebugInfo() const;

private:
    // Asset storage
    Vector<AssetInfo> m_assets;
    HashMap<String, uint32_t> m_assetIndexMap;
    Vector<String> m_rootPaths;

    // Filtering
    AssetFilter m_filter;
    Vector<uint32_t> m_filteredIndices;
    bool m_filterDirty;

    // Sorting
    AssetSortMode m_sortMode;
    bool m_sortAscending;

    // View
    AssetViewMode m_viewMode;
    uint32_t m_iconSize;

    // Selection
    Vector<String> m_selectedPaths;

    // Favorites
    Vector<String> m_favorites;

    // Collections
    Vector<AssetCollection> m_collections;

    // Drag & Drop
    bool m_isDragging;
    Vector<String> m_draggedAssets;

    // Thumbnails
    HashMap<String, uint32_t> m_thumbnailMap;
    Vector<String> m_thumbnailQueue;

    // Callbacks
    AssetSelectedCallback m_onAssetSelected;
    AssetDoubleClickedCallback m_onAssetDoubleClicked;
    AssetContextMenuCallback m_onAssetContextMenu;
    void* m_callbackUserData;

    // State
    bool m_initialized;
    bool m_debugEnabled;

    // Internal
    void RebuildFilter();
    void UpdateFilteredIndices();
    int32_t CompareAssets(const AssetInfo& a, const AssetInfo& b) const;
    AssetType DetectAssetType(const String& path) const;
    void LoadFavorites();
    void SaveFavorites();
    void LoadCollections();
    void SaveCollections();
};

MMV2_NAMESPACE_END
