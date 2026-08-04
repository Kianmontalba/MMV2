// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Animation Chooser System (UE5 Visual Style)
// ============================================================================
// Data structure designed for visual editing like UE5's Chooser.
// Each row has cells that can be: Any (+), True, or False.
// ============================================================================

#pragma once

#include "MMV2/Core/Config.h"
#include "MMV2/Core/Vector.h"
#include "MMV2/Core/HashMap.h"
#include "MMV2/Core/String.h"

MMV2_NAMESPACE_BEGIN

// ============================================================================
// Chooser Cell Value (Any, False, True)
// ============================================================================

enum class ChooserCellValue : uint32_t
{
    Any = 0,        // +  — matches BOTH true and false
    False = 1,      // False — only matches false
    True = 2        // True — only matches true
};

// ============================================================================
// Chooser Column Definition
// ============================================================================

struct ChooserColumnDef
{
    String name;                    // e.g., "IsGrounded", "IsMoving"
    ChooserParamType type;          // Bool, Float, Int
    float weight;                   // Importance of this column
    bool enabled;                   // Is this column active?

    ChooserColumnDef() : type(ChooserParamType::Bool), weight(1.0f), enabled(true) {}
};

// ============================================================================
// Chooser Cell (per row, per column)
// ============================================================================

struct ChooserCell
{
    ChooserCellValue boolValue;     // Any, False, True
    float floatMin;                 // For float columns
    float floatMax;
    int32_t intMin;                 // For int columns
    int32_t intMax;

    ChooserCell() : boolValue(ChooserCellValue::Any),
                    floatMin(0), floatMax(0),
                    intMin(0), intMax(0) {}
};

// ============================================================================
// Chooser Row (Animation Entry)
// ============================================================================

struct ChooserRow
{
    String name;                    // Display name
    uint32_t animationIndex;        // Result: which animation
    float rowWeight;                // Priority weight
    bool enabled;                   // Is row active?
    Vector<ChooserCell> cells;      // One cell per column

    ChooserRow() : animationIndex(0), rowWeight(1.0f), enabled(true) {}
};

// ============================================================================
// Chooser Result
// ============================================================================

struct ChooserResult
{
    uint32_t animationIndex;
    uint32_t rowIndex;
    float score;
    bool valid;

    ChooserResult() : animationIndex(0), rowIndex(0), score(0), valid(false) {}
};

// ============================================================================
// Chooser Parameter (Runtime Input)
// ============================================================================

enum class ChooserParamType : uint32_t
{
    Bool = 0,
    Float,
    Int
};

struct ChooserParameter
{
    String name;
    ChooserParamType type;
    union
    {
        bool boolValue;
        float floatValue;
        int32_t intValue;
    };

    ChooserParameter() : type(ChooserParamType::Bool), boolValue(false) {}

    static ChooserParameter MakeBool(const String& name, bool value)
    {
        ChooserParameter p; p.name = name; p.type = ChooserParamType::Bool; p.boolValue = value; return p;
    }
    static ChooserParameter MakeFloat(const String& name, float value)
    {
        ChooserParameter p; p.name = name; p.type = ChooserParamType::Float; p.floatValue = value; return p;
    }
    static ChooserParameter MakeInt(const String& name, int32_t value)
    {
        ChooserParameter p; p.name = name; p.type = ChooserParamType::Int; p.intValue = value; return p;
    }
};

// ============================================================================
// Main Chooser Table (UE5-Style)
// ============================================================================

class MMV2_API ChooserTable
{
public:
    ChooserTable();

    // === COLUMN MANAGEMENT (Visual: top row) ===
    uint32_t AddColumn(const ChooserColumnDef& def);
    void RemoveColumn(uint32_t index);
    void MoveColumn(uint32_t from, uint32_t to);
    void ClearColumns();
    uint32_t GetColumnCount() const { return static_cast<uint32_t>(m_columns.Size()); }
    const ChooserColumnDef& GetColumn(uint32_t index) const { return m_columns[index]; }
    ChooserColumnDef& GetColumn(uint32_t index) { return m_columns[index]; }

    // === ROW MANAGEMENT (Visual: left column with + button) ===
    uint32_t AddRow(const ChooserRow& row);
    void RemoveRow(uint32_t index);
    void MoveRow(uint32_t from, uint32_t to);
    void ClearRows();
    uint32_t GetRowCount() const { return static_cast<uint32_t>(m_rows.Size()); }
    const ChooserRow& GetRow(uint32_t index) const { return m_rows[index]; }
    ChooserRow& GetRow(uint32_t index) { return m_rows[index]; }

    // === CELL EDITING (Visual: clicking Any/True/False boxes) ===
    void SetCellBool(uint32_t row, uint32_t col, ChooserCellValue value);
    void SetCellFloat(uint32_t row, uint32_t col, float min, float max);
    void SetCellInt(uint32_t row, uint32_t col, int32_t min, int32_t max);
    ChooserCellValue GetCellBool(uint32_t row, uint32_t col) const;

    // === DATABASE LINK ===
    void SetDatabase(class PoseDatabase* database) { m_database = database; }
    class PoseDatabase* GetDatabase() const { return m_database; }
    void RefreshAnimationNames();  // Auto-fill row names from database

    // === EVALUATION ===
    void SetParameter(const ChooserParameter& param);
    void SetParameters(const Vector<ChooserParameter>& params);
    void ClearParameters();

    ChooserResult Evaluate() const;
    Vector<ChooserResult> EvaluateAll() const;

    // Convenience: evaluate with single param
    ChooserResult EvaluateBool(const String& name, bool value);
    ChooserResult EvaluateFloat(const String& name, float value);
    ChooserResult EvaluateInt(const String& name, int32_t value);

    // === DEBUG ===
    String GetDebugInfo() const;
    void SetDebugEnabled(bool enabled) { m_debugEnabled = enabled; }

    // === SERIALIZATION ===
    void Serialize(class BinarySerializer& serializer) const;
    void Deserialize(class BinarySerializer& serializer);

    // === VISUAL HELPERS (for UI rendering) ===
    String GetCellDisplayText(uint32_t row, uint32_t col) const;
    bool IsCellEditable(uint32_t row, uint32_t col) const;

private:
    Vector<ChooserColumnDef> m_columns;
    Vector<ChooserRow> m_rows;
    HashMap<String, ChooserParameter> m_parameters;
    class PoseDatabase* m_database;
    bool m_debugEnabled;

    float EvaluateCell(const ChooserCell& cell, const ChooserParameter& param) const;
    float EvaluateRow(uint32_t rowIndex) const;
    void EnsureCellCount(ChooserRow& row);
};

// ============================================================================
// Chooser Factory (Pre-made choosers)
// ============================================================================

class MMV2_API ChooserFactory
{
public:
    // Locomotion: IsGrounded, IsMoving, IsSprinting, Speed
    static UniquePtr<ChooserTable> CreateLocomotionChooser();

    // Combat: IsAttacking, IsBlocking, IsHit, IsDead
    static UniquePtr<ChooserTable> CreateCombatChooser();

    // Simple bool chooser
    static UniquePtr<ChooserTable> CreateBoolChooser(const String& paramName);
};

// ============================================================================
// Chooser Database (Stores multiple choosers)
// ============================================================================

class MMV2_API ChooserDatabase
{
public:
    void AddChooser(const String& name, UniquePtr<ChooserTable> chooser);
    void RemoveChooser(const String& name);
    ChooserTable* GetChooser(const String& name);
    const ChooserTable* GetChooser(const String& name) const;

    Vector<String> GetChooserNames() const;
    uint32_t GetCount() const { return static_cast<uint32_t>(m_choosers.Size()); }

    void Clear();

    void Serialize(class BinarySerializer& serializer) const;
    void Deserialize(class BinarySerializer& serializer);

private:
    HashMap<String, UniquePtr<ChooserTable>> m_choosers;
};

MMV2_NAMESPACE_END
