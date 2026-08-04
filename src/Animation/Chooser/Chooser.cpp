// ============================================================================
// MMV2 - Motion Matching Framework v2.0
// Animation Chooser Implementation (UE5 Visual Style)
// ============================================================================

#include "MMV2/Animation/Chooser/Chooser.h"
#include "MMV2/Core/Math.h"
#include "MMV2/Core/Serializer.h"
#include "MMV2/Database/Database.h"

MMV2_NAMESPACE_BEGIN

// ============================================================================
// ChooserTable
// ============================================================================

ChooserTable::ChooserTable() : m_database(nullptr), m_debugEnabled(false)
{
}

// === COLUMN MANAGEMENT ===

uint32_t ChooserTable::AddColumn(const ChooserColumnDef& def)
{
    uint32_t index = static_cast<uint32_t>(m_columns.Size());
    m_columns.PushBack(def);

    // Add empty cell to all existing rows
    for (auto& row : m_rows)
    {
        row.cells.Resize(m_columns.Size());
    }

    return index;
}

void ChooserTable::RemoveColumn(uint32_t index)
{
    if (index >= m_columns.Size()) return;

    m_columns.Erase(index);

    // Remove cell from all rows
    for (auto& row : m_rows)
    {
        if (index < row.cells.Size())
            row.cells.Erase(index);
    }
}

void ChooserTable::MoveColumn(uint32_t from, uint32_t to)
{
    if (from >= m_columns.Size() || to >= m_columns.Size()) return;

    // Swap columns
    ChooserColumnDef temp = m_columns[from];
    m_columns[from] = m_columns[to];
    m_columns[to] = temp;

    // Swap cells in all rows
    for (auto& row : m_rows)
    {
        if (from < row.cells.Size() && to < row.cells.Size())
        {
            ChooserCell cellTemp = row.cells[from];
            row.cells[from] = row.cells[to];
            row.cells[to] = cellTemp;
        }
    }
}

void ChooserTable::ClearColumns()
{
    m_columns.Clear();
    for (auto& row : m_rows)
        row.cells.Clear();
}

// === ROW MANAGEMENT ===

uint32_t ChooserTable::AddRow(const ChooserRow& row)
{
    uint32_t index = static_cast<uint32_t>(m_rows.Size());
    m_rows.PushBack(row);
    EnsureCellCount(m_rows.Back());
    return index;
}

void ChooserTable::RemoveRow(uint32_t index)
{
    if (index < m_rows.Size())
        m_rows.Erase(index);
}

void ChooserTable::MoveRow(uint32_t from, uint32_t to)
{
    if (from >= m_rows.Size() || to >= m_rows.Size()) return;

    ChooserRow temp = m_rows[from];
    m_rows[from] = m_rows[to];
    m_rows[to] = temp;
}

void ChooserTable::ClearRows()
{
    m_rows.Clear();
}

// === CELL EDITING ===

void ChooserTable::SetCellBool(uint32_t row, uint32_t col, ChooserCellValue value)
{
    if (row >= m_rows.Size()) return;
    if (col >= m_columns.Size()) return;

    EnsureCellCount(m_rows[row]);
    m_rows[row].cells[col].boolValue = value;
}

void ChooserTable::SetCellFloat(uint32_t row, uint32_t col, float min, float max)
{
    if (row >= m_rows.Size()) return;
    if (col >= m_columns.Size()) return;

    EnsureCellCount(m_rows[row]);
    m_rows[row].cells[col].floatMin = min;
    m_rows[row].cells[col].floatMax = max;
}

void ChooserTable::SetCellInt(uint32_t row, uint32_t col, int32_t min, int32_t max)
{
    if (row >= m_rows.Size()) return;
    if (col >= m_columns.Size()) return;

    EnsureCellCount(m_rows[row]);
    m_rows[row].cells[col].intMin = min;
    m_rows[row].cells[col].intMax = max;
}

ChooserCellValue ChooserTable::GetCellBool(uint32_t row, uint32_t col) const
{
    if (row >= m_rows.Size()) return ChooserCellValue::Any;
    if (col >= m_columns.Size()) return ChooserCellValue::Any;
    if (col >= m_rows[row].cells.Size()) return ChooserCellValue::Any;

    return m_rows[row].cells[col].boolValue;
}

// === DATABASE LINK ===

void ChooserTable::RefreshAnimationNames()
{
    if (!m_database) return;

    for (auto& row : m_rows)
    {
        if (row.name.Empty() && row.animationIndex < m_database->GetAnimationCount())
        {
            row.name = m_database->GetAnimationName(row.animationIndex);
        }
    }
}

// === EVALUATION ===

void ChooserTable::SetParameter(const ChooserParameter& param)
{
    m_parameters[param.name] = param;
}

void ChooserTable::SetParameters(const Vector<ChooserParameter>& params)
{
    for (const auto& p : params)
        m_parameters[p.name] = p;
}

void ChooserTable::ClearParameters()
{
    m_parameters.Clear();
}

ChooserResult ChooserTable::Evaluate() const
{
    Vector<ChooserResult> results = EvaluateAll();

    if (results.Empty())
        return ChooserResult();

    // Find best score
    uint32_t bestIdx = 0;
    float bestScore = results[0].score;

    for (uint32_t i = 1; i < results.Size(); ++i)
    {
        if (results[i].score > bestScore)
        {
            bestScore = results[i].score;
            bestIdx = i;
        }
    }

    return results[bestIdx];
}

Vector<ChooserResult> ChooserTable::EvaluateAll() const
{
    Vector<ChooserResult> results;
    results.Reserve(m_rows.Size());

    for (uint32_t i = 0; i < m_rows.Size(); ++i)
    {
        if (!m_rows[i].enabled)
            continue;

        float score = EvaluateRow(i);

        if (score > 0.0f)
        {
            ChooserResult result;
            result.animationIndex = m_rows[i].animationIndex;
            result.rowIndex = i;
            result.score = score * m_rows[i].rowWeight;
            result.valid = true;
            results.PushBack(result);
        }
    }

    return results;
}

ChooserResult ChooserTable::EvaluateBool(const String& name, bool value)
{
    SetParameter(ChooserParameter::MakeBool(name, value));
    return Evaluate();
}

ChooserResult ChooserTable::EvaluateFloat(const String& name, float value)
{
    SetParameter(ChooserParameter::MakeFloat(name, value));
    return Evaluate();
}

ChooserResult ChooserTable::EvaluateInt(const String& name, int32_t value)
{
    SetParameter(ChooserParameter::MakeInt(name, value));
    return Evaluate();
}

// === PRIVATE ===

float ChooserTable::EvaluateCell(const ChooserCell& cell, const ChooserParameter& param) const
{
    switch (param.type)
    {
        case ChooserParamType::Bool:
        {
            switch (cell.boolValue)
            {
                case ChooserCellValue::Any:
                    return 1.0f;  // + matches everything
                case ChooserCellValue::False:
                    return param.boolValue ? 0.0f : 1.0f;
                case ChooserCellValue::True:
                    return param.boolValue ? 1.0f : 0.0f;
            }
            break;
        }

        case ChooserParamType::Float:
        {
            if (param.floatValue >= cell.floatMin && param.floatValue <= cell.floatMax)
                return 1.0f;
            return 0.0f;
        }

        case ChooserParamType::Int:
        {
            if (param.intValue >= cell.intMin && param.intValue <= cell.intMax)
                return 1.0f;
            return 0.0f;
        }
    }

    return 0.0f;
}

float ChooserTable::EvaluateRow(uint32_t rowIndex) const
{
    if (rowIndex >= m_rows.Size())
        return 0.0f;

    const ChooserRow& row = m_rows[rowIndex];

    if (row.cells.Size() != m_columns.Size())
        return 0.0f;

    float totalScore = 0.0f;
    float totalWeight = 0.0f;

    for (uint32_t c = 0; c < m_columns.Size(); ++c)
    {
        if (!m_columns[c].enabled)
            continue;

        const ChooserParameter* param = nullptr;
        auto it = m_parameters.Find(m_columns[c].name);
        if (it != m_parameters.End())
            param = &it->second;

        if (!param || param->type != m_columns[c].type)
            continue;

        float cellScore = EvaluateCell(row.cells[c], *param);
        float weight = m_columns[c].weight;

        totalScore += cellScore * weight;
        totalWeight += weight;
    }

    return (totalWeight > 0.0f) ? (totalScore / totalWeight) : 0.0f;
}

void ChooserTable::EnsureCellCount(ChooserRow& row)
{
    uint32_t colCount = static_cast<uint32_t>(m_columns.Size());
    if (row.cells.Size() < colCount)
        row.cells.Resize(colCount);
}

// === DEBUG ===

String ChooserTable::GetDebugInfo() const
{
    String info;
    info += "=== ChooserTable ===
";
    info += "Columns: " + String::FromInt(static_cast<int32_t>(m_columns.Size())) + "
";
    info += "Rows: " + String::FromInt(static_cast<int32_t>(m_rows.Size())) + "

";

    // Print header
    info += "                    ";
    for (const auto& col : m_columns)
    {
        info += col.name + "  ";
    }
    info += "
";

    // Print rows
    for (uint32_t r = 0; r < m_rows.Size(); ++r)
    {
        info += m_rows[r].name;
        for (uint32_t c = 0; c < m_columns.Size() && c < m_rows[r].cells.Size(); ++c)
        {
            info += "  ";
            switch (m_rows[r].cells[c].boolValue)
            {
                case ChooserCellValue::Any: info += "[+]"; break;
                case ChooserCellValue::False: info += "[F]"; break;
                case ChooserCellValue::True: info += "[T]"; break;
            }
        }
        info += "
";
    }

    info += "
Parameters:
";
    for (const auto& pair : m_parameters)
    {
        info += "  " + pair.first + " = ";
        switch (pair.second.type)
        {
            case ChooserParamType::Bool: info += pair.second.boolValue ? "true" : "false"; break;
            case ChooserParamType::Float: info += String::FromFloat(pair.second.floatValue); break;
            case ChooserParamType::Int: info += String::FromInt(pair.second.intValue); break;
        }
        info += "
";
    }

    return info;
}

String ChooserTable::GetCellDisplayText(uint32_t row, uint32_t col) const
{
    if (row >= m_rows.Size() || col >= m_columns.Size())
        return "?";

    if (col >= m_rows[row].cells.Size())
        return "+";

    const ChooserCell& cell = m_rows[row].cells[col];

    switch (m_columns[col].type)
    {
        case ChooserParamType::Bool:
            switch (cell.boolValue)
            {
                case ChooserCellValue::Any: return "+";
                case ChooserCellValue::False: return "False";
                case ChooserCellValue::True: return "True";
            }
            break;

        case ChooserParamType::Float:
            return String::FromFloat(cell.floatMin) + " - " + String::FromFloat(cell.floatMax);

        case ChooserParamType::Int:
            return String::FromInt(cell.intMin) + " - " + String::FromInt(cell.intMax);
    }

    return "?";
}

bool ChooserTable::IsCellEditable(uint32_t row, uint32_t col) const
{
    return row < m_rows.Size() && col < m_columns.Size();
}

// === SERIALIZATION ===

void ChooserTable::Serialize(BinarySerializer& serializer) const
{
    // Columns
    serializer.Write(static_cast<uint32_t>(m_columns.Size()));
    for (const auto& col : m_columns)
    {
        serializer.Write(col.name);
        serializer.Write(static_cast<uint32_t>(col.type));
        serializer.Write(col.weight);
        serializer.Write(col.enabled);
    }

    // Rows
    serializer.Write(static_cast<uint32_t>(m_rows.Size()));
    for (const auto& row : m_rows)
    {
        serializer.Write(row.name);
        serializer.Write(row.animationIndex);
        serializer.Write(row.rowWeight);
        serializer.Write(row.enabled);

        // Cells
        serializer.Write(static_cast<uint32_t>(row.cells.Size()));
        for (const auto& cell : row.cells)
        {
            serializer.Write(static_cast<uint32_t>(cell.boolValue));
            serializer.Write(cell.floatMin);
            serializer.Write(cell.floatMax);
            serializer.Write(cell.intMin);
            serializer.Write(cell.intMax);
        }
    }
}

void ChooserTable::Deserialize(BinarySerializer& serializer)
{
    ClearColumns();
    ClearRows();

    // Columns
    uint32_t colCount;
    serializer.Read(colCount);
    m_columns.Resize(colCount);

    for (auto& col : m_columns)
    {
        serializer.Read(col.name);
        uint32_t type;
        serializer.Read(type);
        col.type = static_cast<ChooserParamType>(type);
        serializer.Read(col.weight);
        serializer.Read(col.enabled);
    }

    // Rows
    uint32_t rowCount;
    serializer.Read(rowCount);
    m_rows.Resize(rowCount);

    for (auto& row : m_rows)
    {
        serializer.Read(row.name);
        serializer.Read(row.animationIndex);
        serializer.Read(row.rowWeight);
        serializer.Read(row.enabled);

        // Cells
        uint32_t cellCount;
        serializer.Read(cellCount);
        row.cells.Resize(cellCount);

        for (auto& cell : row.cells)
        {
            uint32_t boolVal;
            serializer.Read(boolVal);
            cell.boolValue = static_cast<ChooserCellValue>(boolVal);
            serializer.Read(cell.floatMin);
            serializer.Read(cell.floatMax);
            serializer.Read(cell.intMin);
            serializer.Read(cell.intMax);
        }
    }
}

// ============================================================================
// ChooserFactory
// ============================================================================

UniquePtr<ChooserTable> ChooserFactory::CreateLocomotionChooser()
{
    auto chooser = MakeUnique<ChooserTable>();

    // Columns (top row in visual editor)
    ChooserColumnDef grounded; grounded.name = "IsGrounded"; grounded.type = ChooserParamType::Bool;
    ChooserColumnDef moving; moving.name = "IsMoving"; moving.type = ChooserParamType::Bool;
    ChooserColumnDef sprinting; sprinting.name = "IsSprinting"; sprinting.type = ChooserParamType::Bool;
    ChooserColumnDef speed; speed.name = "Speed"; speed.type = ChooserParamType::Float; speed.weight = 2.0f;

    chooser->AddColumn(grounded);
    chooser->AddColumn(moving);
    chooser->AddColumn(sprinting);
    chooser->AddColumn(speed);

    // Rows (left side with + button)
    // Row 0: Idle
    ChooserRow idle; idle.name = "Idle"; idle.animationIndex = 0;
    chooser->AddRow(idle);
    chooser->SetCellBool(0, 0, ChooserCellValue::True);   // IsGrounded = True
    chooser->SetCellBool(0, 1, ChooserCellValue::False);  // IsMoving = False
    chooser->SetCellBool(0, 2, ChooserCellValue::False);  // IsSprinting = False
    chooser->SetCellFloat(0, 3, 0.0f, 0.1f);              // Speed = 0-0.1

    // Row 1: Walk
    ChooserRow walk; walk.name = "Walk"; walk.animationIndex = 1;
    chooser->AddRow(walk);
    chooser->SetCellBool(1, 0, ChooserCellValue::True);   // IsGrounded = True
    chooser->SetCellBool(1, 1, ChooserCellValue::True);   // IsMoving = True
    chooser->SetCellBool(1, 2, ChooserCellValue::False);  // IsSprinting = False
    chooser->SetCellFloat(1, 3, 0.1f, 2.0f);              // Speed = 0.1-2.0

    // Row 2: Run
    ChooserRow run; run.name = "Run"; run.animationIndex = 2;
    chooser->AddRow(run);
    chooser->SetCellBool(2, 0, ChooserCellValue::True);   // IsGrounded = True
    chooser->SetCellBool(2, 1, ChooserCellValue::True);   // IsMoving = True
    chooser->SetCellBool(2, 2, ChooserCellValue::False);  // IsSprinting = False
    chooser->SetCellFloat(2, 3, 2.0f, 5.0f);              // Speed = 2.0-5.0

    // Row 3: Sprint
    ChooserRow sprint; sprint.name = "Sprint"; sprint.animationIndex = 3;
    chooser->AddRow(sprint);
    chooser->SetCellBool(3, 0, ChooserCellValue::True);   // IsGrounded = True
    chooser->SetCellBool(3, 1, ChooserCellValue::True);   // IsMoving = True
    chooser->SetCellBool(3, 2, ChooserCellValue::True);   // IsSprinting = True
    chooser->SetCellFloat(3, 3, 5.0f, 10.0f);             // Speed = 5.0-10.0

    // Row 4: Jump
    ChooserRow jump; jump.name = "Jump"; jump.animationIndex = 4;
    chooser->AddRow(jump);
    chooser->SetCellBool(4, 0, ChooserCellValue::False);  // IsGrounded = False
    chooser->SetCellBool(4, 1, ChooserCellValue::Any);    // IsMoving = + (any)
    chooser->SetCellBool(4, 2, ChooserCellValue::Any);    // IsSprinting = + (any)
    chooser->SetCellFloat(4, 3, 0.0f, 100.0f);            // Speed = any

    return chooser;
}

UniquePtr<ChooserTable> ChooserFactory::CreateCombatChooser()
{
    auto chooser = MakeUnique<ChooserTable>();

    ChooserColumnDef attacking; attacking.name = "IsAttacking"; attacking.type = ChooserParamType::Bool;
    ChooserColumnDef blocking; blocking.name = "IsBlocking"; blocking.type = ChooserParamType::Bool;
    ChooserColumnDef hit; hit.name = "IsHit"; hit.type = ChooserParamType::Bool;
    ChooserColumnDef dead; dead.name = "IsDead"; dead.type = ChooserParamType::Bool;

    chooser->AddColumn(attacking);
    chooser->AddColumn(blocking);
    chooser->AddColumn(hit);
    chooser->AddColumn(dead);

    // Row 0: Idle
    ChooserRow idle; idle.name = "Idle"; idle.animationIndex = 10;
    chooser->AddRow(idle);
    chooser->SetCellBool(0, 0, ChooserCellValue::False);
    chooser->SetCellBool(0, 1, ChooserCellValue::False);
    chooser->SetCellBool(0, 2, ChooserCellValue::False);
    chooser->SetCellBool(0, 3, ChooserCellValue::False);

    // Row 1: Attack
    ChooserRow attack; attack.name = "Attack"; attack.animationIndex = 11;
    chooser->AddRow(attack);
    chooser->SetCellBool(1, 0, ChooserCellValue::True);
    chooser->SetCellBool(1, 1, ChooserCellValue::False);
    chooser->SetCellBool(1, 2, ChooserCellValue::False);
    chooser->SetCellBool(1, 3, ChooserCellValue::False);

    // Row 2: Block
    ChooserRow block; block.name = "Block"; block.animationIndex = 12;
    chooser->AddRow(block);
    chooser->SetCellBool(2, 0, ChooserCellValue::False);
    chooser->SetCellBool(2, 1, ChooserCellValue::True);
    chooser->SetCellBool(2, 2, ChooserCellValue::False);
    chooser->SetCellBool(2, 3, ChooserCellValue::False);

    // Row 3: Hit
    ChooserRow hitRow; hitRow.name = "Hit"; hitRow.animationIndex = 13;
    chooser->AddRow(hitRow);
    chooser->SetCellBool(3, 0, ChooserCellValue::False);
    chooser->SetCellBool(3, 1, ChooserCellValue::False);
    chooser->SetCellBool(3, 2, ChooserCellValue::True);
    chooser->SetCellBool(3, 3, ChooserCellValue::False);

    // Row 4: Death
    ChooserRow death; death.name = "Death"; death.animationIndex = 14;
    chooser->AddRow(death);
    chooser->SetCellBool(4, 0, ChooserCellValue::Any);
    chooser->SetCellBool(4, 1, ChooserCellValue::Any);
    chooser->SetCellBool(4, 2, ChooserCellValue::Any);
    chooser->SetCellBool(4, 3, ChooserCellValue::True);

    return chooser;
}

UniquePtr<ChooserTable> ChooserFactory::CreateBoolChooser(const String& paramName)
{
    auto chooser = MakeUnique<ChooserTable>();

    ChooserColumnDef col; col.name = paramName; col.type = ChooserParamType::Bool;
    chooser->AddColumn(col);

    ChooserRow falseRow; falseRow.name = "False"; falseRow.animationIndex = 0;
    chooser->AddRow(falseRow);
    chooser->SetCellBool(0, 0, ChooserCellValue::False);

    ChooserRow trueRow; trueRow.name = "True"; trueRow.animationIndex = 1;
    chooser->AddRow(trueRow);
    chooser->SetCellBool(1, 0, ChooserCellValue::True);

    return chooser;
}

// ============================================================================
// ChooserDatabase
// ============================================================================

void ChooserDatabase::AddChooser(const String& name, UniquePtr<ChooserTable> chooser)
{
    m_choosers[name] = std::move(chooser);
}

void ChooserDatabase::RemoveChooser(const String& name)
{
    m_choosers.Erase(name);
}

ChooserTable* ChooserDatabase::GetChooser(const String& name)
{
    auto it = m_choosers.Find(name);
    return (it != m_choosers.End()) ? it->second.Get() : nullptr;
}

const ChooserTable* ChooserDatabase::GetChooser(const String& name) const
{
    auto it = m_choosers.Find(name);
    return (it != m_choosers.End()) ? it->second.Get() : nullptr;
}

Vector<String> ChooserDatabase::GetChooserNames() const
{
    Vector<String> names;
    names.Reserve(m_choosers.Size());
    for (const auto& pair : m_choosers)
        names.PushBack(pair.first);
    return names;
}

void ChooserDatabase::Clear()
{
    m_choosers.Clear();
}

void ChooserDatabase::Serialize(BinarySerializer& serializer) const
{
    serializer.Write(static_cast<uint32_t>(m_choosers.Size()));
    for (const auto& pair : m_choosers)
    {
        serializer.Write(pair.first);
        pair.second->Serialize(serializer);
    }
}

void ChooserDatabase::Deserialize(BinarySerializer& serializer)
{
    Clear();

    uint32_t count;
    serializer.Read(count);

    for (uint32_t i = 0; i < count; ++i)
    {
        String name;
        serializer.Read(name);

        auto chooser = MakeUnique<ChooserTable>();
        chooser->Deserialize(serializer);
        m_choosers[name] = std::move(chooser);
    }
}

MMV2_NAMESPACE_END
