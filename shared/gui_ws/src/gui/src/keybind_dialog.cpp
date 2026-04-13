#include "gui/keybind_dialog.hpp"
#include "gui/app_settings.hpp"

#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QFrame>

const QStringList KeybindDialog::FUNCTION_NAMES = {
    "None",           // 0
    "Traction Fwd",   // 1
    "Traction Turn",  // 2
    "Flippers (all)", // 3
    "Flipper FL",     // 4
    "Flipper FR",     // 5
    "Flipper RL",     // 6
    "Flipper RR",     // 7
    "Arm (IK)",       // 8  legacy / generic
    "E-Stop",         // 9
    "Arm X",          // 10 forward / back
    "Arm Y",          // 11 lateral
    "Arm Z",          // 12 up / down
    "Arm Pitch",      // 13
    "Arm Yaw",        // 14
    "Arm Roll",       // 15
    "Gripper",        // 16
};

const char* KeybindDialog::MODE_NAMES[3] = {
    "Mode 1 (Low)",   // Ch5 ~1000us
    "Mode 2 (Mid)",   // Ch5 ~1500us
    "Mode 3 (High)",  // Ch5 ~2000us
};

const char* KeybindDialog::CHANNEL_NAMES[4] = {
    "Ch1", "Ch2", "Ch3", "Ch4"
};

KeybindDialog::KeybindDialog(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle("Keybind Configuration");
    setMinimumWidth(520);
    setStyleSheet(
        "QDialog { background-color: #1e1e2e; }"
        "QGroupBox { color: #aaa; border: 1px solid #3a3a55; border-radius: 4px; "
        "           margin-top: 8px; padding: 8px 6px 6px 6px; font-size: 11px; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 8px; color: #4fc3f7; }"
        "QLabel { color: #aaa; font-size: 11px; }"
        "QComboBox { background: #2d2d45; color: #e0e0e0; border: 1px solid #3a3a55; "
        "            border-radius: 3px; padding: 2px 5px; font-size: 11px; min-width: 90px; }"
        "QComboBox::drop-down { border: none; width: 16px; }"
        "QComboBox QAbstractItemView { background: #2d2d45; color: #e0e0e0; "
        "                              selection-background-color: #3a3a7a; border: 1px solid #3a3a55; }"
        "QPushButton { background-color: #2d2d45; color: #e0e0e0; padding: 5px 14px; "
        "              border: 1px solid #3a3a55; border-radius: 3px; font-size: 11px; }"
        "QPushButton:hover { background-color: #3a3a55; }"
    );

    auto* root = new QVBoxLayout(this);
    root->setSpacing(8);
    root->setContentsMargins(12, 12, 12, 12);

    // Explanation
    auto* info = new QLabel(
        "Ch5 is always the mode switch (3-position lever).\n"
        "Ch6 is a dedicated hardware E-Stop (not configurable).\n"
        "Configure what channels 1-4 do in each mode position.", this);
    info->setStyleSheet("color: #888; font-size: 10px;");
    info->setWordWrap(true);
    root->addWidget(info);

    // ── Grid of combos ───────────────────────────────────────────────────────
    auto* grid_group = new QGroupBox("Channel Assignments", this);
    auto* grid = new QGridLayout(grid_group);
    grid->setSpacing(4);
    grid->setContentsMargins(6, 6, 6, 6);

    // Column headers
    grid->addWidget(new QLabel("", this), 0, 0);
    for (int c = 0; c < 4; ++c) {
        auto* lbl = new QLabel(CHANNEL_NAMES[c], this);
        lbl->setAlignment(Qt::AlignHCenter);
        lbl->setStyleSheet("color: #4fc3f7; font-weight: bold;");
        grid->addWidget(lbl, 0, c + 1);
    }

    // Rows: one per mode
    for (int m = 0; m < 3; ++m) {
        auto* row_lbl = new QLabel(MODE_NAMES[m], this);
        row_lbl->setStyleSheet("color: #ccc; font-weight: bold;");
        grid->addWidget(row_lbl, m + 1, 0);

        for (int c = 0; c < 4; ++c) {
            combos_[m][c] = new QComboBox(grid_group);
            combos_[m][c]->addItems(FUNCTION_NAMES);
            grid->addWidget(combos_[m][c], m + 1, c + 1);
        }
    }

    root->addWidget(grid_group);

    // ── Presets ──────────────────────────────────────────────────────────────
    auto* preset_row = new QHBoxLayout();
    preset_row->setSpacing(6);
    auto* preset_lbl = new QLabel("Presets:", this);
    preset_lbl->setStyleSheet("color: #888;");
    preset_row->addWidget(preset_lbl);

    auto* default_btn = new QPushButton("Default (Traction/Flipper/Arm)", this);
    default_btn->setStyleSheet(
        "QPushButton { background-color: #2a4a2a; border-color: #3a6a3a; }"
        "QPushButton:hover { background-color: #3a6a3a; }");
    connect(default_btn, &QPushButton::clicked, this, &KeybindDialog::onPresetDefault);
    preset_row->addWidget(default_btn);

    auto* mixed_btn = new QPushButton("Mixed (Traction always + Flipper switch)", this);
    mixed_btn->setStyleSheet(
        "QPushButton { background-color: #4a4a2a; border-color: #6a6a3a; }"
        "QPushButton:hover { background-color: #6a6a3a; }");
    connect(mixed_btn, &QPushButton::clicked, this, &KeybindDialog::onPresetMixed);
    preset_row->addWidget(mixed_btn);

    preset_row->addStretch();
    root->addLayout(preset_row);

    // ── Buttons ──────────────────────────────────────────────────────────────
    root->addStretch();
    auto* sep = new QFrame(this);
    sep->setFrameShape(QFrame::HLine);
    sep->setStyleSheet("color: #3a3a55;");
    root->addWidget(sep);

    auto* btn_row = new QHBoxLayout();
    btn_row->addStretch();

    auto* apply_btn = new QPushButton("Apply", this);
    apply_btn->setStyleSheet(
        "QPushButton { background-color: #1a4a8a; border-color: #2a5a9a; }"
        "QPushButton:hover { background-color: #2a5a9a; }");
    connect(apply_btn, &QPushButton::clicked, this, &KeybindDialog::onApply);

    auto* close_btn = new QPushButton("Close", this);
    connect(close_btn, &QPushButton::clicked, this, &QDialog::accept);

    btn_row->addWidget(apply_btn);
    btn_row->addWidget(close_btn);
    root->addLayout(btn_row);
}

void KeybindDialog::reloadFromSettings()
{
    auto& S = AppSettings::instance();
    std::lock_guard<std::mutex> lk(S.keybind_mutex);
    for (int m = 0; m < 3; ++m)
        for (int c = 0; c < 4; ++c) {
            int val = S.keybind[m][c];
            if (val >= 0 && val < FUNCTION_NAMES.size())
                combos_[m][c]->setCurrentIndex(val);
            else
                combos_[m][c]->setCurrentIndex(0);
        }
}

void KeybindDialog::applyToSettings()
{
    auto& S = AppSettings::instance();
    std::lock_guard<std::mutex> lk(S.keybind_mutex);
    for (int m = 0; m < 3; ++m) {
        for (int c = 0; c < 4; ++c)
            S.keybind[m][c] = static_cast<uint8_t>(combos_[m][c]->currentIndex());
        S.keybind[m][4] = 0;  // Ch6 slot always NONE (dedicated ESTOP)
    }
}

void KeybindDialog::onApply()
{
    applyToSettings();
    emit keybindChanged();
}

void KeybindDialog::onPresetDefault()
{
    // Default: mode0=traction+flipper, mode1=arm movement, mode2=arm orientation
    int preset[3][4] = {
        {3, 1, 0, 2},          // mode0 (low): FLIPPER_ALL, TRACTION_FWD, NONE, TRACTION_TURN
        {11, 10, 12, 0},       // mode1 (mid): ARM_Y, ARM_X, ARM_Z, NONE
        {13, 14, 15, 0},       // mode2 (high): ARM_PITCH, ARM_YAW, ARM_ROLL, NONE
    };
    for (int m = 0; m < 3; ++m)
        for (int c = 0; c < 4; ++c)
            combos_[m][c]->setCurrentIndex(preset[m][c]);
}

void KeybindDialog::onPresetMixed()
{
    // Mixed: Ch1/Ch2 always traction, lever changes which flippers Ch3/Ch4 control
    int preset[3][4] = {
        {1, 2, 4, 5},   // mode0 (low): FWD, TURN, FL, FR
        {1, 2, 6, 7},   // mode1 (mid): FWD, TURN, RL, RR
        {1, 2, 0, 0},   // mode2 (high): FWD, TURN, NONE, NONE
    };
    for (int m = 0; m < 3; ++m)
        for (int c = 0; c < 4; ++c)
            combos_[m][c]->setCurrentIndex(preset[m][c]);
}
