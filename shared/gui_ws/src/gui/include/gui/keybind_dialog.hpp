#pragma once

#include <cstdint>
#include <QDialog>
#include <QComboBox>
#include <QLabel>

// Channel function enum — must match firmware ChannelFunction in robot_types.h
enum class ChannelFunction : uint8_t {
    NONE         = 0,
    TRACTION_FWD = 1,   // forward / back
    TRACTION_TURN = 2,  // left / right differential
    FLIPPER_ALL  = 3,   // all flippers move together
    FLIPPER_FL   = 4,   // individual front-left
    FLIPPER_FR   = 5,   // individual front-right
    FLIPPER_RL   = 6,   // individual rear-left
    FLIPPER_RR   = 7,   // individual rear-right
    ARM_FWD      = 8,   // forward to mini-PC for arm IK (legacy)
    ESTOP        = 9,   // virtual e-stop
    ARM_X        = 10,  // arm Cartesian +X (forward / back)
    ARM_Y        = 11,  // arm Cartesian +Y (lateral)
    ARM_Z        = 12,  // arm Cartesian +Z (up / down)
    ARM_PITCH    = 13,  // arm pitch rotation
    ARM_YAW      = 14,  // arm yaw rotation
    ARM_ROLL     = 15,  // arm roll rotation
    GRIPPER      = 16,  // gripper open/close
};

class KeybindDialog : public QDialog {
    Q_OBJECT
public:
    explicit KeybindDialog(QWidget* parent = nullptr);

    void reloadFromSettings();

signals:
    void keybindChanged();

private slots:
    void onApply();
    void onPresetDefault();
    void onPresetMixed();

private:
    void applyToSettings();

    static const QStringList FUNCTION_NAMES;
    static const char* MODE_NAMES[3];
    static const char* CHANNEL_NAMES[4];

    // 3 modes x 4 channel slots (Ch1-Ch4; Ch6 is dedicated ESTOP)
    QComboBox* combos_[3][4];
};
