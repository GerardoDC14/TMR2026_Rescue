#pragma once

#include <QWidget>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFrame>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <atomic>
#include <array>

class OdometryPanel : public QWidget {
    Q_OBJECT
public:
    explicit OdometryPanel(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr);

    // 0 = Jaguar (2 traction + 1 flipper set), 1 = Dicerox (2 traction + 4 flippers)
    void setRobotType(int type);

signals:
    // Thread-safe signal bridge (ROS callbacks -> Qt main thread)
    void telemetryUpdated(float spd_l, float spd_r, float flip_angle, float uptime);
    void flipperExtUpdated(float fl, float fr, float rl, float rr);
    void modeUpdated(const QString& mode);
    void flagsUpdated(int flags);
    void tracksUpdated(double left_rpm, double right_rpm);
    void vescStatusUpdated(int id, float erpm, float current, float duty,
                           float temp_fet, float temp_motor, float voltage);
    void mainMotorUpdated(float left_duty, float right_duty, float flipper_duty);
    void odriveArmUpdated(int joint_idx, float iq_a, float vbus_v);

private slots:
    void onTelemetryUpdated(float spd_l, float spd_r, float flip_angle, float uptime);
    void onFlipperExtUpdated(float fl, float fr, float rl, float rr);
    void onModeUpdated(const QString& mode);
    void onFlagsUpdated(int flags);
    void onTracksUpdated(double left_rpm, double right_rpm);
    void onVescStatusUpdated(int id, float erpm, float current, float duty,
                             float temp_fet, float temp_motor, float voltage);
    void onMainMotorUpdated(float left_duty, float right_duty, float flipper_duty);
    void onOdriveArmUpdated(int joint_idx, float iq_a, float vbus_v);

private:
    void buildLayout();
    void rebuildFlipperSection();
    void updateMotorSectionVisibility();
    QLabel* makeValueLabel(const QString& initial = "--");
    QLabel* makeHeaderLabel(const QString& text);
    QLabel* makeAxisLabel(const QString& text);
    QFrame* makeHSep();

    rclcpp::Node::SharedPtr node_;
    int robot_type_{0};  // 0=Jaguar, 1=Dicerox

    // Traction
    QLabel* trac_left_rpm_;
    QLabel* trac_right_rpm_;

    // Flippers — Jaguar: only flip_angle_ used; Dicerox: fl/fr/rl/rr used
    QLabel* flip_angle_;
    QLabel* flip_fl_;
    QLabel* flip_fr_;
    QLabel* flip_rl_;
    QLabel* flip_rr_;

    // Status (uptime removed — now in dashboard panel)
    QLabel* mode_label_;
    QLabel* flags_label_;

    // Flipper container (swapped when robot type changes)
    QWidget* flipper_container_{nullptr};
    QVBoxLayout* main_layout_{nullptr};
    QFrame* flipper_sep_{nullptr};

    // ── Motor telemetry sections ─────────────────────────────────────────────

    // VESC section (secondary robot) — one row per motor ID 1-6
    QWidget* vesc_section_{nullptr};
    struct VescRow {
        QLabel* erpm;
        QLabel* current;
        QLabel* duty;
        QLabel* temp_fet;
        QLabel* temp_motor;
    };
    std::array<VescRow, 7> vesc_rows_{};   // index 0 unused; IDs 1-6

    // Main motor section (primary robot)
    QWidget* main_motor_section_{nullptr};
    QLabel* main_left_duty_{nullptr};
    QLabel* main_right_duty_{nullptr};
    QLabel* main_flip_duty_{nullptr};

    // Arm ODrive telemetry section (primary robot, J1–J3)
    QWidget* odrive_arm_section_{nullptr};
    QLabel*  arm_vbus_{nullptr};
    QLabel*  arm_iq_[3]{};

    // ROS subscriptions
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr telem_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr flipper_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr flags_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr tracks_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr vesc_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr main_motor_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr odrive_sub_;
};
