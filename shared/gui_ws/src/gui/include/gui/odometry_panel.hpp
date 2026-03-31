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

private slots:
    void onTelemetryUpdated(float spd_l, float spd_r, float flip_angle, float uptime);
    void onFlipperExtUpdated(float fl, float fr, float rl, float rr);
    void onModeUpdated(const QString& mode);
    void onFlagsUpdated(int flags);
    void onTracksUpdated(double left_rpm, double right_rpm);

protected:
    void mousePressEvent(QMouseEvent* event) override;

private:
    void buildLayout();
    void rebuildFlipperSection();
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
    QLabel* flip_angle_;     // Jaguar single flipper
    QLabel* flip_fl_;        // Dicerox front-left
    QLabel* flip_fr_;        // Dicerox front-right
    QLabel* flip_rl_;        // Dicerox rear-left
    QLabel* flip_rr_;        // Dicerox rear-right

    // Status
    QLabel* mode_label_;
    QLabel* uptime_label_;
    QLabel* flags_label_;

    // Flipper container (swapped when robot type changes)
    QWidget* flipper_container_{nullptr};
    QVBoxLayout* main_layout_{nullptr};
    QFrame* flipper_sep_{nullptr};

    // ROS subscriptions
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr telem_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr flipper_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr flags_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr tracks_sub_;
};
