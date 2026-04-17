#pragma once

#include <QWidget>
#include <string>
#include <QLabel>
#include <QPushButton>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QTimer>
#include <QFrame>
#include <QFont>
#include <QGraphicsOpacityEffect>
#include <QPropertyAnimation>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>

class SpeechProcessor;

class DashboardPanel : public QWidget {
    Q_OBJECT
public:
    explicit DashboardPanel(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr);

signals:
    void resetSourcesRequested();
    void settingsRequested();
    void magnetometerUpdated(double x, double y, double z);
    void gasUpdated(double value);
    void imuUpdated(double yaw, double pitch, double roll);
    void telemetryReceived();   // used as heartbeat
    void uptimeUpdated(float uptime_s);
    void hwEstopChanged(bool active);  // firmware reports hardware ESTOP state
    void solverModeReceived(const QString& mode);  // marshals ROS cb onto Qt thread

private slots:
    void onEstopToggled(bool checked);
    void onAudioToggled(bool checked);
    void onMagnetometerUpdated(double x, double y, double z);
    void onGasUpdated(double value);
    void onImuUpdated(double yaw, double pitch, double roll);
    void onTranscriptionUpdated(const QString& text);
    void onTelemetryReceived();
    void onHeartbeatCheck();
    void onUptimeUpdated(float uptime_s);
    void onClearAll();
    void publishEstopState();
    void onSensorToggled();
    void onHwEstopChanged(bool active);
    void onSolverToggled(bool checked);
    void onSolverModeReceived(const QString& mode);

private:
    void setConnState(const QString& color, const QString& label);
    void publishSensorMask();

    rclcpp::Node::SharedPtr node_;

    // Connection status
    QLabel*              conn_indicator_;
    QLabel*              conn_label_;
    QLabel*              uptime_label_;
    QTimer*              heartbeat_timer_;
    QPropertyAnimation*  pulse_anim_{nullptr};
    bool                 hb_received_{false};
    int                  hb_miss_count_{3};

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr telemetry_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr flags_sub_;
    std::atomic<bool> hw_estop_active_{false};

    // Magnetometer
    QLabel* mag_x_;
    QLabel* mag_y_;
    QLabel* mag_z_;

    // Gas sensor
    QLabel* gas_value_;

    // IMU orientation
    QLabel* imu_yaw_;
    QLabel* imu_pitch_;
    QLabel* imu_roll_;

    // Transcription
    QTextEdit* transcription_;
    SpeechProcessor* speech_processor_;

    // Sensor subscriptions
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gas_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // Sensor enable mask toggles (bit0=mag, bit1=thermal, bit2=gas, bit3=imu)
    // Thermal (bit1) is controlled by VideoPanel source selection, not a button.
    QPushButton* mag_toggle_;
    QPushButton* gas_toggle_;
    QPushButton* imu_toggle_;
    uint8_t      sensor_mask_{0};
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr sensor_mask_pub_;

    // Buttons
    QPushButton* audio_btn_;
    QPushButton* clear_btn_;
    QPushButton* reset_btn_;
    QPushButton* settings_btn_;
    QPushButton* estop_btn_;
    QPushButton* solver_btn_;

    // Solver mode (/arm/solver_mode: "servo" | "planner")
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    solver_mode_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr solver_mode_sub_;

public:
    void setSpeechGrammar(const std::string& words_csv);
    void setPlaybackEnabled(bool enabled);

public slots:
    // Called by VideoPanel when any widget selects / deselects the thermal source.
    // Sets or clears SENSOR_BIT_THERMAL (bit 1) and republishes the mask.
    void setThermalEnabled(bool enabled);

public:
    // E-STOP
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
    QTimer* estop_timer_;
    std::atomic<bool> estop_active_{false};

    // Audio enable
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr audio_pub_;
    std::atomic<bool> audio_active_{false};
};
