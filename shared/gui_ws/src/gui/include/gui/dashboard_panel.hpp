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
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>

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
    void gasUpdated(int value);
    void heartbeatReceived();

private slots:
    void onEstopToggled(bool checked);
    void onAudioToggled(bool checked);
    void onMagnetometerUpdated(double x, double y, double z);
    void onGasUpdated(int value);
    void onTranscriptionUpdated(const QString& text);
    void onHeartbeatReceived();
    void onHeartbeatCheck();          // fires every 1s to count missed beats
    void onClearAll();
    void publishEstopState();

private:
    void setConnState(const QString& color, const QString& label);

    rclcpp::Node::SharedPtr node_;

    // Connection status
    QLabel*              conn_indicator_;
    QLabel*              conn_label_;
    QTimer*              heartbeat_timer_;     // 1s repeating check
    QPropertyAnimation*  pulse_anim_{nullptr};
    bool                 hb_received_{false};  // did we get a beat this interval?
    int                  hb_miss_count_{3};    // start in offline state

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;

    // Magnetometer
    QLabel* mag_x_;
    QLabel* mag_y_;
    QLabel* mag_z_;

    // Gas sensor
    QLabel* gas_value_;

    // Transcription
    QTextEdit* transcription_;
    SpeechProcessor* speech_processor_;

    // Sensor subscriptions
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr mag_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gas_sub_;

    // Buttons
    QPushButton* audio_btn_;
    QPushButton* clear_btn_;
    QPushButton* reset_btn_;
    QPushButton* settings_btn_;
    QPushButton* estop_btn_;

public:
    void setSpeechGrammar(const std::string& words_csv);
    void setPlaybackEnabled(bool enabled);

    // E-STOP
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
    QTimer* estop_timer_;
    std::atomic<bool> estop_active_{false};

    // Audio enable
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr audio_pub_;
    std::atomic<bool> audio_active_{false};
};
