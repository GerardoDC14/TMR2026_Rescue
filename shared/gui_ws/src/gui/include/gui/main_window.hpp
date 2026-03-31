#pragma once

#include <QMainWindow>
#include <QSplitter>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <atomic>
#include <memory>
#include <thread>

class CameraHub;
class VideoPanel;
class DashboardPanel;
class DigitalTwinPanel;
class SourceManager;
class SettingsDialog;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr);
    ~MainWindow() override;

private slots:
    void onSourcesUpdated();
    void onSettingsRequested();
    void onSettingsApplied();
    void onRobotTypeChanged(int type);
    void publishKeybind();

private:
    void startRosSpinThread();

    rclcpp::Node::SharedPtr node_;

    VideoPanel* video_panel_;
    DigitalTwinPanel* digital_twin_panel_;
    DashboardPanel* dashboard_panel_;
    SourceManager* source_manager_;
    std::shared_ptr<CameraHub> camera_hub_;

    SettingsDialog* settings_dialog_{nullptr};

    // Keybind publisher
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr keybind_pub_;

    std::thread ros_thread_;
    std::atomic<bool> ros_running_{true};
};
