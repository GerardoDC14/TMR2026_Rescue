#pragma once

#include <QObject>
#include <QString>
#include <QStringList>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class CameraHub;

class SourceManager : public QObject {
    Q_OBJECT
public:
    explicit SourceManager(rclcpp::Node::SharedPtr node,
                           std::shared_ptr<CameraHub> hub,
                           QObject* parent = nullptr);
    ~SourceManager() override;

    void discoverSources();

    QStringList sourceNames() const;
    QStringList sourceIdentifiers() const;

signals:
    void sourcesUpdated();

private:
    void probeLocalCameras();
    void onConfigReceived(const std_msgs::msg::String::SharedPtr msg);
    void rebuildSourceList();

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<CameraHub> camera_hub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_sub_;

    mutable std::mutex mutex_;
    QStringList source_names_;
    QStringList source_ids_;

    std::mutex local_mutex_;
    std::vector<int> local_camera_ids_;

    std::mutex config_mutex_;
    std::vector<std::string> config_topics_;
    std::vector<std::string> thermal_topics_;

    std::thread probe_thread_;
    std::atomic<bool> probing_{false};
};
