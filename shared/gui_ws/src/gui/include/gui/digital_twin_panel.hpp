#pragma once

#include <QWidget>
#include <QVBoxLayout>

#include <rclcpp/rclcpp.hpp>

#include "gui/urdf_viewer.hpp"

class DigitalTwinPanel : public QWidget {
    Q_OBJECT
public:
    explicit DigitalTwinPanel(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr)
        : QWidget(parent)
    {
        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(0, 0, 0, 0);
        viewer_ = new UrdfViewer(node, this);
        layout->addWidget(viewer_);
    }

private:
    UrdfViewer* viewer_;
};
