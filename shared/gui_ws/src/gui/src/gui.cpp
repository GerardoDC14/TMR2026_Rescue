#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include "gui/main_window.hpp"
#include "gui/filters.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    // Dark palette for the whole application
    QPalette dark;
    dark.setColor(QPalette::Window, QColor(30, 30, 46));
    dark.setColor(QPalette::WindowText, QColor(224, 224, 224));
    dark.setColor(QPalette::Base, QColor(25, 25, 40));
    dark.setColor(QPalette::AlternateBase, QColor(35, 35, 55));
    dark.setColor(QPalette::Text, QColor(224, 224, 224));
    dark.setColor(QPalette::Button, QColor(45, 45, 65));
    dark.setColor(QPalette::ButtonText, QColor(224, 224, 224));
    dark.setColor(QPalette::Highlight, QColor(70, 100, 180));
    dark.setColor(QPalette::HighlightedText, Qt::white);
    app.setPalette(dark);

    auto node = rclcpp::Node::make_shared("gui_node");

    registerFilters();

    int result;
    {
        MainWindow window(node);
        window.show();
        result = app.exec();
    } // window and all child ROS objects destroyed here, before shutdown

    node.reset();        // release the last node reference while context is still valid
    rclcpp::shutdown();
    shutdownFilters();   // drop CUDA session before libcudart unloads
    return result;
}
