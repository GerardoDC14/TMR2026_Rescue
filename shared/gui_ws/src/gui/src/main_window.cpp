#include "gui/main_window.hpp"
#include "gui/camera_hub.hpp"
#include "gui/video_panel.hpp"
#include "gui/digital_twin_panel.hpp"
#include "gui/dashboard_panel.hpp"
#include "gui/source_manager.hpp"
#include "gui/filter_registry.hpp"
#include "gui/app_settings.hpp"
#include "gui/settings_dialog.hpp"
#include "gui/odometry_panel.hpp"

#include <std_msgs/msg/u_int16_multi_array.hpp>

MainWindow::MainWindow(rclcpp::Node::SharedPtr node, QWidget* parent)
    : QMainWindow(parent), node_(node)
{
    setWindowTitle("Rescue GUI 2.0");
    resize(1600, 900);

    // Shared camera hub for local video sources
    camera_hub_ = std::make_shared<CameraHub>();

    // Create panels
    video_panel_ = new VideoPanel(node_, camera_hub_, this);
    digital_twin_panel_ = new DigitalTwinPanel(node_, this);
    dashboard_panel_ = new DashboardPanel(node_, this);

    // Right side: digital twin (top) + dashboard (bottom)
    auto* right_splitter = new QSplitter(Qt::Vertical, this);
    right_splitter->addWidget(digital_twin_panel_);
    right_splitter->addWidget(dashboard_panel_);
    right_splitter->setSizes({450, 450});

    // Main horizontal splitter: video (left ~3/4) + right panels (~1/4)
    auto* main_splitter = new QSplitter(Qt::Horizontal, this);
    main_splitter->addWidget(video_panel_);
    main_splitter->addWidget(right_splitter);
    main_splitter->setSizes({1200, 400});

    setCentralWidget(main_splitter);

    // Source manager
    source_manager_ = new SourceManager(node_, camera_hub_, this);
    connect(source_manager_, &SourceManager::sourcesUpdated,
            this, &MainWindow::onSourcesUpdated);
    connect(dashboard_panel_, &DashboardPanel::resetSourcesRequested,
            source_manager_, &SourceManager::discoverSources);

    // Selecting/deselecting a thermal video source auto-enables the sensor mask bit
    connect(video_panel_, &VideoPanel::thermalActiveChanged,
            dashboard_panel_, &DashboardPanel::setThermalEnabled);

    // Populate filters from registry
    QStringList filter_names;
    filter_names << "None";
    for (const auto& name : FilterRegistry::instance().getFilterNames())
        filter_names << QString::fromStdString(name);
    video_panel_->updateFilters(filter_names);

    // Load persisted settings before anything else runs
    AppSettings::instance().load();

    // Apply initial robot type to odometry panel
    video_panel_->odometryPanel()->setRobotType(AppSettings::instance().robot_type.load());

    connect(dashboard_panel_, &DashboardPanel::settingsRequested,
            this, &MainWindow::onSettingsRequested);

    // Publishers
    keybind_pub_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "/robot/keybind", rclcpp::QoS(1).reliable().transient_local());
    ppm_calib_pub_ = node_->create_publisher<std_msgs::msg::UInt16MultiArray>(
        "/robot/ppm_calib", rclcpp::QoS(1).reliable().transient_local());

    // Start ROS spin thread, then discover sources
    startRosSpinThread();
    source_manager_->discoverSources();

    // Publish initial keybind and PPM calibration
    publishKeybind();
    publishPpmCalib();
}

MainWindow::~MainWindow()
{
    // Stop ROS spin thread first so no callbacks fire during widget destruction
    ros_running_ = false;
    if (ros_thread_.joinable())
        ros_thread_.join();
}

void MainWindow::startRosSpinThread()
{
    ros_thread_ = std::thread([this]() {
        while (ros_running_) {
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
}

void MainWindow::onSettingsRequested()
{
    if (!settings_dialog_) {
        settings_dialog_ = new SettingsDialog(node_, this);
        connect(settings_dialog_, &SettingsDialog::settingsApplied,
                this, &MainWindow::onSettingsApplied);
        connect(settings_dialog_, &SettingsDialog::robotTypeChanged,
                this, &MainWindow::onRobotTypeChanged);
    }
    settings_dialog_->reloadFromSettings();
    settings_dialog_->show();
    settings_dialog_->activateWindow();
    settings_dialog_->raise();
}

void MainWindow::onSettingsApplied()
{
    auto& S = AppSettings::instance();

    std::string grammar;
    {
        std::lock_guard<std::mutex> lk(S.strings_mutex);
        grammar = S.vosk_grammar;
    }
    dashboard_panel_->setSpeechGrammar(grammar);
    dashboard_panel_->setPlaybackEnabled(S.audio_start_enabled.load());

    // Republish keybind and PPM calibration whenever settings change
    publishKeybind();
    publishPpmCalib();
}

void MainWindow::onRobotTypeChanged(int type)
{
    video_panel_->odometryPanel()->setRobotType(type);
}

void MainWindow::publishKeybind()
{
    auto& S = AppSettings::instance();
    std_msgs::msg::UInt8MultiArray msg;
    // Flat array: 15 bytes = 3 modes × 5 channels
    msg.data.resize(15);
    {
        std::lock_guard<std::mutex> lk(S.keybind_mutex);
        for (int m = 0; m < 3; ++m)
            for (int c = 0; c < 5; ++c)
                msg.data[m * 5 + c] = S.keybind[m][c];
    }
    keybind_pub_->publish(msg);
}

void MainWindow::publishPpmCalib()
{
    auto& S = AppSettings::instance();
    std_msgs::msg::UInt16MultiArray msg;
    // 6 channels × 3 values (min, neutral, max) = 18 uint16 values
    // + 1 uint16 for global deadband × 1000 = 19 values total
    msg.data.resize(19);
    {
        std::lock_guard<std::mutex> lk(S.ppm_calib_mutex);
        for (int c = 0; c < 6; ++c) {
            msg.data[c * 3 + 0] = static_cast<uint16_t>(S.ppm_calib[c].min_us);
            msg.data[c * 3 + 1] = static_cast<uint16_t>(S.ppm_calib[c].neutral_us);
            msg.data[c * 3 + 2] = static_cast<uint16_t>(S.ppm_calib[c].max_us);
        }
    }
    msg.data[18] = static_cast<uint16_t>(S.ppm_deadband_x1000.load());
    ppm_calib_pub_->publish(msg);
}

void MainWindow::onSourcesUpdated()
{
    video_panel_->updateSources(
        source_manager_->sourceNames(),
        source_manager_->sourceIdentifiers());
}
