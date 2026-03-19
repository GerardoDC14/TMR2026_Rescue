#include "gui/main_window.hpp"
#include "gui/camera_hub.hpp"
#include "gui/video_panel.hpp"
#include "gui/digital_twin_panel.hpp"
#include "gui/dashboard_panel.hpp"
#include "gui/source_manager.hpp"
#include "gui/filter_registry.hpp"
#include "gui/app_settings.hpp"
#include "gui/settings_dialog.hpp"

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

    // Main horizontal splitter: video (left 2/3) + right panels (1/3)
    auto* main_splitter = new QSplitter(Qt::Horizontal, this);
    main_splitter->addWidget(video_panel_);
    main_splitter->addWidget(right_splitter);
    main_splitter->setSizes({1066, 534});

    setCentralWidget(main_splitter);

    // Source manager
    source_manager_ = new SourceManager(node_, camera_hub_, this);
    connect(source_manager_, &SourceManager::sourcesUpdated,
            this, &MainWindow::onSourcesUpdated);
    connect(dashboard_panel_, &DashboardPanel::resetSourcesRequested,
            source_manager_, &SourceManager::discoverSources);

    // Populate filters from registry
    QStringList filter_names;
    filter_names << "None";
    for (const auto& name : FilterRegistry::instance().getFilterNames())
        filter_names << QString::fromStdString(name);
    video_panel_->updateFilters(filter_names);

    // Load persisted settings before anything else runs
    AppSettings::instance().load();

    connect(dashboard_panel_, &DashboardPanel::settingsRequested,
            this, &MainWindow::onSettingsRequested);

    // Start ROS spin thread, then discover sources
    startRosSpinThread();
    source_manager_->discoverSources();
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
        settings_dialog_ = new SettingsDialog(this);
        connect(settings_dialog_, &SettingsDialog::settingsApplied,
                this, &MainWindow::onSettingsApplied);
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
}

void MainWindow::onSourcesUpdated()
{
    video_panel_->updateSources(
        source_manager_->sourceNames(),
        source_manager_->sourceIdentifiers());
}
