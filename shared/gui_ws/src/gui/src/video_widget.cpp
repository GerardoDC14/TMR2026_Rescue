
#include "gui/video_widget.hpp"
#include "gui/camera_hub.hpp"
#include "gui/filter_registry.hpp"
#include "gui/app_settings.hpp"

#include <QButtonGroup>
#include <QPushButton>
#include <QSlider>

#include <chrono>

// ── Shared styles for filter options ────────────────────────────────────────

static const QString TOGGLE_BTN_STYLE =
    "QPushButton { background-color: #2d2d45; color: #888; padding: 2px 8px; "
    "border: 1px solid #3a3a55; border-radius: 3px; font-size: 10px; "
    "max-height: 24px; }"
    "QPushButton:checked { background-color: #2a6a2a; color: white; "
    "border-color: #4aba4a; }"
    "QPushButton:hover { background-color: #3a3a55; }";

static const QString OPTION_CONTAINER_STYLE =
    "QLabel { color: #aaa; font-size: 10px; margin: 0; padding: 0; }"
    "QSlider { max-height: 14px; }"
    "QSlider::groove:horizontal { background: #2d2d45; height: 6px; "
    "border-radius: 2px; }"
    "QSlider::handle:horizontal { background: #4a9af5; width: 10px; "
    "margin: -3px 0; border-radius: 5px; }";

// ── Construction / destruction ──────────────────────────────────────────────

VideoWidget::VideoWidget(rclcpp::Node::SharedPtr node,
                         std::shared_ptr<CameraHub> hub,
                         QWidget* parent)
    : QWidget(parent), node_(node), camera_hub_(hub)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(2, 2, 2, 2);
    layout->setSpacing(2);

    auto* top_bar = new QHBoxLayout();
    source_combo_ = new QComboBox(this);
    filter_combo_ = new QComboBox(this);
    source_combo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    filter_combo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    source_combo_->addItem("None", "");
    filter_combo_->addItem("None");
    filter_combo_->setEnabled(false);
    filter_combo_->setStyleSheet(
        "QComboBox:disabled { color: #555; background-color: #1e1e2e; "
        "border: 1px solid #2a2a3a; }");
    top_bar->addWidget(source_combo_);
    top_bar->addWidget(filter_combo_);
    layout->addLayout(top_bar);

    display_ = new ClickableLabel(this);
    display_->setAlignment(Qt::AlignCenter);
    display_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    display_->setMinimumSize(160, 120);
    display_->setStyleSheet(
        "background-color: #000; border: 1px solid #333; color: #555;");
    display_->setText("No Source");
    layout->addWidget(display_);

    // ── Image controls (brightness / contrast) ─────────────────────────────
    // Always visible; sit above the per-filter options block.
    image_controls_container_ = new QWidget(this);
    image_controls_container_->setStyleSheet(OPTION_CONTAINER_STYLE);
    image_controls_container_->setSizePolicy(
        QSizePolicy::Preferred, QSizePolicy::Fixed);
    auto* ic = new QVBoxLayout(image_controls_container_);
    ic->setContentsMargins(2, 0, 2, 0);
    ic->setSpacing(0);

    auto build_slider_row = [this](const QString& label_text,
                                   int range_min, int range_max,
                                   int initial, int reset_value,
                                   std::function<QString(int)> fmt,
                                   std::function<void(int)> on_change) {
        auto* row = new QHBoxLayout();
        auto* lbl = new QLabel(label_text, image_controls_container_);
        lbl->setMinimumWidth(56);
        auto* sl  = new QSlider(Qt::Horizontal, image_controls_container_);
        sl->setRange(range_min, range_max);
        sl->setValue(initial);
        auto* val = new QLabel(fmt(initial), image_controls_container_);
        val->setStyleSheet("color: #4fc3f7; font-size: 11px; "
                           "min-width: 36px;");
        auto* reset = new QPushButton("⟲", image_controls_container_);
        reset->setFixedSize(18, 18);
        reset->setToolTip("Reset");
        reset->setStyleSheet(
            "QPushButton { background-color: #2d2d45; color: #aaa; "
            "border: 1px solid #3a3a55; border-radius: 3px; "
            "font-size: 10px; padding: 0; }"
            "QPushButton:hover { background-color: #3a3a55; color: white; }");
        QObject::connect(sl, &QSlider::valueChanged,
            [val, fmt, on_change](int v) {
                val->setText(fmt(v));
                on_change(v);
            });
        QObject::connect(reset, &QPushButton::clicked,
            [sl, reset_value]() { sl->setValue(reset_value); });
        row->addWidget(lbl);
        row->addWidget(sl, 1);
        row->addWidget(val);
        row->addWidget(reset);
        return row;
    };

    ic->addLayout(build_slider_row(
        "Brightness:", -100, 100, 0, 0,
        [](int v) { return (v > 0 ? "+" : "") + QString::number(v); },
        [this](int v) { brightness_.store(v); }));

    ic->addLayout(build_slider_row(
        "Contrast:", 50, 300, 100, 100,
        [](int v) { return QString::number(v / 100.0, 'f', 2) + "x"; },
        [this](int v) { contrast_x100_.store(v); }));

    layout->addWidget(image_controls_container_);

    connect(source_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &VideoWidget::onSourceChanged);
    connect(filter_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &VideoWidget::onFilterChanged);
    connect(display_, &ClickableLabel::clicked,
            this, &VideoWidget::displayClicked);
    connect(this, &VideoWidget::frameReady,
            this, &VideoWidget::onFrameReady,
            Qt::QueuedConnection);

    worker_ = std::thread(&VideoWidget::workerLoop, this);
}

VideoWidget::~VideoWidget()
{
    running_ = false;
    if (worker_.joinable())
        worker_.join();

    int id = current_local_id_.load();
    if (id >= 0 && camera_hub_)
        camera_hub_->unsubscribe(id);
}

// ── Public API ──────────────────────────────────────────────────────────────

void VideoWidget::setPaused(bool paused)
{
    paused_ = paused;
}

void VideoWidget::updateSources(const QStringList& names,
                                const QStringList& identifiers)
{
    QString current_id = source_combo_->currentData().toString();

    source_combo_->blockSignals(true);
    source_combo_->clear();
    for (int i = 0; i < names.size() && i < identifiers.size(); ++i)
        source_combo_->addItem(names[i], identifiers[i]);

    int idx = source_combo_->findData(current_id);
    source_combo_->setCurrentIndex(idx >= 0 ? idx : 0);
    source_combo_->blockSignals(false);

    if (idx < 0)
        onSourceChanged(0);
}

void VideoWidget::updateFilters(const QStringList& names)
{
    QString current = filter_combo_->currentText();

    filter_combo_->blockSignals(true);
    filter_combo_->clear();
    for (const auto& name : names)
        filter_combo_->addItem(name);

    int idx = filter_combo_->findText(current);
    filter_combo_->setCurrentIndex(idx >= 0 ? idx : 0);
    filter_combo_->blockSignals(false);
}

// ── Source / filter change ──────────────────────────────────────────────────

void VideoWidget::onSourceChanged(int index)
{
    if (index < 0) return;
    QString source = source_combo_->itemData(index).toString();

    // Unsubscribe from old local camera
    int old_id = current_local_id_.exchange(-1);
    if (old_id >= 0 && camera_hub_)
        camera_hub_->unsubscribe(old_id);

    // Tear down existing subscriptions
    {
        std::lock_guard<std::mutex> lock(sub_mutex_);
        image_sub_.reset();
        thermal_sub_.reset();
    }
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_ros_frame_ = cv::Mat();
    }

    // If we were on thermal and are switching away, notify
    if (source_type_.load() == SourceType::THERMAL && !source.startsWith("thermal:"))
        emit thermalActiveChanged(false);

    if (source.startsWith("topic:")) {
        QString topic_name = source.mid(6); 

        if (topic_name.startsWith("/camera/video")) {
            // ── INTERCEPTAR EL STREAM UDP DE LA JETSON ──
            int jetson_index = topic_name.mid(13).toInt();
            int target_cam_id = 5000 + jetson_index; // <-- Variable declarada aquí

            if (camera_hub_) {
                camera_hub_->subscribe(target_cam_id); // <-- Usamos target_cam_id
            }

            current_local_id_ = target_cam_id;         // <-- Usamos target_cam_id
            source_type_ = SourceType::LOCAL; 
            
        } else {
            // ── TÓPICOS DE IMAGEN NATIVOS DE ROS 2 ──
            std::string topic = topic_name.toStdString();
            auto sub = node_->create_subscription<sensor_msgs::msg::Image>(
                topic, rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::Image::SharedPtr msg) {
                    onImageReceived(msg);
                });
            {
                std::lock_guard<std::mutex> lock(sub_mutex_);
                image_sub_ = sub;
            }
            source_type_ = SourceType::ROS_TOPIC;
        }
    } else if (source.startsWith("local:")) {
        int cam_id = source.mid(6).toInt();
        if (camera_hub_)
            camera_hub_->subscribe(cam_id);
        current_local_id_ = cam_id;
        source_type_ = SourceType::LOCAL;
    } else if (source.startsWith("thermal:")) {
        std::string topic = source.mid(8).toStdString();
        auto sub = node_->create_subscription<sensor_msgs::msg::Image>(
            topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::SharedPtr msg) {
                onThermalReceived(msg);
            });
        {
            std::lock_guard<std::mutex> lock(sub_mutex_);
            thermal_sub_ = sub;
        }
        source_type_ = SourceType::THERMAL;
        emit thermalActiveChanged(true);
    } else {
        source_type_ = SourceType::NONE;
        emit thermalActiveChanged(false);
        display_->setPixmap(QPixmap());
        display_->setText("No Source");
    }

    // Gray out filter when no source selected
    if (source_type_.load() == SourceType::NONE) {
        filter_combo_->blockSignals(true);
        filter_combo_->setCurrentIndex(0);
        filter_combo_->blockSignals(false);
        filter_combo_->setEnabled(false);
        {
            std::lock_guard<std::mutex> lock(filter_mutex_);
            current_filter_func_ = nullptr;
            filter_config_.reset();
        }
        clearFilterOptions();
    } else {
        filter_combo_->setEnabled(true);
    }
}

void VideoWidget::onFilterChanged(int index)
{
    if (index < 0) return;
    std::string name = filter_combo_->itemText(index).toStdString();

    auto config = std::make_shared<FilterConfig>();
    std::function<cv::Mat(const cv::Mat&)> func;

    if (name != "None")
        func = FilterRegistry::instance().createFilter(name, config);

    {
        std::lock_guard<std::mutex> lock(filter_mutex_);
        current_filter_func_ = func;
        filter_config_ = config;
    }

    setupFilterOptions(name);
}

// ── Filter options UI ───────────────────────────────────────────────────────

void VideoWidget::clearFilterOptions()
{
    delete options_container_;
    options_container_ = nullptr;
}

void VideoWidget::setupFilterOptions(const std::string& name)
{
    clearFilterOptions();

    std::shared_ptr<FilterConfig> config;
    {
        std::lock_guard<std::mutex> lock(filter_mutex_);
        config = filter_config_;
    }
    if (!config || name == "None" || name.empty())
        return;

    options_container_ = new QWidget(this);
    options_container_->setStyleSheet(OPTION_CONTAINER_STYLE);
    options_container_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    auto* opts = new QVBoxLayout(options_container_);
    opts->setContentsMargins(2, 0, 2, 0);
    opts->setSpacing(0);

    if (name == "QR Code") {
        // Two toggle buttons: OpenCV (default) | zbar
        auto* row = new QHBoxLayout();
        auto* btn_cv   = new QPushButton("OpenCV", options_container_);
        auto* btn_zbar = new QPushButton("zbar",   options_container_);
        btn_cv->setCheckable(true);
        btn_zbar->setCheckable(true);
        btn_cv->setChecked(true);
        btn_cv->setStyleSheet(TOGGLE_BTN_STYLE);
        btn_zbar->setStyleSheet(TOGGLE_BTN_STYLE);

        auto* grp = new QButtonGroup(options_container_);
        grp->addButton(btn_cv,   0);
        grp->addButton(btn_zbar, 1);
        grp->setExclusive(true);

        connect(grp, QOverload<int>::of(&QButtonGroup::idClicked),
                [config](int id) { config->use_zbar.store(id == 1); });

        row->addWidget(btn_cv);
        row->addWidget(btn_zbar);
        opts->addLayout(row);

    } else if (name == "Hazmat") {
        // Confidence threshold slider
        auto* row = new QHBoxLayout();
        auto* lbl = new QLabel("Confidence:", options_container_);
        auto* slider = new QSlider(Qt::Horizontal, options_container_);
        auto* val_lbl = new QLabel("25%", options_container_);
        val_lbl->setStyleSheet("color: #4fc3f7; font-size: 11px; "
                               "min-width: 30px;");
        slider->setRange(1, 100);
        slider->setValue(25);

        connect(slider, &QSlider::valueChanged,
                [config, val_lbl](int v) {
                    config->conf_threshold_pct.store(v);
                    val_lbl->setText(QString::number(v) + "%");
                });

        row->addWidget(lbl);
        row->addWidget(slider, 1);
        row->addWidget(val_lbl);
        opts->addLayout(row);

    } else if (name == "Detect Shape") {
        // Threshold slider
        auto* r1 = new QHBoxLayout();
        auto* t_lbl = new QLabel("Threshold:", options_container_);
        auto* t_sl  = new QSlider(Qt::Horizontal, options_container_);
        auto* t_val = new QLabel("100", options_container_);
        t_val->setStyleSheet("color: #4fc3f7; font-size: 11px; "
                             "min-width: 30px;");
        t_sl->setRange(0, 255);
        t_sl->setValue(100);
        connect(t_sl, &QSlider::valueChanged,
                [config, t_val](int v) {
                    config->shape_threshold.store(v);
                    t_val->setText(QString::number(v));
                });
        r1->addWidget(t_lbl);
        r1->addWidget(t_sl, 1);
        r1->addWidget(t_val);
        opts->addLayout(r1);

        // Tolerance slider
        auto* r2 = new QHBoxLayout();
        auto* tol_lbl = new QLabel("Tolerance:", options_container_);
        auto* tol_sl  = new QSlider(Qt::Horizontal, options_container_);
        auto* tol_val = new QLabel("0.04", options_container_);
        tol_val->setStyleSheet("color: #4fc3f7; font-size: 11px; "
                               "min-width: 30px;");
        tol_sl->setRange(1, 100);
        tol_sl->setValue(4);
        connect(tol_sl, &QSlider::valueChanged,
                [config, tol_val](int v) {
                    config->shape_tolerance_pct.store(v);
                    tol_val->setText(
                        QString::number(v * 0.01, 'f', 2));
                });
        r2->addWidget(tol_lbl);
        r2->addWidget(tol_sl, 1);
        r2->addWidget(tol_val);
        opts->addLayout(r2);

        // Mode buttons
        auto* r3 = new QHBoxLayout();
        auto* btn1 = new QPushButton("Mode1", options_container_);
        auto* btn2 = new QPushButton("Mode2", options_container_);
        btn1->setCheckable(true);
        btn2->setCheckable(true);
        btn1->setChecked(true);
        btn1->setStyleSheet(TOGGLE_BTN_STYLE);
        btn2->setStyleSheet(TOGGLE_BTN_STYLE);

        auto* grp = new QButtonGroup(options_container_);
        grp->addButton(btn1, 1);
        grp->addButton(btn2, 2);
        grp->setExclusive(true);

        connect(grp, QOverload<int>::of(&QButtonGroup::idClicked),
                [config](int id) { config->shape_mode.store(id); });

        r3->addWidget(btn1);
        r3->addWidget(btn2);
        opts->addLayout(r3);

    } else if (name == "Shape 2") {
        // Threshold slider (shares shape_threshold with "Detect Shape")
        auto* r1 = new QHBoxLayout();
        auto* t_lbl = new QLabel("Threshold:", options_container_);
        auto* t_sl  = new QSlider(Qt::Horizontal, options_container_);
        auto* t_val = new QLabel("100", options_container_);
        t_val->setStyleSheet("color: #4fc3f7; font-size: 11px; "
                             "min-width: 30px;");
        t_sl->setRange(0, 255);
        t_sl->setValue(config->shape_threshold.load());
        connect(t_sl, &QSlider::valueChanged,
                [config, t_val](int v) {
                    config->shape_threshold.store(v);
                    t_val->setText(QString::number(v));
                });
        r1->addWidget(t_lbl);
        r1->addWidget(t_sl, 1);
        r1->addWidget(t_val);
        opts->addLayout(r1);

        // Tolerance slider (shares shape_tolerance_pct with "Detect Shape")
        auto* r2 = new QHBoxLayout();
        auto* tol_lbl = new QLabel("Tolerance:", options_container_);
        auto* tol_sl  = new QSlider(Qt::Horizontal, options_container_);
        auto* tol_val = new QLabel("0.04", options_container_);
        tol_val->setStyleSheet("color: #4fc3f7; font-size: 11px; "
                               "min-width: 30px;");
        tol_sl->setRange(1, 100);
        tol_sl->setValue(config->shape_tolerance_pct.load());
        tol_val->setText(QString::number(
            config->shape_tolerance_pct.load() * 0.01, 'f', 2));
        connect(tol_sl, &QSlider::valueChanged,
                [config, tol_val](int v) {
                    config->shape_tolerance_pct.store(v);
                    tol_val->setText(
                        QString::number(v * 0.01, 'f', 2));
                });
        r2->addWidget(tol_lbl);
        r2->addWidget(tol_sl, 1);
        r2->addWidget(tol_val);
        opts->addLayout(r2);

        // Corner selector: TL / TR / BL / BR
        auto* r3 = new QHBoxLayout();
        auto* b_tl = new QPushButton("TL", options_container_);
        auto* b_tr = new QPushButton("TR", options_container_);
        auto* b_bl = new QPushButton("BL", options_container_);
        auto* b_br = new QPushButton("BR", options_container_);
        for (auto* b : {b_tl, b_tr, b_bl, b_br}) {
            b->setCheckable(true);
            b->setStyleSheet(TOGGLE_BTN_STYLE);
        }
        b_tl->setChecked(true);

        auto* corner_grp = new QButtonGroup(options_container_);
        corner_grp->addButton(b_tl, 0);
        corner_grp->addButton(b_tr, 1);
        corner_grp->addButton(b_bl, 2);
        corner_grp->addButton(b_br, 3);
        corner_grp->setExclusive(true);

        connect(corner_grp, QOverload<int>::of(&QButtonGroup::idClicked),
                [config](int id) { config->shape2_corner.store(id); });

        r3->addWidget(b_tl);
        r3->addWidget(b_tr);
        r3->addWidget(b_bl);
        r3->addWidget(b_br);
        opts->addLayout(r3);

        // Mode toggle: Contour sector / HoughCircles sector
        auto* r4 = new QHBoxLayout();
        auto* b_cont = new QPushButton("Contour", options_container_);
        auto* b_circ = new QPushButton("Circle", options_container_);
        b_cont->setCheckable(true);
        b_circ->setCheckable(true);
        b_cont->setChecked(true);
        b_cont->setStyleSheet(TOGGLE_BTN_STYLE);
        b_circ->setStyleSheet(TOGGLE_BTN_STYLE);

        auto* mode_grp = new QButtonGroup(options_container_);
        mode_grp->addButton(b_cont, 0);
        mode_grp->addButton(b_circ, 1);
        mode_grp->setExclusive(true);

        connect(mode_grp, QOverload<int>::of(&QButtonGroup::idClicked),
                [config](int id) {
                    config->shape2_circle_mode.store(id == 1);
                });

        r4->addWidget(b_cont);
        r4->addWidget(b_circ);
        opts->addLayout(r4);
    }

    static_cast<QVBoxLayout*>(layout())->addWidget(options_container_);
}

// ── Frame display ───────────────────────────────────────────────────────────

void VideoWidget::onFrameReady(const QImage& image)
{
    if (source_type_.load() == SourceType::NONE)
        return;

    if (!image.isNull()) {
        display_->setPixmap(QPixmap::fromImage(image).scaled(
            display_->size(), Qt::KeepAspectRatio, Qt::FastTransformation));
    }
}

// ── ROS image callbacks ─────────────────────────────────────────────────────

void VideoWidget::onImageReceived(
    const sensor_msgs::msg::Image::SharedPtr& msg)
{
    int cv_type = CV_8UC3;
    const auto& enc = msg->encoding;
    if (enc == "mono8")
        cv_type = CV_8UC1;
    else if (enc == "rgba8" || enc == "bgra8")
        cv_type = CV_8UC4;

    cv::Mat raw(msg->height, msg->width, cv_type,
                const_cast<uint8_t*>(msg->data.data()), msg->step);

    cv::Mat bgr;
    if (enc == "rgb8")       cv::cvtColor(raw, bgr, cv::COLOR_RGB2BGR);
    else if (enc == "bgr8")  bgr = raw.clone();
    else if (enc == "mono8") cv::cvtColor(raw, bgr, cv::COLOR_GRAY2BGR);
    else if (enc == "rgba8") cv::cvtColor(raw, bgr, cv::COLOR_RGBA2BGR);
    else if (enc == "bgra8") cv::cvtColor(raw, bgr, cv::COLOR_BGRA2BGR);
    else                     bgr = raw.clone();

    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_ros_frame_ = std::move(bgr);
}

void VideoWidget::onThermalReceived(
    const sensor_msgs::msg::Image::SharedPtr& msg)
{
    int rows = static_cast<int>(msg->height);
    int cols = static_cast<int>(msg->width);
    if (rows <= 0 || cols <= 0) return;

    // Decode 32FC1: each pixel is a float32 temperature in °C
    cv::Mat temp;
    if (msg->encoding == "32FC1") {
        if (msg->data.size() < static_cast<size_t>(rows * cols * 4)) return;
        cv::Mat raw(rows, cols, CV_32FC1,
                    const_cast<uint8_t*>(msg->data.data()), msg->step);
        temp = raw.clone();
    } else {
        // Fallback: try to interpret as 8-bit grayscale
        cv::Mat raw(rows, cols, CV_8UC1,
                    const_cast<uint8_t*>(msg->data.data()), msg->step);
        raw.convertTo(temp, CV_32F);
    }

    double min_val, max_val;
    cv::minMaxLoc(temp, &min_val, &max_val);

    cv::Mat normalized;
    if (max_val > min_val) {
        temp.convertTo(normalized, CV_8U, 255.0 / (max_val - min_val),
                       -min_val * 255.0 / (max_val - min_val));
    } else {
        normalized = cv::Mat::zeros(rows, cols, CV_8U);
    }

    auto& S = AppSettings::instance();
    cv::Mat colored;
    cv::applyColorMap(normalized, colored, S.thermal_colormap.load());

    int tw = S.thermal_upscale_w.load();
    int th = S.thermal_upscale_h.load();
    if (tw <= 0 || th <= 0) {
        int scale = std::max(1, 480 / rows);
        tw = cols * scale;
        th = rows * scale;
    }
    cv::Mat upscaled;
    cv::resize(colored, upscaled, cv::Size(tw, th), 0, 0, S.thermal_interp.load());

    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_ros_frame_ = std::move(upscaled);
}

// ── Utilities ───────────────────────────────────────────────────────────────

QImage VideoWidget::matToQImage(const cv::Mat& mat)
{
    if (mat.empty()) return QImage();

    if (mat.channels() == 1) {
        return QImage(mat.data, mat.cols, mat.rows,
                      static_cast<int>(mat.step),
                      QImage::Format_Grayscale8).copy();
    }

    cv::Mat rgb;
    if (mat.channels() == 4) {
        cv::cvtColor(mat, rgb, cv::COLOR_BGRA2RGBA);
        return QImage(rgb.data, rgb.cols, rgb.rows,
                      static_cast<int>(rgb.step),
                      QImage::Format_RGBA8888).copy();
    }

    cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
    return QImage(rgb.data, rgb.cols, rgb.rows,
                  static_cast<int>(rgb.step),
                  QImage::Format_RGB888).copy();
}

// ── Worker thread ───────────────────────────────────────────────────────────

void VideoWidget::workerLoop()
{
    using clock = std::chrono::steady_clock;

    while (running_) {
        auto frame_start = clock::now();

        if (paused_.load() || source_type_.load() == SourceType::NONE) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        cv::Mat frame;
        SourceType type = source_type_.load();

        if (type == SourceType::LOCAL) {
            int cam_id = current_local_id_.load();
            if (cam_id >= 0 && camera_hub_)
                frame = camera_hub_->getLatestFrame(cam_id);
        } else if (type == SourceType::ROS_TOPIC ||
                   type == SourceType::THERMAL) {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (!latest_ros_frame_.empty())
                frame = latest_ros_frame_.clone();
        }

        if (frame.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            continue;
        }

        // Apply brightness/contrast (output = alpha * input + beta).
        // Skip when both at default to avoid the per-pixel pass cost.
        int brightness = brightness_.load();
        int contrast_x100 = contrast_x100_.load();
        if (brightness != 0 || contrast_x100 != 100) {
            double alpha = contrast_x100 / 100.0;
            cv::convertScaleAbs(frame, frame, alpha, brightness);
        }

        // Apply filter
        std::function<cv::Mat(const cv::Mat&)> filter;
        {
            std::lock_guard<std::mutex> lock(filter_mutex_);
            filter = current_filter_func_;
        }

        if (filter)
            frame = filter(frame);

        QImage qimg = matToQImage(frame);
        emit frameReady(qimg);

        auto elapsed = clock::now() - frame_start;
        auto remaining = std::chrono::milliseconds(33) - elapsed;
        if (remaining.count() > 0)
            std::this_thread::sleep_for(remaining);
    }
}
