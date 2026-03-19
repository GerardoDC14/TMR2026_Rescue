#include "gui/dashboard_panel.hpp"
#include "gui/speech_processor.hpp"
#include "gui/app_settings.hpp"

DashboardPanel::DashboardPanel(rclcpp::Node::SharedPtr node, QWidget* parent)
    : QWidget(parent), node_(node)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(10, 10, 10, 10);
    layout->setSpacing(8);

    auto hdr_style = QString("color: #aaa; font-weight: bold;");
    auto lbl_style = QString("color: #888;");
    auto val_style = QString("color: #4fc3f7; font-size: 13px;");

    // ── Connection Status ─────────────────────────────────────────────────────
    auto* conn_hdr = new QLabel("Connection Status", this);
    conn_hdr->setAlignment(Qt::AlignHCenter);
    conn_hdr->setStyleSheet(hdr_style);
    layout->addWidget(conn_hdr);

    auto* conn_row = new QHBoxLayout();
    conn_row->setSpacing(6);
    conn_indicator_ = new QLabel("●", this);
    conn_indicator_->setStyleSheet("color: #cc3333; font-size: 14px;");
    conn_label_ = new QLabel("Offline", this);
    conn_label_->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
    conn_label_->setStyleSheet(lbl_style);
    conn_row->addStretch();
    conn_row->addWidget(conn_indicator_);
    conn_row->addWidget(conn_label_);
    conn_row->addStretch();
    layout->addLayout(conn_row);

    // Smooth opacity pulse on heartbeat
    auto* opacity = new QGraphicsOpacityEffect(conn_indicator_);
    conn_indicator_->setGraphicsEffect(opacity);
    pulse_anim_ = new QPropertyAnimation(opacity, "opacity", this);
    pulse_anim_->setDuration(800);
    pulse_anim_->setKeyValueAt(0.0, 1.0);
    pulse_anim_->setKeyValueAt(0.4, 0.5);
    pulse_anim_->setKeyValueAt(1.0, 1.0);
    pulse_anim_->setEasingCurve(QEasingCurve::InOutSine);

    auto* sep2 = new QFrame(this);
    sep2->setFrameShape(QFrame::HLine);
    sep2->setStyleSheet("color: #444;");
    layout->addWidget(sep2);

    // ── Magnetometer | Gas ────────────────────────────────────────────────────
    auto make_val = [&]() {
        auto* l = new QLabel("--", this);
        l->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter); // change to AlignLeft to left-align
        l->setStyleSheet(val_style);
        return l;
    };
    auto make_axis = [&](const char* text) {
        auto* l = new QLabel(text, this);
        l->setAlignment(Qt::AlignHCenter); // change to AlignLeft to left-align
        l->setStyleSheet(lbl_style);
        return l;
    };
    auto make_vsep = [&]() {
        auto* s = new QFrame(this);
        s->setFrameShape(QFrame::VLine);
        s->setStyleSheet("color: #333;");
        return s;
    };

    auto* sensors_row = new QHBoxLayout();
    sensors_row->setSpacing(8);

    // Left: Magnetometer
    auto* mag_col = new QVBoxLayout();
    mag_col->setSpacing(3);
    auto* mag_hdr = new QLabel("Magnetometer (µT)", this);
    mag_hdr->setAlignment(Qt::AlignHCenter);
    mag_hdr->setStyleSheet(hdr_style);
    mag_col->addWidget(mag_hdr);
    mag_x_ = make_val(); mag_y_ = make_val(); mag_z_ = make_val();
    for (auto [lbl, val] : {std::pair{"X", mag_x_}, {"Y", mag_y_}, {"Z", mag_z_}}) {
        auto* row = new QHBoxLayout();
        row->addWidget(make_axis(lbl));
        row->addWidget(val, 1);
        mag_col->addLayout(row);
    }

    // Right: Gas
    auto* gas_col = new QVBoxLayout();
    gas_col->setSpacing(3);
    auto* gas_hdr = new QLabel("Gas (ppm)", this);
    gas_hdr->setAlignment(Qt::AlignHCenter);
    gas_hdr->setStyleSheet(hdr_style);
    gas_col->addWidget(gas_hdr);
    gas_value_ = make_val();
    gas_col->addWidget(gas_value_);
    gas_col->addStretch();

    sensors_row->addLayout(mag_col, 1);
    sensors_row->addWidget(make_vsep());
    sensors_row->addLayout(gas_col, 1);
    layout->addLayout(sensors_row);

    auto* sep3 = new QFrame(this);
    sep3->setFrameShape(QFrame::HLine);
    sep3->setStyleSheet("color: #444;");
    layout->addWidget(sep3);

    // ── Transcription ────────────────────────────────────────────────────────
    auto* trans_label = new QLabel("Transcription", this);
    trans_label->setAlignment(Qt::AlignHCenter);
    trans_label->setStyleSheet("color: #aaa; font-weight: bold;");
    layout->addWidget(trans_label);

    transcription_ = new QTextEdit(this);
    transcription_->setReadOnly(true);
    transcription_->setMaximumHeight(75);
    transcription_->setStyleSheet(
        "background-color: #1a1a2e; color: #c0c0c0; border: 1px solid #333; "
        "font-size: 14px; padding: 4px;");
    transcription_->setPlaceholderText("Waiting for audio...");
    layout->addWidget(transcription_);

    // ── Subscriptions ─────────────────────────────────────────────────────────
    connect(this, &DashboardPanel::magnetometerUpdated,
            this, &DashboardPanel::onMagnetometerUpdated, Qt::QueuedConnection);
    connect(this, &DashboardPanel::gasUpdated,
            this, &DashboardPanel::onGasUpdated, Qt::QueuedConnection);
    connect(this, &DashboardPanel::heartbeatReceived,
            this, &DashboardPanel::onHeartbeatReceived, Qt::QueuedConnection);

    auto sensor_qos = rclcpp::QoS(10).best_effort();

    mag_sub_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
        "/sensor/magnetometer", sensor_qos,
        [this](geometry_msgs::msg::Vector3::SharedPtr msg) {
            emit magnetometerUpdated(msg->x, msg->y, msg->z);
        });

    gas_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        "/sensor/gas", sensor_qos,
        [this](std_msgs::msg::Int32::SharedPtr msg) {
            emit gasUpdated(msg->data);
        });

    heartbeat_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
        "/heartbeat", rclcpp::QoS(10).best_effort(),
        [this](std_msgs::msg::Empty::SharedPtr) { emit heartbeatReceived(); });

    // 1s repeating timer — counts missed heartbeat intervals
    heartbeat_timer_ = new QTimer(this);
    heartbeat_timer_->setInterval(1000);
    connect(heartbeat_timer_, &QTimer::timeout, this, &DashboardPanel::onHeartbeatCheck);
    heartbeat_timer_->start();

    // ── Speech processor ─────────────────────────────────────────────────────
    speech_processor_ = new SpeechProcessor(node_, this);
    connect(speech_processor_, &SpeechProcessor::transcriptionUpdated,
            this, &DashboardPanel::onTranscriptionUpdated, Qt::QueuedConnection);

    layout->addStretch();

    auto* sep4 = new QFrame(this);
    sep4->setFrameShape(QFrame::HLine);
    sep4->setStyleSheet("color: #444;");
    layout->addWidget(sep4);

    // ── Controls ─────────────────────────────────────────────────────────────
    auto btn_style = [](const char* bg, const char* hover, const char* pressed) {
        return QString(
            "QPushButton { background-color: %1; color: white; padding: 6px; "
            "border: 1px solid %2; border-radius: 3px; }"
            "QPushButton:hover { background-color: %2; }"
            "QPushButton:pressed { background-color: %3; }")
            .arg(bg, hover, pressed);
    };

    audio_btn_ = new QPushButton("Audio: OFF", this);
    audio_btn_->setCheckable(true);
    audio_btn_->setStyleSheet(
        "QPushButton { background-color: #2a4a2a; color: white; padding: 6px; "
        "border: 1px solid #3a6a3a; border-radius: 3px; }"
        "QPushButton:hover { background-color: #3a6a3a; }"
        "QPushButton:checked { background-color: #2a8a2a; border-color: #4aba4a; }");
    connect(audio_btn_, &QPushButton::toggled, this, &DashboardPanel::onAudioToggled);
    layout->addWidget(audio_btn_);

    audio_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/audio_enable", 10);


    layout->addSpacing(4);

    estop_btn_ = new QPushButton("E-STOP", this);
    estop_btn_->setCheckable(true);
    estop_btn_->setMinimumHeight(50);
    QFont estop_font = estop_btn_->font();
    estop_font.setPointSize(16);
    estop_font.setBold(true);
    estop_btn_->setFont(estop_font);
    estop_btn_->setStyleSheet(
        "QPushButton { background-color: #5a1a1a; color: white; padding: 10px; "
        "border: 2px solid #8a2a2a; border-radius: 5px; }"
        "QPushButton:hover { background-color: #6a2a2a; }"
        "QPushButton:checked { background-color: #cc0000; border-color: #ff3333; }");
    connect(estop_btn_, &QPushButton::toggled, this, &DashboardPanel::onEstopToggled);
    layout->addWidget(estop_btn_);

    // ── Utility row (below E-STOP) ────────────────────────────────────────────
    auto* btn_row = new QHBoxLayout();
    btn_row->setSpacing(4);

    reset_btn_ = new QPushButton("Reset Sources", this);
    reset_btn_->setMinimumHeight(28);
    reset_btn_->setStyleSheet(btn_style("#2a4a7f", "#3a5a9f", "#1a3a6f"));
    connect(reset_btn_, &QPushButton::clicked, this, [this]() {
        emit resetSourcesRequested();
    });
    btn_row->addWidget(reset_btn_);

    clear_btn_ = new QPushButton("Clear Data", this);
    clear_btn_->setMinimumHeight(28);
    clear_btn_->setStyleSheet(btn_style("#4a4a2a", "#6a6a3a", "#3a3a1a"));
    connect(clear_btn_, &QPushButton::clicked, this, &DashboardPanel::onClearAll);
    btn_row->addWidget(clear_btn_);

    settings_btn_ = new QPushButton(this);
    settings_btn_->setFixedSize(28, 28);
    settings_btn_->setToolTip("Settings");
    {
        QIcon icon = QIcon::fromTheme("preferences-system");
        if (!icon.isNull()) {
            settings_btn_->setIcon(icon);
            settings_btn_->setIconSize(QSize(16, 16));
        } else {
            settings_btn_->setText("⚙");
        }
    }
    settings_btn_->setStyleSheet(
        "QPushButton { background-color: #2d2d45; color: #ccc; padding: 2px; "
        "border: 1px solid #3a3a55; border-radius: 3px; }"
        "QPushButton:hover { background-color: #3a3a55; }");
    connect(settings_btn_, &QPushButton::clicked, this, [this]() {
        emit settingsRequested();
    });
    btn_row->addWidget(settings_btn_);

    layout->addLayout(btn_row);

    auto estop_qos = rclcpp::QoS(10).reliable().transient_local();
    estop_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/estop", estop_qos);

    estop_timer_ = new QTimer(this);
    connect(estop_timer_, &QTimer::timeout, this, &DashboardPanel::publishEstopState);
    estop_timer_->start(100);
}

// ── Helpers ───────────────────────────────────────────────────────────────────

void DashboardPanel::setConnState(const QString& color, const QString& label)
{
    conn_indicator_->setStyleSheet(
        QString("color: %1; font-size: 14px;").arg(color));
    conn_label_->setText(label);
}

// ── Slots ─────────────────────────────────────────────────────────────────────

void DashboardPanel::onHeartbeatReceived()
{
    hb_received_ = true;
    conn_label_->setText("Online");
    conn_indicator_->setStyleSheet("color: #33cc33; font-size: 14px;");

    if (pulse_anim_) {
        pulse_anim_->stop();
        pulse_anim_->start();
    }
}

void DashboardPanel::onHeartbeatCheck()
{
    if (hb_received_) {
        hb_received_ = false;
        hb_miss_count_ = 0;
        // Visual already handled in onHeartbeatReceived
        return;
    }

    if (hb_miss_count_ < 100) hb_miss_count_++;

    if (hb_miss_count_ == 1) {
        setConnState("#ccaa00", "Pending");
    } else if (hb_miss_count_ == 5) {
        setConnState("#cc3333", "Offline");
    }
}

void DashboardPanel::onEstopToggled(bool checked)
{
    estop_active_ = checked;
    publishEstopState();
}

void DashboardPanel::onAudioToggled(bool checked)
{
    audio_active_ = checked;
    audio_btn_->setText(checked ? "Audio: ON" : "Audio: OFF");
    auto msg = std_msgs::msg::Bool();
    msg.data = checked;
    audio_pub_->publish(msg);
}

void DashboardPanel::onMagnetometerUpdated(double x, double y, double z)
{
    mag_x_->setText(QString::number(x, 'f', 2));
    mag_y_->setText(QString::number(y, 'f', 2));
    mag_z_->setText(QString::number(z, 'f', 2));
}

void DashboardPanel::onGasUpdated(int value)
{
    gas_value_->setText(QString::number(value));
}

void DashboardPanel::onTranscriptionUpdated(const QString& text)
{
    transcription_->setText(text);
    auto cursor = transcription_->textCursor();
    cursor.movePosition(QTextCursor::End);
    transcription_->setTextCursor(cursor);
}

void DashboardPanel::onClearAll()
{
    mag_x_->setText("--");
    mag_y_->setText("--");
    mag_z_->setText("--");
    gas_value_->setText("--");
    speech_processor_->clearTranscription();
}

void DashboardPanel::setSpeechGrammar(const std::string& words_csv)
{
    if (speech_processor_)
        speech_processor_->setGrammar(words_csv);
}

void DashboardPanel::setPlaybackEnabled(bool enabled)
{
    if (speech_processor_)
        speech_processor_->setPlaybackEnabled(enabled);
}

void DashboardPanel::publishEstopState()
{
    auto msg = std_msgs::msg::Bool();
    msg.data = estop_active_.load();
    estop_pub_->publish(msg);
}
