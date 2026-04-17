#include "gui/dashboard_panel.hpp"
#include "gui/speech_processor.hpp"
#include "gui/app_settings.hpp"

#include <cmath>

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
    uptime_label_ = new QLabel("--", this);
    uptime_label_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    uptime_label_->setStyleSheet("color: #4fc3f7; font-size: 12px;");
    conn_row->addStretch();
    conn_row->addWidget(conn_indicator_);
    conn_row->addWidget(conn_label_);
    conn_row->addWidget(uptime_label_);
    conn_row->addStretch();
    layout->addLayout(conn_row);

    // Smooth opacity pulse on telemetry received
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

    // ── Helper lambdas ────────────────────────────────────────────────────────
    auto make_val = [&]() {
        auto* l = new QLabel("--", this);
        l->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
        l->setStyleSheet(val_style);
        return l;
    };
    auto make_axis = [&](const char* text) {
        auto* l = new QLabel(text, this);
        l->setAlignment(Qt::AlignHCenter);
        l->setStyleSheet(lbl_style);
        return l;
    };
    auto make_vsep = [&]() {
        auto* s = new QFrame(this);
        s->setFrameShape(QFrame::VLine);
        s->setStyleSheet("color: #333;");
        return s;
    };

    // Sensor toggle button factory — small inline enable/disable toggle
    auto make_sensor_toggle = [&]() {
        auto* btn = new QPushButton("OFF", this);
        btn->setCheckable(true);
        btn->setFixedSize(36, 18);
        QFont f = btn->font();
        f.setPointSize(7);
        btn->setFont(f);
        btn->setStyleSheet(
            "QPushButton { background-color: #3a2a2a; color: #888; padding: 1px; "
            "border: 1px solid #5a3a3a; border-radius: 3px; }"
            "QPushButton:hover { background-color: #4a3030; }"
            "QPushButton:checked { background-color: #1a5a1a; color: #8afa8a; "
            "border-color: #2a8a2a; }");
        return btn;
    };

    // ── Magnetometer | Gas ────────────────────────────────────────────────────
    auto* sensors_row = new QHBoxLayout();
    sensors_row->setSpacing(8);

    // Left: Magnetometer
    auto* mag_col = new QVBoxLayout();
    mag_col->setSpacing(3);
    {
        auto* mag_hdr_row = new QHBoxLayout();
        mag_hdr_row->setSpacing(4);
        auto* mag_hdr = new QLabel("Magnetometer (µT)", this);
        mag_hdr->setStyleSheet(hdr_style);
        mag_toggle_ = make_sensor_toggle();
        connect(mag_toggle_, &QPushButton::toggled, this, &DashboardPanel::onSensorToggled);
        mag_hdr_row->addWidget(mag_hdr, 1);
        mag_hdr_row->addWidget(mag_toggle_);
        mag_col->addLayout(mag_hdr_row);
    }
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
    {
        auto* gas_hdr_row = new QHBoxLayout();
        gas_hdr_row->setSpacing(4);
        auto* gas_hdr = new QLabel("Gas (Rs/Ro)", this);
        gas_hdr->setStyleSheet(hdr_style);
        gas_toggle_ = make_sensor_toggle();
        connect(gas_toggle_, &QPushButton::toggled, this, &DashboardPanel::onSensorToggled);
        gas_hdr_row->addWidget(gas_hdr, 1);
        gas_hdr_row->addWidget(gas_toggle_);
        gas_col->addLayout(gas_hdr_row);
    }
    gas_value_ = make_val();
    gas_col->addWidget(gas_value_);
    gas_col->addStretch();

    sensors_row->addLayout(mag_col, 1);
    sensors_row->addWidget(make_vsep());
    sensors_row->addLayout(gas_col, 1);
    layout->addLayout(sensors_row);

    auto* sep_imu = new QFrame(this);
    sep_imu->setFrameShape(QFrame::HLine);
    sep_imu->setStyleSheet("color: #444;");
    layout->addWidget(sep_imu);

    // ── IMU Orientation ──────────────────────────────────────────────────────
    {
        auto* imu_hdr_row = new QHBoxLayout();
        imu_hdr_row->setSpacing(4);
        auto* imu_hdr = new QLabel("Orientation", this);
        //imu_hdr->setAlignment(Qt::AlignHCenter);
        imu_hdr->setStyleSheet(hdr_style);
        imu_toggle_ = make_sensor_toggle();
        connect(imu_toggle_, &QPushButton::toggled, this, &DashboardPanel::onSensorToggled);
        imu_hdr_row->addWidget(imu_hdr);
        imu_hdr_row->addStretch();
        imu_hdr_row->addWidget(imu_toggle_);
        layout->addLayout(imu_hdr_row);
    }

    auto* imu_row = new QHBoxLayout();
    imu_row->setSpacing(8);
    imu_yaw_ = make_val(); imu_pitch_ = make_val(); imu_roll_ = make_val();
    for (auto [lbl, val] : {std::pair{"Yaw", imu_yaw_}, {"Pitch", imu_pitch_}, {"Roll", imu_roll_}}) {
        auto* col = new QVBoxLayout();
        col->setSpacing(1);
        col->addWidget(make_axis(lbl));
        col->addWidget(val);
        imu_row->addLayout(col);
    }
    layout->addLayout(imu_row);

    auto* sep3 = new QFrame(this);
    sep3->setFrameShape(QFrame::HLine);
    sep3->setStyleSheet("color: #444;");
    layout->addWidget(sep3);

    // ── Transcription ────────────────────────────────────────────────────────
    {
        auto* trans_hdr_row = new QHBoxLayout();
        trans_hdr_row->setSpacing(4);
        auto* trans_label = new QLabel("Transcription", this);
        trans_label->setStyleSheet(hdr_style);
        audio_btn_ = make_sensor_toggle();
        connect(audio_btn_, &QPushButton::toggled, this, &DashboardPanel::onAudioToggled);
        trans_hdr_row->addWidget(trans_label);
        trans_hdr_row->addStretch();
        trans_hdr_row->addWidget(audio_btn_);
        layout->addLayout(trans_hdr_row);
    }

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
    connect(this, &DashboardPanel::imuUpdated,
            this, &DashboardPanel::onImuUpdated, Qt::QueuedConnection);
    connect(this, &DashboardPanel::telemetryReceived,
            this, &DashboardPanel::onTelemetryReceived, Qt::QueuedConnection);
    connect(this, &DashboardPanel::uptimeUpdated,
            this, &DashboardPanel::onUptimeUpdated, Qt::QueuedConnection);
    connect(this, &DashboardPanel::hwEstopChanged,
            this, &DashboardPanel::onHwEstopChanged, Qt::QueuedConnection);

    auto sensor_qos = rclcpp::QoS(10).best_effort();

    // Magnetometer: /sensors/mag (sensor_msgs/MagneticField)
    mag_sub_ = node_->create_subscription<sensor_msgs::msg::MagneticField>(
        "/sensors/mag", sensor_qos,
        [this](sensor_msgs::msg::MagneticField::SharedPtr msg) {
            emit magnetometerUpdated(
                msg->magnetic_field.x,
                msg->magnetic_field.y,
                msg->magnetic_field.z);
        });

    // Gas sensor: /sensors/gas (std_msgs/Float32)
    gas_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
        "/sensors/gas", sensor_qos,
        [this](std_msgs::msg::Float32::SharedPtr msg) {
            emit gasUpdated(static_cast<double>(msg->data));
        });

    // IMU: /sensors/imu (sensor_msgs/Imu) — quaternion → euler
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "/sensors/imu", sensor_qos,
        [this](sensor_msgs::msg::Imu::SharedPtr msg) {
            double qw = msg->orientation.w;
            double qx = msg->orientation.x;
            double qy = msg->orientation.y;
            double qz = msg->orientation.z;

            double sinr = 2.0 * (qw * qx + qy * qz);
            double cosr = 1.0 - 2.0 * (qx * qx + qy * qy);
            double roll = std::atan2(sinr, cosr) * 180.0 / M_PI;

            double sinp = 2.0 * (qw * qy - qz * qx);
            double pitch;
            if (std::abs(sinp) >= 1.0)
                pitch = std::copysign(90.0, sinp);
            else
                pitch = std::asin(sinp) * 180.0 / M_PI;

            double siny = 2.0 * (qw * qz + qx * qy);
            double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
            double yaw = std::atan2(siny, cosy) * 180.0 / M_PI;

            emit imuUpdated(yaw, pitch, roll);
        });

    // Connection status + uptime: /robot/telemetry as heartbeat (50 Hz)
    telemetry_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/robot/telemetry", sensor_qos,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            emit telemetryReceived();
            if (msg->data.size() >= 4)
                emit uptimeUpdated(msg->data[3]);
        });

    // Hardware ESTOP override: subscribe to /robot/flags (bit3 = estop)
    flags_sub_ = node_->create_subscription<std_msgs::msg::UInt8>(
        "/robot/flags", sensor_qos,
        [this](std_msgs::msg::UInt8::SharedPtr msg) {
            bool fw_estop = (msg->data & 0x08) != 0;
            bool prev = hw_estop_active_.exchange(fw_estop);
            if (fw_estop != prev)
                emit hwEstopChanged(fw_estop);
        });

    // 1s repeating timer — counts missed heartbeat intervals
    heartbeat_timer_ = new QTimer(this);
    heartbeat_timer_->setInterval(1000);
    connect(heartbeat_timer_, &QTimer::timeout, this, &DashboardPanel::onHeartbeatCheck);
    heartbeat_timer_->start();

    // ── Sensor enable mask publisher ─────────────────────────────────────────
    sensor_mask_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/sensors/enable_mask", 10);
    // Publish 0x00 immediately so firmware starts with all sensors off
    publishSensorMask();

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

    audio_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/audio_enable", 10);

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

    // Solver toggle — picks between real-time DLS servo and MoveIt plan-and-execute.
    // Checked (green) = "servo"  (damped_servo takes joint trajectories from twist cmds)
    // Unchecked (gray) = "planner" (damped_servo paused; RViz / move_group plan-and-execute
    //                                can drive the arm through the JointTrajectoryController)
    solver_btn_ = new QPushButton("Solver: Servo", this);
    solver_btn_->setCheckable(true);
    solver_btn_->setChecked(true);
    solver_btn_->setMinimumHeight(28);
    solver_btn_->setToolTip(
        "Servo: real-time jogging (RC / Xbox / keyboard)\n"
        "Planner: MoveIt plan-and-execute (RViz markers)");
    solver_btn_->setStyleSheet(
        "QPushButton { background-color: #3a3a5a; color: #bbb; padding: 6px; "
        "border: 1px solid #4a4a7a; border-radius: 3px; }"
        "QPushButton:hover { background-color: #4a4a7a; }"
        "QPushButton:checked { background-color: #1a5a3a; color: #8afa8a; "
        "border-color: #2a8a5a; }");
    connect(solver_btn_, &QPushButton::toggled, this, &DashboardPanel::onSolverToggled);
    btn_row->addWidget(solver_btn_);

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

    // E-STOP publisher
    auto estop_qos = rclcpp::QoS(10).reliable().transient_local();
    estop_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/robot/estop", estop_qos);

    estop_timer_ = new QTimer(this);
    connect(estop_timer_, &QTimer::timeout, this, &DashboardPanel::publishEstopState);
    estop_timer_->start(100);

    // ── Solver-mode topic wiring ──────────────────────────────────────────────
    // Latched (transient_local) so a late-joining damped_servo picks up the
    // current GUI selection even if it was chosen before the servo came up.
    auto solver_qos = rclcpp::QoS(1).reliable().transient_local();
    solver_mode_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "/arm/solver_mode", solver_qos);
    solver_mode_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/arm/solver_mode", solver_qos,
        [this](std_msgs::msg::String::SharedPtr msg) {
            emit solverModeReceived(QString::fromStdString(msg->data));
        });
    connect(this, &DashboardPanel::solverModeReceived,
            this, &DashboardPanel::onSolverModeReceived, Qt::QueuedConnection);
}

// ── Helpers ───────────────────────────────────────────────────────────────────

void DashboardPanel::setConnState(const QString& color, const QString& label)
{
    conn_indicator_->setStyleSheet(
        QString("color: %1; font-size: 14px;").arg(color));
    conn_label_->setText(label);
}

void DashboardPanel::publishSensorMask()
{
    auto msg = std_msgs::msg::UInt8();
    msg.data = sensor_mask_;
    sensor_mask_pub_->publish(msg);
}

// ── Slots ─────────────────────────────────────────────────────────────────────

void DashboardPanel::onTelemetryReceived()
{
    hb_received_ = true;
    conn_label_->setText("Online");
    conn_indicator_->setStyleSheet("color: #33cc33; font-size: 14px;");

    if (pulse_anim_) {
        pulse_anim_->stop();
        pulse_anim_->start();
    }
}

void DashboardPanel::onUptimeUpdated(float uptime_s)
{
    int secs  = static_cast<int>(uptime_s);
    int mins  = secs / 60;
    int hours = mins / 60;
    QString text;
    if (hours > 0)
        text = QString("%1h%2m").arg(hours).arg(mins % 60);
    else if (mins > 0)
        text = QString("%1m%2s").arg(mins).arg(secs % 60);
    else
        text = QString("%1s").arg(secs);
    uptime_label_->setText(text);
}

void DashboardPanel::onHeartbeatCheck()
{
    if (hb_received_) {
        hb_received_ = false;
        hb_miss_count_ = 0;
        return;
    }

    if (hb_miss_count_ < 100) hb_miss_count_++;

    if (hb_miss_count_ == 1) {
        setConnState("#ccaa00", "Intermitent");
    } else if (hb_miss_count_ >= 2) {
        setConnState("#cc3333", "Offline");
        uptime_label_->setText("--");
    }
}

void DashboardPanel::onSensorToggled()
{
    // Preserve SENSOR_BIT_THERMAL (bit 1) — it is controlled by setThermalEnabled()
    sensor_mask_ &= (1 << 1);

    if (mag_toggle_->isChecked()) {
        sensor_mask_ |= (1 << 0);   // SENSOR_BIT_MAG
        mag_toggle_->setText("ON");
    } else {
        mag_toggle_->setText("OFF");
    }
    if (gas_toggle_->isChecked()) {
        sensor_mask_ |= (1 << 2);   // SENSOR_BIT_GAS
        gas_toggle_->setText("ON");
    } else {
        gas_toggle_->setText("OFF");
    }
    if (imu_toggle_->isChecked()) {
        sensor_mask_ |= (1 << 3);   // SENSOR_BIT_IMU
        imu_toggle_->setText("ON");
    } else {
        imu_toggle_->setText("OFF");
    }
    publishSensorMask();
}

void DashboardPanel::setThermalEnabled(bool enabled)
{
    if (enabled)
        sensor_mask_ |=  static_cast<uint8_t>(1 << 1);   // SENSOR_BIT_THERMAL
    else
        sensor_mask_ &= ~static_cast<uint8_t>(1 << 1);
    publishSensorMask();
}

void DashboardPanel::onSolverToggled(bool checked)
{
    solver_btn_->setText(checked ? "Solver: Servo" : "Solver: Planner");
    std_msgs::msg::String msg;
    msg.data = checked ? "servo" : "planner";
    solver_mode_pub_->publish(msg);
}

void DashboardPanel::onSolverModeReceived(const QString& mode)
{
    const bool is_servo = (mode.trimmed().toLower() == "servo");
    if (is_servo == solver_btn_->isChecked()) return;  // already in sync
    // Update the button without retriggering onSolverToggled (which would
    // publish right back and bounce on the topic).
    QSignalBlocker block(solver_btn_);
    solver_btn_->setChecked(is_servo);
    solver_btn_->setText(is_servo ? "Solver: Servo" : "Solver: Planner");
}

void DashboardPanel::onEstopToggled(bool checked)
{
    estop_active_ = checked;
    publishEstopState();
}

void DashboardPanel::onAudioToggled(bool checked)
{
    audio_active_ = checked;
    audio_btn_->setText(checked ? "ON" : "OFF");
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

void DashboardPanel::onGasUpdated(double value)
{
    gas_value_->setText(QString::number(value, 'f', 2));
}

void DashboardPanel::onImuUpdated(double yaw, double pitch, double roll)
{
    imu_yaw_->setText(QString::number(yaw, 'f', 1) + "°");
    imu_pitch_->setText(QString::number(pitch, 'f', 1) + "°");
    imu_roll_->setText(QString::number(roll, 'f', 1) + "°");
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
    imu_yaw_->setText("--");
    imu_pitch_->setText("--");
    imu_roll_->setText("--");
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

void DashboardPanel::onHwEstopChanged(bool active)
{
    if (active) {
        // Hardware ESTOP overrides: force button checked and disable it
        estop_btn_->blockSignals(true);
        estop_btn_->setChecked(true);
        estop_btn_->blockSignals(false);
        estop_btn_->setEnabled(false);
        estop_btn_->setText("E-STOP (HW)");
        estop_active_ = true;
    } else {
        // Hardware ESTOP cleared: re-enable button and clear
        estop_btn_->setEnabled(true);
        estop_btn_->blockSignals(true);
        estop_btn_->setChecked(false);
        estop_btn_->blockSignals(false);
        estop_btn_->setText("E-STOP");
        estop_active_ = false;
        publishEstopState();   // send clear to firmware
    }
}

void DashboardPanel::publishEstopState()
{
    auto msg = std_msgs::msg::Bool();
    msg.data = estop_active_.load();
    estop_pub_->publish(msg);
}
