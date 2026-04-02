#include "gui/settings_dialog.hpp"
#include "gui/keybind_dialog.hpp"
#include "gui/ppm_calib_dialog.hpp"
#include "gui/app_settings.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QTimer>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSlider>
#include <QVBoxLayout>

// ── cv enum value tables ──────────────────────────────────────────────────────

const int SettingsDialog::COLORMAPS[5] = {14, 2, 11, 15, 16}; // Inferno Jet Hot Plasma Viridis
const int SettingsDialog::INTERPS[4]   = {0, 1, 2, 4};        // Nearest Linear Cubic Lanczos4
const int SettingsDialog::UPSCALE_W[4] = {0, 160, 320, 640};
const int SettingsDialog::UPSCALE_H[4] = {0, 120, 240, 480};

int SettingsDialog::indexOfColormap(int v) {
    for (int i = 0; i < 5; ++i) if (COLORMAPS[i] == v) return i;
    return 0;
}
int SettingsDialog::indexOfInterp(int v) {
    for (int i = 0; i < 4; ++i) if (INTERPS[i] == v) return i;
    return 2; // default Cubic
}
int SettingsDialog::indexOfUpscale(int w) {
    for (int i = 0; i < 4; ++i) if (UPSCALE_W[i] == w) return i;
    return 0; // default Auto
}

// ── Constructor ───────────────────────────────────────────────────────────────

SettingsDialog::SettingsDialog(rclcpp::Node::SharedPtr node, QWidget* parent)
    : QDialog(parent), node_(node)
{
    setWindowTitle("Settings");
    setMinimumWidth(380);
    setStyleSheet(
        "QDialog { background-color: #1e1e2e; }"
        "QGroupBox { color: #aaa; border: 1px solid #3a3a55; border-radius: 4px; "
        "           margin-top: 8px; padding: 8px 6px 6px 6px; font-size: 11px; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 8px; color: #4fc3f7; }"
        "QLabel { color: #aaa; font-size: 11px; }"
        "QLineEdit { background: #2d2d45; color: #e0e0e0; border: 1px solid #3a3a55; "
        "            border-radius: 3px; padding: 3px 5px; font-size: 11px; }"
        "QComboBox { background: #2d2d45; color: #e0e0e0; border: 1px solid #3a3a55; "
        "            border-radius: 3px; padding: 2px 5px; font-size: 11px; }"
        "QComboBox::drop-down { border: none; width: 16px; }"
        "QComboBox QAbstractItemView { background: #2d2d45; color: #e0e0e0; "
        "                              selection-background-color: #3a3a7a; border: 1px solid #3a3a55; }"
        "QPushButton { background-color: #2d2d45; color: #e0e0e0; padding: 5px 14px; "
        "              border: 1px solid #3a3a55; border-radius: 3px; font-size: 11px; }"
        "QPushButton:hover { background-color: #3a3a55; }"
        "QPushButton:pressed { background-color: #252535; }"
        "QSlider::groove:horizontal { background: #2d2d45; height: 5px; border-radius: 2px; }"
        "QSlider::handle:horizontal { background: #4a9af5; width: 13px; margin: -4px 0; border-radius: 6px; }"
    );

    auto* root = new QVBoxLayout(this);
    root->setSpacing(8);
    root->setContentsMargins(12, 12, 12, 12);

    auto make_form = [](QGroupBox* box) {
        auto* fl = new QFormLayout(box);
        fl->setSpacing(6);
        fl->setContentsMargins(6, 4, 6, 6);
        fl->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);
        return fl;
    };

    // ── Robot ─────────────────────────────────────────────────────────────────
    auto* robot_group = new QGroupBox("Robot", this);
    auto* rf = make_form(robot_group);

    robot_type_combo_ = new QComboBox(robot_group);
    robot_type_combo_->addItems({"Jaguar (Main)", "Dicerox (Secondary)"});
    rf->addRow("Robot:", robot_type_combo_);

    keybind_btn_ = new QPushButton("Change Keybinds...", robot_group);
    keybind_btn_->setStyleSheet(
        "QPushButton { background-color: #2a4a7f; border-color: #3a5a9f; }"
        "QPushButton:hover { background-color: #3a5a9f; }");
    connect(keybind_btn_, &QPushButton::clicked, this, &SettingsDialog::onKeybindClicked);
    rf->addRow("Controls:", keybind_btn_);

    ppm_calib_btn_ = new QPushButton("RC Calibration...", robot_group);
    ppm_calib_btn_->setStyleSheet(
        "QPushButton { background-color: #2a4a7f; border-color: #3a5a9f; }"
        "QPushButton:hover { background-color: #3a5a9f; }");
    connect(ppm_calib_btn_, &QPushButton::clicked, this, &SettingsDialog::onPpmCalibClicked);
    rf->addRow("RC Channels:", ppm_calib_btn_);

    root->addWidget(robot_group);

    // ── Audio ──────────────────────────────────────────────────────────────────
    auto* audio_group = new QGroupBox("Audio", this);
    auto* af = new QVBoxLayout(audio_group);
    af->setContentsMargins(6, 4, 6, 6);
    audio_check_ = new QCheckBox("Enable audio playback", audio_group);
    audio_check_->setStyleSheet(
        "QCheckBox { color: #ccc; font-size: 12px; spacing: 8px; }"
        "QCheckBox::indicator { width: 16px; height: 16px; }"
        "QCheckBox::indicator:unchecked { border: 1px solid #555; border-radius: 2px; background: #1e1e2e; }"
        "QCheckBox::indicator:checked   { border: 1px solid #4aba4a; border-radius: 2px; background: #2a8a2a; }");
    af->addWidget(audio_check_);
    root->addWidget(audio_group);

    // ── Speech ─────────────────────────────────────────────────────────────────
    auto* speech_group = new QGroupBox("Speech Recognition", this);
    auto* sf = make_form(speech_group);
    grammar_edit_ = new QLineEdit(speech_group);
    grammar_edit_->setPlaceholderText("Comma-separated list");
    // Defer palette: Qt stylesheet polishing overwrites it during construction.
    QTimer::singleShot(0, grammar_edit_, [this]() {
        auto pal = grammar_edit_->palette();
        pal.setColor(QPalette::PlaceholderText, QColor("#7070a0"));
        grammar_edit_->setPalette(pal);
    });
    sf->addRow("Vocabulary:", grammar_edit_);
    root->addWidget(speech_group);

    // ── Thermal ────────────────────────────────────────────────────────────────
    auto* thermal_group = new QGroupBox("Thermal Camera", this);
    auto* tf = make_form(thermal_group);

    colormap_combo_ = new QComboBox(thermal_group);
    colormap_combo_->addItems({"Inferno", "Jet", "Hot", "Plasma", "Viridis"});
    tf->addRow("Colormap:", colormap_combo_);

    interp_combo_ = new QComboBox(thermal_group);
    interp_combo_->addItems({"Nearest Neighbor", "Bilinear", "Bicubic", "Lanczos4"});
    tf->addRow("Interpolation:", interp_combo_);

    upscale_combo_ = new QComboBox(thermal_group);
    upscale_combo_->addItems({"Auto", "160x120", "320x240", "640x480"});
    tf->addRow("Upscale:", upscale_combo_);

    root->addWidget(thermal_group);

    // ── Detection labels ───────────────────────────────────────────────────────
    auto* det_group = new QGroupBox("Detection Labels", this);
    auto* df = make_form(det_group);

    auto* slider_row = new QHBoxLayout();
    font_scale_slider_ = new QSlider(Qt::Horizontal, det_group);
    font_scale_slider_->setRange(20, 150);
    font_scale_slider_->setSingleStep(5);
    font_scale_val_ = new QLabel("0.55", det_group);
    font_scale_val_->setStyleSheet("color: #4fc3f7; min-width: 30px;");
    font_scale_val_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    slider_row->addWidget(font_scale_slider_);
    slider_row->addWidget(font_scale_val_);
    connect(font_scale_slider_, &QSlider::valueChanged, [this](int v) {
        font_scale_val_->setText(QString::number(v / 100.0, 'f', 2));
    });
    df->addRow("Font size:", slider_row);

    root->addWidget(det_group);

    // ── Buttons ────────────────────────────────────────────────────────────────
    root->addStretch();
    auto* sep = new QFrame(this);
    sep->setFrameShape(QFrame::HLine);
    sep->setStyleSheet("color: #3a3a55;");
    root->addWidget(sep);

    auto* btn_row = new QHBoxLayout();
    btn_row->addStretch();

    auto* apply_btn = new QPushButton("Apply", this);
    auto* save_btn  = new QPushButton("Save Config", this);
    auto* cancel_btn = new QPushButton("Reset", this);
    apply_btn->setStyleSheet(
        "QPushButton { background-color: #1a4a8a; border-color: #2a5a9a; }"
        "QPushButton:hover { background-color: #2a5a9a; }");
    save_btn->setStyleSheet(
        "QPushButton { background-color: #1a5a2a; border-color: #2a6a3a; }"
        "QPushButton:hover { background-color: #2a6a3a; }");

    connect(apply_btn,  &QPushButton::clicked, this, &SettingsDialog::onApply);
    connect(save_btn,   &QPushButton::clicked, this, &SettingsDialog::onSave);
    connect(cancel_btn, &QPushButton::clicked, this, &SettingsDialog::onCancel);

    btn_row->addWidget(apply_btn);
    btn_row->addWidget(save_btn);
    btn_row->addWidget(cancel_btn);
    root->addLayout(btn_row);
}

// ── Sync UI ↔ AppSettings ─────────────────────────────────────────────────────

void SettingsDialog::reloadFromSettings()
{
    captureOriginals();

    auto& S = AppSettings::instance();
    audio_check_->setChecked(S.audio_start_enabled.load());
    colormap_combo_->setCurrentIndex(indexOfColormap(S.thermal_colormap.load()));
    interp_combo_->setCurrentIndex(indexOfInterp(S.thermal_interp.load()));
    upscale_combo_->setCurrentIndex(indexOfUpscale(S.thermal_upscale_w.load()));
    font_scale_slider_->setValue(S.label_font_scale_x100.load());
    robot_type_combo_->setCurrentIndex(S.robot_type.load());

    std::string grammar;
    {
        std::lock_guard<std::mutex> lk(S.strings_mutex);
        grammar = S.vosk_grammar;
    }
    grammar_edit_->setText(QString::fromStdString(grammar));
}

void SettingsDialog::captureOriginals()
{
    auto& S = AppSettings::instance();
    orig_audio_enabled_   = S.audio_start_enabled.load();
    orig_colormap_        = S.thermal_colormap.load();
    orig_interp_          = S.thermal_interp.load();
    orig_upscale_w_       = S.thermal_upscale_w.load();
    orig_upscale_h_       = S.thermal_upscale_h.load();
    orig_font_scale_x100_ = S.label_font_scale_x100.load();
    orig_robot_type_      = S.robot_type.load();
    {
        std::lock_guard<std::mutex> lk(S.strings_mutex);
        orig_grammar_ = S.vosk_grammar;
    }
}

void SettingsDialog::restoreOriginals()
{
    auto& S = AppSettings::instance();
    S.audio_start_enabled .store(orig_audio_enabled_);
    S.thermal_colormap    .store(orig_colormap_);
    S.thermal_interp      .store(orig_interp_);
    S.thermal_upscale_w   .store(orig_upscale_w_);
    S.thermal_upscale_h   .store(orig_upscale_h_);
    S.label_font_scale_x100.store(orig_font_scale_x100_);
    S.robot_type          .store(orig_robot_type_);
    {
        std::lock_guard<std::mutex> lk(S.strings_mutex);
        S.vosk_grammar = orig_grammar_;
    }
}

void SettingsDialog::applyToSettings()
{
    auto& S = AppSettings::instance();
    int ci = colormap_combo_->currentIndex();
    int ii = interp_combo_->currentIndex();
    int ui = upscale_combo_->currentIndex();

    S.audio_start_enabled .store(audio_check_->isChecked());
    S.thermal_colormap    .store(COLORMAPS[ci]);
    S.thermal_interp      .store(INTERPS[ii]);
    S.thermal_upscale_w   .store(UPSCALE_W[ui]);
    S.thermal_upscale_h   .store(UPSCALE_H[ui]);
    S.label_font_scale_x100.store(font_scale_slider_->value());

    int new_robot_type = robot_type_combo_->currentIndex();
    int old_robot_type = S.robot_type.load();
    S.robot_type.store(new_robot_type);
    if (new_robot_type != old_robot_type)
        emit robotTypeChanged(new_robot_type);


    {
        std::lock_guard<std::mutex> lk(S.strings_mutex);
        S.vosk_grammar = grammar_edit_->text().trimmed().toStdString();
    }
}

// ── Button slots ──────────────────────────────────────────────────────────────

void SettingsDialog::onApply()
{
    applyToSettings();
    emit settingsApplied();
}

void SettingsDialog::onSave()
{
    applyToSettings();
    AppSettings::instance().save();
    emit settingsApplied();
    accept();
}

void SettingsDialog::onCancel()
{
    restoreOriginals();
    reloadFromSettings();     // sync widgets back to the restored values
    emit settingsApplied();   // let MainWindow revert grammar in SpeechProcessor
}

void SettingsDialog::onKeybindClicked()
{
    if (!keybind_dialog_) {
        keybind_dialog_ = new KeybindDialog(this);
        connect(keybind_dialog_, &KeybindDialog::keybindChanged,
                this, &SettingsDialog::settingsApplied);
    }
    keybind_dialog_->reloadFromSettings();
    keybind_dialog_->show();
    keybind_dialog_->activateWindow();
    keybind_dialog_->raise();
}

void SettingsDialog::onPpmCalibClicked()
{
    if (!ppm_calib_dialog_) {
        ppm_calib_dialog_ = new PpmCalibDialog(node_, this);
        connect(ppm_calib_dialog_, &PpmCalibDialog::calibChanged,
                this, &SettingsDialog::settingsApplied);
    }
    ppm_calib_dialog_->reloadFromSettings();
    ppm_calib_dialog_->show();
    ppm_calib_dialog_->activateWindow();
    ppm_calib_dialog_->raise();
}
