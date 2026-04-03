#pragma once

#include <QDialog>
#include <QVector>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

class QLabel;
class QProgressBar;
class QSpinBox;
class QPushButton;

// ─── PpmCalibDialog ───────────────────────────────────────────────────────────
// Per-channel RC PPM calibration window.
//
// Shows each channel's live raw µs value from /robot/ppm and lets the user set
// per-channel min/neutral/max limits that are saved to AppSettings and
// published to the firmware via calibChanged → SettingsDialog::settingsApplied.
//
// Layout per channel row:
//   [Ch#]  [bar — live position relative to calibrated range]  [µsValue]
//          Min: [spin]   Ctr: [spin]   Max: [spin]   [◎ Capture Center]

class PpmCalibDialog : public QDialog {
    Q_OBJECT
public:
    explicit PpmCalibDialog(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr);
    ~PpmCalibDialog() override;

    // Sync spinboxes with current AppSettings (call before show()).
    void reloadFromSettings();

signals:
    void calibChanged();        // emitted after Apply; triggers MainWindow to republish

    // Internal signal used to marshal the ROS callback onto the GUI thread.
    void ppmReceived(QVector<int> values);

private slots:
    void onApply();
    void onPpmReceived(QVector<int> values);
    void onCaptureCenter(int ch);

private:
    void applyToSettings();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr ppm_sub_;

    static const char* CH_NAMES[6];

    // Per-channel widgets (index 0 = Ch1 … 5 = Ch6)
    QLabel*       val_labels_[6];
    QProgressBar* bars_[6];
    QSpinBox*     min_spins_[6];
    QSpinBox*     neutral_spins_[6];
    QSpinBox*     max_spins_[6];

    int live_us_[6] = {};   // latest raw values — used by "Capture Center"
};
