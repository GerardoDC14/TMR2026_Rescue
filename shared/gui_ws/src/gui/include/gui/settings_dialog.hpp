#pragma once

#include <QDialog>
#include <string>

class QCheckBox;
class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSlider;
class KeybindDialog;

class SettingsDialog : public QDialog {
    Q_OBJECT
public:
    explicit SettingsDialog(QWidget* parent = nullptr);

    // Sync UI widgets with current AppSettings (call before show()).
    void reloadFromSettings();

signals:
    void settingsApplied();
    void robotTypeChanged(int type);  // 0=Jaguar, 1=Dicerox

private slots:
    void onApply();
    void onSave();
    void onCancel();
    void onKeybindClicked();

private:
    void applyToSettings();
    void captureOriginals();
    void restoreOriginals();

    // Snapshot taken at dialog open — used by Cancel to undo any Apply
    bool        orig_audio_enabled_;
    int         orig_colormap_, orig_interp_, orig_upscale_w_, orig_upscale_h_;
    int         orig_font_scale_x100_;
    int         orig_robot_type_;
    std::string orig_grammar_;

    // Widgets
    QCheckBox* audio_check_;
    QLineEdit* grammar_edit_;
    QComboBox* colormap_combo_;
    QComboBox* interp_combo_;
    QComboBox* upscale_combo_;
    QSlider*   font_scale_slider_;
    QLabel*    font_scale_val_;
    QComboBox* robot_type_combo_;
    QPushButton* keybind_btn_;

    KeybindDialog* keybind_dialog_{nullptr};

    // Combo index ↔ cv enum value tables
    static const int COLORMAPS[5];
    static const int INTERPS[4];
    static const int UPSCALE_W[4];
    static const int UPSCALE_H[4];

    static int indexOfColormap(int cv_val);
    static int indexOfInterp(int cv_val);
    static int indexOfUpscale(int w);
};
