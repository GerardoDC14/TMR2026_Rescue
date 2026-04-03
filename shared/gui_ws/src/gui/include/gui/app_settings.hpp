#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

// Global application settings — persisted to ~/.config/robocup_gui/settings.json.
// Atomic members are safe to read from worker threads (thermal, filter).
// vosk_grammar must be accessed under strings_mutex.
struct AppSettings {
    static AppSettings& instance() {
        static AppSettings s;
        return s;
    }

    void load();
    void save();

    // Thermal camera
    // colormap: 14=Inferno, 2=Jet, 11=Hot, 15=Plasma, 16=Viridis
    std::atomic<int>  thermal_colormap{14};
    // interp: 0=Nearest, 1=Linear, 2=Cubic, 4=Lanczos4
    std::atomic<int>  thermal_interp{2};
    // 0×0 = auto-fit to widget
    std::atomic<int>  thermal_upscale_w{0};
    std::atomic<int>  thermal_upscale_h{0};

    // Detection labels — actual scale = value / 100.0f
    std::atomic<int>  label_font_scale_x100{80};

    // Audio
    std::atomic<bool> audio_start_enabled{true};

    // Robot type: 0 = Jaguar (ROBOT_MAIN), 1 = Dicerox (ROBOT_SECONDARY)
    std::atomic<int> robot_type{0};

    // RC PPM calibration — per-channel (6 channels × {min, neutral, max} µs)
    struct PpmChannelCalib { int min_us{1000}, neutral_us{1500}, max_us{2000}; };
    std::mutex       ppm_calib_mutex;
    PpmChannelCalib  ppm_calib[6] = {};

    // Keybind configuration — 3 modes × 5 channel slots (Ch1,Ch2,Ch3,Ch4,Ch6)
    // Values are ChannelFunction enum (see keybind_dialog.hpp)
    std::mutex  keybind_mutex;
    uint8_t     keybind[3][5] = {
        // Default Jaguar: mode0=FLIPPER, mode1=NORMAL, mode2=ARM
        {3, 1, 0, 2, 0},   // mode0: FLIPPER_ALL, TRACTION_FWD, NONE, TRACTION_TURN, NONE
        {3, 1, 0, 2, 0},   // mode1: FLIPPER_ALL, TRACTION_FWD, NONE, TRACTION_TURN, NONE
        {8, 8, 8, 8, 0},   // mode2: ARM x4, NONE
    };

    // Speech — comma-separated vocabulary, empty = unrestricted
    std::mutex  strings_mutex;
    std::string vosk_grammar;

private:
    AppSettings() = default;
    AppSettings(const AppSettings&) = delete;
    AppSettings& operator=(const AppSettings&) = delete;
};
