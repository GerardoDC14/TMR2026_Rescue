#pragma once

#include <atomic>
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

    // Speech — comma-separated vocabulary, empty = unrestricted
    std::mutex  strings_mutex;
    std::string vosk_grammar;

private:
    AppSettings() = default;
    AppSettings(const AppSettings&) = delete;
    AppSettings& operator=(const AppSettings&) = delete;
};
