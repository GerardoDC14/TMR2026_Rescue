#pragma once

#include <opencv2/opencv.hpp>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <atomic>

/// Thread-safe filter settings shared between UI thread (writes) and worker thread (reads).
struct FilterConfig {
    // QR Code
    std::atomic<bool> use_zbar{false};

    // YOLO Detect
    std::atomic<int> conf_threshold_pct{25}; // 0-100

    // Detect Shape
    std::atomic<int> shape_threshold{100};     // 0-255
    std::atomic<int> shape_tolerance_pct{4};   // 1-100 → actual = value * 0.01
    std::atomic<int> shape_mode{1};            // 1 or 2

    // Shape 2 (single-corner variant, mirrors reference.cpp::detectShape)
    std::atomic<int> shape2_corner{0};         // 0=TL, 1=TR, 2=BL, 3=BR
    std::atomic<bool> shape2_circle_mode{false}; // false = contour sector, true = HoughCircles sector
};

using FilterFunc    = std::function<cv::Mat(const cv::Mat&)>;
using FilterFactory = std::function<FilterFunc(std::shared_ptr<FilterConfig>)>;

class FilterRegistry {
public:
    static FilterRegistry& instance() {
        static FilterRegistry reg;
        return reg;
    }

    void registerFilter(const std::string& name, FilterFactory factory) {
        factories_[name] = std::move(factory);
    }

    /// Create a per-widget filter instance that captures the given config.
    FilterFunc createFilter(const std::string& name,
                            std::shared_ptr<FilterConfig> config) const {
        auto it = factories_.find(name);
        return (it != factories_.end()) ? it->second(config) : nullptr;
    }

    std::vector<std::string> getFilterNames() const {
        std::vector<std::string> names;
        names.reserve(factories_.size());
        for (const auto& [name, _] : factories_)
            names.push_back(name);
        return names;
    }

private:
    FilterRegistry() = default;

    FilterRegistry(const FilterRegistry&) = delete;
    FilterRegistry& operator=(const FilterRegistry&) = delete;

    std::map<std::string, FilterFactory> factories_;
};
