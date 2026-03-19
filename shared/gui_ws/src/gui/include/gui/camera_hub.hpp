#pragma once

#include <opencv2/opencv.hpp>

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

// Owns shared VideoCapture instances so multiple VideoWidgets can view the
// same local camera simultaneously.  Each camera device gets one capture
// thread; consumers call getLatestFrame() to grab a clone of the most recent
// frame.  Reference-counted: the capture thread is started on the first
// subscribe() and stopped when the last consumer calls unsubscribe().
class CameraHub {
public:
    CameraHub() = default;
    ~CameraHub();

    CameraHub(const CameraHub&) = delete;
    CameraHub& operator=(const CameraHub&) = delete;

    void subscribe(int camera_id);
    void unsubscribe(int camera_id);
    cv::Mat getLatestFrame(int camera_id);
    std::vector<int> activeCameraIds() const;

private:
    struct Stream {
        cv::VideoCapture capture;
        std::thread thread;
        std::atomic<bool> running{true};
        std::mutex frame_mutex;
        cv::Mat latest_frame;
        int ref_count{0};
    };

    void captureLoop(int camera_id, Stream* stream);

    mutable std::mutex streams_mutex_;
    std::map<int, std::unique_ptr<Stream>> streams_;
};
