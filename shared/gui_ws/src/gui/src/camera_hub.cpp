#include "gui/camera_hub.hpp"

#include <chrono>

CameraHub::~CameraHub()
{
    std::lock_guard<std::mutex> lock(streams_mutex_);
    for (auto& [id, stream] : streams_)
        stream->running = false;
    // captureLoop never locks streams_mutex_, so joining under lock is safe
    for (auto& [id, stream] : streams_) {
        if (stream->thread.joinable())
            stream->thread.join();
    }
}

void CameraHub::subscribe(int camera_id)
{
    std::lock_guard<std::mutex> lock(streams_mutex_);
    auto it = streams_.find(camera_id);
    if (it != streams_.end()) {
        it->second->ref_count++;
        return;
    }

    auto stream = std::make_unique<Stream>();
    stream->ref_count = 1;
    auto* ptr = stream.get();
    streams_[camera_id] = std::move(stream);
    streams_[camera_id]->thread =
        std::thread(&CameraHub::captureLoop, this, camera_id, ptr);
}

void CameraHub::unsubscribe(int camera_id)
{
    std::unique_ptr<Stream> to_destroy;
    {
        std::lock_guard<std::mutex> lock(streams_mutex_);
        auto it = streams_.find(camera_id);
        if (it == streams_.end()) return;

        it->second->ref_count--;
        if (it->second->ref_count <= 0) {
            it->second->running = false;
            to_destroy = std::move(it->second);
            streams_.erase(it);
        }
    }
    // Join outside the lock
    if (to_destroy && to_destroy->thread.joinable())
        to_destroy->thread.join();
}

cv::Mat CameraHub::getLatestFrame(int camera_id)
{
    std::lock_guard<std::mutex> lock(streams_mutex_);
    auto it = streams_.find(camera_id);
    if (it == streams_.end()) return cv::Mat();

    std::lock_guard<std::mutex> flock(it->second->frame_mutex);
    if (it->second->latest_frame.empty()) return cv::Mat();
    return it->second->latest_frame.clone();
}

std::vector<int> CameraHub::activeCameraIds() const
{
    std::lock_guard<std::mutex> lock(streams_mutex_);
    std::vector<int> ids;
    ids.reserve(streams_.size());
    for (const auto& [id, _] : streams_)
        ids.push_back(id);
    return ids;
}

void CameraHub::captureLoop(int camera_id, Stream* stream)
{
    stream->capture.open(camera_id, cv::CAP_V4L2);

    while (stream->running) {
        if (!stream->capture.isOpened()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        cv::Mat frame;
        if (!stream->capture.read(frame) || frame.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(stream->frame_mutex);
            stream->latest_frame = std::move(frame);
        }
    }

    stream->capture.release();
}
