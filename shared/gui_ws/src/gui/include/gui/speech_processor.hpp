#pragma once

#include <QObject>
#include <QString>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

struct VoskModel;
struct VoskRecognizer;
struct OpusDecoder;
typedef struct pa_simple pa_simple;

class SpeechProcessor : public QObject {
    Q_OBJECT
public:
    explicit SpeechProcessor(rclcpp::Node::SharedPtr node, QObject* parent = nullptr);
    ~SpeechProcessor() override;

    void clearTranscription();
    bool isModelLoaded() const { return vosk_model_ != nullptr; }
    void setPlaybackEnabled(bool enabled);
    void setGrammar(const std::string& words_csv);

signals:
    void transcriptionUpdated(const QString& text);

private:
    void onAudioReceived(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
    void audioWorkerLoop();
    bool loadModel();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr audio_sub_;

    // Opus decoder (16 kHz mono voice). Buffer sized for the largest Opus
    // frame at 16 kHz: 60 ms = 960 samples.
    OpusDecoder* opus_decoder_{nullptr};
    std::vector<int16_t> pcm_buf_;
    std::uint64_t opus_decode_errors_{0};

    VoskModel* vosk_model_{nullptr};
    VoskRecognizer* vosk_recognizer_{nullptr};

    std::mutex recognizer_mutex_;
    std::mutex text_mutex_;
    QString full_transcription_;

    pa_simple* pa_{nullptr};
    bool playback_enabled_{true};

    // Worker thread isolates blocking pa_simple_write + Vosk inference from
    // the ROS spin thread so /image_raw callbacks keep flowing while audio
    // is active. Bounded queue drops oldest on overflow.
    std::thread audio_worker_;
    std::atomic<bool> audio_running_{false};
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::deque<std::vector<std::uint8_t>> packet_queue_;
    std::uint64_t queue_drops_{0};
    static constexpr size_t kMaxQueueDepth = 8;

    // Diagnostic counters (reset each periodic log)
    std::chrono::steady_clock::time_point last_log_time_{std::chrono::steady_clock::now()};
    std::uint64_t msgs_received_{0};
    std::uint64_t samples_received_{0};
    std::int64_t rms_sq_sum_{0};
    std::int16_t peak_{0};
    std::uint64_t pa_write_errors_{0};
};
