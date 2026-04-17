#pragma once

#include <QObject>
#include <QString>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

#include <chrono>
#include <cstdint>
#include <mutex>

struct VoskModel;
struct VoskRecognizer;
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
    void onAudioReceived(const std_msgs::msg::Int16MultiArray::SharedPtr msg);
    bool loadModel();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr audio_sub_;

    VoskModel* vosk_model_{nullptr};
    VoskRecognizer* vosk_recognizer_{nullptr};

    std::mutex recognizer_mutex_;
    std::mutex text_mutex_;
    QString full_transcription_;

    pa_simple* pa_{nullptr};
    bool playback_enabled_{true};

    // Diagnostic counters (reset each periodic log)
    std::chrono::steady_clock::time_point last_log_time_{std::chrono::steady_clock::now()};
    std::uint64_t msgs_received_{0};
    std::uint64_t samples_received_{0};
    std::int64_t rms_sq_sum_{0};
    std::int16_t peak_{0};
    std::uint64_t pa_write_errors_{0};
};
