#include "gui/speech_processor.hpp"
#include "gui/app_settings.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <vosk_api.h>
#include <pulse/simple.h>
#include <pulse/error.h>

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <filesystem>

static const std::string& voskModelPath() {
    static const std::string path =
        ament_index_cpp::get_package_share_directory("gui") + "/assets/audio";
    return path;
}

SpeechProcessor::SpeechProcessor(rclcpp::Node::SharedPtr node, QObject* parent)
    : QObject(parent), node_(node)
{
    playback_enabled_ = AppSettings::instance().audio_start_enabled.load();
    vosk_set_log_level(-1);

    if (!loadModel()) {
        RCLCPP_WARN(node_->get_logger(), "Vosk model not found. Speech transcription disabled");
        return;
    }

    vosk_recognizer_ = vosk_recognizer_new(vosk_model_, 16000.0f);
    if (!vosk_recognizer_) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create Vosk recognizer");
        return;
    }

    // Set up PulseAudio output
    pa_sample_spec ss{PA_SAMPLE_S16LE, 16000, 1};
    int pa_error;
    pa_ = pa_simple_new(nullptr, "gui", PA_STREAM_PLAYBACK, nullptr,
                        "Audio Monitor", &ss, nullptr, nullptr, &pa_error);
    if (!pa_)
        RCLCPP_WARN(node_->get_logger(), "[Audio] PulseAudio init failed: %s",
                    pa_strerror(pa_error));

    audio_sub_ = node_->create_subscription<std_msgs::msg::Int16MultiArray>(
        "/audio", rclcpp::SensorDataQoS(),
        [this](std_msgs::msg::Int16MultiArray::SharedPtr msg) {
            onAudioReceived(msg);
        });

    RCLCPP_INFO(node_->get_logger(), "Speech processor ready, listening on /audio");
}

SpeechProcessor::~SpeechProcessor()
{
    if (pa_) {
        pa_simple_drain(pa_, nullptr);
        pa_simple_free(pa_);
    }
    if (vosk_recognizer_) vosk_recognizer_free(vosk_recognizer_);
    if (vosk_model_) vosk_model_free(vosk_model_);
}

bool SpeechProcessor::loadModel()
{
    if (std::filesystem::is_directory(voskModelPath())) {
        vosk_model_ = vosk_model_new(voskModelPath().c_str());
        if (vosk_model_) {
            RCLCPP_INFO(node_->get_logger(), "Loaded Vosk model: %s", voskModelPath().c_str());
            return true;
        }
    }
    return false;
}

void SpeechProcessor::setPlaybackEnabled(bool enabled)
{
    playback_enabled_ = enabled;
}

void SpeechProcessor::clearTranscription()
{
    {
        std::lock_guard<std::mutex> lock(recognizer_mutex_);
        if (vosk_recognizer_)
            vosk_recognizer_reset(vosk_recognizer_);
    }
    std::lock_guard<std::mutex> lock(text_mutex_);
    full_transcription_.clear();
    emit transcriptionUpdated("");
}

void SpeechProcessor::setGrammar(const std::string& words_csv)
{
    if (!vosk_model_) return;

    // Convert "word1, word2" → JSON ["word1","word2","[unk]"]
    std::string grammar_json;
    if (!words_csv.empty()) {
        QString q = QString::fromStdString(words_csv);
        QJsonArray arr;
        for (const auto& w : q.split(',', Qt::SkipEmptyParts)) {
            QString t = w.trimmed().toLower();
            if (!t.isEmpty()) arr.append(t);
        }
        arr.append("[unk]");
        grammar_json = QJsonDocument(arr).toJson(QJsonDocument::Compact).toStdString();
    }

    std::lock_guard<std::mutex> lock(recognizer_mutex_);
    if (vosk_recognizer_) vosk_recognizer_free(vosk_recognizer_);

    if (!grammar_json.empty())
        vosk_recognizer_ = vosk_recognizer_new_grm(vosk_model_, 16000.0f, grammar_json.c_str());
    else
        vosk_recognizer_ = vosk_recognizer_new(vosk_model_, 16000.0f);

    RCLCPP_INFO(node_->get_logger(), "[Speech] Grammar %s",
                grammar_json.empty() ? "cleared (unrestricted)" : "updated");
}

void SpeechProcessor::onAudioReceived(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
{
    if (msg->data.empty()) return;

    if (playback_enabled_ && pa_) {
        int error;
        pa_simple_write(pa_, msg->data.data(),
                        msg->data.size() * sizeof(int16_t), &error);
    }

    std::lock_guard<std::mutex> lock(recognizer_mutex_);
    if (!vosk_recognizer_) return;

    int result = vosk_recognizer_accept_waveform_s(
        vosk_recognizer_, msg->data.data(),
        static_cast<int>(msg->data.size()));

    if (result > 0) {
        const char* json_str = vosk_recognizer_result(vosk_recognizer_);
        QJsonDocument doc = QJsonDocument::fromJson(QByteArray(json_str));
        QString text = doc.object().value("text").toString().trimmed();
        if (!text.isEmpty()) {
            std::lock_guard<std::mutex> lock2(text_mutex_);
            if (!full_transcription_.isEmpty())
                full_transcription_ += "\n";
            full_transcription_ += text;
            emit transcriptionUpdated(full_transcription_);
        }
    }
}
