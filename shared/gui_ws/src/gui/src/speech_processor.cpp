#include "gui/speech_processor.hpp"
#include "gui/app_settings.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <vosk_api.h>
#include <pulse/simple.h>
#include <pulse/error.h>
#include <opus/opus.h>

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <cmath>
#include <cstdlib>
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

    // Opus decoder: 16 kHz mono, matches robot audio_node defaults.
    // Max frame at 16 kHz = 60 ms = 960 samples.
    {
        int err = 0;
        opus_decoder_ = opus_decoder_create(16000, 1, &err);
        if (err != OPUS_OK || !opus_decoder_) {
            RCLCPP_ERROR(node_->get_logger(),
                "[Audio] opus_decoder_create failed: %s",
                opus_strerror(err));
        } else {
            RCLCPP_INFO(node_->get_logger(),
                "[Audio] Opus decoder ready (16kHz mono)");
        }
        pcm_buf_.resize(960);
    }

    // Set up PulseAudio output
    pa_sample_spec ss{PA_SAMPLE_S16LE, 16000, 1};
    int pa_error;
    const char* pa_server = std::getenv("PULSE_SERVER");
    pa_ = pa_simple_new(nullptr, "gui", PA_STREAM_PLAYBACK, nullptr,
                        "Audio Monitor", &ss, nullptr, nullptr, &pa_error);
    if (!pa_) {
        RCLCPP_ERROR(node_->get_logger(),
            "[Audio] PulseAudio init FAILED: %s. "
            "Playback disabled. PULSE_SERVER=%s",
            pa_strerror(pa_error), pa_server ? pa_server : "<unset>");
    } else {
        RCLCPP_INFO(node_->get_logger(),
            "[Audio] PulseAudio playback stream opened "
            "(S16LE 16000Hz mono). PULSE_SERVER=%s",
            pa_server ? pa_server : "<unset — using default>");
    }

    audio_sub_ = node_->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/audio", rclcpp::SensorDataQoS(),
        [this](std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
            onAudioReceived(msg);
        });
    last_log_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(node_->get_logger(),
        "[Audio] Subscribed to /audio (SensorDataQoS: BEST_EFFORT). "
        "Playback enabled=%s. Publish true on /audio_enable from the robot "
        "to start the stream.",
        playback_enabled_ ? "true" : "false");
}

SpeechProcessor::~SpeechProcessor()
{
    if (pa_) {
        pa_simple_drain(pa_, nullptr);
        pa_simple_free(pa_);
    }
    if (vosk_recognizer_) vosk_recognizer_free(vosk_recognizer_);
    if (vosk_model_) vosk_model_free(vosk_model_);
    if (opus_decoder_) opus_decoder_destroy(opus_decoder_);
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
    if (playback_enabled_ != enabled)
        RCLCPP_INFO(node_->get_logger(),
                    "[Audio] Playback %s", enabled ? "ENABLED" : "DISABLED");
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

void SpeechProcessor::onAudioReceived(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
    if (msg->data.empty() || !opus_decoder_) return;

    // ── Decode Opus → PCM int16 ─────────────────────────────────────────────
    int decoded = opus_decode(
        opus_decoder_,
        msg->data.data(),
        static_cast<opus_int32>(msg->data.size()),
        pcm_buf_.data(),
        static_cast<int>(pcm_buf_.size()),
        /*decode_fec=*/0);

    if (decoded < 0) {
        ++opus_decode_errors_;
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "[Audio] opus_decode failed: %s", opus_strerror(decoded));
        return;
    }
    const size_t n_samples = static_cast<size_t>(decoded);
    const int16_t* pcm = pcm_buf_.data();

    // ── Diagnostic stats ────────────────────────────────────────────────────
    ++msgs_received_;
    samples_received_ += n_samples;
    for (size_t i = 0; i < n_samples; ++i) {
        int16_t s = pcm[i];
        int16_t a = s < 0 ? static_cast<int16_t>(-s) : s;
        if (a > peak_) peak_ = a;
        rms_sq_sum_ += static_cast<int64_t>(s) * s;
    }

    auto now = std::chrono::steady_clock::now();
    auto since_log = std::chrono::duration<double>(now - last_log_time_).count();
    if (since_log >= 2.0) {
        double rms = samples_received_ > 0
            ? std::sqrt(static_cast<double>(rms_sq_sum_) / samples_received_)
            : 0.0;
        double dbfs = rms > 0 ? 20.0 * std::log10(rms / 32767.0) : -120.0;
        RCLCPP_INFO(node_->get_logger(),
            "[Audio] rx opus frames/s=%.1f samples/s=%.0f "
            "peak=%d rms=%.0f (%+.1f dBFS) "
            "pa_err=%lu opus_err=%lu%s",
            msgs_received_ / since_log,
            samples_received_ / since_log,
            peak_, rms, dbfs,
            static_cast<unsigned long>(pa_write_errors_),
            static_cast<unsigned long>(opus_decode_errors_),
            dbfs < -60 ? " — SILENT (mic on robot muted/gain low?)" : "");
        last_log_time_ = now;
        msgs_received_ = 0;
        samples_received_ = 0;
        rms_sq_sum_ = 0;
        peak_ = 0;
    }

    if (playback_enabled_ && pa_) {
        int error;
        if (pa_simple_write(pa_, pcm,
                            n_samples * sizeof(int16_t), &error) < 0) {
            ++pa_write_errors_;
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "[Audio] pa_simple_write failed: %s", pa_strerror(error));
        }
    } else if (!pa_) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
            "[Audio] Receiving audio but PulseAudio is not initialised — "
            "nothing will be heard.");
    }

    std::lock_guard<std::mutex> lock(recognizer_mutex_);
    if (!vosk_recognizer_) return;

    int result = vosk_recognizer_accept_waveform_s(
        vosk_recognizer_, pcm, static_cast<int>(n_samples));

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
