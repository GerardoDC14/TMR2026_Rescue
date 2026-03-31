#include "gui/app_settings.hpp"

#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

static QString configPath()
{
    return QDir::homePath() + "/.config/robocup_gui/settings.json";
}

void AppSettings::load()
{
    QFile f(configPath());
    if (!f.open(QIODevice::ReadOnly)) return;

    auto doc = QJsonDocument::fromJson(f.readAll());
    if (!doc.isObject()) return;
    auto o = doc.object();

    auto get_i = [&](const char* k, int def) {
        return o.contains(k) ? o[k].toInt(def) : def;
    };

    thermal_colormap    .store(get_i("thermal_colormap",     14));
    thermal_interp      .store(get_i("thermal_interp",        2));
    thermal_upscale_w   .store(get_i("thermal_upscale_w",     0));
    thermal_upscale_h   .store(get_i("thermal_upscale_h",     0));
    label_font_scale_x100.store(get_i("label_font_scale_x100", 80));
    audio_start_enabled .store(o.value("audio_start_enabled").toBool(true));

    robot_type.store(get_i("robot_type", 0));

    if (o.contains("keybind")) {
        auto arr = o["keybind"].toArray();
        std::lock_guard<std::mutex> lk(keybind_mutex);
        for (int m = 0; m < 3 && m < arr.size(); ++m) {
            auto row = arr[m].toArray();
            for (int c = 0; c < 5 && c < row.size(); ++c)
                keybind[m][c] = static_cast<uint8_t>(row[c].toInt(0));
        }
    }

    if (o.contains("vosk_grammar")) {
        std::lock_guard<std::mutex> lk(strings_mutex);
        vosk_grammar = o["vosk_grammar"].toString().toStdString();
    }
}

void AppSettings::save()
{
    QJsonObject o;
    o["thermal_colormap"]      = thermal_colormap.load();
    o["thermal_interp"]        = thermal_interp.load();
    o["thermal_upscale_w"]     = thermal_upscale_w.load();
    o["thermal_upscale_h"]     = thermal_upscale_h.load();
    o["label_font_scale_x100"] = label_font_scale_x100.load();
    o["audio_start_enabled"]   = audio_start_enabled.load();
    o["robot_type"]            = robot_type.load();
    {
        std::lock_guard<std::mutex> lk(keybind_mutex);
        QJsonArray kb_arr;
        for (int m = 0; m < 3; ++m) {
            QJsonArray row;
            for (int c = 0; c < 5; ++c)
                row.append(static_cast<int>(keybind[m][c]));
            kb_arr.append(row);
        }
        o["keybind"] = kb_arr;
    }
    {
        std::lock_guard<std::mutex> lk(strings_mutex);
        o["vosk_grammar"] = QString::fromStdString(vosk_grammar);
    }

    QDir dir(QDir::homePath() + "/.config/robocup_gui");
    if (!dir.exists()) dir.mkpath(".");

    QFile f(configPath());
    if (f.open(QIODevice::WriteOnly | QIODevice::Truncate))
        f.write(QJsonDocument(o).toJson());
}
