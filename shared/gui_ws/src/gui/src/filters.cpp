#include "gui/filters.hpp"
#include "gui/filter_registry.hpp"
#include "gui/app_settings.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/dnn.hpp>
#include <opencv2/objdetect.hpp>
#include <onnxruntime_cxx_api.h>
#include <zbar.h>
using namespace zbar;

#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <thread>
#include <vector>
#include <numeric>

// ── YOLO model path ──────────────────────────────────────────────────────────
static const std::string& yoloModelPath() {
    static const std::string path =
        ament_index_cpp::get_package_share_directory("gui") + "/assets/vision/best.onnx";
    return path;
}
// ─────────────────────────────────────────────────────────────────────────────

static const float YOLO_NMS_THRESH = 0.45f;
static const int   YOLO_INPUT_SIZE = 640;

// Box colours (BGR)
static const cv::Scalar BOX_COLORS[] = {
    {0,255,0},{255,0,0},{0,0,255},{0,255,255},
    {255,0,255},{255,255,0},{0,128,255},{128,255,0},
    {255,128,0},{128,0,255},{0,255,128},{255,128,128},
};
static constexpr int NUM_COLORS = sizeof(BOX_COLORS) / sizeof(BOX_COLORS[0]);

// ── Helpers ─────────────────────────────────────────────────────────────────

// Reads class labels from the ONNX model's "names" metadata field.
// YOLO models exported with Ultralytics embed a proto-encoded Python-style
// dict in metadata_props: key="names", value="{0: 'dog', 1: 'cat', ...}".
static std::vector<std::string> loadClassNamesFromOnnx(const std::string& onnx_path)
{
    std::vector<std::string> names;
    std::ifstream f(onnx_path, std::ios::binary);
    if (!f.is_open()) return names;

    std::vector<uint8_t> data(std::istreambuf_iterator<char>(f), {});

    // Search for the protobuf-encoded key "names":
    //   field 1 (key), wire type 2 → tag 0x0A, length 5, "names"
    const uint8_t needle[] = {0x0A, 0x05, 'n', 'a', 'm', 'e', 's'};
    constexpr size_t nlen = sizeof(needle);

    for (size_t i = 0; i + nlen + 2 < data.size(); ++i) {
        if (std::memcmp(data.data() + i, needle, nlen) != 0) continue;

        // Expect field 2 (value), wire type 2 → tag 0x12
        size_t pos = i + nlen;
        if (data[pos] != 0x12) continue;
        ++pos;

        // Decode varint length
        size_t vlen = 0, shift = 0;
        while (pos < data.size()) {
            uint8_t b = data[pos++];
            vlen |= size_t(b & 0x7F) << shift;
            if (!(b & 0x80)) break;
            shift += 7;
        }
        if (pos + vlen > data.size()) continue;

        std::string val(reinterpret_cast<const char*>(data.data() + pos), vlen);

        // Parse Python-style dict: {0: 'dog', 1: 'cat', ...}
        std::map<int, std::string> cls_map;
        size_t p = 0;
        while (p < val.size()) {
            while (p < val.size() && !std::isdigit((unsigned char)val[p])) ++p;
            if (p >= val.size()) break;
            int key = 0;
            while (p < val.size() && std::isdigit((unsigned char)val[p]))
                key = key * 10 + (val[p++] - '0');
            while (p < val.size() && val[p] != '\'' && val[p] != '"') ++p;
            if (p >= val.size()) break;
            char q = val[p++];
            size_t s = p;
            while (p < val.size() && val[p] != q) ++p;
            cls_map[key] = val.substr(s, p - s);
            if (p < val.size()) ++p;
        }

        if (!cls_map.empty()) {
            for (int k = 0; k < (int)cls_map.size(); ++k)
                names.push_back(cls_map.count(k) ? cls_map[k]
                                                 : "Class " + std::to_string(k));
            return names;
        }
    }

    return names;
}

static void drawLabel(cv::Mat& img, const std::string& text,
                      cv::Point origin, const cv::Scalar& color)
{
    const float scale = AppSettings::instance().label_font_scale_x100.load() / 100.0f;
    const int   thick = (scale > 0.8f) ? 2 : 1;
    int baseline = 0;
    auto sz = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, scale, thick, &baseline);
    cv::Point tl(origin.x, origin.y - sz.height - 6);
    cv::Point br(origin.x + sz.width + 6, origin.y);
    cv::rectangle(img, tl, br, color, -1);
    cv::putText(img, text, cv::Point(origin.x + 3, origin.y - 4),
                cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0, 0, 0), thick);
}

static cv::Mat letterbox(const cv::Mat& src, int target,
                         float& scale, int& dx, int& dy)
{
    int h = src.rows, w = src.cols;
    scale = std::min(float(target) / h, float(target) / w);
    int nw = int(w * scale), nh = int(h * scale);
    dx = (target - nw) / 2;
    dy = (target - nh) / 2;

    cv::Mat resized;
    cv::resize(src, resized, cv::Size(nw, nh));

    cv::Mat padded(target, target, src.type(), cv::Scalar(114, 114, 114));
    resized.copyTo(padded(cv::Rect(dx, dy, nw, nh)));
    return padded;
}

// ── Zbar thin wrapper ───────────────────────────────────────────────────────

struct ZbarScanner {
    zbar_image_scanner_t* scanner;
    ZbarScanner() {
        scanner = zbar_image_scanner_create();
        zbar_image_scanner_set_config(scanner, ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
    }
    ~ZbarScanner() { zbar_image_scanner_destroy(scanner); }
    ZbarScanner(const ZbarScanner&) = delete;
    ZbarScanner& operator=(const ZbarScanner&) = delete;
};

// ── YOLO global singleton ────────────────────────────────────────────────────
// Loaded once at startup in a background thread, shared across all widgets.
// ORT Session::Run() is thread-safe — no mutex needed for concurrent inference.

struct YoloGlobal {
    std::atomic<bool> loaded{false};
    std::atomic<bool> failed{false};

    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "yolo"};
    std::unique_ptr<Ort::Session> session;

    std::vector<std::string> input_names_storage;
    std::vector<std::string> output_names_storage;
    std::vector<const char*> input_names;
    std::vector<const char*> output_names;
    std::vector<std::string> class_names;

    void loadAsync() {
        std::thread([this]() {
            const std::string& path = yoloModelPath();
            if (!std::ifstream(path).good()) {
                fprintf(stderr, "[YOLO] Model not found: %s\n", path.c_str());
                failed.store(true);
                return;
            }
            try {
                Ort::SessionOptions opts;
                opts.SetIntraOpNumThreads(1);
                opts.SetGraphOptimizationLevel(
                    GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

                bool cuda_ok = false;
                try {
                    OrtCUDAProviderOptions cuda_opts{};
                    cuda_opts.device_id = 0;
                    opts.AppendExecutionProvider_CUDA(cuda_opts);
                    cuda_ok = true;
                } catch (...) {}
                fprintf(stderr, "[YOLO] Using %s backend\n",
                        cuda_ok ? "CUDA" : "CPU");

                session = std::make_unique<Ort::Session>(env, path.c_str(), opts);

                Ort::AllocatorWithDefaultOptions alloc;
                for (size_t i = 0; i < session->GetInputCount(); ++i) {
                    auto n = session->GetInputNameAllocated(i, alloc);
                    input_names_storage.emplace_back(n.get());
                }
                for (size_t i = 0; i < session->GetOutputCount(); ++i) {
                    auto n = session->GetOutputNameAllocated(i, alloc);
                    output_names_storage.emplace_back(n.get());
                }
                for (auto& s : input_names_storage)  input_names.push_back(s.c_str());
                for (auto& s : output_names_storage) output_names.push_back(s.c_str());

                class_names = loadClassNamesFromOnnx(path);
                fprintf(stderr, "[YOLO] Loaded OK, %zu classes\n",
                        class_names.size());
                loaded.store(true);  // release — all writes above are visible after this
            } catch (const std::exception& e) {
                fprintf(stderr, "[YOLO] Load failed: %s\n", e.what());
                failed.store(true);
            }
        }).detach();
    }
};

static YoloGlobal g_yolo;

void shutdownFilters()
{
    // Reset the ORT session while the CUDA driver is still alive.
    // g_yolo's own destructor runs after main() returns, by which point
    // libcudart may already be unloading and cudaFreeHost would crash.
    g_yolo.session.reset();
}

// ── Filter registration ─────────────────────────────────────────────────────

void registerFilters()
{
    auto& reg = FilterRegistry::instance();

    g_yolo.loadAsync();

    // ═══ 1. QR Code Scanner ═════════════════════════════════════════════════
    reg.registerFilter("QR Code",
        [](std::shared_ptr<FilterConfig> config) -> FilterFunc
    {
        auto cv_det = std::make_shared<cv::QRCodeDetector>();
        auto zbar_s = std::make_shared<ZbarScanner>();

        return [cv_det, zbar_s, config](const cv::Mat& frame) -> cv::Mat {
            cv::Mat result = frame.clone();

            if (config->use_zbar.load()) {
                // ── zbar path ───────────────────────────────────────────
                cv::Mat gray;
                cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

                zbar_image_t* img = zbar_image_create();
                zbar_image_set_format(img, zbar_fourcc('Y','8','0','0'));
                zbar_image_set_size(img,
                    static_cast<unsigned>(gray.cols),
                    static_cast<unsigned>(gray.rows));
                zbar_image_set_data(img, gray.data,
                    static_cast<unsigned long>(gray.cols * gray.rows),
                    nullptr);
                zbar_scan_image(zbar_s->scanner, img);

                const zbar_symbol_t* sym = zbar_image_first_symbol(img);
                while (sym) {
                    const char* data = zbar_symbol_get_data(sym);
                    unsigned n = zbar_symbol_get_loc_size(sym);
                    if (n >= 4 && data && data[0] != '\0') {
                        for (unsigned i = 0; i < n; ++i) {
                            cv::Point p1(
                                zbar_symbol_get_loc_x(sym, i),
                                zbar_symbol_get_loc_y(sym, i));
                            cv::Point p2(
                                zbar_symbol_get_loc_x(sym, (i+1) % n),
                                zbar_symbol_get_loc_y(sym, (i+1) % n));
                            cv::line(result, p1, p2,
                                     cv::Scalar(0, 255, 0), 3);
                        }
                        int y = std::max(10,
                            zbar_symbol_get_loc_y(sym, 0) - 10);
                        drawLabel(result, std::string(data),
                            cv::Point(zbar_symbol_get_loc_x(sym, 0), y),
                            cv::Scalar(0, 255, 0));
                    }
                    sym = zbar_symbol_next(sym);
                }
                zbar_image_destroy(img);
            } else {
                // ── OpenCV path ─────────────────────────────────────────
                std::vector<std::string> decoded;
                cv::Mat points;
                bool found = cv_det->detectAndDecodeMulti(
                    result, decoded, points);

                if (found && !decoded.empty()) {
                    const auto* pts =
                        reinterpret_cast<const cv::Point2f*>(points.data);
                    for (size_t q = 0; q < decoded.size(); ++q) {
                        int base = static_cast<int>(q) * 4;
                        for (int i = 0; i < 4; ++i)
                            cv::line(result,
                                pts[base + i],
                                pts[base + (i + 1) % 4],
                                cv::Scalar(0, 255, 0), 3);
                        if (!decoded[q].empty()) {
                            int y = std::max(10,
                                int(pts[base].y) - 10);
                            drawLabel(result, decoded[q],
                                cv::Point(int(pts[base].x), y),
                                cv::Scalar(0, 255, 0));
                        }
                    }
                }
            }
            return result;
        };
    });

    // ═══ 2. YOLO Detection ══════════════════════════════════════════════════
    reg.registerFilter("Hazmat",
        [](std::shared_ptr<FilterConfig> config) -> FilterFunc
    {
        return [config](const cv::Mat& frame) -> cv::Mat {
            if (g_yolo.failed.load()) {
                cv::Mat r = frame.clone();
                cv::putText(r, "YOLO model not loaded",
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                    0.7, cv::Scalar(0, 0, 255), 2);
                return r;
            }
            if (!g_yolo.loaded.load()) {
                cv::Mat r = frame.clone();
                cv::putText(r, "Loading YOLO model...",
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                    0.7, cv::Scalar(0, 200, 255), 2);
                return r;
            }

            float conf_thresh = config->conf_threshold_pct.load() / 100.0f;

            float scale; int dx, dy;
            cv::Mat lb = letterbox(frame, YOLO_INPUT_SIZE, scale, dx, dy);
            cv::Mat blob = cv::dnn::blobFromImage(
                lb, 1.0 / 255.0,
                cv::Size(YOLO_INPUT_SIZE, YOLO_INPUT_SIZE),
                cv::Scalar(), true, false);

            std::array<int64_t, 4> in_shape{1, 3, YOLO_INPUT_SIZE, YOLO_INPUT_SIZE};
            auto mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
            Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
                mem_info, blob.ptr<float>(), blob.total(),
                in_shape.data(), in_shape.size());

            std::vector<Ort::Value> ort_outputs;
            try {
                ort_outputs = g_yolo.session->Run(
                    Ort::RunOptions{nullptr},
                    g_yolo.input_names.data(), &input_tensor, 1,
                    g_yolo.output_names.data(), g_yolo.output_names.size());
            } catch (const Ort::Exception& e) {
                fprintf(stderr, "[YOLO] Inference failed: %s\n", e.what());
                return frame.clone();
            }
            if (ort_outputs.empty()) return frame.clone();

            auto out_shape = ort_outputs[0].GetTensorTypeAndShapeInfo().GetShape();
            int nch = (int)out_shape[1], ndet = (int)out_shape[2];
            int nc  = nch - 4;
            if (nc <= 0) return frame.clone();

            cv::Mat det(nch, ndet, CV_32F,
                        ort_outputs[0].GetTensorMutableData<float>());
            cv::Mat detT;
            cv::transpose(det, detT);

            std::vector<cv::Rect> boxes;
            std::vector<float>    confs;
            std::vector<int>      cls_ids;

            for (int i = 0; i < ndet; ++i) {
                const float* row = detT.ptr<float>(i);
                float cx = row[0], cy = row[1], w = row[2], h = row[3];
                float best = 0; int best_cls = 0;
                for (int j = 0; j < nc; ++j)
                    if (row[4+j] > best) { best = row[4+j]; best_cls = j; }
                if (best < conf_thresh) continue;
                float x1 = (cx - w/2 - dx) / scale;
                float y1 = (cy - h/2 - dy) / scale;
                boxes.emplace_back(int(x1), int(y1), int(w/scale), int(h/scale));
                confs.push_back(best);
                cls_ids.push_back(best_cls);
            }

            std::vector<int> idx;
            cv::dnn::NMSBoxes(boxes, confs, conf_thresh, YOLO_NMS_THRESH, idx);

            cv::Mat result = frame.clone();
            for (int i : idx) {
                auto& box = boxes[i];
                int   cls = cls_ids[i];
                auto  col = BOX_COLORS[cls % NUM_COLORS];
                cv::rectangle(result, box, col, 2);
                std::string name = (cls < (int)g_yolo.class_names.size())
                    ? g_yolo.class_names[cls]
                    : "Class " + std::to_string(cls);
                char label[128];
                snprintf(label, sizeof(label), "%s %.0f%%", name.c_str(), confs[i] * 100.0f);
                drawLabel(result, label, cv::Point(box.x, box.y), col);
            }
            return result;
        };
    });

    // ═══ 3. Detect Shape ════════════════════════════════════════════════════
    reg.registerFilter("Detect Shape",
        [](std::shared_ptr<FilterConfig> config) -> FilterFunc
    {
        return [config](const cv::Mat& frame) -> cv::Mat {
            cv::Mat result = frame.clone();

            int    threshold      = config->shape_threshold.load();
            double shape_tolerance = config->shape_tolerance_pct.load() * 0.01;
            int    mode           = config->shape_mode.load();

            double img_scale = 1.0;
            cv::Mat gray_frame, gray_resized, inv_thresh;
            cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
            cv::resize(gray_frame, gray_resized, cv::Size(),
                       1.0 / img_scale, 1.0 / img_scale, cv::INTER_AREA);
            cv::threshold(gray_resized, inv_thresh,
                          threshold, 255, cv::THRESH_BINARY_INV);

            if (mode == 2) {
                // ── Mode 2: HoughCircles sector ─────────────────────────
                std::vector<cv::Vec3f> circles;
                cv::HoughCircles(gray_resized, circles,
                    cv::HOUGH_GRADIENT, 1,
                    gray_resized.rows / 8.0,
                    100, 50,
                    gray_resized.rows / 8,
                    gray_resized.rows / 4);

                double min_dis = DBL_MAX;
                cv::Vec3f circ_sector;
                for (size_t i = 0; i < circles.size(); ++i) {
                    double x = circles[i][0] * img_scale;
                    double y = circles[i][1] * img_scale;
                    double r = circles[i][2] * img_scale;
                    double dis = x * x +
                        (frame.rows - y) * (frame.rows - y);
                    cv::circle(result, cv::Point(int(x), int(y)),
                               int(r), cv::Scalar(255, 0, 0), 4);
                    if (dis < min_dis) {
                        min_dis = dis;
                        circ_sector = cv::Vec3f(
                            float(x), float(y), float(r));
                    }
                }
                if (min_dis == DBL_MAX) return result;

                cv::Mat mask = cv::Mat::zeros(
                    inv_thresh.size(), CV_8UC1);
                cv::circle(mask,
                    cv::Point(int(circ_sector[0]), int(circ_sector[1])),
                    int(circ_sector[2]), cv::Scalar(255), -1);
                cv::Mat inv_task_sector;
                cv::bitwise_and(inv_thresh, inv_thresh,
                                inv_task_sector, mask);
                cv::Rect sector = cv::boundingRect(mask);
                if (sector.width <= 0 || sector.height <= 0)
                    return result;
                inv_task_sector = inv_task_sector(sector);

                cv::Mat task_sector;
                cv::bitwise_not(inv_task_sector, task_sector);

                std::vector<std::vector<cv::Point>> shapes;
                cv::findContours(task_sector, shapes,
                    cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

                double s_min = DBL_MAX;
                std::vector<cv::Point> best_shape;
                cv::Point s_center(task_sector.cols / 2,
                                   task_sector.rows / 2);

                for (size_t i = 0; i < shapes.size(); ++i) {
                    double area = cv::contourArea(shapes[i]);
                    if (area <= 100.0) continue;
                    cv::Rect box = cv::boundingRect(shapes[i]);
                    std::vector<cv::Point> hull;
                    cv::convexHull(shapes[i], hull);
                    if (box.height == 0 || hull.empty()) continue;
                    double ar = double(box.width) / box.height;
                    double sol = area / cv::contourArea(hull);
                    if (ar < 1.0 - shape_tolerance ||
                        ar > 1.0 + shape_tolerance ||
                        sol < 1.0 - shape_tolerance) continue;
                    cv::Point sc(box.x + box.width / 2,
                                 box.y + box.height / 2);
                    double d = double(sc.x - s_center.x) *
                                      (sc.x - s_center.x) +
                               double(sc.y - s_center.y) *
                                      (sc.y - s_center.y);
                    if (d < s_min) {
                        s_min = d;
                        best_shape = shapes[i];
                    }
                }
                if (s_min == DBL_MAX) return result;

                cv::Rect box = cv::boundingRect(best_shape);
                cv::Rect fr;
                fr.x = std::max(0, box.x + sector.x - 10);
                fr.y = std::max(0, box.y + sector.y - 10);
                fr.width  = std::min(box.width  + 20,
                                     result.cols - fr.x);
                fr.height = std::min(box.height + 20,
                                     result.rows - fr.y);
                cv::rectangle(result, fr, cv::Scalar(0, 255, 0), 5);
                cv::rectangle(result, sector,
                              cv::Scalar(0, 0, 255), 3);

            } else {
                // ── Mode 1: contour sectors – all 4 corners ─────────────
                const cv::Point corners[4] = {
                    {0, 0},
                    {inv_thresh.cols, 0},
                    {0, inv_thresh.rows},
                    {inv_thresh.cols, inv_thresh.rows}
                };

                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(inv_thresh, contours,
                    cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                std::sort(contours.begin(), contours.end(),
                    [](const std::vector<cv::Point>& a,
                       const std::vector<cv::Point>& b) {
                        return cv::contourArea(a) >
                               cv::contourArea(b);
                    });

                std::set<int> processed;

                for (int c = 0; c < 4; ++c) {
                    int best_idx = -1;
                    double min_dis = DBL_MAX;

                    for (int i = 0; i < (int)contours.size(); ++i) {
                        if (processed.count(i)) continue;
                        if (cv::contourArea(contours[i]) < 1000)
                            break; // sorted by area descending
                        cv::Rect r = cv::boundingRect(contours[i]);
                        cv::Point ctr(r.x + r.width / 2,
                                      r.y + r.height / 2);
                        double d = cv::norm(ctr - corners[c]);
                        if (d < min_dis) {
                            min_dis = d;
                            best_idx = i;
                        }
                    }
                    if (best_idx < 0) continue;
                    processed.insert(best_idx);

                    cv::Rect sector =
                        cv::boundingRect(contours[best_idx]);
                    if (sector.width <= 0 || sector.height <= 0)
                        continue;
                    cv::rectangle(result, sector,
                                  cv::Scalar(0, 0, 255), 3);

                    cv::Mat inv_sector = inv_thresh(sector);
                    cv::Mat task_sector;
                    cv::bitwise_not(inv_sector, task_sector);

                    std::vector<std::vector<cv::Point>> shapes;
                    cv::findContours(task_sector, shapes,
                        cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

                    double s_min = DBL_MAX;
                    std::vector<cv::Point> best_shape;
                    cv::Point s_center(task_sector.cols / 2,
                                       task_sector.rows / 2);

                    for (size_t i = 0; i < shapes.size(); ++i) {
                        double area = cv::contourArea(shapes[i]);
                        if (area <= 100.0) continue;
                        cv::Rect box = cv::boundingRect(shapes[i]);
                        std::vector<cv::Point> hull;
                        cv::convexHull(shapes[i], hull);
                        if (box.height == 0 || hull.empty()) continue;
                        double ar = double(box.width) / box.height;
                        double sol = area / cv::contourArea(hull);
                        if (ar < 1.0 - shape_tolerance ||
                            ar > 1.0 + shape_tolerance ||
                            sol < 1.0 - shape_tolerance) continue;
                        cv::Point sc(box.x + box.width / 2,
                                     box.y + box.height / 2);
                        double d = double(sc.x - s_center.x) *
                                          (sc.x - s_center.x) +
                                   double(sc.y - s_center.y) *
                                          (sc.y - s_center.y);
                        if (d < s_min) {
                            s_min = d;
                            best_shape = shapes[i];
                        }
                    }
                    if (s_min == DBL_MAX) continue;

                    cv::Rect box = cv::boundingRect(best_shape);
                    cv::Rect fr;
                    fr.x = std::max(0,
                        box.x + sector.x - 10);
                    fr.y = std::max(0,
                        box.y + sector.y - 10);
                    fr.width  = std::min(box.width  + 20,
                                         result.cols - fr.x);
                    fr.height = std::min(box.height + 20,
                                         result.rows - fr.y);
                    cv::rectangle(result, fr,
                                  cv::Scalar(0, 255, 0), 5);
                }
            }

            return result;
        };
    });

    // ═══ 4. Shape 2 (single-corner, mirrors reference.cpp::detectShape) ═════
    reg.registerFilter("Shape 2",
        [](std::shared_ptr<FilterConfig> config) -> FilterFunc
    {
        return [config](const cv::Mat& frame) -> cv::Mat {
            cv::Mat result = frame.clone();

            const int    corner          = config->shape2_corner.load();          // 0..3
            const bool   circle_mode     = config->shape2_circle_mode.load();
            const int    threshold       = config->shape_threshold.load();
            const double shape_tolerance = config->shape_tolerance_pct.load() * 0.01;

            const double scale = 1.0;
            cv::Mat gray_frame, gray_resized, inv_thresh;
            cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
            cv::resize(gray_frame, gray_resized, cv::Size(),
                       1.0 / scale, 1.0 / scale, cv::INTER_AREA);
            cv::threshold(gray_resized, inv_thresh,
                          threshold, 255, cv::THRESH_BINARY_INV);

            cv::Rect sector;
            cv::Mat  inv_task_sector;
            double   min_dis = DBL_MAX;

            if (circle_mode) {
                std::vector<cv::Vec3f> circles;
                cv::HoughCircles(gray_resized, circles,
                    cv::HOUGH_GRADIENT, 1,
                    gray_resized.rows / 8.0,
                    100, 50,
                    gray_resized.rows / 8,
                    gray_resized.rows / 4);

                cv::Vec3f circ_sector;
                for (size_t i = 0; i < circles.size(); ++i) {
                    double x = circles[i][0] * scale;
                    double y = circles[i][1] * scale;
                    double r = circles[i][2] * scale;
                    double dis = x * x +
                        (frame.rows - y) * (frame.rows - y);
                    cv::circle(result, cv::Point(int(x), int(y)),
                               int(r), cv::Scalar(255, 0, 0), 4);
                    if (dis < min_dis) {
                        min_dis = dis;
                        circ_sector = cv::Vec3f(
                            float(x), float(y), float(r));
                    }
                }
                if (min_dis == DBL_MAX) return result;

                cv::Mat mask = cv::Mat::zeros(
                    inv_thresh.size(), CV_8UC1);
                cv::circle(mask,
                    cv::Point(int(circ_sector[0]), int(circ_sector[1])),
                    int(circ_sector[2]), cv::Scalar(255), -1);
                cv::bitwise_and(inv_thresh, inv_thresh,
                                inv_task_sector, mask);
                sector = cv::boundingRect(mask);
                if (sector.width <= 0 || sector.height <= 0)
                    return result;
                inv_task_sector = inv_task_sector(sector);
            } else {
                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(inv_thresh, contours,
                    cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                std::sort(contours.begin(), contours.end(),
                    [](const std::vector<cv::Point>& a,
                       const std::vector<cv::Point>& b) {
                        return cv::contourArea(a) >
                               cv::contourArea(b);
                    });

                // corner lookup table matches reference.cpp indexing
                //   0=TL(0,0)  1=TR(cols,0)  2=BL(0,rows)  3=BR(cols,rows)
                const cv::Point corner_pt(
                    (corner == 0 || corner == 2) ? 0 : inv_thresh.cols,
                    (corner <= 1)                ? 0 : inv_thresh.rows);

                for (size_t i = 0; i < contours.size(); ++i) {
                    if (cv::contourArea(contours[i]) < 1000) break;
                    cv::Rect r = cv::boundingRect(contours[i]);
                    cv::Point ctr(r.x + r.width / 2,
                                  r.y + r.height / 2);
                    double dis = cv::norm(ctr - corner_pt);
                    if (dis < min_dis) {
                        min_dis = dis;
                        sector = r;
                    }
                }
                if (min_dis == DBL_MAX) return result;
                if (sector.width <= 0 || sector.height <= 0)
                    return result;
                inv_task_sector = inv_thresh(sector);
            }
            cv::rectangle(result, sector, cv::Scalar(0, 0, 255), 3);

            cv::Mat task_sector;
            cv::bitwise_not(inv_task_sector, task_sector);

            std::vector<std::vector<cv::Point>> shapes;
            cv::findContours(task_sector, shapes,
                cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

            double s_min = DBL_MAX;
            std::vector<cv::Point> best_shape;
            cv::Point s_center(task_sector.cols / 2,
                               task_sector.rows / 2);

            for (size_t i = 0; i < shapes.size(); ++i) {
                double area = cv::contourArea(shapes[i]);
                if (area <= 100.0) continue;
                cv::Rect box = cv::boundingRect(shapes[i]);
                std::vector<cv::Point> hull;
                cv::convexHull(shapes[i], hull);
                if (box.height == 0 || hull.empty()) continue;
                double ar = double(box.width) / box.height;
                double sol = area / cv::contourArea(hull);
                if (ar < 1.0 - shape_tolerance ||
                    ar > 1.0 + shape_tolerance ||
                    sol < 1.0 - shape_tolerance) continue;
                cv::Point sc(box.x + box.width / 2,
                             box.y + box.height / 2);
                double d = double(sc.x - s_center.x) *
                                  (sc.x - s_center.x) +
                           double(sc.y - s_center.y) *
                                  (sc.y - s_center.y);
                if (d < s_min) {
                    s_min = d;
                    best_shape = shapes[i];
                }
            }
            if (s_min == DBL_MAX) return result;

            cv::Rect box = cv::boundingRect(best_shape);
            cv::Rect fr;
            fr.x = std::max(0, box.x + sector.x - 10);
            fr.y = std::max(0, box.y + sector.y - 10);
            fr.width  = std::min(box.width  + 20,
                                 result.cols - fr.x);
            fr.height = std::min(box.height + 20,
                                 result.rows - fr.y);
            cv::rectangle(result, fr, cv::Scalar(0, 255, 0), 5);
            return result;
        };
    });

    // ═══ 5. Circle Gap Detector ══════════════════════════════════════════════════
    reg.registerFilter("Circle Gap",
        [](std::shared_ptr<FilterConfig> config) -> FilterFunc
    {
        return [config](const cv::Mat& frame) -> cv::Mat {
            cv::Mat result = frame.clone();
            
            // Reemplazo de los valores que faltaban en FilterConfig
            int rad_checks = 360; 
            float scale = 2.0f; 

            // 1. Preprocesamiento
            cv::Mat gray;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            
            // Detección inicial de ROI (Círculo exterior)
            std::vector<cv::Vec3f> outer_circles;
            cv::HoughCircles(gray, outer_circles, cv::HOUGH_GRADIENT, 1, gray.rows / 8, 100, 50, gray.rows / 8, gray.rows / 3);

            if (outer_circles.empty()) {
                cv::putText(result, "MISSING TASK SECTOR", cv::Point(30, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
                return result;
            }

            // Tomamos el primer círculo detectado y armamos su caja delimitadora (ROI) matemáticamente
            int cx = std::round(outer_circles[0][0]);
            int cy = std::round(outer_circles[0][1]);
            int r  = std::round(outer_circles[0][2]);
            cv::Rect roi_rect(cx - r, cy - r, 2 * r, 2 * r);
            
            // Ajuste de límites del ROI para que no se salga de la imagen
            roi_rect &= cv::Rect(0, 0, gray.cols, gray.rows);
            
            if (roi_rect.width < 8 || roi_rect.height < 8) return result;

            cv::Mat frame_roi = gray(roi_rect);

            // 2. Detección de círculo interno
            std::vector<cv::Vec3f> inn_circles;
            cv::HoughCircles(frame_roi, inn_circles, cv::HOUGH_GRADIENT, 1, frame_roi.rows / 8, 100, 50, frame_roi.rows / 8, frame_roi.rows / 3);

            if (inn_circles.empty()) {
                cv::putText(result, "MISSING INNER CIRCLE", cv::Point(30, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
                return result;
            }

            // Crear máscara para aislar el área de interés
            int inn_cx = std::round(inn_circles[0][0]);
            int inn_cy = std::round(inn_circles[0][1]);
            int inn_r  = std::round(inn_circles[0][2]) - 10;
            if (inn_r <= 0) inn_r = 1; // Salvaguarda: si el radio es muy pequeño, evitamos un círculo negativo

            cv::Mat mask_roi = cv::Mat::zeros(frame_roi.size(), CV_8UC1);
            cv::circle(mask_roi, cv::Point(inn_cx, inn_cy), inn_r, 255, -1);

            cv::Mat final_img;
            cv::bitwise_and(frame_roi, frame_roi, final_img, mask_roi);

            cv::Rect final_rect = cv::boundingRect(mask_roi);
            final_img = final_img(final_rect);
            cv::resize(final_img, final_img, cv::Size(), scale, scale, cv::INTER_LINEAR);

            // Segmentación
            cv::Mat thresh;
            cv::threshold(final_img, thresh, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
            
            cv::Mat kernel = cv::Mat::ones(3, 3, CV_8UC1);
            cv::morphologyEx(thresh, thresh, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 2);

            // 3. Encontrar contornos y procesar brecha
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(thresh, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

            std::vector<std::vector<cv::Point>> filtered_contours;
            for (const auto& cnt : contours) {
                if (cv::contourArea(cnt) > (100 * scale))
                    filtered_contours.push_back(cnt);
            }

            std::sort(filtered_contours.begin(), filtered_contours.end(), [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                return cv::contourArea(a) > cv::contourArea(b);
            });

            std::vector<cv::Scalar> colors = { cv::Scalar(0,255,0), cv::Scalar(255,0,0), cv::Scalar(0,0,255) };

            for (std::size_t i = 0; i < filtered_contours.size() && i < 3; ++i) {
                cv::Rect bbox = cv::boundingRect(filtered_contours[i]);
                float aspect_ratio = static_cast<float>(bbox.width) / bbox.height;

                if (aspect_ratio > 0.85f && aspect_ratio < 1.15f) {
                    cv::Point2f center;
                    float r_enc;
                    cv::minEnclosingCircle(filtered_contours[i], center, r_enc);

                    int min_touch = INT_MAX;
                    double best_angle = 0.0;
                    std::vector<double> angles;

                    cv::Mat mask = cv::Mat::zeros(final_img.size(), CV_8UC1);
                    cv::drawContours(mask, filtered_contours, (int)i, 255, -1);

                    for (int j = 0; j < rad_checks; ++j) {
                        double angle = 2.0 * CV_PI * j / rad_checks;
                        int x3 = static_cast<int>(center.x + r_enc * std::cos(angle));
                        int y3 = static_cast<int>(center.y + r_enc * std::sin(angle));

                        cv::LineIterator it(mask, center, cv::Point(x3, y3));
                        int overlap_count = 0;
                        for(int k = 0; k < it.count; k++, ++it) {
                            if (*(const uchar*)*it > 0) overlap_count++;
                        }

                        if (overlap_count < min_touch) {
                            min_touch = overlap_count;
                            best_angle = angle;
                        }
                        if (overlap_count == 0) angles.push_back(angle);
                    }

                    if (!angles.empty()) {
                        double sum = 0;
                        for(double a : angles) sum += a;
                        best_angle = sum / angles.size();
                    }

                    // Proyección de regreso al frame original
                    int gap_x = static_cast<int>(center.x + r_enc * std::cos(best_angle));
                    int gap_y = static_cast<int>(center.y + r_enc * std::sin(best_angle));

                    cv::Point pt1(static_cast<int>((center.x / scale) + roi_rect.x + final_rect.x),
                                  static_cast<int>((center.y / scale) + roi_rect.y + final_rect.y));
                    cv::Point pt2(static_cast<int>((gap_x / scale) + roi_rect.x + final_rect.x),
                                  static_cast<int>((gap_y / scale) + roi_rect.y + final_rect.y));

                    cv::line(result, pt1, pt2, colors[i % 3], 4);
                }
            }

            return result;
        };
    });
}
