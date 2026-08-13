#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <cerrno>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include <zmq.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shm_msgs/msg/image1m.hpp>
#include <shm_msgs/msg/image2m.hpp>
#include <shm_msgs/msg/image6m.hpp>
#include <sensor_msgs/msg/image.hpp>

// Minimal JSON parser — extracts integer values for known keys.
// Avoids nlohmann-json system dependency for a trivial {"width":N,"height":N} payload.
namespace tinyjson {
inline int get_int(const std::string& json, const std::string& key) {
    auto pos = json.find("\"" + key + "\"");
    if (pos == std::string::npos) return -1;
    auto colon = json.find(':', pos + key.size() + 2);
    if (colon == std::string::npos) return -1;
    size_t i = colon + 1;
    while (i < json.size() && (json[i] == ' ' || json[i] == '\t' || json[i] == '\n' || json[i] == '\r')) i++;
    int sign = 1;
    if (i < json.size() && json[i] == '-') { sign = -1; i++; }
    int val = 0;
    bool found = false;
    while (i < json.size() && json[i] >= '0' && json[i] <= '9') {
        val = val * 10 + (json[i] - '0');
        i++; found = true;
    }
    return found ? val * sign : -1;
}
inline std::string get_string(const std::string& json, const std::string& key) {
    auto pos = json.find("\"" + key + "\"");
    if (pos == std::string::npos) return "";
    auto colon = json.find(':', pos + key.size() + 2);
    if (colon == std::string::npos) return "";
    auto val_start = json.find('"', colon + 1);
    if (val_start == std::string::npos) return "";
    auto val_end = json.find('"', val_start + 1);
    if (val_end == std::string::npos) return "";
    return json.substr(val_start + 1, val_end - val_start - 1);
}
} // namespace tinyjson

// Camera spec: topic + optional msg_type
struct CameraSpec {
    std::string topic;
    std::string msg_type = "Image2m";
};

// Parse camera_topics JSON. Supports two formats:
//   Old: {"name":"topic"}                        → msg_type defaults to Image2m
//   New: {"name":{"topic":"...","msg_type":"..."}} → per-camera msg_type
static std::map<std::string, CameraSpec> parse_camera_topics(const std::string& json) {
    std::map<std::string, CameraSpec> map;
    if (json.empty()) return map;
    size_t pos = 0;
    while ((pos = json.find('"', pos)) != std::string::npos) {
        size_t key_start = pos + 1;
        size_t key_end = json.find('"', key_start);
        if (key_end == std::string::npos) break;
        std::string key = json.substr(key_start, key_end - key_start);
        // Skip internal keys like "topic" and "msg_type" when inside a spec object
        if (key == "topic" || key == "msg_type") {
            pos = key_end + 1;
            continue;
        }
        size_t val_start = json.find('"', key_end + 1);
        if (val_start == std::string::npos) break;
        // Peek ahead: if the character after ':' is '{', it's the new format
        size_t colon = json.rfind(':', val_start);
        bool is_object = false;
        for (size_t c = colon + 1; c < val_start; c++) {
            if (json[c] == '{') { is_object = true; break; }
        }
        if (is_object) {
            CameraSpec spec;
            spec.topic = tinyjson::get_string(json.substr(colon, json.find('}', colon) - colon + 1), "topic");
            std::string mt = tinyjson::get_string(json.substr(colon, json.find('}', colon) - colon + 1), "msg_type");
            if (!mt.empty()) spec.msg_type = mt;
            map[key] = spec;
            pos = json.find('}', val_start) + 1;
        } else {
            size_t val_end = json.find('"', val_start + 1);
            if (val_end == std::string::npos) break;
            std::string val = json.substr(val_start + 1, val_end - val_start - 1);
            if (!key.empty() && !val.empty()) {
                CameraSpec spec;
                spec.topic = val;
                spec.msg_type = "Image2m";  // old format default
                map[key] = spec;
            }
            pos = val_end + 1;
        }
    }
    return map;
}

struct BridgeConfig {
    int zmq_port = 5557;
    std::string rgb_topic = "/ob_camera_head/color/image_raw";
    std::string depth_topic = "/ob_camera_head/depth/image_raw";
    // "Image2m" (shm_msgs/Image2m, default — walker S2) or "Image" (sensor_msgs/Image — 天工)
    std::string msg_type = "Image2m";
    // camera_name → CameraSpec (multi-camera routing, Walker S2)
    std::map<std::string, CameraSpec> camera_topics;
};

static BridgeConfig parse_args(int argc, char* argv[]) {
    BridgeConfig cfg;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "--zmq-port") && i + 1 < argc) {
            cfg.zmq_port = std::stoi(argv[++i]);
        } else if ((arg == "--rgb-topic") && i + 1 < argc) {
            cfg.rgb_topic = argv[++i];
        } else if ((arg == "--depth-topic") && i + 1 < argc) {
            cfg.depth_topic = argv[++i];
        } else if ((arg == "--msg-type") && i + 1 < argc) {
            cfg.msg_type = argv[++i];
        } else if ((arg == "--camera-topics") && i + 1 < argc) {
            cfg.camera_topics = parse_camera_topics(argv[++i]);
        } else if (arg == "--help") {
            std::cout << "Usage: zmq_image_bridge [OPTIONS]\n"
                      << "Options:\n"
                      << "  --zmq-port PORT       ZMQ image port (default: 5557)\n"
                      << "  --rgb-topic TOPIC     RGB image topic (default: /ob_camera_head/color/image_raw)\n"
                      << "  --depth-topic TOPIC   Depth image topic (default: /ob_camera_head/depth/image_raw)\n"
                      << "  --msg-type TYPE       Image type: Image (sensor_msgs/Image) or Image2m (shm_msgs/Image2m, default)\n";
            exit(0);
        }
    }
    return cfg;
}

static void set_shm_string(shm_msgs::msg::String& dst, const std::string& src) {
    std::fill(dst.data.begin(), dst.data.end(), '\0');
    const size_t n = std::min(src.size(), dst.data.size());
    std::memcpy(dst.data.data(), src.data(), n);
    dst.size = static_cast<uint8_t>(std::min<size_t>(n, 255));
}

// Template: fill any shm_msgs Image* message (Image1m/Image2m/Image6m/…)
template<typename MsgT>
static bool fill_image_shm(
    MsgT& msg,
    const rclcpp::Time& stamp,
    const std::string& frame_id,
    int width,
    int height,
    const std::string& encoding,
    uint32_t step,
    const void* data,
    size_t size)
{
    const size_t byte_count = static_cast<size_t>(height) * step;
    if (size < byte_count) {
        return false;
    }
    if (byte_count > msg.data.size()) {
        return false;
    }

    msg.header.stamp = stamp;
    set_shm_string(msg.header.frame_id, frame_id);
    msg.height = static_cast<uint32_t>(height);
    msg.width = static_cast<uint32_t>(width);
    set_shm_string(msg.encoding, encoding);
    msg.is_bigendian = 0;
    msg.step = step;
    std::fill(msg.data.begin(), msg.data.end(), 0);
    std::memcpy(msg.data.data(), data, byte_count);
    return true;
}

// Keep a named alias for backward-compatible calls (fill_image2m → fill_image_shm<Image2m>)
static bool fill_image2m(
    shm_msgs::msg::Image2m& msg,
    const rclcpp::Time& stamp,
    const std::string& frame_id,
    int width, int height,
    const std::string& encoding,
    uint32_t step,
    const void* data, size_t size)
{
    return fill_image_shm<shm_msgs::msg::Image2m>(msg, stamp, frame_id, width, height, encoding, step, data, size);
}

// Fill a sensor_msgs::msg::Image (variable-length data, no 2MB cap). Used for 天工.
static bool fill_image(
    sensor_msgs::msg::Image& msg,
    const rclcpp::Time& stamp,
    const std::string& frame_id,
    int width,
    int height,
    const std::string& encoding,
    uint32_t step,
    const void* data,
    size_t size)
{
    const size_t byte_count = static_cast<size_t>(height) * step;
    if (size < byte_count) {
        return false;
    }

    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.height = static_cast<uint32_t>(height);
    msg.width = static_cast<uint32_t>(width);
    msg.encoding = encoding;
    msg.is_bigendian = 0;
    msg.step = step;
    const uint8_t* src = static_cast<const uint8_t*>(data);
    msg.data.assign(src, src + byte_count);
    return true;
}

class ZmqImageBridge : public rclcpp::Node
{
public:
    ZmqImageBridge(const BridgeConfig& cfg) : Node("zmq_image_bridge"), msg_type_(cfg.msg_type)
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
        if (msg_type_ == "Image") {
            pub_rgb_img_ = this->create_publisher<sensor_msgs::msg::Image>(cfg.rgb_topic, qos);
            pub_depth_img_ = this->create_publisher<sensor_msgs::msg::Image>(cfg.depth_topic, qos);
        } else {
            // default: Image2m (shm_msgs) — walker S2 path
            msg_type_ = "Image2m";
            pub_rgb_2m_ = this->create_publisher<shm_msgs::msg::Image2m>(cfg.rgb_topic, qos);
            pub_depth_2m_ = this->create_publisher<shm_msgs::msg::Image2m>(cfg.depth_topic, qos);
        }

        // Per-camera publishers (multi-camera routing, type-aware)
        for (const auto& [cam_name, spec] : cfg.camera_topics) {
            if (spec.msg_type == "Image1m") {
                auto pub = this->create_publisher<shm_msgs::msg::Image1m>(spec.topic, qos);
                publishers_1m_[cam_name] = pub;
            } else if (spec.msg_type == "Image6m") {
                auto pub = this->create_publisher<shm_msgs::msg::Image6m>(spec.topic, qos);
                publishers_6m_[cam_name] = pub;
            } else {
                auto pub = this->create_publisher<shm_msgs::msg::Image2m>(spec.topic, qos);
                publishers_2m_[cam_name] = pub;
            }
            RCLCPP_INFO(this->get_logger(), "  Camera '%s' → %s (%s)",
                        cam_name.c_str(), spec.topic.c_str(), spec.msg_type.c_str());
        }

        context_ = zmq::context_t(1);
        subscriber_ = zmq::socket_t(context_, ZMQ_SUB);

        std::string zmq_addr = "tcp://127.0.0.1:" + std::to_string(cfg.zmq_port);
        RCLCPP_INFO(this->get_logger(), "Connecting to ZMQ Image Server at %s", zmq_addr.c_str());
        subscriber_.connect(zmq_addr);
        subscriber_.set(zmq::sockopt::subscribe, "");
        subscriber_.set(zmq::sockopt::rcvhwm, 8);
        // RCVTIMEO: recv 带超时，避免无帧时永久阻塞致 stop() 的 join() 死锁。
        // 100ms：正常 30Hz 流(33ms/帧)不会触发，仅在空闲/暂停时定期醒来检查 running_。
        subscriber_.set(zmq::sockopt::rcvtimeo, 100);

        RCLCPP_INFO(this->get_logger(),
                    "C++ ZMQ Image Bridge Started (msg_type=%s, rgb: %s, depth: %s, cameras: %zu)",
                    msg_type_.c_str(), cfg.rgb_topic.c_str(), cfg.depth_topic.c_str(),
                    cfg.camera_topics.size());

        // Keep rclcpp::spin() alive — the executor needs at least one timer/subscription
        // as a work item, otherwise spin() returns immediately and the process exits.
        // The actual frame processing happens in receive_loop (separate std::thread).
        keepalive_timer_ = this->create_wall_timer(
            std::chrono::seconds(60), []() {});

        receive_thread_ = std::thread(&ZmqImageBridge::receive_loop, this);
    }

    ~ZmqImageBridge()
    {
        running_ = false;
        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }
    }

private:
    void receive_loop()
    {
        while (rclcpp::ok() && running_) {
            zmq::message_t meta_msg, rgb_msg, depth_msg;

            try {
                auto res = subscriber_.recv(meta_msg, zmq::recv_flags::none);
                if (!res) continue;

                if (!subscriber_.get(zmq::sockopt::rcvmore)) continue;
                (void)subscriber_.recv(rgb_msg, zmq::recv_flags::none);

                bool has_depth = false;
                if (subscriber_.get(zmq::sockopt::rcvmore)) {
                    (void)subscriber_.recv(depth_msg, zmq::recv_flags::none);
                    has_depth = true;
                }

                publish_images(meta_msg, rgb_msg, depth_msg, has_depth);

            } catch (const zmq::error_t& e) {
                // EAGAIN 是 RCVTIMEO 无帧超时，静默重试以检查 running_/rclcpp::ok()；
                // 其余 ZMQ 错误才记录并退避。
                if (zmq_errno() != EAGAIN) {
                    RCLCPP_ERROR(this->get_logger(), "ZMQ Error: %s", e.what());
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Unhandled exception in receive loop: %s", e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "Unknown exception in receive loop");
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }

    // Helper: allocate a shm_msgs image on the heap and fill + publish it.
    // shm_msgs::msg::Image*M contain multi-MB fixed arrays; they MUST live on
    // the heap — stacking three of them (Image1m ~1 MB + Image2m ~2 MB +
    // Image6m ~6 MB) blows past the default 8 MB pthread stack and segfaults.
    template<typename MsgT>
    bool _publish_shm(MsgT& msg, const std::string& cam_name,
                      const rclcpp::Time& stamp, int w, int h,
                      const void* data, size_t size,
                      typename rclcpp::Publisher<MsgT>::SharedPtr pub)
    {
        if (fill_image_shm(msg, stamp, cam_name, w, h, "rgb8",
                           static_cast<uint32_t>(w * 3), data, size)) {
            pub->publish(msg);
            return true;
        }
        return false;
    }

    void publish_images(zmq::message_t& meta_msg, zmq::message_t& rgb_msg,
                        zmq::message_t& depth_msg, bool has_depth)
    {
        std::string meta_str(static_cast<char*>(meta_msg.data()), meta_msg.size());
        int w = tinyjson::get_int(meta_str, "width");
        int h = tinyjson::get_int(meta_str, "height");
        if (w <= 0 || h <= 0) {
            RCLCPP_WARN(this->get_logger(), "Invalid metadata, skipping frame");
            return;
        }

        std::string cam_name = tinyjson::get_string(meta_str, "camera");
        auto current_time = this->now();

        // --- Path A: no "camera" field → fallback to default rgb_topic (old controller / 天工 Pro) ---
        if (cam_name.empty()) {
            if (msg_type_ == "Image" && pub_rgb_img_) {
                sensor_msgs::msg::Image img_msg;  // uses std::vector — OK on stack
                if (fill_image(img_msg, current_time, "camera", w, h, "rgb8",
                               static_cast<uint32_t>(w * 3), rgb_msg.data(), rgb_msg.size())) {
                    pub_rgb_img_->publish(img_msg);
                }
                if (has_depth && depth_msg.size() > 0 && pub_depth_img_) {
                    sensor_msgs::msg::Image d_msg;
                    if (fill_image(d_msg, current_time, "camera_depth", w, h, "16UC1",
                                   static_cast<uint32_t>(w * 2), depth_msg.data(), depth_msg.size())) {
                        pub_depth_img_->publish(d_msg);
                    }
                }
            } else if (pub_rgb_2m_) {
                auto img_msg = std::make_unique<shm_msgs::msg::Image2m>();
                if (fill_image2m(*img_msg, current_time, "camera", w, h, "rgb8",
                                 static_cast<uint32_t>(w * 3), rgb_msg.data(), rgb_msg.size())) {
                    pub_rgb_2m_->publish(*img_msg);
                }
                if (has_depth && depth_msg.size() > 0 && pub_depth_2m_) {
                    auto d_msg = std::make_unique<shm_msgs::msg::Image2m>();
                    if (fill_image2m(*d_msg, current_time, "camera_depth", w, h, "16UC1",
                                     static_cast<uint32_t>(w * 2), depth_msg.data(), depth_msg.size())) {
                        pub_depth_2m_->publish(*d_msg);
                    }
                }
            }
            return;
        }

        // --- Path B: "camera" field present → route to per-camera publisher (type-aware) ---
        // Allocate the heavy shm_msgs image on the heap (see class comment).
        bool published = false;
        auto it1 = publishers_1m_.find(cam_name);
        if (it1 != publishers_1m_.end()) {
            auto img_msg = std::make_unique<shm_msgs::msg::Image1m>();
            if (_publish_shm(*img_msg, cam_name, current_time, w, h,
                            rgb_msg.data(), rgb_msg.size(), it1->second)) {
                published = true;
            }
        }
        auto it2 = publishers_2m_.find(cam_name);
        if (it2 != publishers_2m_.end()) {
            auto img_msg = std::make_unique<shm_msgs::msg::Image2m>();
            if (_publish_shm(*img_msg, cam_name, current_time, w, h,
                            rgb_msg.data(), rgb_msg.size(), it2->second)) {
                published = true;
            }
        }
        auto it6 = publishers_6m_.find(cam_name);
        if (it6 != publishers_6m_.end()) {
            auto img_msg = std::make_unique<shm_msgs::msg::Image6m>();
            if (_publish_shm(*img_msg, cam_name, current_time, w, h,
                            rgb_msg.data(), rgb_msg.size(), it6->second)) {
                published = true;
            }
        }
        if (!published) {
            RCLCPP_WARN(this->get_logger(), "Unknown camera '%s', dropping frame",
                        cam_name.c_str());
        }

        // stereo_left also publishes to legacy stereo rgb_topic for backward compat
        if (cam_name == "stereo_left" && pub_rgb_2m_) {
            auto stereo_msg = std::make_unique<shm_msgs::msg::Image2m>();
            if (fill_image2m(*stereo_msg, current_time, cam_name, w, h, "rgb8",
                             static_cast<uint32_t>(w * 3), rgb_msg.data(), rgb_msg.size())) {
                pub_rgb_2m_->publish(*stereo_msg);
            }
        }
    }

    // Only one pair is non-null, selected by msg_type_ at construction.
    rclcpp::Publisher<shm_msgs::msg::Image2m>::SharedPtr pub_rgb_2m_;
    rclcpp::Publisher<shm_msgs::msg::Image2m>::SharedPtr pub_depth_2m_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rgb_img_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_img_;
    // Per-camera publishers by type (multi-camera routing)
    std::map<std::string, rclcpp::Publisher<shm_msgs::msg::Image1m>::SharedPtr> publishers_1m_;
    std::map<std::string, rclcpp::Publisher<shm_msgs::msg::Image2m>::SharedPtr> publishers_2m_;
    std::map<std::string, rclcpp::Publisher<shm_msgs::msg::Image6m>::SharedPtr> publishers_6m_;
    std::string msg_type_;

    zmq::context_t context_;
    zmq::socket_t subscriber_;
    std::thread receive_thread_;
    std::atomic<bool> running_{true};
    rclcpp::TimerBase::SharedPtr keepalive_timer_;
};

int main(int argc, char * argv[])
{
    BridgeConfig cfg = parse_args(argc, argv);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ZmqImageBridge>(cfg);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
