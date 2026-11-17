// --ros without a camera. The program needs an rc_visard, so the streams are checked on the node itself (node.hpp)
// with a session that takes part in ROS 2 as the program's does with --ros (Options::session_options gives it
// SessionOptions::ros), fed with synthetic grabs (Mono8 left and right, a disparity image):
// - With ROS: what ROS 2 sees, read through Durchblick's ROS reader (Transport::Ros, ros::Subscriber): left, right and
//   depth with their camera_info, /tf_static, the diagnostics; every ROS data key with rmw_zenoh's attachment; the
//   publisher tokens of the node; no ROS form of the RVL depth.
// - Without: no key or token of rmw_zenoh.
// The program itself: --ros makes its session a ROS node before it looks for the device (it then stops, as there is
// none), a ROS option without --ros is refused.
#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <rc_genicam_api/pixel_formats.h>

#include "node.hpp"
#include "source.hpp"
#include "stereo.hpp"
#include "xblick/camera.hpp"
#include "xblick/protocol.hpp"
#include "xblick/ros.hpp"

using namespace std::chrono_literals;
using namespace xblick;
using namespace xblick::rc_node;

namespace
{
    int failures = 0;

    void expect(bool condition, const std::string& what)
    {
        std::fprintf(stderr, "%s: %s\n", condition ? "ok" : "FAILED", what.c_str());
        failures += condition ? 0 : 1;
    }

    bool wait_for(const std::function<bool()>& done, std::chrono::milliseconds timeout)
    {
        const auto end = std::chrono::steady_clock::now() + timeout;
        while (not done())
        {
            if (std::chrono::steady_clock::now() > end)
            {
                return false;
            }
            std::this_thread::sleep_for(20ms);
        }
        return true;
    }

    int free_port()
    {
        std::random_device random;
        return 20000 + int(random() % 20000);
    }

    constexpr int kDomain = 95;  // ROS_DOMAIN_ID of nothing else here
    const std::string kBase = "t/rc_ros";
    constexpr std::uint32_t kW = 64, kH = 48, kDW = 32, kDH = 24;
    const StereoParameters kStereo{.focal_length_factor = 0.8, .baseline = 0.065, .disparity_scale = 1.0 / 16.0};

    struct Images
    {
        std::vector<std::uint8_t> left = std::vector<std::uint8_t>(kW * kH, 60);
        std::vector<std::uint8_t> right = std::vector<std::uint8_t>(kW * kH, 120);
        std::vector<std::uint8_t> disparity = std::vector<std::uint8_t>(kDW * kDH * 2);

        Images()
        {
            for (std::size_t i = 0; i < kDW * kDH; ++i)
            {
                const std::uint16_t d = std::uint16_t(16 * (20 + i % kDW));
                disparity[2 * i] = std::uint8_t(d & 0xFF);
                disparity[2 * i + 1] = std::uint8_t(d >> 8);
            }
        }

        Grab grab(std::int64_t stamp) const
        {
            Grab g;
            g.stamp_ns = stamp;
            g.left = RawImage{left.data(), Mono8, kW, kH, 0, false};
            g.right = RawImage{right.data(), Mono8, kW, kH, 0, false};
            g.disparity = RawImage{disparity.data(), Coord3D_C16, kDW, kDH, 0, false};
            return g;
        }
    };

    struct FakeSource : FrameSource
    {
        std::optional<Grab> next(std::chrono::milliseconds) override { return std::nullopt; }
        const StereoParameters& stereo() const override { return kStereo; }
        std::uint64_t dropped() const override { return 0; }
        std::uint64_t restarts() const override { return 0; }
        std::vector<std::pair<std::string, std::string>> info() const override
        {
            return {{"model", "test"}, {"serial", "12345678"}};
        }
    };

    std::int64_t now_ns()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::system_clock::now().time_since_epoch())
            .count();
    }

    // Every key the session sees (data and liveliness), with the attachment of the last sample on each data key.
    struct Observer
    {
        Observer(zenoh::Session& session, const std::string& data, const std::string& tokens)
        {
            data_.emplace(session.declare_subscriber(
                zenoh::KeyExpr(data),
                [this](const zenoh::Sample& sample) {
                    const auto a = sample.get_attachment();
                    std::lock_guard lock(mutex_);
                    attachments_[std::string(sample.get_keyexpr().as_string_view())] =
                        a ? a->get().as_vector() : std::vector<std::uint8_t>{};
                },
                zenoh::closures::none));
            zenoh::Session::LivelinessSubscriberOptions options;
            options.history = true;
            tokens_.emplace(session.liveliness_declare_subscriber(
                zenoh::KeyExpr(tokens),
                [this](const zenoh::Sample& sample) {
                    if (sample.get_kind() == Z_SAMPLE_KIND_PUT)
                    {
                        std::lock_guard lock(mutex_);
                        token_keys_.insert(std::string(sample.get_keyexpr().as_string_view()));
                    }
                },
                zenoh::closures::none, std::move(options)));
        }

        std::map<std::string, std::vector<std::uint8_t>> data()
        {
            std::lock_guard lock(mutex_);
            return attachments_;
        }
        std::set<std::string> tokens()
        {
            std::lock_guard lock(mutex_);
            return token_keys_;
        }

    private:
        std::mutex mutex_;
        std::map<std::string, std::vector<std::uint8_t>> attachments_;
        std::set<std::string> token_keys_;
        std::optional<zenoh::Subscriber<void>> data_, tokens_;
    };

    std::string ros_key(const std::string& key, const std::string& type)
    {
        return ros::data_key(kDomain, ros::topic_of(key), type, ros::hash_of(type));
    }

    // ------------------------------------------------------------------------------------------ the node, with ROS

    void test_node_with_ros()
    {
        const int port = free_port();
        const std::string endpoint = "127.0.0.1:" + std::to_string(port);
        // the node's session as the program's with --ros --ros-domain 95
        zenoh::Session node_session = open_session(
            {.listen = {endpoint}, .declare = false, .ros = ros::Options{.domain = kDomain, .node = "xblick_roboception"}});
        zenoh::Session reader = open_session(
            {.connect = {endpoint}, .declare = false, .ros = ros::Options{.domain = kDomain, .node = "rc_ros_test"}});
        const std::shared_ptr<ros::Ros> part = ros::of(reader);
        Observer observer(reader, std::to_string(kDomain) + "/**", "@ros2_lv/" + std::to_string(kDomain) + "/**");
        std::mutex mutex;
        std::optional<TFMessage> tf_static;
        std::vector<DiagnosticStatus> statuses;
        // the C++ API reads ROS volatile only: the /tf_static reader takes the transforms as the node publishes them
        ros::Subscriber tf_reader(reader, *part, part->node(), "/tf_static",
                                  std::string(protocol::kTFMessageType.name),
                                  [&](std::vector<std::uint8_t>&& payload, const ros::Attachment&) {
                                      std::lock_guard lock(mutex);
                                      tf_static = TFMessage::deserialize(payload);
                                  });
        ros::Subscriber diagnostics_reader(reader, *part, part->node(), ros::topic_of(kBase + "/diagnostics"),
                                           std::string(protocol::kDiagnosticArrayType.name),
                                           [&](std::vector<std::uint8_t>&& payload, const ros::Attachment&) {
                                               std::lock_guard lock(mutex);
                                               for (const DiagnosticStatus& s :
                                                    DiagnosticArray::deserialize(payload).status)
                                               {
                                                   statuses.push_back(s);
                                               }
                                           });
        const SubscriberOptions through_ros{.transport = Transport::Ros, .camera_info = true};
        ImageSubscriber left(reader, kBase + "/left", through_ros);
        ImageSubscriber right(reader, kBase + "/right", through_ros);
        ImageSubscriber depth(reader, kBase + "/depth", through_ros);
        std::this_thread::sleep_for(500ms);  // the readers reach the node's session

        const Images images;
        Node node(node_session, NodeOptions{.base = kBase, .source = "test"}, kStereo);
        std::optional<Frame> l, r, d;
        const auto start = std::chrono::steady_clock::now();
        for (int seq = 0; std::chrono::steady_clock::now() - start < 10s and not(l and r and d); ++seq)
        {
            node.publish(images.grab(now_ns()));
            std::this_thread::sleep_for(40ms);
            l = l ? l : left.latest(true, 0ms);
            r = r ? r : right.latest(true, 0ms);
            d = d ? d : depth.latest(true, 0ms);
        }
        // the camera_info of a frame comes on its own topic: wait for it, publishing on
        wait_for(
            [&] {
                node.publish(images.grab(now_ns()));
                return left.camera_info() and right.camera_info() and depth.camera_info();
            },
            5s);
        expect(l and l->encoding == "mono8" and l->width == kW and l->height == kH and l->data() != nullptr and
                   l->data()[0] == 60 and l->header.frame_id == "Roboception_LeftCamera" and not l->seq,
               "ROS: /" + kBase + "/left/image_raw, mono8, in the left camera's frame, no attachment of ours");
        expect(r and r->encoding == "mono8" and r->data() != nullptr and r->data()[0] == 120 and
                   r->header.frame_id == "Roboception_RightCamera",
               "ROS: /" + kBase + "/right/image_raw in the right camera's frame");
        expect(d and d->encoding == "16UC1" and d->width == kDW and d->height == kDH,
               "ROS: /" + kBase + "/depth/image_raw, 16UC1 at the disparity's resolution");
        const std::optional<CameraInfo> ri = right.camera_info(), di = depth.camera_info();
        expect(left.camera_info() and ri and std::abs(ri->p[3] - -0.8 * kW * 0.065) < 1e-9 and di and
                   di->width == kDW,
               "ROS: the camera_info of left, right (P with the baseline) and depth");

        node.report(FakeSource{});
        const bool came = wait_for(
            [&] {
                std::lock_guard lock(mutex);
                return tf_static and not statuses.empty();
            },
            5s);
        {
            std::lock_guard lock(mutex);
            expect(came and tf_static->transforms.size() == 1 and
                       tf_static->transforms[0].header.frame_id == "Roboception_LeftCamera" and
                       tf_static->transforms[0].child_frame_id == "Roboception_RightCamera" and
                       tf_static->transforms[0].transform.translation.x == 0.065,
                   "ROS: /tf_static, the right camera 0.065 m along the left camera's x");
            expect(came and statuses.front().name == "roboception" and statuses.front().hardware_id == "12345678",
                   "ROS: /" + kBase + "/diagnostics");
        }

        // rmw_zenoh's keys with its attachment; the JPEG forms of left and right, no form of the RVL depth
        std::vector<std::pair<std::string, std::string>> topics{
            {kBase + "/diagnostics", "diagnostic_msgs/msg/DiagnosticArray"},
            {"tf_static", "tf2_msgs/msg/TFMessage"},
            {kBase + "/left/image_raw/compressed", "sensor_msgs/msg/CompressedImage"},
            {kBase + "/right/image_raw/compressed", "sensor_msgs/msg/CompressedImage"}};
        for (const char* stream : {"left", "right", "depth"})
        {
            topics.emplace_back(kBase + "/" + stream + "/image_raw", "sensor_msgs/msg/Image");
            topics.emplace_back(kBase + "/" + stream + "/camera_info", "sensor_msgs/msg/CameraInfo");
        }
        wait_for(
            [&] {
                node.publish(images.grab(now_ns()));
                return observer.data().size() >= topics.size();
            },
            5s);
        const auto data = observer.data();
        for (const auto& [key, type] : topics)
        {
            const auto it = data.find(ros_key(key, type));
            bool attached = false;
            if (it != data.end())
            {
                try
                {
                    const ros::Attachment a = ros::decode_attachment(it->second);
                    attached = a.seq >= 1 and std::abs(now_ns() - a.source_ns) < 60'000'000'000;
                }
                catch (const ros::RosError&)
                {
                }
            }
            expect(attached, "ROS: " + ros_key(key, type) + " with rmw_zenoh's attachment");
        }
        expect(not data.contains(ros_key(kBase + "/depth/image_raw/compressed", "sensor_msgs/msg/CompressedImage")),
               "ROS: no ROS form of the RVL depth");
        std::set<std::string> publishers;
        bool node_token = false;
        for (const std::string& key : observer.tokens())
        {
            const ros::Token token = ros::Token::parse(key);
            node_token = node_token or (token.kind == ros::kNode and token.fully_qualified_node() == "/xblick_roboception");
            if (token.kind == ros::kPublisher and token.node == "xblick_roboception")
            {
                publishers.insert(*token.topic + " " + *token.type_name +
                                  (token.qos and token.qos->durability == ros::kTransientLocal ? " transient" : ""));
            }
        }
        expect(node_token and publishers.contains("/" + kBase + "/left/image_raw sensor_msgs/msg/Image") and
                   publishers.contains("/" + kBase + "/depth/image_raw sensor_msgs/msg/Image") and
                   publishers.contains("/tf_static tf2_msgs/msg/TFMessage transient"),
               "ROS: the node /xblick_roboception and its publisher tokens, /tf_static transient local (" +
                   std::to_string(publishers.size()) + " publishers)");
    }

    // --------------------------------------------------------------------------------------- the node, without

    void test_node_without_ros()
    {
        const int port = free_port();
        const std::string endpoint = "127.0.0.1:" + std::to_string(port);
        zenoh::Session node_session = open_session({.listen = {endpoint}, .declare = false});
        zenoh::Session reader = open_session({.connect = {endpoint}, .declare = false});
        Observer observer(reader, "**", "@ros2_lv/**");
        std::this_thread::sleep_for(500ms);
        const Images images;
        Node node(node_session, NodeOptions{.base = kBase, .source = "test"}, kStereo);
        const bool frames = wait_for(
            [&] {
                node.publish(images.grab(now_ns()));
                return observer.data().contains(kBase + "/depth/image_raw");
            },
            10s);
        node.report(FakeSource{});
        std::this_thread::sleep_for(1s);
        expect(frames, "without ROS: the streams on our keys");
        std::size_t ros_keys = 0;
        for (const auto& [key, _] : observer.data())
        {
            ros_keys += key.find("::dds_::") != std::string::npos or key.rfind(std::to_string(kDomain) + "/", 0) == 0;
        }
        expect(ros_keys == 0 and observer.tokens().empty(), "without ROS: no data key and no token of rmw_zenoh");
    }

    // ------------------------------------------------------------------------------------------------ the program

    // The program's exit status and output (stdout and stderr); killed after 30 s.
    std::pair<int, std::string> run(std::vector<std::string> argv, const std::string& log)
    {
        const pid_t child = ::fork();
        if (child == 0)
        {
            const int fd = ::open(log.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
            ::dup2(fd, 1);
            ::dup2(fd, 2);
            std::vector<char*> args;
            for (std::string& a : argv)
            {
                args.push_back(a.data());
            }
            args.push_back(nullptr);
            ::execv(args[0], args.data());
            std::_Exit(127);
        }
        int status = 0;
        for (int i = 0; ::waitpid(child, &status, WNOHANG) == 0; ++i)
        {
            if (i == 600)
            {
                ::kill(child, SIGKILL);
                ::waitpid(child, &status, 0);
                break;
            }
            std::this_thread::sleep_for(50ms);
        }
        std::ifstream in(log);
        std::stringstream text;
        text << in.rdbuf();
        return {WIFEXITED(status) ? WEXITSTATUS(status) : -1, text.str()};
    }

    void test_program()
    {
        const std::string tmp = RC_TEST_TMP;
        const auto [refused, refused_text] = run({RC_PROGRAM, "--device", "x", "--ros-domain", "3"},
                                                 tmp + "/test_ros_refused.log");
        expect(refused != 0 and refused_text.find("need --ros") != std::string::npos,
               "program: --ros-domain without --ros stops it with the reason");

        // a peer to connect to; the device is not there, so the program stops after its session opened
        const int port = free_port();
        zenoh::Session peer = open_session({.listen = {"127.0.0.1:" + std::to_string(port)}, .declare = false});
        const std::string connect = "127.0.0.1:" + std::to_string(port);
        const auto [with, with_text] =
            run({RC_PROGRAM, "--device", "no_such_device", "--connect", connect, "--ros", "--ros-domain",
                 std::to_string(kDomain)},
                tmp + "/test_ros_program.log");
        expect(with != 0 and with_text.find("in ROS 2 as node /xblick_roboception on domain " +
                                            std::to_string(kDomain)) != std::string::npos,
               "program: --ros makes its session the ROS node /xblick_roboception");
        const auto [without, without_text] =
            run({RC_PROGRAM, "--device", "no_such_device", "--connect", connect}, tmp + "/test_ros_program_plain.log");
        expect(without != 0 and without_text.find("ROS") == std::string::npos,
               "program: without --ros its session takes no part in ROS");
    }
}  // namespace

int main()
{
    try
    {
        test_program();
        test_node_without_ros();
        test_node_with_ros();
    }
    catch (const std::exception& e)
    {
        std::fprintf(stderr, "FAILED with an exception: %s\n", e.what());
        return 1;
    }
    std::fprintf(stderr, failures ? "%d failures\n" : "ok\n", failures);
    return failures ? 1 : 0;
}
