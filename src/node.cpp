#include "node.hpp"

#include <chrono>
#include <cstdio>
#include <vector>

#include "xblick/camera.hpp"
#include "xblick/image.hpp"

namespace xblick::rc_node
{
    namespace
    {
        std::int64_t wall_ns()
        {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                .count();
        }

        std::string number(double v)
        {
            char text[32];
            std::snprintf(text, sizeof(text), "%.2f", v);
            return text;
        }
    }  // namespace

    struct Node::Impl
    {
        zenoh::Session* session = nullptr;
        NodeOptions o;
        StereoParameters stereo;
        std::string base;
        std::unique_ptr<CameraPublisher> left, right, depth;
        std::unique_ptr<StaticTransformPublisher> tf_static;
        std::unique_ptr<DiagnosticsPublisher> diagnostics;
        std::vector<std::uint8_t> pixels;     // a converted intensity image
        std::vector<std::uint16_t> depth_mm;  // a converted disparity image
        NodeStats stats, reported_stats;
        std::uint64_t reported_dropped = 0, reported_restarts = 0;
        std::chrono::steady_clock::time_point reported = std::chrono::steady_clock::now();

        std::unique_ptr<CameraPublisher> publisher(const std::string& name, const std::string& encoding,
                                                   const CameraInfo& info, const char* codec)
        {
            return std::make_unique<CameraPublisher>(
                *session, base + "/" + name,
                PublisherOptions{.encoding = encoding, .frame_id = info.header.frame_id, .source = o.source,
                                 .camera_info = info, .codec = codec, .slots = o.slots,
                                 .cuda_device = o.cuda_device});
        }

        void publish_intensity(std::unique_ptr<CameraPublisher>& to, const char* name, const RawImage& image,
                               std::int64_t stamp, bool right_camera, std::uint64_t& count)
        {
            if (not to)
            {
                const std::string frame = right_camera ? o.right_frame_id : o.left_frame_id;
                to = publisher(name, encoding(image), camera_info(stereo, image.width, image.height, frame,
                                                                  right_camera),
                               kJpeg);
            }
            if (not to->subscribed())
            {
                return;
            }
            const std::size_t channels = encoding(image) == "rgb8" ? 3 : 1;
            pixels.resize(std::size_t(image.width) * image.height * channels);
            to_pixels(image, pixels.data());
            to->publish(HostImage{pixels.data(), image.height, image.width, 0}, stamp);
            ++count;
        }
    };

    Node::Node(zenoh::Session& session, const NodeOptions& options, const StereoParameters& stereo)
        : impl_(std::make_unique<Impl>())
    {
        Impl& m = *impl_;
        m.session = &session;
        m.o = options;
        m.stereo = stereo;
        m.base = check_name(options.base);
        // the right camera in the left camera's frame: the baseline along x (both rectified, the same orientation)
        TransformStamped right;
        right.header = {Time::from_ns(wall_ns()), options.left_frame_id};
        right.child_frame_id = options.right_frame_id;
        right.transform = Transform::from_matrix({1, 0, 0, 0, 1, 0, 0, 0, 1}, Vector3{stereo.baseline, 0.0, 0.0});
        TFMessage transforms;
        transforms.transforms.push_back(right);
        m.tf_static = std::make_unique<StaticTransformPublisher>(session, m.base, transforms);
        m.diagnostics = std::make_unique<DiagnosticsPublisher>(session, m.base);
    }

    Node::~Node() = default;

    void Node::publish(const Grab& grab)
    {
        Impl& m = *impl_;
        ++m.stats.grabs;
        if (grab.left or grab.disparity)
        {
            m.stats.age_ms += double(wall_ns() - grab.stamp_ns) / 1e6;
        }
        if (grab.left)
        {
            ++m.stats.intensity;
            m.publish_intensity(m.left, "left", *grab.left, grab.stamp_ns, false, m.stats.left);
        }
        if (grab.right)
        {
            m.publish_intensity(m.right, "right", *grab.right, grab.stamp_ns, true, m.stats.right);
        }
        if (grab.disparity)
        {
            ++m.stats.disparity;
            const RawImage& d = *grab.disparity;
            if (not m.depth)
            {
                m.depth = m.publisher("depth", "16UC1", camera_info(m.stereo, d.width, d.height, m.o.left_frame_id,
                                                                    false),
                                      kRvl);
            }
            if (m.depth->subscribed())
            {
                m.depth_mm.resize(std::size_t(d.width) * d.height);
                m.stats.beyond += disparity_to_depth(d, m.stereo, m.depth_mm.data());
                m.depth->publish(HostImage{m.depth_mm.data(), d.height, d.width, 0}, grab.stamp_ns);
                ++m.stats.depth;
            }
        }
    }

    DiagnosticStatus Node::report(const FrameSource& source)
    {
        Impl& m = *impl_;
        const auto now = std::chrono::steady_clock::now();
        const double seconds = std::chrono::duration<double>(now - m.reported).count();
        const NodeStats& a = m.stats;
        const NodeStats& b = m.reported_stats;
        const std::uint64_t dropped = source.dropped() - m.reported_dropped;
        const std::uint64_t restarts = source.restarts() - m.reported_restarts;
        const std::uint64_t images = (a.intensity - b.intensity) + (a.disparity - b.disparity);
        DiagnosticStatus status;
        status.name = "roboception";
        status.values = {{"grabs/s", number(double(a.grabs - b.grabs) / seconds)},
                         {"intensity/s", number(double(a.intensity - b.intensity) / seconds)},
                         {"disparity/s", number(double(a.disparity - b.disparity) / seconds)},
                         {"left/s", number(double(a.left - b.left) / seconds)},
                         {"right/s", number(double(a.right - b.right) / seconds)},
                         {"depth/s", number(double(a.depth - b.depth) / seconds)},
                         {"age ms", number(images ? (a.age_ms - b.age_ms) / double(images) : 0.0)},
                         {"depth beyond 65535 mm", std::to_string(a.beyond - b.beyond)},
                         {"dropped", std::to_string(dropped)},
                         {"restarts", std::to_string(restarts)}};
        if (const std::optional<ClockSync> clock = source.clock_sync())
        {
            status.values.push_back({"clock offset ms", number(double(clock->offset_ns) / 1e6)});
            status.values.push_back({"clock round trip ms", number(double(clock->round_trip_ns) / 1e6)});
        }
        for (const auto& [key, value] : source.info())
        {
            status.values.push_back({key, value});
            if (key == "serial")
            {
                status.hardware_id = value;
            }
        }
        if (a.grabs == b.grabs)
        {
            status.level = DiagnosticStatus::kError;
            status.message = "no images";
        }
        else if (dropped or restarts)
        {
            status.level = DiagnosticStatus::kWarn;
            status.message = number(double(a.grabs - b.grabs) / seconds) + " grabs/s, " + std::to_string(dropped) +
                             " dropped, " + std::to_string(restarts) + " restarts";
        }
        else
        {
            status.message = number(double(a.grabs - b.grabs) / seconds) + " grabs/s";
        }
        DiagnosticArray diagnostics;
        diagnostics.header.stamp = Time::from_ns(wall_ns());
        diagnostics.status = {status};
        m.diagnostics->publish(diagnostics);
        m.reported = now;
        m.reported_stats = a;
        m.reported_dropped = source.dropped();
        m.reported_restarts = source.restarts();
        return status;
    }

    const NodeStats& Node::stats() const { return impl_->stats; }

    std::string describe(const NodeStats& now, const NodeStats& before, std::uint64_t dropped, std::uint64_t restarts,
                         double seconds)
    {
        const std::uint64_t images = (now.intensity - before.intensity) + (now.disparity - before.disparity);
        char text[256];
        std::snprintf(text, sizeof(text),
                      "%.1f grabs/s, dropped %llu, restarts %llu; left %llu right %llu depth %llu, age %.1f ms",
                      double(now.grabs - before.grabs) / seconds, static_cast<unsigned long long>(dropped),
                      static_cast<unsigned long long>(restarts),
                      static_cast<unsigned long long>(now.left - before.left),
                      static_cast<unsigned long long>(now.right - before.right),
                      static_cast<unsigned long long>(now.depth - before.depth),
                      images ? (now.age_ms - before.age_ms) / double(images) : 0.0);
        return text;
    }
}  // namespace xblick::rc_node
