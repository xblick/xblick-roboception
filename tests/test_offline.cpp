// Tests without a camera: the conversions (stereo.hpp) against the SDK's own (rcg::convYCbCr411toRGB) and its point
// cloud's formulas (rcg::storePointCloud), the node end to end on images as the camera sends them (IntensityCombined
// in YCbCr411_8 with padding, disparity in Coord3D_C16), and, when Durchblick was built with it, Durchblick's point
// cloud node on the node's depth and left streams against the SDK's point cloud.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <map>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <sys/wait.h>
#include <unistd.h>

#include <rc_genicam_api/image.h>
#include <rc_genicam_api/pixel_formats.h>

#include "xblick/camera.hpp"
#include "xblick/points.hpp"
#include "controls.hpp"
#include "node.hpp"
#include "stereo.hpp"

using namespace std::chrono_literals;
using namespace xblick;
using namespace xblick::rc_node;

namespace
{
    int failures = 0;

    void expect(bool condition, const std::string& what)
    {
        if (not condition)
        {
            std::fprintf(stderr, "FAILED: %s\n", what.c_str());
            ++failures;
        }
    }

    template <typename F>
    bool throws(F&& f)
    {
        try
        {
            f();
        }
        catch (const std::exception&)
        {
            return true;
        }
        return false;
    }

    // the camera's images: 64 x 48 per eye (IntensityCombined 64 x 96), disparity 32 x 24 (half the resolution, as
    // the depth qualities below Full are)
    constexpr std::uint32_t kW = 64, kH = 48, kDW = 32, kDH = 24, kPad = 8, kDPad = 4;
    const StereoParameters kStereo{.focal_length_factor = 0.8, .baseline = 0.065, .disparity_scale = 1.0 / 16.0};

    struct CameraImages
    {
        std::vector<std::uint8_t> combined;   // YCbCr411_8, rows of kW * 6 / 4 + kPad bytes, left above right
        std::vector<std::uint8_t> disparity;  // Coord3D_C16 little endian, rows of kDW * 2 + kDPad bytes

        RawImage combined_image() const
        {
            return {combined.data(), YCbCr411_8, kW, 2 * kH, kPad, false};
        }
        RawImage disparity_image() const { return {disparity.data(), Coord3D_C16, kDW, kDH, kDPad, false}; }
    };

    std::uint16_t raw_disparity(std::uint32_t u, std::uint32_t v)
    {
        // a tilted plane between about 0.6 and 1.6 m, with holes
        if ((u + 3 * v) % 11 == 0)
        {
            return 0;
        }
        return std::uint16_t(16 * (20 + u) + 5 * v);
    }

    CameraImages camera_images()
    {
        CameraImages images;
        std::mt19937 random(7);
        images.combined.resize((kW / 4 * 6 + kPad) * 2 * kH);
        for (auto& b : images.combined)
        {
            b = std::uint8_t(random());
        }
        images.disparity.resize((kDW * 2 + kDPad) * kDH, 0xEE);  // the padding stays 0xEE
        for (std::uint32_t v = 0; v < kDH; ++v)
        {
            for (std::uint32_t u = 0; u < kDW; ++u)
            {
                const std::uint16_t d = raw_disparity(u, v);
                std::uint8_t* p = images.disparity.data() + v * (kDW * 2 + kDPad) + 2 * u;
                p[0] = std::uint8_t(d & 0xFF);
                p[1] = std::uint8_t(d >> 8);
            }
        }
        return images;
    }

    // the depth the SDK's point cloud gives a disparity pixel (rcg::storePointCloud: z = f t / d, f in pixels)
    double sdk_z(std::uint32_t u, std::uint32_t v)
    {
        return kStereo.focal_length_factor * kDW * kStereo.baseline / (kStereo.disparity_scale * raw_disparity(u, v));
    }

    void test_conversions()
    {
        const CameraImages images = camera_images();
        const RawImage combined = images.combined_image();
        // colour: every pixel of both halves as the SDK converts it
        std::vector<std::uint8_t> left(kW * kH * 3), right(kW * kH * 3);
        to_pixels(top_half(combined), left.data());
        to_pixels(bottom_half(combined), right.data());
        const std::size_t row = kW / 4 * 6 + kPad;
        bool exact = true;
        for (std::uint32_t v = 0; v < kH; ++v)
        {
            for (std::uint32_t u = 0; u < kW; ++u)
            {
                std::uint8_t l[3], r[3];
                rcg::convYCbCr411toRGB(l, images.combined.data() + v * row, int(u));
                rcg::convYCbCr411toRGB(r, images.combined.data() + (kH + v) * row, int(u));
                exact = exact and std::memcmp(l, &left[3 * (v * kW + u)], 3) == 0 and
                        std::memcmp(r, &right[3 * (v * kW + u)], 3) == 0;
            }
        }
        expect(exact and encoding(combined) == "rgb8",
               "left and right halves of IntensityCombined as the SDK's colour");
        // mono with padding
        std::vector<std::uint8_t> mono_raw((kW + kPad) * kH);
        for (std::size_t i = 0; i < mono_raw.size(); ++i)
        {
            mono_raw[i] = std::uint8_t(i * 7);
        }
        std::vector<std::uint8_t> mono(kW * kH);
        to_pixels({mono_raw.data(), Mono8, kW, kH, kPad, false}, mono.data());
        bool mono_exact = true;
        for (std::uint32_t v = 0; v < kH; ++v)
        {
            mono_exact = mono_exact and std::memcmp(&mono[v * kW], &mono_raw[v * (kW + kPad)], kW) == 0;
        }
        expect(mono_exact and encoding({mono_raw.data(), Mono8, kW, kH, kPad, false}) == "mono8",
               "Mono8 without its padding");
        expect(throws([&] { encoding({nullptr, Coord3D_C16, kW, kH, 0, false}); }) and
                   throws([&] { top_half({nullptr, Mono8, kW, 5, 0, false}); }),
               "a disparity image is no intensity image; an odd combined image is refused");

        // depth: z = f t / d in mm, 0 where the disparity is 0; big endian the same
        std::vector<std::uint16_t> depth(kDW * kDH);
        const std::size_t beyond = disparity_to_depth(images.disparity_image(), kStereo, depth.data());
        std::vector<std::uint8_t> swapped = images.disparity;
        for (std::size_t i = 0; i + 1 < swapped.size(); i += 2)
        {
            std::swap(swapped[i], swapped[i + 1]);
        }
        std::vector<std::uint16_t> depth_be(kDW * kDH);
        disparity_to_depth({swapped.data(), Coord3D_C16, kDW, kDH, kDPad, true}, kStereo, depth_be.data());
        bool depth_exact = beyond == 0 and depth == depth_be;
        std::size_t holes = 0;
        for (std::uint32_t v = 0; v < kDH; ++v)
        {
            for (std::uint32_t u = 0; u < kDW; ++u)
            {
                const std::uint16_t got = depth[v * kDW + u];
                if (raw_disparity(u, v) == 0)
                {
                    depth_exact = depth_exact and got == 0;
                    ++holes;
                }
                else
                {
                    depth_exact = depth_exact and got == std::uint16_t(std::lround(1000.0 * sdk_z(u, v)));
                }
            }
        }
        expect(depth_exact and holes > 30, "depth in mm from the disparity, 0 in the holes, either byte order");
        // z = 0.8 * 32 * 100 m / (d / 16) is beyond 65.535 m for every disparity below 625 (16 * (20 + u) + 5 v)
        const StereoParameters far{.focal_length_factor = 0.8, .baseline = 100.0, .disparity_scale = 1.0 / 16.0};
        expect(disparity_to_depth(images.disparity_image(), far, depth.data()) > 0, "depth beyond 65535 mm is counted");

        // camera models
        const CameraInfo l = camera_info(kStereo, kW, kH, "L", false);
        const CameraInfo r = camera_info(kStereo, kW, kH, "R", true);
        expect(l.k[0] == 0.8 * kW and l.k[4] == 0.8 * kW and l.k[2] == 31.5 and l.k[5] == 23.5 and l.p[3] == 0.0 and
                   r.p[3] == -0.8 * kW * 0.065 and r.header.frame_id == "R" and l.distortion_model == "plumb_bob",
               "camera models: f = FocalLengthFactor * width, the centre at (w - 1) / 2, the right P[3] = -f t");
    }

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

    Grab grab(const CameraImages& images, std::int64_t stamp)
    {
        Grab g;
        g.stamp_ns = stamp;
        g.left = top_half(images.combined_image());
        g.right = bottom_half(images.combined_image());
        g.disparity = images.disparity_image();
        return g;
    }

    void test_node(zenoh::Session& node_session, zenoh::Session& client, int port)
    {
        const CameraImages images = camera_images();
        Node node(node_session, NodeOptions{.base = "t/rc", .source = "test"}, kStereo);
        ImageSubscriber left(client, "t/rc/left", {.transport = Transport::Raw, .camera_info = true});
        ImageSubscriber right(client, "t/rc/right", {.transport = Transport::Raw, .camera_info = true});
        ImageSubscriber depth(client, "t/rc/depth", {.transport = Transport::Raw, .camera_info = true});
        std::optional<Frame> l, r, d;
        const auto start = std::chrono::steady_clock::now();
        for (int seq = 0; std::chrono::steady_clock::now() - start < 10s and not(l and r and d); ++seq)
        {
            node.publish(grab(images, 1000000000LL + seq * 40000000LL));
            std::this_thread::sleep_for(20ms);
            l = l ? l : left.latest(true, 0ms);
            r = r ? r : right.latest(true, 0ms);
            d = d ? d : depth.latest(true, 0ms);
        }
        expect(l and r and d, "left, right and depth arrive");
        if (not(l and r and d))
        {
            return;
        }
        std::vector<std::uint8_t> expected_left(kW * kH * 3), expected_right(kW * kH * 3);
        to_pixels(top_half(images.combined_image()), expected_left.data());
        to_pixels(bottom_half(images.combined_image()), expected_right.data());
        expect(l->encoding == "rgb8" and l->width == kW and l->height == kH and
                   std::memcmp(l->data(), expected_left.data(), expected_left.size()) == 0 and
                   l->header.frame_id == "Roboception_LeftCamera",
               "the left image arrives as converted, in the left camera's frame");
        expect(r->width == kW and std::memcmp(r->data(), expected_right.data(), expected_right.size()) == 0 and
                   r->header.frame_id == "Roboception_RightCamera",
               "the right image arrives, in the right camera's frame");
        std::vector<std::uint16_t> expected_depth(kDW * kDH);
        disparity_to_depth(images.disparity_image(), kStereo, expected_depth.data());
        expect(d->encoding == "16UC1" and d->width == kDW and d->height == kDH and
                   std::memcmp(d->data(), expected_depth.data(), expected_depth.size() * 2) == 0 and
                   d->header.frame_id == "Roboception_LeftCamera" and
                   (d->header.stamp.to_ns() - 1000000000LL) % 40000000LL == 0,
               "depth arrives in the left camera's frame with the camera's stamp of its grab");
        std::optional<CameraInfo> ri, di;
        for (int attempt = 0; attempt < 100 and not(ri and di); ++attempt)
        {
            std::this_thread::sleep_for(20ms);
            ri = ri ? ri : right.camera_info();
            di = di ? di : depth.camera_info();
        }
        expect(ri and ri->p[3] == -0.8 * kW * 0.065 and di and di->width == kDW and di->k[0] == 0.8 * kDW,
               "CameraInfo: the right camera's P with the baseline, depth's at the disparity's resolution");

        // tf_static: the right camera at +baseline along x in the left camera's frame
        std::optional<TFMessage> tf;
        for (int attempt = 0; attempt < 50 and not tf; ++attempt)
        {
            zenoh::Session::GetOptions options;
            options.timeout_ms = 500;
            auto replies = client.get(zenoh::KeyExpr("t/rc/tf_static"), "", zenoh::channels::FifoChannel(4),
                                      std::move(options));
            for (auto reply = replies.recv(); std::holds_alternative<zenoh::Reply>(reply); reply = replies.recv())
            {
                const auto& got = std::get<zenoh::Reply>(reply);
                if (got.is_ok())
                {
                    tf = TFMessage::deserialize(got.get_ok().get_payload().as_vector());
                }
            }
        }
        // auto: lookup()'s result is Pose up to Durchblick 0.3.2, RigidTransform from 0.3.3
        const auto pose = tf ? lookup(*tf, "Roboception_LeftCamera", "Roboception_RightCamera")
                             : decltype(lookup(*tf, "", "")){};
        expect(pose and pose->translation.x == 0.065 and pose->translation.y == 0.0 and pose->rotation[0] == 1.0,
               "tf_static: the right camera 0.065 m along the left camera's x");

        const DiagnosticStatus status = node.report(FakeSource{});
        expect(status.name == "roboception" and status.level == DiagnosticStatus::kOk and
                   status.hardware_id == "12345678",
               "diagnostics: status roboception, OK, the serial as hardware id");

#ifdef RC_POINTS_PROGRAM
        // Durchblick's point cloud node on depth and left: every point where the SDK's point cloud has one
        // (rcg::storePointCloud: x = (i + 0.5 - 0.5 w) t / d, y likewise, z = f t / d), up to the depth's rounding
        // to mm
        const pid_t child = ::fork();
        if (child == 0)
        {
            const std::string connect = "127.0.0.1:" + std::to_string(port);
            ::execl(RC_POINTS_PROGRAM, RC_POINTS_PROGRAM, "--connect", connect.c_str(), "--depth", "t/rc/depth",
                    "--color", "t/rc/left", "--cuda-device", "0", static_cast<char*>(nullptr));
            std::_Exit(127);
        }
        PointCloudSubscriber clouds(node_session, "t/rc/points", {.transport = Transport::Shm});
        std::optional<Cloud> cloud;
        const auto cloud_start = std::chrono::steady_clock::now();
        for (int seq = 0; std::chrono::steady_clock::now() - cloud_start < 30s and not cloud; ++seq)
        {
            node.publish(grab(images, 5000000000LL + seq * 40000000LL));
            cloud = clouds.latest(true, 30ms);
        }
        ::kill(child, SIGTERM);
        int child_status = 0;
        ::waitpid(child, &child_status, 0);
        expect(cloud.has_value(), "a point cloud of the node's depth and left image arrives");
        if (cloud)
        {
            const auto* points = reinterpret_cast<const float*>(cloud->data());
            std::size_t sdk = 0, agree = 0, extra = 0;
            double worst = 0.0;
            for (std::uint32_t v = 0; v < kDH; ++v)
            {
                for (std::uint32_t u = 0; u < kDW; ++u)
                {
                    const float* p = points + 4 * (std::size_t(v) * kDW + u);
                    if (raw_disparity(u, v) == 0)
                    {
                        extra += std::isfinite(p[2]) ? 1 : 0;
                        continue;
                    }
                    ++sdk;
                    const double t_d = kStereo.baseline / (kStereo.disparity_scale * raw_disparity(u, v));
                    const double x = (u + 0.5 - 0.5 * kDW) * t_d, y = (v + 0.5 - 0.5 * kDH) * t_d, z = sdk_z(u, v);
                    const double error =
                        std::max({std::abs(p[0] - x), std::abs(p[1] - y), std::abs(p[2] - z)}) * 1000.0;
                    worst = std::max(worst, error);
                    agree += error <= 0.6;  // z rounded to mm by the depth image (0.5), x, y scale with it; float
                }
            }
            std::fprintf(stderr,
                         "point cloud: %zu of %zu points within 0.6 mm of the SDK's (worst %.3f mm), %zu extra\n",
                         agree, sdk, worst, extra);
            expect(cloud->width == kDW and cloud->height == kDH and cloud->header.frame_id == "Roboception_LeftCamera",
                   "the cloud is organized as the depth image, in the left camera's frame");
            expect(sdk > 600 and agree == sdk and extra == 0, "every point where the SDK's point cloud has it");
        }
#else
        (void)port;
#endif
    }

    // a camera's settings as the device has them: exposure or gain switch its automatic exposure off, a ratio its
    // automatic white balance; it refuses values outside its ranges (as GenICam does)
    class SettingsSource : public FrameSource
    {
    public:
        std::optional<Grab> next(std::chrono::milliseconds) override { return std::nullopt; }
        const StereoParameters& stereo() const override { return stereo_; }
        std::uint64_t dropped() const override { return 0; }
        std::uint64_t restarts() const override { return 0; }
        std::vector<std::pair<std::string, std::string>> info() const override { return {}; }
        double setting(Setting s) const override { return values_.at(int(s)); }
        SettingRange range(Setting s) const override
        {
            switch (s)
            {
                case Setting::ExposureMs: return {0.066, 18.0};
                case Setting::GainDb: return {0.0, 18.0};
                case Setting::WhiteBalanceRed:
                case Setting::WhiteBalanceBlue: return {0.125, 8.0};
                default: return {0.0, 1.0};
            }
        }
        void set(Setting s, double v) override
        {
            const SettingRange r = range(s);
            if (v < r.min or v > r.max)
            {
                throw std::runtime_error("the camera refuses " + std::to_string(v));
            }
            if (s == Setting::ExposureMs or s == Setting::GainDb)
            {
                values_[int(Setting::AutoExposure)] = 0.0;
            }
            if (s == Setting::WhiteBalanceRed or s == Setting::WhiteBalanceBlue)
            {
                values_[int(Setting::AutoWhiteBalance)] = 0.0;
            }
            values_[int(s)] = v;
        }
        std::map<int, double> values_{{0, 1.0}, {1, 5.0}, {2, 3.0}, {3, 1.0}, {4, 1.0}, {5, 1.0}};  // a last run's

    private:
        StereoParameters stereo_;
    };

    void test_settings()
    {
        SettingsSource source;
        Parameters parameters(nullptr, "rc_visard", "cam");
        parameters.add_override("gain_db:=6.0");
        DeviceOptions start;  // exposure 12 ms, gain 0, white balance 1.2 2.4, both automatics off
        declare_settings(parameters, source, start);
        parameters.check_overrides();
        parameters.apply_all();  // the start's values on the device, the --param one over the option
        expect(source.values_[int(Setting::ExposureMs)] == 12.0, "exposure at the start");
        expect(source.values_[int(Setting::GainDb)] == 6.0, "gain from --param");
        expect(source.values_[int(Setting::AutoExposure)] == 0.0, "auto exposure off at the start");
        expect(source.values_[int(Setting::WhiteBalanceBlue)] == 2.4, "white balance at the start");
        expect(parameters.set("auto_exposure", ParameterValue::of(true)).successful, "auto exposure on");
        source.values_[int(Setting::ExposureMs)] = 7.5;  // what the camera chose
        expect(parameters.get("exposure_ms").double_value == 7.5, "the exposure the camera chose");
        expect(parameters.set("exposure_ms", ParameterValue::of(18.0)).successful, "exposure 18 ms");
        expect(not parameters.get("auto_exposure").bool_value, "setting the exposure switches auto off");
        expect(not parameters.set("exposure_ms", ParameterValue::of(40.0)).successful, "beyond the camera's range");
        expect(source.values_[int(Setting::ExposureMs)] == 18.0, "a refused value leaves the camera");
        expect(parameters.descriptor("exposure_ms").description.find("0.066 to 18 ms") != std::string::npos,
               "the range in the description");
        DeviceOptions mono;
        mono.color = false;
        Parameters grey(nullptr, "rc_visard", "cam");
        declare_settings(grey, source, mono);
        expect(not grey.has("white_balance_red"), "no white balance in Mono8");
    }
}  // namespace

int main()
{
    try
    {
        test_conversions();
        test_settings();
        std::random_device random;
        const int port = 20000 + int(random() % 20000);
        zenoh::Session node_session = open_session({.listen = {"127.0.0.1:" + std::to_string(port)}});
        zenoh::Session client = open_session({.connect = {"127.0.0.1:" + std::to_string(port)}});
        test_node(node_session, client, port);
    }
    catch (const std::exception& e)
    {
        std::fprintf(stderr, "FAILED with an exception: %s\n", e.what());
        return 1;
    }
    std::fprintf(stderr, failures ? "%d failures\n" : "ok\n", failures);
    return failures ? 1 : 0;
}
