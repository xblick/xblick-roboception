// xblick-roboception: a Roboception rc_visard as Durchblick streams (node.hpp).
//
//   xblick-roboception --device <id> [--base cam/rc_visard] [--fps 25]
//                          [--mono] [--no-right] [--no-depth] [--depth-quality Low|Medium|High|Full]
//                          [--auto-exposure | --exposure-ms 12] [--gain-db 0]
//                          [--auto-white-balance | --white-balance 1.2 2.4] [--no-ptp] [--sync-clock]
//                          [--left-frame-id Roboception_LeftCamera] [--right-frame-id Roboception_RightCamera]
//                          [--cuda-device 0] [--source robot] [--status-period 5]
//   xblick-roboception --list   (the devices the SDK sees)
//
// Zenoh: the PC's router (127.0.0.1:7447), or --listen/--connect ENDPOINT..., or --zenoh-config FILE
// (xblick::open_session). Every option is also a parameter of the node /<base> (xblick/options.hpp; --param and
// --params-file set them at the start): read-only, but exposure, gain and white balance, which change while the camera
// streams (controls.hpp: exposure_ms, gain_db, auto_exposure, white_balance_red, white_balance_blue,
// auto_white_balance, each in the camera's range; `xblick`, tab Parameters, or `xblick param`). The defaults:
// exposure 12 ms, white balance red 1.2, blue 2.4, colour, depth quality High, left, right and depth together. The
// SDK finds its GenTL producer through GENICAM_GENTL64_PATH. --sync-clock puts the stamps on the host's wall clock
// without a PTP master: the offset to the camera's clock is measured with TimestampLatch at the start and every second.
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <iostream>

#include "xblick/camera.hpp"
#include "xblick/options.hpp"
#include "xblick/parameters.hpp"
#include "controls.hpp"
#include "node.hpp"
#include "session.hpp"
#include "source.hpp"

namespace
{
    std::atomic<bool> stop{false};
}  // namespace

int main(int argc, char** argv)
{
    using namespace xblick;
    using namespace xblick::rc_node;
    std::signal(SIGINT, [](int) { stop = true; });
    std::signal(SIGTERM, [](int) { stop = true; });
    const DeviceOptions device_defaults;
    const NodeOptions node_defaults;
    Options options("xblick-roboception", "a Roboception rc_visard as Durchblick streams");
    options.add_flag("list", {.help = "list the devices the SDK sees, and end"});
    options.add("device", "", {.help = "the GenICam device ID, serial number or name (--list shows them)"});
    options.add("base", node_defaults.base, {.help = "the streams' base <namespace>/<camera>; the node /<base>"});
    options.add("fps", device_defaults.fps,
                {.help = "frames per second (AcquisitionFrameRate): 1 to 25, the rc_visard's range",
                 .parameter = {.unit = "Hz", .floating_range = FloatingPointRange{1.0, 25.0, 0.0}}});
    options.add_flag("mono", {.help = "Mono8 instead of colour"});
    options.add_flag("no-right", {.help = "the left image only"});
    options.add_flag("no-depth", {.help = "without depth"});
    options.add("depth-quality", device_defaults.depth_quality,
                {.help = "the depth image's size and its most frames per second: Low 214 x 160 (25 Hz), Medium "
                         "320 x 240 (15 Hz), High 640 x 480 (3 Hz), Full 1280 x 960 (0.8 Hz; needs its licence)",
                 .parameter = {.choices = {ParameterValue::of("Low"), ParameterValue::of("Medium"),
                                           ParameterValue::of("High"), ParameterValue::of("Full")}}});
    options.add_flag("auto-exposure", {.help = "the camera's automatic exposure, which sets the gain too (live)"});
    options.add_optional("exposure-ms", parameter_type::kDouble,
                         {.help = "the exposure without --auto-exposure (default 12; the rc_visard: 0.066 to 18; live)",
                          .metavar = "MS"});
    options.add("gain-db", device_defaults.gain_db,
                {.help = "the gain (the rc_visard: 0 to 18; 6 dB doubles the brightness; live)",
                 .parameter = {.unit = "dB"}});
    options.add_flag("auto-white-balance", {.help = "the camera's automatic white balance (live)"});
    options.add_list("white-balance", {},
                     {.help = "red and blue to green without --auto-white-balance (default 1.2 2.4; live as "
                              "white_balance_red and white_balance_blue)",
                      .metavar = "RED BLUE"});
    options.add_flag("no-ptp", {.help = "without PTP time synchronisation"});
    options.add_flag("sync-clock", {.help = "stamps on the host's wall clock: the offset to the camera's clock "
                                            "measured with TimestampLatch every second (without a PTP master)"});
    options.add("left-frame-id", node_defaults.left_frame_id, {.help = "the left camera's frame"});
    options.add("right-frame-id", node_defaults.right_frame_id, {.help = "the right camera's frame"});
    options.add_optional("cuda-device", parameter_type::kInteger, {.help = "also offer CUDA rings on this GPU"});
    options.add("source", node_defaults.source, {.help = "the source in every frame's attachment"});
    options.add("status-period", 5.0, {.help = "seconds between the status lines", .parameter = {.unit = "s"}});
    options.add_session_options();
    options.add_parameter_options();
    options.parse(argc, argv);
    try
    {
        if (options.flag("list"))
        {
            for (const std::string& line : list_devices())
            {
                std::cout << line << std::endl;
            }
            release_sdk();
            return 0;
        }
        // the parameters before the options are used: a value given with --param or a params file is the one used;
        // every option a read-only parameter of /<base> (the device takes these settings when it opens), but the
        // live ones, declared once the camera is open with its ranges (controls.hpp)
        zenoh::Session session = open_session(options.session_options());
        const std::string base = check_name(options.text("base"));
        const auto [node, ns] = node_name(base);
        Parameters parameters(session, node, ns);
        parameters.configure(options.parameter_startup());
        options.declare(parameters, {},
                        {"base", "list", "auto-exposure", "exposure-ms", "gain-db", "auto-white-balance",
                         "white-balance"});
        DeviceOptions d;
        d.device_id = options.text("device");
        if (d.device_id.empty())
        {
            throw std::invalid_argument("--device <GenICam ID, serial number or name> is needed (--list shows them)");
        }
        d.fps = options.number("fps");
        d.color = not options.flag("mono");
        d.right = not options.flag("no-right");
        d.depth = not options.flag("no-depth");
        d.depth_quality = options.text("depth-quality");
        d.auto_exposure = options.flag("auto-exposure");
        if (options.has_value("exposure-ms"))
        {
            if (d.auto_exposure)
            {
                throw std::invalid_argument("--exposure-ms and --auto-exposure exclude each other");
            }
            d.exposure_ms = options.number("exposure-ms");
        }
        d.gain_db = options.number("gain-db");
        d.auto_white_balance = options.flag("auto-white-balance");
        const std::vector<std::string> balance = options.list("white-balance");
        if (not balance.empty())
        {
            if (balance.size() != 2 or d.auto_white_balance)
            {
                throw std::invalid_argument("--white-balance <red> <blue>, not with --auto-white-balance");
            }
            d.white_balance_red = std::stod(balance[0]);
            d.white_balance_blue = std::stod(balance[1]);
        }
        d.ptp = not options.flag("no-ptp");
        d.sync_clock = options.flag("sync-clock");
        NodeOptions n;
        n.base = base;
        n.source = options.text("source");
        n.left_frame_id = options.text("left-frame-id");
        n.right_frame_id = options.text("right-frame-id");
        if (options.has_value("cuda-device"))
        {
            n.cuda_device = static_cast<int>(options.integer("cuda-device"));
        }
        const double status_period = options.number("status-period");

        // the source goes before the SDK is released (both on return and on an exception)
        struct Sdk
        {
            ~Sdk() { release_sdk(); }
        } sdk;
        std::unique_ptr<FrameSource> source = open_device(d);
        declare_settings(parameters, *source, d);
        parameters.check_overrides();
        parameters.apply_all();  // the start's values (--param ones included) on the camera
        Node publisher(session, n, source->stereo());
        std::cout << "xblick-roboception: " << d.device_id << " -> " << n.base << "/{left,right,depth,tf_static,"
                  << "diagnostics}" << std::endl;
        std::cout << "ready" << std::endl;
        auto last = std::chrono::steady_clock::now();
        auto last_report = last;
        NodeStats before = publisher.stats();
        std::uint64_t dropped_before = 0, restarts_before = 0;
        while (not stop)
        {
            if (auto grab = source->next(std::chrono::milliseconds(1000)))
            {
                publisher.publish(*grab);
            }
            const auto now = std::chrono::steady_clock::now();
            if (now - last_report >= std::chrono::seconds(1))
            {
                publisher.report(*source);
                parameters.refresh();  // what the automatic exposure chose
                last_report = now;
            }
            const double span = std::chrono::duration<double>(now - last).count();
            if (span >= status_period)
            {
                const NodeStats s = publisher.stats();
                std::printf("[rc] %s\n", describe(s, before, source->dropped() - dropped_before,
                                                  source->restarts() - restarts_before, span)
                                             .c_str());
                std::fflush(stdout);
                before = s;
                dropped_before = source->dropped();
                restarts_before = source->restarts();
                last = now;
            }
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "xblick-roboception: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
