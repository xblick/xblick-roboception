// Where the images come from: a Roboception rc_visard through its GenICam SDK (rc_genicam_api), or anything else that
// gives the same images (the tests). A grab is what one buffer of the camera holds: its intensity image (left, and
// right with IntensityCombined) and/or its disparity image, each with the camera's timestamp. The camera sends the
// intensity images at the frame rate and the disparity images at the rate of its depth quality; an intensity and a
// disparity image of the same exposure carry the same timestamp, which pairs them for a reader (Durchblick's point
// cloud node).
#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "stereo.hpp"

namespace xblick::rc_node
{
    struct DeviceOptions
    {
        std::string device_id;          // GenICam device ID, serial number or user-defined name
        double fps = 25.0;              // AcquisitionFrameRate, 1 to 25 on the rc_visard
        bool color = true;              // YCbCr411_8 (colour), else Mono8
        bool right = true;              // IntensityCombined (left and right), else Intensity (left only)
        bool depth = true;              // Disparity
        std::string depth_quality = "High";  // Low (214x160, 25 Hz), Medium (320x240, 15), High (640x480, 3), Full
        bool auto_exposure = false;
        double exposure_ms = 12.0;      // with auto_exposure off
        double gain_db = 0.0;
        bool auto_white_balance = false;
        double white_balance_red = 1.2;   // red to green, with auto_white_balance off
        double white_balance_blue = 2.4;  // blue to green
        bool ptp = true;                  // GevIEEE1588: the camera's clock follows the network's PTP master
        // The stamps on the host's wall clock without PTP: the offset host - camera is measured with TimestampLatch
        // (the fastest of 5 round trips) at the start and every sync_period, and added to every stamp.
        bool sync_clock = false;
        std::chrono::milliseconds sync_period{1000};
    };

    // The last measurement of sync_clock: the host's wall clock minus the camera's clock, and its round trip.
    struct ClockSync
    {
        std::int64_t offset_ns = 0;
        std::int64_t round_trip_ns = 0;
    };

    struct Grab
    {
        // the camera's clock (with PTP: the PTP master's, usually the host's wall clock); with sync_clock the host's
        // wall clock
        std::int64_t stamp_ns = 0;
        std::optional<RawImage> left, right, disparity;
        std::shared_ptr<const void> keep;  // what holds the pixels until the next grab
    };

    // The settings that change while the camera streams (controls.hpp makes them the node's live parameters). The
    // auto ones are 0 or 1. Setting exposure or gain switches the automatic exposure off, a white balance ratio the
    // automatic white balance (the camera's automatic exposure sets the gain too).
    enum class Setting
    {
        AutoExposure,       // ExposureAuto: Continuous (1) or Off (0)
        ExposureMs,         // ExposureTime [ms]
        GainDb,             // Gain [dB]
        AutoWhiteBalance,   // BalanceWhiteAuto: Continuous (1) or Off (0); colour only
        WhiteBalanceRed,    // BalanceRatio of Red to green; colour only
        WhiteBalanceBlue,   // BalanceRatio of Blue to green; colour only
    };

    struct SettingRange
    {
        double min = 0.0, max = 0.0;
    };

    class FrameSource
    {
    public:
        virtual ~FrameSource() = default;
        // The next buffer, waiting at most `timeout`; empty if none came (or it was incomplete: counted in dropped()).
        virtual std::optional<Grab> next(std::chrono::milliseconds timeout) = 0;
        virtual const StereoParameters& stereo() const = 0;
        virtual std::uint64_t dropped() const = 0;
        virtual std::uint64_t restarts() const = 0;
        // What the diagnostics tell about the camera (model, serial, firmware, settings), as key and value.
        virtual std::vector<std::pair<std::string, std::string>> info() const = 0;
        // The last clock measurement with sync_clock, else none.
        virtual std::optional<ClockSync> clock_sync() const { return std::nullopt; }
        // A setting's value now (with the automatic exposure: what the camera chose), its range, and a change; safe
        // from another thread than next()'s. A source without settings throws std::logic_error; a value the camera
        // refuses throws std::runtime_error with its reason.
        virtual double setting(Setting) const { throw std::logic_error("this source has no settings"); }
        virtual SettingRange range(Setting) const { throw std::logic_error("this source has no settings"); }
        virtual void set(Setting, double) { throw std::logic_error("this source has no settings"); }
    };

    // Opens the camera (exclusively) and applies the options; a setting the camera refuses is an error, not a
    // warning. Streaming starts with the first next(). A grab that times out restarts the stream once; a second timeout in a row is raised.
    std::unique_ptr<FrameSource> open_device(const DeviceOptions& options);

    // The devices the SDK sees, one line each (ID, model, serial, access status), for an error message or a listing.
    std::vector<std::string> list_devices();

    // Releases the SDK's GenTL producers (rcg::System::clearSystems). Call it once at the end, after every source is
    // gone: without it the program crashes in the producers' static destructors at exit.
    void release_sdk();
}  // namespace xblick::rc_node
