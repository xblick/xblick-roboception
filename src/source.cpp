#include "source.hpp"

#include <limits>
#include <mutex>
#include <stdexcept>

#include <rc_genicam_api/buffer.h>
#include <rc_genicam_api/config.h>
#include <rc_genicam_api/device.h>
#include <rc_genicam_api/pixel_formats.h>
#include <rc_genicam_api/stream.h>
#include <rc_genicam_api/system.h>

#include <Base/GCException.h>

namespace xblick::rc_node
{
    namespace
    {
        // GenICam's exceptions are no std::exception: turned into one where the SDK is called
        template <typename F>
        auto sdk(const std::string& what, F&& f) -> decltype(f())
        {
            try
            {
                return f();
            }
            catch (const GENICAM_NAMESPACE::GenericException& e)
            {
                throw std::runtime_error(what + ": " + e.GetDescription());
            }
        }

        std::int64_t wall_ns()
        {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                .count();
        }

        void enable(const std::shared_ptr<GenApi::CNodeMapRef>& nodemap, const char* component, bool on)
        {
            rcg::setEnum(nodemap, "ComponentSelector", component, true);
            rcg::setBoolean(nodemap, "ComponentEnable", on, true);
        }

        RawImage raw_image(const rcg::Buffer& buffer, std::uint32_t part)
        {
            return RawImage{.pixels = static_cast<const std::uint8_t*>(buffer.getBase(part)),
                            .pixel_format = buffer.getPixelFormat(part),
                            .width = std::uint32_t(buffer.getWidth(part)),
                            .height = std::uint32_t(buffer.getHeight(part)),
                            .xpadding = std::uint32_t(buffer.getXPadding(part)),
                            .big_endian = buffer.isBigEndian()};
        }

        class DeviceSource : public FrameSource
        {
        public:
            explicit DeviceSource(const DeviceOptions& o) : options_(o)
            {
                device_ = rcg::getDevice(o.device_id.c_str());
                if (not device_)
                {
                    std::string known;
                    for (const std::string& line : list_devices())
                    {
                        known += "\n  " + line;
                    }
                    throw std::runtime_error("Roboception device '" + o.device_id + "' not found; the SDK sees:" +
                                             (known.empty() ? std::string(" none") : known));
                }
                device_->open(rcg::Device::EXCLUSIVE);
                nodemap_ = device_->getRemoteNodeMap();
                configure();
                stereo_.focal_length_factor = rcg::getFloat(nodemap_, "FocalLengthFactor", nullptr, nullptr, true);
                stereo_.baseline = rcg::getFloat(nodemap_, "Baseline", nullptr, nullptr, true);
                if (o.depth)
                {
                    rcg::checkFeature(nodemap_, "Scan3dOutputMode", "DisparityC");
                    rcg::checkFeature(nodemap_, "Scan3dCoordinateOffset", "0");
                    stereo_.disparity_scale = rcg::getFloat(nodemap_, "Scan3dCoordinateScale", nullptr, nullptr, true);
                }
                info_ = {{"device", device_->getID()},
                         {"model", device_->getModel()},
                         {"serial", device_->getSerialNumber()},
                         {"version", device_->getVersion()},
                         {"pixel format", rcg::getEnum(nodemap_, "PixelFormat", true)},
                         {"depth quality", o.depth ? rcg::getEnum(nodemap_, "DepthQuality", true) : "off"},
                         {"frame rate", std::to_string(rcg::getFloat(nodemap_, "AcquisitionFrameRate"))},
                         {"ptp", o.ptp ? "on" : "off"},
                         {"sync clock", o.sync_clock ? "on" : "off"}};
                if (o.sync_clock)
                {
                    sync_clock();  // fails at the start when the camera has no TimestampLatch
                }
                const std::vector<std::shared_ptr<rcg::Stream>> streams = device_->getStreams();
                if (streams.empty())
                {
                    throw std::runtime_error("Roboception device '" + o.device_id + "' has no stream");
                }
                stream_ = streams[0];
            }

            ~DeviceSource() override
            {
                if (streaming_)
                {
                    stream_->stopStreaming();
                    stream_->close();
                }
                device_->close();
            }

            std::optional<Grab> next(std::chrono::milliseconds timeout) override
            {
                return sdk("Roboception device '" + options_.device_id + "'", [&] { return grab(timeout); });
            }

            const StereoParameters& stereo() const override { return stereo_; }
            std::uint64_t dropped() const override { return dropped_; }
            std::uint64_t restarts() const override { return restarts_; }
            std::vector<std::pair<std::string, std::string>> info() const override
            {
                std::vector<std::pair<std::string, std::string>> out = info_;
                out.emplace_back("exposure ms", std::to_string(setting(Setting::ExposureMs)));
                out.emplace_back("gain dB", std::to_string(setting(Setting::GainDb)));
                out.emplace_back("auto exposure", setting(Setting::AutoExposure) != 0.0 ? "on" : "off");
                return out;
            }

            double setting(Setting s) const override
            {
                return sdk("Roboception device '" + options_.device_id + "'", [&] {
                    std::lock_guard lock(mutex_);
                    switch (s)
                    {
                        case Setting::AutoExposure:
                            return rcg::getEnum(nodemap_, "ExposureAuto", true) == "Off" ? 0.0 : 1.0;
                        case Setting::ExposureMs:
                            return rcg::getFloat(nodemap_, "ExposureTime", nullptr, nullptr, true) / 1000.0;
                        case Setting::GainDb:
                            return rcg::getFloat(nodemap_, "Gain", nullptr, nullptr, true);
                        case Setting::AutoWhiteBalance:
                            return rcg::getEnum(nodemap_, "BalanceWhiteAuto", true) == "Off" ? 0.0 : 1.0;
                        case Setting::WhiteBalanceRed:
                        case Setting::WhiteBalanceBlue:
                            rcg::setEnum(nodemap_, "BalanceRatioSelector", ratio(s), true);
                            return rcg::getFloat(nodemap_, "BalanceRatio", nullptr, nullptr, true);
                    }
                    throw std::logic_error("unknown setting");
                });
            }

            SettingRange range(Setting s) const override
            {
                return sdk("Roboception device '" + options_.device_id + "'", [&] {
                    std::lock_guard lock(mutex_);
                    SettingRange r{0.0, 1.0};
                    switch (s)
                    {
                        case Setting::AutoExposure:
                        case Setting::AutoWhiteBalance:
                            break;
                        case Setting::ExposureMs:
                            rcg::getFloat(nodemap_, "ExposureTime", &r.min, &r.max, true);
                            r = {r.min / 1000.0, r.max / 1000.0};
                            break;
                        case Setting::GainDb:
                            rcg::getFloat(nodemap_, "Gain", &r.min, &r.max, true);
                            break;
                        case Setting::WhiteBalanceRed:
                        case Setting::WhiteBalanceBlue:
                            rcg::setEnum(nodemap_, "BalanceRatioSelector", ratio(s), true);
                            rcg::getFloat(nodemap_, "BalanceRatio", &r.min, &r.max, true);
                            break;
                    }
                    return r;
                });
            }

            void set(Setting s, double value) override
            {
                sdk("Roboception device '" + options_.device_id + "'", [&] {
                    std::lock_guard lock(mutex_);
                    switch (s)
                    {
                        case Setting::AutoExposure:
                            rcg::setEnum(nodemap_, "ExposureAuto", value != 0.0 ? "Continuous" : "Off", true);
                            break;
                        case Setting::ExposureMs:
                            rcg::setEnum(nodemap_, "ExposureAuto", "Off", true);
                            rcg::setFloat(nodemap_, "ExposureTime", value * 1000.0, true);  // microseconds
                            break;
                        case Setting::GainDb:
                            rcg::setEnum(nodemap_, "ExposureAuto", "Off", true);
                            rcg::setFloat(nodemap_, "Gain", value, true);
                            break;
                        case Setting::AutoWhiteBalance:
                            rcg::setEnum(nodemap_, "BalanceWhiteAuto", value != 0.0 ? "Continuous" : "Off", true);
                            break;
                        case Setting::WhiteBalanceRed:
                        case Setting::WhiteBalanceBlue:
                            rcg::setEnum(nodemap_, "BalanceWhiteAuto", "Off", true);
                            rcg::setEnum(nodemap_, "BalanceRatioSelector", ratio(s), true);
                            rcg::setFloat(nodemap_, "BalanceRatio", value, true);
                            break;
                    }
                });
            }
            std::optional<ClockSync> clock_sync() const override { return clock_; }

        private:
            static const char* ratio(Setting s) { return s == Setting::WhiteBalanceRed ? "Red" : "Blue"; }

            std::optional<Grab> grab(std::chrono::milliseconds timeout)
            {
                if (not streaming_)
                {
                    start();
                }
                const rcg::Buffer* buffer = stream_->grab(timeout.count());
                if (buffer == nullptr)
                {
                    if (timed_out_)
                    {
                        throw std::runtime_error("Roboception device '" + options_.device_id +
                                                 "': no image after a restart of its stream");
                    }
                    timed_out_ = true;
                    stream_->stopStreaming();
                    stream_->close();
                    streaming_ = false;
                    ++restarts_;
                    return std::nullopt;
                }
                timed_out_ = false;
                if (buffer->getIsIncomplete())
                {
                    ++dropped_;
                    return std::nullopt;
                }
                Grab result;
                result.stamp_ns = std::int64_t(buffer->getTimestampNS());
                std::lock_guard lock(mutex_);  // the node map, which a setting's change uses from another thread
                if (options_.sync_clock)
                {
                    if (std::chrono::steady_clock::now() - synced_ >= options_.sync_period)
                    {
                        sync_clock();
                    }
                    result.stamp_ns += clock_->offset_ns;
                }
                for (std::uint32_t part = 0; part < buffer->getNumberOfParts(); ++part)
                {
                    if (not buffer->getImagePresent(part))
                    {
                        continue;
                    }
                    const std::string component = rcg::getComponentOfPart(nodemap_, buffer, part);
                    const RawImage image = raw_image(*buffer, part);
                    if (component == "IntensityCombined")
                    {
                        result.left = top_half(image);
                        result.right = bottom_half(image);
                    }
                    else if (component == "Intensity")
                    {
                        result.left = image;
                    }
                    else if (component == "Disparity")
                    {
                        result.disparity = image;
                    }
                }
                return result;
            }

            void configure()
            {
                const DeviceOptions& o = options_;
                // the components: only what the node publishes
                std::vector<std::string> components;
                rcg::getEnum(nodemap_, "ComponentSelector", components, true);
                for (const std::string& c : components)
                {
                    const bool on = (c == "IntensityCombined" and o.right) or (c == "Intensity" and not o.right) or
                                    (c == "Disparity" and o.depth);
                    enable(nodemap_, c.c_str(), on);
                }
                // intensity and disparity of one exposure in one buffer where the device can (multi-part)
                rcg::setString(nodemap_, "AcquisitionMultiPartMode", "SynchronizedComponents", false);
                rcg::setEnum(nodemap_, "ComponentSelector", o.right ? "IntensityCombined" : "Intensity", true);
                rcg::setEnum(nodemap_, "PixelFormat", o.color ? "YCbCr411_8" : "Mono8", true);
                if (o.depth)
                {
                    rcg::setEnum(nodemap_, "DepthQuality", o.depth_quality.c_str(), true);
                }
                rcg::setFloat(nodemap_, "AcquisitionFrameRate", o.fps, true);
                rcg::setEnum(nodemap_, "ExposureAuto", o.auto_exposure ? "Continuous" : "Off", true);
                if (not o.auto_exposure)
                {
                    rcg::setFloat(nodemap_, "ExposureTime", o.exposure_ms * 1000.0, true);  // microseconds
                }
                rcg::setFloat(nodemap_, "Gain", o.gain_db, true);
                if (o.color)
                {
                    rcg::setEnum(nodemap_, "BalanceWhiteAuto", o.auto_white_balance ? "Continuous" : "Off", true);
                    if (not o.auto_white_balance)
                    {
                        rcg::setEnum(nodemap_, "BalanceRatioSelector", "Red", true);
                        rcg::setFloat(nodemap_, "BalanceRatio", o.white_balance_red, true);
                        rcg::setEnum(nodemap_, "BalanceRatioSelector", "Blue", true);
                        rcg::setFloat(nodemap_, "BalanceRatio", o.white_balance_blue, true);
                    }
                }
                rcg::setBoolean(nodemap_, "GevIEEE1588", o.ptp, true);
            }

            // The host's wall clock minus the camera's: the camera latches its clock between two readings of the host's
            // clock, whose middle is taken. The fastest of 5 round trips (about 0.1 ms on a direct link) is kept.
            void sync_clock()
            {
                ClockSync best{.offset_ns = 0, .round_trip_ns = std::numeric_limits<std::int64_t>::max()};
                for (int i = 0; i < 5; ++i)
                {
                    const std::int64_t before = wall_ns();
                    rcg::callCommand(nodemap_, "TimestampLatch", true);
                    const std::int64_t after = wall_ns();
                    const std::int64_t camera = rcg::getInteger(nodemap_, "TimestampLatchValue", nullptr, nullptr, true);
                    if (after - before < best.round_trip_ns)
                    {
                        best = {.offset_ns = before + (after - before) / 2 - camera, .round_trip_ns = after - before};
                    }
                }
                clock_ = best;
                synced_ = std::chrono::steady_clock::now();
            }

            void start()
            {
                stream_->open();
                stream_->startStreaming();
                streaming_ = true;
            }

            DeviceOptions options_;
            std::shared_ptr<rcg::Device> device_;
            std::shared_ptr<GenApi::CNodeMapRef> nodemap_;
            std::shared_ptr<rcg::Stream> stream_;
            StereoParameters stereo_;
            std::vector<std::pair<std::string, std::string>> info_;
            std::optional<ClockSync> clock_;
            std::chrono::steady_clock::time_point synced_;
            mutable std::mutex mutex_;  // the node map: grab()'s thread and the settings' (Zenoh's)
            bool streaming_ = false, timed_out_ = false;
            std::uint64_t dropped_ = 0, restarts_ = 0;
        };
    }  // namespace

    std::unique_ptr<FrameSource> open_device(const DeviceOptions& options)
    {
        return sdk("Roboception device '" + options.device_id + "'",
                   [&]() -> std::unique_ptr<FrameSource> { return std::make_unique<DeviceSource>(options); });
    }

    void release_sdk() { rcg::System::clearSystems(); }

    std::vector<std::string> list_devices()
    {
        return sdk("listing the Roboception devices", [] {
            std::vector<std::string> lines;
            for (const std::shared_ptr<rcg::Device>& d : rcg::getDevices())
            {
                lines.push_back(d->getID() + " (" + d->getModel() + ", serial " + d->getSerialNumber() + ", " +
                                d->getAccessStatus() + ")");
            }
            return lines;
        });
    }
}  // namespace xblick::rc_node
