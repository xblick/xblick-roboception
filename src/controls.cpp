#include "controls.hpp"

#include <cstdio>
#include <iostream>

namespace xblick::rc_node
{
    namespace
    {
        std::string text(double v)
        {
            char buffer[32];
            std::snprintf(buffer, sizeof buffer, "%g", v);
            return buffer;
        }

        // the setting's writer (the camera's reason when it refuses) and reader
        std::pair<Parameters::Writer, Parameters::Reader> access(FrameSource& source, Setting s, bool flag)
        {
            Parameters::Writer write = [&source, s, flag](const ParameterValue& v) -> std::string {
                try
                {
                    source.set(s, flag ? (v.bool_value ? 1.0 : 0.0) : v.double_value);
                    return "";
                }
                catch (const std::exception& error)
                {
                    return error.what();
                }
            };
            Parameters::Reader read = [&source, s, flag] {
                const double v = source.setting(s);
                return flag ? ParameterValue::of(v != 0.0) : ParameterValue::of(v);
            };
            return {write, read};
        }

        void declare_number(Parameters& parameters, FrameSource& source, Setting s, const std::string& name,
                            double start, const std::string& unit, const std::string& what)
        {
            const SettingRange r = source.range(s);
            ParameterOptions o{.description = what + ", " + text(r.min) + " to " + text(r.max) + " " + unit +
                                              " (the camera's range)",
                               .unit = unit,
                               .floating_range = FloatingPointRange{r.min, r.max, 0.0}};
            auto [write, read] = access(source, s, false);
            parameters.declare(name, start, o, write, read);
        }

        void declare_flag(Parameters& parameters, FrameSource& source, Setting s, const std::string& name, bool start,
                          const std::string& what)
        {
            auto [write, read] = access(source, s, true);
            parameters.declare(name, start, ParameterOptions{.description = what}, write, read);
        }
    }  // namespace

    void declare_settings(Parameters& parameters, FrameSource& source, const DeviceOptions& start)
    {
        declare_number(parameters, source, Setting::ExposureMs, "exposure_ms", start.exposure_ms, "ms",
                       "exposure time; setting it switches auto_exposure off; longer: brighter, more motion blur");
        declare_number(parameters, source, Setting::GainDb, "gain_db", start.gain_db, "dB",
                       "gain; setting it switches auto_exposure off; higher: brighter, more noise (6 dB: twice)");
        declare_flag(parameters, source, Setting::AutoExposure, "auto_exposure", start.auto_exposure,
                     "the camera sets exposure and gain (ExposureAuto Continuous): true or false");
        if (start.color)
        {
            declare_number(parameters, source, Setting::WhiteBalanceRed, "white_balance_red", start.white_balance_red,
                           "", "red to green; setting it switches auto_white_balance off");
            declare_number(parameters, source, Setting::WhiteBalanceBlue, "white_balance_blue",
                           start.white_balance_blue, "", "blue to green; setting it switches auto_white_balance off");
            declare_flag(parameters, source, Setting::AutoWhiteBalance, "auto_white_balance", start.auto_white_balance,
                         "the camera sets the white balance (BalanceWhiteAuto Continuous): true or false");
        }
    }

    std::pair<std::string, std::string> node_name(const std::string& base)
    {
        const auto slash = base.rfind('/');
        return slash == std::string::npos ? std::pair{base, std::string()}
                                          : std::pair{base.substr(slash + 1), base.substr(0, slash)};
    }
}  // namespace xblick::rc_node
