// The camera's settings that change while it streams, as live parameters of the node /<base> (xblick-roboception
// and programs built on its library): exposure_ms, gain_db, auto_exposure and, in colour, white_balance_red,
// white_balance_blue, auto_white_balance. Each one's range is the camera's (read when it opens); its value is the
// camera's (with the automatic exposure the exposure and gain it chose: Parameters::refresh announces them).
// Declared in this order, so that apply_all ends as the auto ones say: writing exposure or gain switches the automatic
// exposure off, writing a ratio the automatic white balance.
#pragma once

#include "xblick/parameters.hpp"
#include "source.hpp"

namespace xblick::rc_node
{
    // `start`: the values at the start (the options or properties); a value given with --param or a params file wins.
    // Call Parameters::apply_all after it (and check_overrides).
    void declare_settings(Parameters& parameters, FrameSource& source, const DeviceOptions& start);

    // The node's name and namespace of a base <namespace>/<camera>: /cam/rc_visard -> ("rc_visard", "cam").
    std::pair<std::string, std::string> node_name(const std::string& base);
}  // namespace xblick::rc_node
