// A Roboception rc_visard's images as Durchblick streams below one base name (e.g. cam/rc_visard):
//
//   <base>/left         rgb8 (or mono8), the rectified left image
//   <base>/right        the rectified right image (with IntensityCombined)
//   <base>/depth        16UC1 [mm] from the disparity image, the left camera at the depth quality's resolution
//   <base>/tf_static    the right camera's pose in the left camera's frame (the baseline along x)
//   <base>/diagnostics  diagnostic_msgs/DiagnosticArray from report(): rates, drops, restarts, image age, settings
//
// Each image stream carries its CameraInfo (stereo.hpp: rectified, no distortion; the right one with P[3] = -fx t).
// An image is converted only while somebody subscribes to its stream. Stamps: the camera's timestamp (with PTP the
// PTP master's clock); depth carries the stamp of the exposure it was computed from, as the left image of that
// exposure does, which pairs them for a reader (Durchblick's point cloud node: --depth <base>/depth --color
// <base>/left).
#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <zenoh.hxx>

#include "xblick/msg.hpp"
#include "source.hpp"

namespace xblick::rc_node
{
    struct NodeOptions
    {
        std::string base = "cam/rc_visard";
        std::string source = "robot";  // in every frame's attachment
        std::string left_frame_id = "Roboception_LeftCamera";
        std::string right_frame_id = "Roboception_RightCamera";
        std::optional<int> cuda_device;  // also offer CUDA rings on this device
        std::uint32_t slots = 32;  // ring slots per stream (a reader has slots - 1 frame periods)
    };

    struct NodeStats
    {
        std::uint64_t grabs = 0;
        std::uint64_t left = 0, right = 0, depth = 0;  // frames published
        std::uint64_t intensity = 0, disparity = 0;    // images that came
        std::uint64_t beyond = 0;                      // depth pixels beyond 65535 mm (published as 0)
        double age_ms = 0;                             // now - stamp of the images that came, summed
    };

    class Node
    {
    public:
        Node(zenoh::Session& session, const NodeOptions& options, const StereoParameters& stereo);
        ~Node();
        Node(const Node&) = delete;
        Node& operator=(const Node&) = delete;

        // Publishes what is subscribed of one grab. The publishers of a stream start with its first image (its size
        // is the device's mode).
        void publish(const Grab& grab);
        // How the node is doing since the last report, on <base>/diagnostics, and returned (for a log or
        // another monitor). About once a second.
        DiagnosticStatus report(const FrameSource& source);

        const NodeStats& stats() const;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

    // "12.0 grabs/s, dropped 0, restarts 0; left 12 right 12 depth 3, age 4.1 ms": the rates between two stats()
    // taken `seconds` apart
    std::string describe(const NodeStats& now, const NodeStats& before, std::uint64_t dropped, std::uint64_t restarts,
                         double seconds);
}  // namespace xblick::rc_node
