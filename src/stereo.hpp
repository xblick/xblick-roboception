// What a Roboception rc_visard sends, turned into Durchblick's images: its rectified left and right images (Mono8 or
// YCbCr411_8, the combined image IntensityCombined has the left one in its top half and the right one below) and its
// disparity (Coord3D_C16: 16 bit, times Scan3dCoordinateScale, 0 invalid). The camera is described by three numbers
// of its GenICam node map: FocalLengthFactor (the focal length divided by the image width), Baseline (metres) and
// Scan3dCoordinateScale. The images are rectified on the camera: no distortion, the principal point in the image's
// centre as the SDK's own point cloud (rcg::storePointCloud: x = (i + 0.5 - 0.5 w) t / d) places it, cx = (w - 1) / 2.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "xblick/msg.hpp"

namespace xblick::rc_node
{
    struct StereoParameters
    {
        double focal_length_factor = 0.0;  // focal length / image width
        double baseline = 0.0;             // metres, between the left and the right camera
        double disparity_scale = 0.0;      // Scan3dCoordinateScale: disparity in pixels per unit of the 16-bit value
    };

    // An image as the camera sends it (pixels owned by the grabbed buffer).
    struct RawImage
    {
        const std::uint8_t* pixels = nullptr;
        std::uint64_t pixel_format = 0;  // PFNC: Mono8, YCbCr411_8, Coord3D_C16, ...
        std::uint32_t width = 0, height = 0;
        std::uint32_t xpadding = 0;      // bytes after each row
        bool big_endian = false;         // of 16-bit pixels
    };

    // Bytes of one row (with its padding); throws for a format this node does not read.
    std::size_t row_bytes(std::uint64_t pixel_format, std::uint32_t width, std::uint32_t xpadding);

    // The two halves of IntensityCombined (left above, right below).
    RawImage top_half(const RawImage& combined);
    RawImage bottom_half(const RawImage& combined);

    // The CameraInfo of an image of this size (left camera, or the right one: P[3] = -fx * baseline as in ROS's
    // stereo convention); plumb_bob without distortion, R = identity.
    xblick::CameraInfo camera_info(const StereoParameters& stereo, std::uint32_t width, std::uint32_t height,
                                   const std::string& frame_id, bool right);

    // Depth in millimetres (16UC1, as ROS's REP 118 and the Azure Kinect node; RVL keeps it lossless over the network)
    // of a disparity image: z = f t / d with f = FocalLengthFactor * width, rounded; 0 where the disparity is invalid
    // (0) or the depth beyond 65535 mm. Returns how many were beyond.
    std::size_t disparity_to_depth(const RawImage& disparity, const StereoParameters& stereo, std::uint16_t* depth);

    // rgb8 (YCbCr411_8, RGB8) or mono8 (Mono8) pixels of an intensity image, rows without padding; the encoding is
    // what encoding() says. Throws for another format.
    std::string encoding(const RawImage& image);
    void to_pixels(const RawImage& image, std::uint8_t* out);
}  // namespace xblick::rc_node
