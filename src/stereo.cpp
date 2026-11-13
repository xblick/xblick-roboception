#include "stereo.hpp"

#include <cmath>
#include <stdexcept>

#include <rc_genicam_api/image.h>
#include <rc_genicam_api/pixel_formats.h>

namespace xblick::rc_node
{
    std::size_t row_bytes(std::uint64_t pixel_format, std::uint32_t width, std::uint32_t xpadding)
    {
        switch (pixel_format)
        {
            case Mono8:
            case Confidence8:
            case Error8:
                return std::size_t(width) + xpadding;
            case YCbCr411_8:
                if (width % 4 != 0)
                {
                    throw std::invalid_argument("YCbCr411_8 needs a width that is a multiple of 4");
                }
                return std::size_t(width) / 4 * 6 + xpadding;
            case RGB8:
                return std::size_t(width) * 3 + xpadding;
            case Coord3D_C16:
                return std::size_t(width) * 2 + xpadding;
        }
        throw std::invalid_argument("pixel format " + std::to_string(pixel_format) + " is not read by this node");
    }

    RawImage top_half(const RawImage& combined)
    {
        if (combined.height % 2 != 0)
        {
            throw std::invalid_argument("IntensityCombined of odd height " + std::to_string(combined.height));
        }
        RawImage half = combined;
        half.height = combined.height / 2;
        return half;
    }

    RawImage bottom_half(const RawImage& combined)
    {
        RawImage half = top_half(combined);
        half.pixels = combined.pixels + half.height * row_bytes(combined.pixel_format, combined.width,
                                                                 combined.xpadding);
        return half;
    }

    xblick::CameraInfo camera_info(const StereoParameters& stereo, std::uint32_t width, std::uint32_t height,
                                   const std::string& frame_id, bool right)
    {
        if (not(stereo.focal_length_factor > 0.0) or not(stereo.baseline > 0.0))
        {
            throw std::invalid_argument("FocalLengthFactor and Baseline must be positive");
        }
        const double f = stereo.focal_length_factor * width;
        xblick::CameraInfo info =
            xblick::CameraInfo::pinhole(width, height, f, f, (width - 1) / 2.0, (height - 1) / 2.0, frame_id);
        if (right)
        {
            info.p[3] = -f * stereo.baseline;
        }
        return info;
    }

    std::size_t disparity_to_depth(const RawImage& disparity, const StereoParameters& stereo, std::uint16_t* depth)
    {
        if (disparity.pixel_format != Coord3D_C16)
        {
            throw std::invalid_argument("disparity: Coord3D_C16 expected, got " +
                                        std::to_string(disparity.pixel_format));
        }
        if (not(stereo.disparity_scale > 0.0))
        {
            throw std::invalid_argument("Scan3dCoordinateScale must be positive");
        }
        const double ft_mm = stereo.focal_length_factor * disparity.width * stereo.baseline * 1000.0;
        std::size_t beyond = 0;
        const std::size_t step = row_bytes(Coord3D_C16, disparity.width, disparity.xpadding);
        for (std::uint32_t v = 0; v < disparity.height; ++v)
        {
            const std::uint8_t* row = disparity.pixels + v * step;
            std::uint16_t* out = depth + std::size_t(v) * disparity.width;
            for (std::uint32_t u = 0; u < disparity.width; ++u)
            {
                const std::uint8_t* p = row + 2 * u;
                const unsigned raw = disparity.big_endian ? (unsigned(p[0]) << 8 | p[1]) : (unsigned(p[1]) << 8 | p[0]);
                const double z = raw == 0 ? 0.0 : std::round(ft_mm / (stereo.disparity_scale * raw));
                beyond += z > 65535.0;
                out[u] = z > 65535.0 ? 0 : std::uint16_t(z);
            }
        }
        return beyond;
    }

    std::string encoding(const RawImage& image)
    {
        switch (image.pixel_format)
        {
            case Mono8:
                return "mono8";
            case YCbCr411_8:
            case RGB8:
                return "rgb8";
        }
        throw std::invalid_argument("intensity image of pixel format " + std::to_string(image.pixel_format) +
                                    ": Mono8, YCbCr411_8 or RGB8 expected");
    }

    void to_pixels(const RawImage& image, std::uint8_t* out)
    {
        const bool mono = encoding(image) == "mono8";
        if (not rcg::convertImage(mono ? nullptr : out, mono ? out : nullptr, image.pixels, image.pixel_format,
                                  image.width, image.height, image.xpadding))
        {
            throw std::invalid_argument("the SDK cannot convert this image (format " +
                                        std::to_string(image.pixel_format) + ", " + std::to_string(image.width) +
                                        " x " + std::to_string(image.height) + ")");
        }
    }
}  // namespace xblick::rc_node
