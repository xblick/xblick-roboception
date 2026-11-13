# Design of xblick-roboception

This page explains how the node works inside, which decisions we made and which special cases of Roboception's SDK
the code handles.

## Structure

The project is a plain CMake project with the program `xblick-roboception` and the library `xblick_rc`.
Other programs can build on the installed CMake package `xblick_rc`. xblick-azure-kinect has the same
structure.

| Part | Files | Work |
| --- | --- | --- |
| Device | `src/source.*` | Opens the GenICam device, applies the settings, reads multi-part buffers, restarts the stream. |
| Conversion | `src/stereo.*` | Converts YCbCr411 and Mono8, computes depth from disparity, builds the camera models. |
| Node | `src/node.*`, `src/session.hpp` | Publishes on Durchblick and reports diagnostics. |

## Decisions

**The node stops on a refused setting.** A setting that the camera refuses stops the node at its start. So do an
unknown device and an unknown option.

**The camera models follow the SDK.** The principal point and the focal length are those of the SDK's own point
cloud, and the right camera's P follows ROS's stereo convention. [streams.md](streams.md#camera-models) has the
values. The point cloud node and the stereo nodes depend on them, so a change breaks those nodes.

**Depth is 16-bit millimetres at the camera's resolution.** A reader scales the image if it needs another size.

**One stamp per exposure.** Depth and the left image of one exposure carry the same stamp, so that the point cloud
node can pair them.

**Work only for subscribers.** The node produces each representation only while a subscriber asks for it.

**The camera's pose belongs to the robot model.** The node publishes `tf_static` for the right camera only.
The node does not scale, rotate or translate the point cloud.

## Special cases of the SDK

| Case | What the code does |
| --- | --- |
| The SDK crashes with a segmentation fault in its static destructors at exit. | The program calls `rcg::System::clearSystems()` before it exits (`src/source.hpp`). |
| GenICam's `GenericException` does not derive from `std::exception`. | `src/source.cpp` converts it. A new SDK call outside that conversion escapes `catch (std::exception&)`. |
| `librc_genicam_api` has no RPATH, so it does not find its GenICam libraries by itself. | The programs link with `-Wl,--disable-new-dtags`. The SDK's library directory then enters the program as `DT_RPATH`, which also serves the GenICam libraries. `LD_LIBRARY_PATH` is not needed. A `RUNPATH` does not work here. |
| The SDK finds its GenTL producer through `GENICAM_GENTL64_PATH`. | Without the variable the SDK sees no device. |
| A grab fails. | The node restarts the stream once. A second failure in a row stops the node. |
| The camera sends left and right as one image `IntensityCombined`, in YCbCr411_8 with row padding or in Mono8. | The node splits the halves and converts them as the SDK's `convYCbCr411toRGB` does. |

## Status and limits

- The settings follow the SDK's example `gc_pointcloud`. Nobody has tried them on a camera.
- The camera has confidence and error images. The node does not publish them.
