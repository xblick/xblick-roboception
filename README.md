# xblick-roboception

[![xblick core](https://img.shields.io/pypi/v/xblick?label=xblick%20core&color=blue)](https://github.com/xblick/xblick)
[![checks](https://github.com/xblick/xblick-roboception/actions/workflows/checks.yml/badge.svg)](https://github.com/xblick/xblick-roboception/actions/workflows/checks.yml)
[![licence](https://img.shields.io/badge/licence-Apache--2.0-blue)](LICENSE)
[![platform](https://img.shields.io/badge/platform-Linux%20x86__64-lightgrey)](README.md#requirements)

`xblick-roboception` publishes the rectified left and right images of a Roboception rc_visard stereo camera as
[Durchblick](https://github.com/xblick/xblick) streams, at 25 frames per second by default. It
also publishes a depth image, which it computes from the disparity image of the camera.

The node produces each representation of a stream only while a subscriber asks for it.

## What it publishes

The node publishes below one base name. The default base is `cam/rc_visard`.

| Topic | Type | Content |
| --- | --- | --- |
| `<base>/left` | image, `rgb8` | The rectified left image. With `--mono` the encoding is `mono8`. |
| `<base>/right` | image, `rgb8` | The rectified right image. With `--mono` the encoding is `mono8`. |
| `<base>/depth` | image, `16UC1`, mm | The depth in the left camera's frame. It is 0 where the disparity is invalid. |
| `<base>/tf_static` | `tf2_msgs/TFMessage` | The pose of the right camera in the left camera's frame. |
| `<base>/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Rates, image age, drops, restarts and the device's settings, about once a second. |

Every image stream carries its `CameraInfo`. Durchblick's point cloud node computes coloured point clouds from
`depth` and `left` on `<base>/points`. [docs/streams.md](docs/streams.md) describes the streams, the camera
models and the stamps.

## Requirements

| Need | For |
| --- | --- |
| Durchblick from its branch `ros2` (commit `93c618d`, not released), C++ package | everything |
| The CUDA toolkit that Durchblick's package was built with | Durchblick's CMake package |
| Roboception's `rc_genicam_api` 2.6 or newer (tested: 2.8.9) | the camera |
| `GENICAM_GENTL64_PATH` set to the SDK's GenTL producer | finding a device |
| A PTP master on the network, or `--sync-clock` | stamps on the host's clock |

Every program connects to the Zenoh router of its PC (`zenohd` on port 7447). Durchblick's
[sessions page](https://github.com/xblick/xblick/blob/main/docs/concepts/sessions.md) explains the routers.

## Install

```bash
cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH="<xblick>;<zenoh>" \
  -DRC_GENICAM_API_DIR=<SDK>/lib/rc_genicam_api -DCUDAToolkit_ROOT=<CUDA toolkit>
cmake --build build
ctest --test-dir build --output-on-failure
cmake --install build --prefix <xblick_rc>
```

Replace each `<...>` with the install prefix of that package. The test needs no camera.
[docs/build.md](docs/build.md) has the details.

## Run

List the devices that the SDK sees, then start the node with one of them.

```bash
xblick-roboception --list
xblick-roboception --device <device id>
```

Other programs read the streams of the node.

```bash
# coloured point clouds on cam/rc_visard/points, with Durchblick's point cloud node
xblick-points --depth cam/rc_visard/depth --color cam/rc_visard/left
# a viewer on another host
xblick rerun cam/rc_visard/left cam/rc_visard/right cam/rc_visard/depth \
  --points cam/rc_visard/points --spawn --connect <camera host>
```

Most users change these options:

| Option | Default | Effect |
| --- | --- | --- |
| `--device ID` | required | The GenICam ID, the serial number or the name of the camera. |
| `--base NAME` | `cam/rc_visard` | The base name of all topics. |
| `--fps N` | `25` | The frame rate. |
| `--depth-quality Low\|Medium\|High\|Full` | `High` | The resolution of the depth image. |
| `--exposure-ms MS`, `--auto-exposure` | 12 ms | The exposure. |
| `--cuda-device N` | none | Also offer CUDA rings on this GPU. |

`--help` lists every option, and the program stops at an unknown option. Every option is also a parameter of the
node `<base>`, which Durchblick's terminal interface (`xblick`, tab Parameters) shows; `--param NAME:=VALUE` or
`--params-file FILE` set them at the start. Exposure, gain and white balance change while the node runs, within the
camera's ranges. The others apply at the next start. [docs/options.md](docs/options.md) lists every option and
parameter.

## ROS 2

The branch `ros2` of this repository builds on the core's branch `ros2`, which is not released. Build that core at
commit `93c618d` ([docs/build.md](docs/build.md)). The core adds the option `--ros`: the node then also takes part in
ROS 2 on `rmw_zenoh` (ROS 2 Jazzy), and its own topics stay as they are. Without `--ros` nothing changes.

```bash
xblick-roboception --device <device id> --ros
```

| ROS 2 topic | ROS type |
| --- | --- |
| `/<base>/left/image_raw`, `/<base>/right/image_raw`, `/<base>/depth/image_raw` | `sensor_msgs/msg/Image` |
| `/<base>/<stream>/camera_info` for the same streams, with the header of each image | `sensor_msgs/msg/CameraInfo` |
| `/<base>/left/image_raw/compressed`, `/<base>/right/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` (JPEG) |
| `/tf_static`, transient local | `tf2_msgs/msg/TFMessage` |
| `/<base>/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` |

ROS does not get the RVL form of the depth (`depth/image_raw/compressed`), which has no ROS format, nor the rings in
shared memory and on the GPU. ROS gets the depth's `image_raw`.

The node sends a topic to ROS only while a ROS subscriber reads it. `camera_info` goes out with the images: a ROS
subscriber that reads only `camera_info` gets nothing until something takes the stream's images. The transform on
`/tf_static` goes out once and stays in the publisher's cache for ROS subscribers that start later. The program is
the ROS node `/xblick_roboception` (`--ros-node`), and its parameters are the ROS node `/<base>`. ROS clients read the
parameters and may set them only when the node was started with `--ros-params-writable`.

Each part of `--base` has to be a ROS name: letters, digits and `_`, not starting with a digit. The node checks this
at its start with or without `--ros`. Every default topic is such a name.

The node needs a camera, so the test `ros` checks the streams on the node itself, with synthetic images and a
session that takes part in ROS 2 as the program's does with `--ros`. It reads the topics above through Durchblick's
ROS reader, checks ROS's attachment and tokens, and checks that without ROS nothing of ROS is on the wire. Of the
program it checks that `--ros` makes it the ROS node `/xblick_roboception`. Nothing was tested with a camera or with
ROS 2's own programs.
[Durchblick and ROS 2](https://github.com/xblick/xblick/blob/ros2/docs/integrations/ros2.md) describes the option,
the other `--ros-*` options and ROS 2's tools.

## Limits

- We tested the node without a camera. Nobody has tried the GenICam settings, the multi-part buffers, the PTP
  stamps, the depth qualities with their rates, or the restart on a device.
- The node does not publish the confidence and error images of the camera.
- The node stops at its start when the camera refuses a setting.

[docs/hardware-checks.md](docs/hardware-checks.md) is the checklist for a session with the camera attached.

## Documentation

| Page | Content |
| --- | --- |
| [docs/streams.md](docs/streams.md) | The streams, the camera models, the stamps, the point clouds. |
| [docs/options.md](docs/options.md) | Every command-line option. |
| [docs/design.md](docs/design.md) | How the node works inside and the special cases of the SDK. |
| [docs/build.md](docs/build.md) | Dependencies, build, tests. |
| [docs/hardware-checks.md](docs/hardware-checks.md) | The checks that need a camera. |

Durchblick's concept pages explain the terms that these pages use:
[streams and topics](https://github.com/xblick/xblick/blob/main/docs/concepts/streams-and-topics.md), [transports](https://github.com/xblick/xblick/blob/main/docs/concepts/transports.md),
[rings](https://github.com/xblick/xblick/blob/main/docs/concepts/rings.md) and [timestamps](https://github.com/xblick/xblick/blob/main/docs/concepts/timestamps.md).

## Licence and credits

The code of this repository is under the Apache License 2.0 ([LICENSE](LICENSE)). The repository does not contain
Roboception's SDK `rc_genicam_api`. You install the SDK separately, and the build links it. The SDK has its own
licences: Roboception's code is under the 3-clause BSD licence, and the GenICam reference implementation and Baumer's
GenTL producer that come with it have licences of their own. [NOTICE](NOTICE) names them.
