# Streams of durchblick-roboception

This page describes the streams of the node, their camera models and their stamps. The README has the short table.

## The streams

`<base>` is the base name of the node. Its default is `robot/rc_visard`. The node produces every representation
only while a subscriber asks for it.

| Topic | Encoding | Content |
| --- | --- | --- |
| `<base>/left` | `rgb8`, with `--mono`: `mono8` | The rectified left image at the frame rate of the camera. |
| `<base>/right` | `rgb8`, with `--mono`: `mono8` | The rectified right image. The node takes it from the camera's `IntensityCombined` image. `--no-right` turns it off. |
| `<base>/depth` | `16UC1`, mm | The depth in the left camera's frame. It is 0 where the disparity is invalid. `--no-depth` turns it off. |
| `<base>/tf_static` | `tf2_msgs/TFMessage` | The pose of the right camera in the left camera's frame, which is the baseline along x. |
| `<base>/diagnostics` | `diagnostic_msgs/DiagnosticArray` | The status `roboception`, about once a second. |

### Depth

The camera computes a disparity image. The node converts it with z = f t / d, where f is the focal length, t the
baseline and d the disparity. The depth image has the resolution of the depth quality:

| `--depth-quality` | Resolution |
| --- | --- |
| `Low` | 214 x 160 |
| `Medium` | 320 x 240 |
| `High` | 640 x 480 |
| `Full` | 1280 x 960 |

The quality `Full` needs a licence on the camera.

### Diagnostics

The status `roboception` contains the rates, the state of the stream and the device:

- grabs per second, intensity images and disparity images per second, published frames per second,
- the image age, drops and restarts,
- the model, serial number, firmware and settings of the device.

## Camera models

The camera rectifies the images itself. Every `CameraInfo` is therefore a pinhole camera without distortion, with
the model `plumb_bob` and zeros as coefficients.

| Quantity | Value |
| --- | --- |
| Focal length f | `FocalLengthFactor` x width |
| Principal point | ((w - 1) / 2, (h - 1) / 2) |
| P of the right camera | P[3] = -f t, which is ROS's stereo convention |
| Model of `depth` | the left camera's model at the resolution of the depth image |

The principal point is where the SDK's own point cloud puts it. `rcg::storePointCloud` computes
x = (i + 0.5 - 0.5 w) t / d.

Nodes that read a stereo pair, such as durchblick-fast-foundationstereo, read the baseline from P[3] of the right
camera.

## Stamps

The stamp of a frame is the camera's timestamp of the exposure. Depth and the left image of one exposure carry the
same stamp. The point cloud node pairs them by it.

The camera has its own clock. With PTP (`GevIEEE1588`) that clock follows the PTP master of the network. PTP is on
by default, and `--no-ptp` turns it off.

The diagnostics report the image age, which is the current host time minus the stamp. The age shows whether the
clock of the camera is the clock of the host. A large or a negative age means that no PTP master exists or that the
camera follows another clock. Do not compare the stamps with other clocks until the age is plausible.

Without a PTP master, `--sync-clock` puts the stamps on the host's wall clock. The node latches the camera's clock
(`TimestampLatch`, `TimestampLatchValue`) between two readings of the host's clock and takes their middle. Of 5 such
round trips it keeps the fastest. It measures at the start and every second, and adds the offset to every stamp. The
diagnostics report the offset and the round trip.

Measured with an rc_visard 160c on a direct link to the host, 2026-11-12: round trip 0.1 ms,
offsets of one measurement within 0.013 ms, drift of the free camera clock 2.5 µs/s. The error of a stamp is thus
below 0.1 ms. The age with the offset was about 140 ms at 25 fps, the time from the exposure to the arrival of the
buffer.
[hardware-checks.md](hardware-checks.md) has the checks for the stamps.

## Point clouds

This node computes no point clouds. Durchblick's point cloud node `durchblick-points` reads `depth` and takes the
colour from `left`.

```bash
durchblick-points --depth robot/rc_visard/depth --color robot/rc_visard/left
```

The cloud is a `sensor_msgs/PointCloud2` in metres, organized like the depth image. Durchblick's
[point cloud guide](https://github.com/xblick/durchblick/blob/main/docs/guides/point-clouds.md) describes that node.
