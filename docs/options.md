# Options of xblick-roboception

This page lists every command-line option, as `--help` does (Durchblick's `xblick::Options`). The header comment
of `src/main.cpp` is the source of this page. The program stops with an error at an unknown option.

| Option | Default | Effect |
| --- | --- | --- |
| `--list` | | Print the devices that the SDK sees, then exit. |
| `--device ID` | required | The GenICam ID, the serial number or the name of the camera. For an unknown device the node lists the devices that the SDK sees. |
| `--base NAME` | `cam/rc_visard` | The base name of all topics. |
| `--fps N` | `25` | The frame rate, 1 to 25 Hz (the rc_visard's range). |
| `--mono` | off, colour | Publish `mono8` images. |
| `--no-right` | off | Do not offer the right image. |
| `--no-depth` | off | Do not offer the depth image. |
| `--depth-quality Low\|Medium\|High\|Full` | `High` | The size of the depth image and its highest rate: `Low` 214 x 160 (25 Hz), `Medium` 320 x 240 (15 Hz), `High` 640 x 480 (3 Hz), `Full` 1280 x 960 (0.8 Hz, needs its licence). |
| `--auto-exposure` | off | Let the camera set the exposure. It excludes `--exposure-ms`. |
| `--exposure-ms MS` | `12` | The exposure time. The rc_visard takes 0.066 to 18 ms. |
| `--gain-db DB` | `0` | The gain. The rc_visard takes 0 to 18 dB. 6 dB doubles the brightness and the noise. |
| `--auto-white-balance` | off | Let the camera set the white balance. |
| `--white-balance RED BLUE` | `1.2 2.4` | The white balance ratios for red and blue. It excludes `--auto-white-balance`. |
| `--no-ptp` | off, PTP on | Turn PTP (`GevIEEE1588`) off. |
| `--sync-clock` | off | Put the stamps on the host's wall clock without a PTP master. The node measures the offset to the camera's clock with `TimestampLatch` at the start and every second ([streams.md](streams.md#stamps)). |
| `--left-frame-id NAME` | `Roboception_LeftCamera` | The frame of `left` and `depth`. |
| `--right-frame-id NAME` | `Roboception_RightCamera` | The frame of `right`. |
| `--cuda-device N` | none | Also offer CUDA rings on this GPU, for readers on the GPU. |
| `--source NAME` | `cam` | The `source` field in the attachment of every frame. |
| `--status-period SECONDS` | `5` | The period of the status line on the terminal. |
| `--connect ENDPOINT...`, `--listen ENDPOINT...`, `--zenoh-config FILE` | the router at `127.0.0.1:7447` | The Zenoh session. |
| `--ros` | off | Also take part in ROS 2 on `rmw_zenoh` (the core's branch `ros2`, [README](../README.md#ros-2)). |
| `--ros-domain ID` | `ROS_DOMAIN_ID`, else 0 | The ROS domain id. Needs `--ros`. |
| `--ros-node NAME` | `xblick_roboception` | The program's ROS node name. Needs `--ros`. |
| `--ros-params-writable` | off | ROS clients may set the parameters. Without it they only read them. Needs `--ros`. |

The node stops at its start when the camera refuses a setting. Examples are colour on a monochrome camera and the
depth quality `Full` without its licence.

## Parameters

Every option is also a parameter of the node `<base>`. Durchblick's terminal interface (`xblick`, tab Parameters) and
`xblick param` show them. `--param NAME:=VALUE` or `--params-file FILE` set them at the start.

The settings of exposure, gain and white balance change while the camera streams. Their ranges are the camera's,
read when it opens, and their descriptions name them. Their values are the camera's: with the automatic exposure,
`exposure_ms` and `gain_db` show what the camera chose, updated once a second.

| Parameter | Option | Effect of a change |
| --- | --- | --- |
| `exposure_ms` | `--exposure-ms` | Sets the exposure time and switches `auto_exposure` off. |
| `gain_db` | `--gain-db` | Sets the gain and switches `auto_exposure` off. |
| `auto_exposure` | `--auto-exposure` | The camera sets exposure and gain (`true`), or keeps the last ones (`false`). |
| `white_balance_red`, `white_balance_blue` | `--white-balance` | Sets a ratio and switches `auto_white_balance` off. Colour only. |
| `auto_white_balance` | `--auto-white-balance` | The camera sets the white balance. Colour only. |

For example, a brighter image: `xblick param --node cam/rc_visard --set exposure_ms:=18`. The other options are
read-only and apply at the next start, because the camera takes them when it opens.
