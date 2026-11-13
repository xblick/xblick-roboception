# AGENTS.md

Roboception rc_visard stereo camera as Durchblick streams below `<base>` (default `robot/rc_visard`): rectified
`left` and `right`, `depth` (16UC1 mm, from the camera's disparity), `tf_static`, `diagnostics`. C++20, plain CMake
project: program `durchblick-roboception`, library `durchblick_rc` (installed as CMake package, for programs
that build on the node). Same structure as durchblick-azure-kinect.
Version 0.2.0, tested without a camera only. Human documentation: `README.md`, then `docs/` (streams, options,
design, build, hardware-checks). Transport-wide traps (Zenoh sessions and routers, CUDA rings, hanging
tests): `AGENTS.md` of the core repository (github.com/xblick/durchblick). Style of all documentation: the core's
`docs/project/writing-docs.md`.

## Commands

```bash
# Build on a local disk, never on a shared filesystem. <...> = install prefixes.
cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH="<durchblick>;<zenoh>" \
  -DRC_GENICAM_API_DIR=<SDK>/lib/rc_genicam_api -DCUDAToolkit_ROOT=<CUDA toolkit>
cmake --build build
ctest --test-dir build --output-on-failure
cmake --install build --prefix <durchblick_rc>   # the CMake package durchblick_rc (durchblick_rc_DIR)
durchblick-roboception --list                    # the devices the SDK sees
tools/check_references.sh                        # the reference check of the CI (allowlist: tools/reference_allowlist.txt)
```

Durations: not recorded. Give `ctest` a timeout.

- Needs Durchblick 0.4 (`find_package(durchblick 0.4)`; its CMake package needs the CUDA toolkit it was built
  with) and `rc_genicam_api` >= 2.6 (tested 2.8.9).
- Test `offline`: no camera. The point cloud part runs only when Durchblick has its point cloud node, and needs a GPU.

## Rules

- Fail fast. A setting the camera refuses (colour on a monochrome camera, depth quality Full without its licence), an
  unknown device or an unknown option stops the node at its start. Keep it so.
- The program defines its options with Durchblick's `durchblick::Options` (`--help` lists them), and every option is
  also a parameter of the node: read-only, but exposure, gain and white balance (`src/controls.*`, live, ranges
  from the camera). Keep the header comment of `src/main.cpp` and docs/options.md in sync with them.
- Do not change the defaults silently.
- Camera models follow the SDK. `plumb_bob` with zero distortion, f = `FocalLengthFactor` x width, principal point
  ((w - 1) / 2, (h - 1) / 2) as `rcg::storePointCloud` puts it, right P[3] = -f t (ROS's stereo convention, which
  durchblick-fast-foundationstereo reads). Depth has the left camera's model at its own resolution. A change breaks
  the point cloud and stereo nodes downstream.
- Depth and the left image of one exposure carry the same stamp. The point cloud node pairs them by it.
- Produce each representation only while a subscriber asks for it.
- Documentation: Markdown only in `README.md` and `docs/`, one purpose per page.

## Traps

| Trap | Symptom | Fix |
| --- | --- | --- |
| SDK static destructors | Segfault at exit. | Call `rcg::System::clearSystems()` before exit (`source.hpp`). |
| GenICam's `GenericException` | It is not a `std::exception` and escapes `catch (std::exception&)`. | `source.cpp` converts it. Put every new SDK call inside that conversion. |
| `librc_genicam_api` has no RPATH | GenICam libraries not found. | Programs link with `-Wl,--disable-new-dtags`: the SDK's library directory is `DT_RPATH`. Do not switch to `RUNPATH`. |
| GenTL producer | No device found. | Set `GENICAM_GENTL64_PATH` to the SDK's GenTL producer. |
| Camera clock | Large or negative image age in the diagnostics. | No PTP master, or another clock. Stamps are the camera's exposure time. With PTP (`GevIEEE1588`, default on) the clock follows the network's PTP master. Without one, `--sync-clock` maps the stamps to the host's clock with `TimestampLatch`. |
| Grab failure | Stream stops. | The node restarts the stream once. A second failure in a row stops the node. |
| SDK build | | The SDK builds with a current system g++ (tested: g++ 14, rc_genicam_api 2.8.9). |
| Zenoh session with only `--connect` to a router | It does not listen and finds nothing. | Every program uses the PC's router (`zenohd` on 7447). |
| NFS | | Build and write large outputs on a local disk. |

## Layout

| Path | Content |
| --- | --- |
| `src/main.cpp` | the program and its options (Durchblick's `durchblick::Options`, header comment) |
| `src/source.*` | the GenICam device: settings, multi-part buffers, restart, `clearSystems` |
| `src/controls.*` | the live settings as parameters of the node |
| `src/stereo.*` | YCbCr411/Mono8 conversion, depth from disparity, camera models |
| `src/node.*`, `session.hpp` | publishing on Durchblick, diagnostics |
| `tests/test_offline.cpp` | the `offline` test |
| `docs/` | human documentation |

## Defaults

- Base `robot/rc_visard`. Topics: `left`, `right`, `depth`, `tf_static`, `diagnostics`. `points` comes from
  `durchblick-points --depth <base>/depth --color <base>/left`.
- Exposure 12 ms, gain 0 dB, white balance red 1.2 and blue 2.4, colour, depth quality High, 25 fps, PTP on.
- Frames `Roboception_LeftCamera`, `Roboception_RightCamera`. `source` `robot`. `--cuda-device` unset.
- Zenoh: the PC's router at 127.0.0.1:7447, or `--connect`, `--listen`, `--zenoh-config`.
- Standalone: `zenohd`, `durchblick-roboception --device <id> [--depth-quality High] [--fps 25]`, look with
  `blick rerun ... --spawn`.
- Not done: everything that needs the device (GenICam settings, multi-part buffers, PTP stamps, depth qualities, the
  restart). Test with a camera before relying on it: docs/hardware-checks.md.
