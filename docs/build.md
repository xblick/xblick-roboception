# Build and test xblick-roboception

This page lists the dependencies, the build commands and what the test checks.

## Dependencies

| Dependency | Version | Note |
| --- | --- | --- |
| Durchblick's C++ package | branch `ros2`, commit `93c618d` (not released, [below](#durchblick-from-its-branch-ros2)) | Its CMake package needs the CUDA toolkit that it was built with. |
| Roboception's `rc_genicam_api` | 2.6 or newer, tested with 2.8.9 | Set `RC_GENICAM_API_DIR` to `<SDK>/lib/rc_genicam_api`. |
| zenoh-c and zenoh-cpp | as Durchblick | |

The SDK builds with the system's g++ 14. We tested this with version 2.8.9.

### Durchblick from its branch ros2

The branch `ros2` of this repository needs the core's branch `ros2` at commit `93c618d`. That commit is not
released. Build its C++ package from the archive of the commit, as the setup script of `xblick-azure-kinect` does:

```bash
curl -fLO https://github.com/xblick/xblick/archive/93c618d5e59b6c80ee0795028e7dda4b588c89b5.tar.gz
echo "444c3c695b9b9bf3efa28a6e6e348e1f4e76634638fb4a6ba278d8e8581b06c1  93c618d5e59b6c80ee0795028e7dda4b588c89b5.tar.gz" | sha256sum --check
tar -xzf 93c618d5e59b6c80ee0795028e7dda4b588c89b5.tar.gz
cmake -S xblick-93c618d5e59b6c80ee0795028e7dda4b588c89b5 -B xblick-build -G Ninja -DCMAKE_PREFIX_PATH=<zenoh> \
  -DCMAKE_INSTALL_PREFIX=<xblick> -DBUILD_TESTING=OFF -DXBLICK_POINTS=ON -DCUDAToolkit_ROOT=<CUDA toolkit> \
  -DCMAKE_CUDA_COMPILER=<CUDA toolkit>/bin/nvcc -DXBLICK_CUDA_INCLUDE_DIR=<CUDA toolkit>/include
cmake --build xblick-build && cmake --install xblick-build
```

The build of this node stops when the core has no `xblick/ros.hpp`.

## Build

```bash
cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH="<xblick>;<zenoh>" \
  -DRC_GENICAM_API_DIR=<SDK>/lib/rc_genicam_api -DCUDAToolkit_ROOT=<CUDA toolkit>
cmake --build build
ctest --test-dir build --output-on-failure
cmake --install build --prefix <xblick_rc>
```

Build in a directory on a local disk. Do not build on a shared filesystem.

The install provides the CMake package `xblick_rc` for programs that build on the node. They find it through
`xblick_rc_DIR`.

The programs get the SDK's library directory as `DT_RPATH`. They therefore find the SDK's GenICam libraries without
`LD_LIBRARY_PATH`.

## Tests

The test `offline` needs no camera. It checks the following, as tested on 2026-10-28 with rc_genicam_api 2.8.9,
Durchblick 0.3.1 and an RTX PRO 6000.

| Part | Check |
| --- | --- |
| Colour conversion | The halves of `IntensityCombined` in YCbCr411_8 with row padding equal the result of the SDK's `convYCbCr411toRGB`, pixel for pixel. |
| Mono conversion | Mono8 arrives without its padding. |
| Depth | The depth in mm follows from the disparity in either byte order. It is 0 in the holes. |
| Camera models | The `CameraInfo` of each stream. |
| Node end to end | Left, right and depth arrive as converted, with their frames and `CameraInfo`. `tf_static` and the diagnostics arrive. |
| Point cloud | Durchblick's point cloud node reads `depth` and `left`. 698 of 698 points are within 0.5 mm of the SDK's point cloud, and the node has no point where the SDK has none. |

The point cloud check compares with the formulas of `rcg::storePointCloud`. The tolerance of 0.5 mm covers the
rounding of the depth to millimetres. The check takes place only when Durchblick has its point cloud node, and it
needs a GPU.

We also started `xblick-roboception --list` without a camera. The SDK sees no device, and the node stops with
"device ... not found".

The test `ros` needs no camera either. The node needs one, so the test checks the streams on the node itself, fed
with synthetic images, with a session that takes part in ROS 2 as the program's does with `--ros`. Durchblick's ROS
reader receives left, right and depth with their `camera_info`, `/tf_static` and the diagnostics. Every ROS key
carries ROS's attachment, the publishers' tokens belong to the node `/xblick_roboception`, and the RVL depth does not
go to ROS. Without ROS no key or token of ROS is on the wire. The test also starts the program without a device:
with `--ros` its session becomes the ROS node `/xblick_roboception` before it stops, and a ROS option without `--ros`
stops it with the reason. Tested on 2026-11-17 with the core at `93c618d`.
