# Build and test xblick-roboception

This page lists the dependencies, the build commands and what the test checks.

## Dependencies

| Dependency | Version | Note |
| --- | --- | --- |
| Durchblick's C++ package | 0.5 (`find_package(xblick 0.5)`) | Its CMake package needs the CUDA toolkit that it was built with. |
| Roboception's `rc_genicam_api` | 2.6 or newer | Set `RC_GENICAM_API_DIR` to `<SDK>/lib/rc_genicam_api`. |
| zenoh-c and zenoh-cpp | as Durchblick | |

The SDK builds with the system's g++ 14.

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

The test `offline` needs no camera. It checks the following.

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
