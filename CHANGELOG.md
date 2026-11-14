# Versions

*The node's [overview](README.md). Versions before 0.3.0 are described in the git history only.*

* **0.3.1** Two defaults change: the base name and the `source` label.

  The default base name is `cam/rc_visard`, no longer `robot/rc_visard`, so that the default fits any
  camera PC and follows the core's scheme `cam/<device>/<stream>`. All topics move with it: `cam/rc_visard/left`,
  `cam/rc_visard/right`, `cam/rc_visard/depth`, `cam/rc_visard/tf_static`, `cam/rc_visard/diagnostics`, and the
  node `/cam/rc_visard` with its parameters. A script that subscribes to `robot/rc_visard/...` needs the new names, or
  the node started with the old name: `xblick-roboception --device <id> --base robot/rc_visard`.

  The default `source` label in every frame's info record is `cam`, no longer `robot`, as in the core's `xblick cam`.
  A reader that filters on `source == "robot"` needs the new label, or the node started with `--source robot`.
  Both old values together: `xblick-roboception --device <id> --base robot/rc_visard --source robot`.

* **0.3.0** Builds against Durchblick 0.5, in which every name in code, on the wire and on disk is `xblick`. The node
  takes the same names; only the prose name of the project stays Durchblick. It works only with programs of
  Durchblick 0.5. Stream names, frame ids, options and parameters are unchanged.

  | What | 0.2.0 | 0.3.0 |
  | --- | --- | --- |
  | Repository | `github.com/xblick/durchblick-roboception` | `github.com/xblick/xblick-roboception` |
  | Program | `durchblick-roboception` | `xblick-roboception` |
  | CMake project | `durchblick_roboception` | `xblick_roboception` |
  | CMake package, library, target | `durchblick_rc`, `durchblick_rc::durchblick_rc` | `xblick_rc`, `xblick_rc::xblick_rc` |
  | Installed paths | `include/durchblick_rc`, `lib/cmake/durchblick_rc` | `include/xblick_rc`, `lib/cmake/xblick_rc` |
  | Core package | `find_package(durchblick 0.4)`, `durchblick::durchblick`, `"durchblick/<header>.hpp"` | `find_package(xblick 0.5)`, `xblick::xblick`, `"xblick/<header>.hpp"` |
  | C++ namespace | `durchblick::rc_node` | `xblick::rc_node` |
  | Point cloud node in the test | `durchblick-points` | `xblick-points` |
