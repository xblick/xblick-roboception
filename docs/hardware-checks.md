# Checks that need the rc_visard attached

We tested the node without a device. This page is the checklist that settles its stamps on a real rc_visard.
[streams.md](streams.md#stamps) explains the stamps. Record the results with date, setup and numbers in
[streams.md](streams.md), and remove the check here.

Besides the stamps, everything that needs the device is untested: the GenICam settings, the multi-part buffers, the
depth qualities with their rates, and the restart of the stream.

## 1. PTP: does the camera's clock follow the host's clock?

**Run.** Start the node with PTP on, which is the default. `xblick diagnostics` shows the image age, which
is the current time minus the stamp. Check on the host whether a PTP master is active. It can be `ptp4l` with
`phc2sys`, or a network switch. Check whether the web GUI of the rc_visard shows the PTP state "slave" with an
offset.

**Expect.** The age is a few tens of ms, steady and positive. Without a master the camera keeps its own clock. The
age is then seconds, or negative.

**Judge.** If the network has no master, start one with `ptp4l` on the interface of the camera PC, or measure the
offset and decide. Never compare the stamps with other clocks until the age is plausible.

## 2. What the stamp marks

**Run.** Do the screen-clock test with the left camera. It is check 1 in the `docs/hardware-checks.md` of
xblick-azure-kinect.

**Expect.** Roboception documents the timestamp as the time of the exposure. The stamp minus the shown time is half
the exposure plus the delay of the monitor, within one refresh period.

**Judge.** A constant beyond that is a bias. Document it, and correct it if it is large.

## 3. Left, right and depth of one exposure

**Run.** Subscribe to `left`, `right` and `depth`, and log the stamps.

**Expect.** The stamps are identical. Depth and the left image of one exposure must carry the same stamp.

**Judge.** Any difference breaks the pairing of the point cloud node and of the stereo nodes. Fix it in
`src/source.cpp`.

## 4. PTP lost or stepped

**Run.** Stop the PTP master for a minute, then start it again.

**Look for.** The ages while the master is off, and the stamps when it returns.

**Expect.** The clock of the camera drifts slowly while it is free. It steps back when PTP returns.

**Judge.** A step backwards in the stamps confuses pairing and recording, because MCAP uses the stamp as log time.
Note it in [streams.md](streams.md). Consider a warning in the diagnostics when the age jumps.
