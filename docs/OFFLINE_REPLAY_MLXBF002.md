# Read-only MLXBF002 offline replay (experimental)

**Branch:** `feature/live-rigid-body-fit`
**Status:** implemented in source; the new Rust tests and GUI playback have **not yet been executed on the user's desktop or a connected board**.

## Purpose

Use an existing `MLXBF002` compact three-board Hall recording to exercise the **actual live application's** sensor trace state, dotted advisory field limits, all-48-channel range/step warnings and, only if there is an **independently verified matching rigid profile**, the existing six-DOF pose pipeline. This removes the need for connected MLX90393 receivers for playback/GUI integration testing.

It is **NOT** a new measurement, a sensor calibration, a digital-wrap correction, a validated motion ground truth, or a live vendor HDMM acquisition. The HDMM plot remains a **Hall-predicted marker displacement** calculated from an enabled rigid fit and a separately confirmed optical marker coordinate.

## Windows workflow

```powershell
cd F:\GitHub\magnetic_board_driver
git pull --ff-only origin feature/live-rigid-body-fit
cargo test -p app
cargo check -p app
cargo run -p app --release
```

In the new **OFFLINE REPLAY** panel near the top:

1. With any live field stream **stopped** and any recording log **closed**, choose **Open MLXBF002 recording**. Browse to a read-only or local copy of a previously acquired .bin; it can be anywhere on the disk, including outside this repository. For the user's original November 6 research data, the existing source filename is `GK11062026.bin` under the **separate analysis repository's private folder**. That external path is **optional**: the standalone app neither imports that repository nor looks for it automatically.
2. After loading, confirm the file path, decoded frame count, elapsed time and detected raw board IDs. Loading does **not** start the playback automatically.
3. Select **1×**, **4×**, or **16×** and press **Play**. Pause and resume without losing the plotted history. **Restart** returns to the first original recorded frame and resets history; **Unload** exits replay mode.
4. On **Live Tracking → Rigid 6-DOF**, the raw Hall traces for Board 0/1/2 sit next to the **Hall-predicted** HDMM marker chart (the two primary experimental monitors). The secondary XYZ translation and rotation charts remain below the predicted-HDMM chart. The separate Sensor Traces and Both tabs still work.
5. Look for the range warning on **every chip** (not just the selected plotted chip), dotted signed X/Y/Z advisory limits, and a session-latched >8mT abrupt-step warning. Pause to inspect the original November 6 superior-board discontinuities around 65–73s. A quiet trace after a jump is NOT evidence that the true physical branch is recovered.

**Without an independently loaded profile matching the original recording and its physical receiver/magnet geometry**, expect raw traces and warnings to work but **NO validated six-DOF displacement, rotation or predicted HDMM displacement**. The app does not invent CT-derived geometry, background, optical marker location or clock registration. The original November 6 CT/HDMM analysis remains a separate source of evidence and does not become validated by a GUI replay.

## Recorded file semantics and safety

- Loads native `MLXBF002` header followed by little-endian `u32` length and Postcard `BoardFrame` records, exactly as `app/src/main.rs` writes them. Source is opened read-only; errors on invalid header, truncated record, malformed payload, invalid board ID/sensor mask, or empty recording. File loading is bounded to **256 MiB** and **250,000 frames** to avoid unintended memory exhaustion.
- The playback scheduler preserves **original board IDs, sensor masks, XYZ values, per-sensor timestamps, record ordering and numeric fields**. Only delivery speed changes. A nonmonotonic cross-board timestamp is reported and makes the scheduler's delivery time nondecreasing without editing stored frames. GUI processing is limited to 256 frames per 50ms timer tick and catches up as needed. Playback speed may not be exact if the host cannot keep up.
- Replay is mutually exclusive with hardware streaming and file logging. Any delayed live-source frames are ignored while replay is active. Log start/append/file-selection and current-controller sensitivity writes are blocked. Live-only GK background/reference collection and new calibration are also blocked. A replayed frame cannot enter the on-disk `MLXBF002` writer.
- Changing the output gain/resolution UI without applying does not change historical data or digital guide levels. **An MLXBF002 file does not contain full per-chip on-chip register snapshots**, so replay resets MLX range provenance to **UNVERIFIED** and displays the deliberately conservative nominal gain7/res0 dotted guides. This may be different from the actual acquisition hardware settings and is NOT proof of analog non-saturation.
- With or without replay, a field step/range warning is an operator aid, not proof of digital wrapping or sensor saturation; the production pose fitter retains its existing QC rather than silently unwrapping data.

## Regressions and acceptance

`app/src/offline_replay.rs` tests cover original logger framing, signed component values and board IDs, speed/clock behavior, pause/resume/restart, bad and truncated files, and bounded delivery. `app/src/main.rs` integration tests cover loading the same format, displaying recorded fields/warnings, blocking hardware stream/logging/setting writes, ignoring a stray hardware frame, clearing history and leaving the source file byte-for-byte intact.

**Do not mark this experimentally validated from compiled unit tests alone.** After the Windows `cargo test` and `cargo check` succeed, launch the app with an original local recording. Verify playback across all three boards, scene updates, paused/finished status, warnings, and consistent GUI performance. The rigid fitter should be compared against independently held-out motion truth only after a valid matching profile and reference timestamps are established.
