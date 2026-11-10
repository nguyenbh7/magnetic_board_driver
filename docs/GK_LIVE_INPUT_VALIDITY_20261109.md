# Gamma Knife live tracking: fail-closed raw-input validity gate

**Status (Nov 9, 2026):** Source implementation committed on
`feature/live-rigid-body-fit`. New Rust unit/integration tests have been
added but **not yet executed on the user's Windows checkout**. The user
previously validated the live layout and observed the November 6 raw Board 0
discontinuities in offline replay. The new fit gate itself has **not yet**
been experimentally or GUI validated.

## Why this gate is needed

The original November 6 SUPERIOR receiver (raw Board 0) had **66
sensor/frame >8 mT steps across 11/16 sensors** near 65–73 seconds.
Those events are consistent with possible digital field wrapping, but the
recording does not independently establish the true magnetic field branch
or analog saturation. The UI previously warned about raw steps/range but
did not invalidate the current 6-DOF pose. A low optimizer RMSE does not
prove measurements from a wrapped/overranged sensor were physically
correct. The new gate prevents a questionable pose being shown as a
**current reliable** estimate.

## Live and replay behavior

After processing **every observed raw Hall sensor** in a complete
`BoardFrame`, but *before dispatching* a new six-DOF fit, the app checks:

- **Critical range event:** any observed X/Y/Z component reaches **80% or
  more** of its axis-specific **advisory** magnitude limit (same limits as
  the existing red `BACK OFF MAGNET` banner and the signed dotted guides).
  The conservative fallback when settings are unknown is the unverified
  gain7/res0 nominal digital half-range: X/Y ±4.9152 mT; Z ±7.929856 mT.
  Firmware-cached configurations can change the nominal limits; the
  conservative 25 mT analog guard applies to wider modes. These are NOT
  verified real per-chip saturation limits.
- **Sudden step event:** any sensor component changes by **more than 8 mT**
  within **0.25 seconds** between its adjacent raw samples, as the
  existing latched `RAW STEP WARNING` logic already reports.
- A **65%-to-80% caution** alone is *not* an invalidity trigger. A missing
  or nonfinite sensor sample is still reported as a raw-data warning and
  subject to existing valid-sensor counts/fit QC; it is not automatically
  classified as saturation or made into an all-session latch.

At the first critical event, the active rigid fitter:

1. Latches a specific **INPUT UNRELIABLE — RIGID FIT SUSPENDED** reason
   identifying the board, firmware sensor index, field component, and
   observed severity. This is a conservative risk screen, NOT a verified
   digital-wrap or analog-saturation diagnosis.
2. Cancels all buffered synchronized frames and invalidates in-flight
   rigid fitter jobs through a generation change. Any delayed completion,
   even with a matching generation while the latch is set, is discarded.
   No subsequent pose job is dispatched while the latch is active.
3. Clears the **current pose**, relative-zero pose, and active
   predicted-HDMM-value readout. Previously accepted displacement,
   rotation, and projected-HDMM curves are **retained only as frozen
   PRE-WARNING history**, with explicit warning labels.
4. Suspends independent fitting too; switching the fitting model cannot
   bypass the latch. An ongoing new calibration background/reference
   capture is aborted if a critical event occurs.
5. Continues the **original raw Hall recording**, all-sensor range/step
   warnings, and selectable Bx/By/Bz traces. Sensor raw samples are **never
   automatically unwrapped, clipped, masked, corrected, or modified** by
   the gate. The source file in Offline Replay remains read-only.

The latch remains active after the raw field settles, after changing the
dashboard/tracking model, and after stopping the stream. A quiet
post-event value **cannot establish which digital field branch is
physical**. The user must correct/inspect the setup and **start a new
live acquisition** (or **Restart** an offline replay) to rearm. An unsafe
new frame triggers the latch again. Replay pause/resume does not rearm.

The optical HDMM instrument is **not** connected to the app. Its chart
is only a Hall-rigid-pose prediction at a separately confirmed reflector
position, never a measurement from the HDMM camera.

## Tests and limits of interpretation

The code contains additional tests that:

- Trigger the latch from a nonselected raw Board 0 sensor at >80% of its
  provisional Z advisory limit, while verifying that raw traces keep
  updating and the latch persists when a later frame becomes quiet.
- Check that a 70%-of-limit **CAUTION** is not latched.
- Trigger the latch from an isolated **>8 mT** jump well below the
  wider-range 80% threshold.
- Verify switching to independent fits cannot bypass invalidity.
- Ensure existing read-only replay tests see input invalidity and that
  replay restart clears the latch.
- Test that delayed fitter completions are ignored, that no current
  pose/HDMM value is returned, and that stop/reset-stream does not
  automatically erase the latched warning.

**This is software safety hardening, not absolute measurement validation.**
Unknown register state and raw values already wrapped into a plausible
numeric range can evade detection; conversely, conservative unknown-range
fallback can produce an early/false-positive latch on a legitimate
wider-range acquisition. Replaying the November 6 file may latch before
the 65–73 s jump episode if a recorded sensor already exceeds the
provisional 80% threshold. This is intentional fail-closed behavior and
must not be presented as proof that saturation occurred at that point.
For valid physical tracking, verify chip settings, background/calibration,
CBCT geometry, the optical reflector location, and independent timing.

## Run on Windows

```powershell
cd F:\GitHub\magnetic_board_driver
git pull --ff-only origin feature/live-rigid-body-fit

cargo test -p app raw_input_fault_latches
cargo test -p app critical_raw_range_latches
cargo test -p app caution_does_not_latch
cargo test -p app offline_replay_is_read_only
cargo test -p app
cargo check -p app
cargo run -p app --release
```

Choose **Offline Replay**, select `GK11062026.bin`, and play through
the problematic Board 0 interval. Verify the INVALID / UNRELIABLE banner,
frozen pose/HDMM curves, still-live raw traces, and failure to recover
a current pose merely by switching tracking modes. Pause, Resume, then
Restart the file to confirm latch-reset semantics. Do not claim the
entire acquisition has usable 6-DOF positions without a matching
independently verified CT/Hall calibration profile.
