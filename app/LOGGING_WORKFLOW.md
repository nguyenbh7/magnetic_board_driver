# Safe Hall measurement logging

The desktop app records `MLXBF002` compact board-frame binary logs.

## Workflow

The desktop app groups the instructions, file-selection controls, current/last
filename, log status, and warnings inside a **Measurement logging** bordered
panel. Follow the numbered instructions shown at the top of the panel.

1. Select **Select File** and choose a **new** `.bin` filename, with the MCU
   connected and the stream stopped if possible. The app creates the file with
   the `MLXBF002` header, refuses existing names, and then automatically
   requests actual MLX90393 registers 0x00/0x01/0x02 from detected chips.
   A new companion `<logname>.mlx_settings.json` is saved beside the `.bin`.
   **Verify the Register snapshot status before starting the stream.**
2. Click **Start Field Stream** to begin acquiring data. The active sensor
   board frames are appended to the open log.
3. Click **Stop Field Stream** if you want to pause. With the log still open,
   **Start Field Stream** resumes appending to that same file.
4. Click **Stop logging** to finish the experiment. The app drains queued frames,
   flushes the output buffer, and closes the log.
5. For the next experiment, select a **new** filename before starting its stream.

**Important:** Starting the field stream with no file selected only displays
live data; it does not save a new file. **Stop logging does not stop the field
stream**; stop the stream separately as needed.

## Per-run register metadata (new in mlx-live-fit)

The live app exports a **read-only hardware-register snapshot** right after
creating each NEW MLXBF002 log. The snapshot is a separate JSON sidecar
(e.g., run01.bin becomes run01.mlx_settings.json).
The existing binary format and magnetic-data-parser remain unchanged.

Each board/index (up to three boards x 16 chips) includes:
- Raw registers 0x00 (gain/Hall phase), 0x01 (TCMP and more), and
  0x02 (separate XYZ resolution, OSR, DIG_FILT, OSR2).
- Decoded gain 0..7, separate res X/Y/Z each 0..3, Hall config
  0x0 or 0xC, TCMP enable, OSR, filter, and original register bytes.
- Presence/read-success masks, sample counts, uniformity flag,
  read timestamp, serial port, and provenance/limitations.

No existing JSON sidecar is overwritten: exclusive file creation is used.
On a missing RPC/firmware, transport failure or failed register read,
the app warns explicitly; incomplete snapshots are marked and
never presented as verified full-device configuration. Sensitivity
setting changes are blocked while a binary log is open. Starting
a new stream is deferred until the initial snapshot request completes;
retry Start Stream after the readback status appears.

**Important:** The preexisting sensitivity UI, now labeled **Read cached setting**, reports
**cached firmware context**, not a genuine readback of all chips.
The new snapshot is an actual, non-mutating register query, but
it only applies to the *time it was captured*; it cannot establish the
configuration of the original historical October/November Hall sweeps.
MLXBF002 stores preconverted microtesla vectors, not gain/res register
values or raw ADC counts.

A stream already active before selecting a filename might have produced
frames before the snapshot was taken. Stop the stream, select a new
filename, verify the readback, and then begin streaming for best
timing provenance.

**CACHED SOFTWARE DEFAULT IS NOT THE CHIP RESET DEFAULT:** The branch's
firmware application-level mlx_sensitivity context initializes to
GAIN_SEL=0, RES=0, HALLCONF=0xC; those values would mean
XY=0.751 µT/count, Z=1.210 µT/count if explicitly programmed.
However, the firmware's Sensor::from_mlx initializes every chip by
calling mlx.reset(), then set_measurement_configuration() **reads the
actual hardware registers into the separate per-sensor state** used for
conversion. Melexis MLX90393 datasheet Rev 012, Table 16 lists
**factory reset GAIN_SEL=7, RES_XYZ=0, HALLCONF=0xC, TCMP=0**,
which corresponds to **XY=0.150 µT/count and Z=0.242 µT/count**
in Table 17 (TCMP disabled, 25°C). The A1 historical MLXBF002
count-lattice report from Nov 4, 2026 strongly matched gain7/res0/0xC
(p95 0.000339 counts), consistent with the physical factory reset
and prior firmware conversion rather than the misleading cached context.
These values **remain inferred historical settings**, not hardware
registers recorded at acquisition. The new sidecar corrects this
future-provenance gap without changing the chip settings.

**Deployment:** both the desktop app and the MCU must be rebuilt and
the MCU reflashed for the new RPC. A new GUI attached to old
firmware must report a missing snapshot; it must not
invent plausible register values.

Run tests from magnetic_board_driver:

    cargo test -p app logging_safety_tests

Then perform one disposable manual GUI check with updated firmware,
stop stream, select a NEW log file, confirm a new JSON sidecar
reports the expected per-chip readbacks, and start/stop a brief
stream. Confirm reusing the same log/sidecar paths does not modify
either existing file. Never modify the original A1/VGM raw logs.

## Overwrite and loss protection

- An existing path is protected with Rust's atomic
  `OpenOptions::create_new(true)`. The application will display an
  **EXISTING FILE PROTECTED** notice and keep the old file unchanged.
- **Select File** is unavailable while a log is open or draining, and
  the async file dialog's completion handler independently checks that
  state. Two simultaneous selection dialogs are prevented.
- **Stop logging** is inactive when no log is open or after a stop was
  requested. Draining the pending queue finishes before an explicit
  flush and close.
- Disk-write and flush failures are displayed as errors. Any affected
  file requires integrity inspection before scientific use.
- The UI shows the current/last log path and log notices.

The native save dialog may also ask for overwrite confirmation. That
confirmation is *not* the app's security boundary; the atomic exclusive
open refuses existing files regardless of the dialog result.

## Suggested filenames

For traceability use a unique experiment identifier, date, and run number,
for example `GK_mannequin_20261027_run01.bin` and
`GK_mannequin_20261027_run02.bin`.

## Developer checks

Run the logging safety tests from the repository root:

```powershell
cargo test -p app logging_safety_tests
```

They verify that existing file bytes are never changed and that a new
recording begins with the correct eight-byte file header.

For a manual GUI smoke test, try selecting an existing disposable test file,
then a new path; start/stop the stream twice while leaving the new log open;
finally stop logging and confirm the file size is preserved when attempting
to select the old path again. Run this with disposable files, not lab data.

### Notes for operators

The offline parser expects every board frame to have a valid length-prefixed
Postcard payload. A log with only the `MLXBF002` header means no
board frames were recorded. A write failure in the middle of a frame can
leave a truncated final payload; check and retain such recordings for
diagnosis rather than treating them as valid experimental results.
