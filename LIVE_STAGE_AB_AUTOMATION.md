# Automated live-stage A/B logging

This workflow compares the Rust live fitter against the validated free-orientation Python fits in `gk_analysis` without manually transcribing live UI values.

## Output pair

Selecting a raw output file in the desktop app creates two synchronized files:

```text
stage_z.bin
stage_z.ab.csv
```

`stage_z.bin` is the existing fixed-record `SensorField` stream. `stage_z.ab.csv` is a companion log containing:

- the per-board/per-sensor background snapshot currently installed in the live fitter;
- one row for each completed Rust fit, keyed by the same nonzero firmware `frame_id` as the raw stream;
- fit timestamp;
- x/y/z position;
- free magnet orientation (`theta_rad`, `phi_rad`);
- residual RMS and objective score;
- sensor count and target moment magnitude;
- signed displacement components dx/dy/dz and total displacement from the current zero.

Orientation remains free. The linear stage supplies translation ground truth only.

## Applying the logging patch on the hardware checkout

The hardware WSL checkout may contain local item-7 UI edits. To avoid replacing those files wholesale, the branch contains a transactional patcher:

```bash
python3 scripts/apply_ab_companion_logging.py
```

It edits `app/src/main.rs` and `app/src/live_fit.rs` in memory and writes them only if every required source transformation succeeds. The companion writer itself is `app/src/ab_log.rs`.

Format only the touched Rust files:

```bash
rustfmt --edition 2024 app/src/main.rs app/src/live_fit.rs app/src/ab_log.rs
cargo test -p app
cargo check -p app
```

Do not use `cargo fmt --all` merely for this patch; the working tree may contain unrelated app/firmware edits.

## Acquisition

1. Start the app/field stream and capture the normal 20-frame no-magnet background.
2. Put the magnet on the stage and allow the fit to stabilize.
3. Reset displacement zero.
4. Select the stage raw output file. Do this **after the background has been captured** so the companion receives the exact background snapshot used by Rust.
5. Keep recording while moving the stage through the known sequence, normally:

```text
0 -> 1 -> 2 -> 3 -> 4 -> 5 -> 0 mm
```

6. Hold each plateau for several seconds. No fit values need to be copied from the UI.
7. Stop recording after the return-to-zero plateau.

## Automated Python comparison

Use the maintained Windows `gk_analysis` checkout on `agent-dev` and its canonical interpreter:

```powershell
$GKPY = "$HOME\.venvs\gk_analysis\Scripts\python.exe"
```

The automated runner accepts the two Rust-generated files:

```powershell
& $GKPY scripts/run_live_stage_ab_auto.py `
    "PATH\TO\stage_z.bin" `
    "PATH\TO\stage_z.ab.csv" `
    --stage-axis z `
    --initial-height-mm 28.175 `
    --skip-v9
```

If the companion path is the normal sibling `<stage stem>.ab.csv`, it may be omitted:

```powershell
& $GKPY scripts/run_live_stage_ab_auto.py `
    "PATH\TO\stage_z.bin" `
    --stage-axis z `
    --initial-height-mm 28.175 `
    --skip-v9
```

After the fast systematic comparison succeeds, remove `--skip-v9` to include the adaptive v9 reference tracker.

The runner subtracts the exact Rust background snapshot, aligns Rust/Python frames by explicit `frame_id`, automatically detects stage transitions from field jumps, computes plateau medians, and reports displacement error plus off-axis leakage. No handwritten `rust_stage_z.csv` or separate background raw file is required for the normal workflow.
