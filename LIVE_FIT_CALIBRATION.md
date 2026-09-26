# Live-fit calibration and coordinate convention

This note records the production live-fit preprocessing added on the `gk_measurement` branch after the Bambu A1 calibration campaign.

## Production fit path

The raw Sensor Trace remains unchanged. Only the live pose fitter converts incoming measurements into the calibrated model frame.

For board `b` and physical sensor `i`, the live fitter uses the finalized A1 scalar response calibration `s[b][i]` and the KiCad-verified 4x4 geometry.

The fit input is

```text
B_fit = T((B_raw - B_background) / s[b][i])
T([Bx, By, Bz]) = [By, -Bx, Bz]
```

Background capture and the A/B companion background records remain in the original raw logger XYZ frame. The scalar normalization and logger-to-model axis transform are applied only after background subtraction, immediately before pose fitting.

## Sensor geometry

With the JST connectors toward `-Y` / the front of the board, the physical sensor-index grid is

```text
12   8   4   0
13   9   5   1
14  10   6   2
15  11   7   3
```

The live fitter derives these positions locally and intentionally ignores transmitted `field.position` values. This protects the fit from stale firmware geometry.

Coordinates are a 13.5 mm square grid centered at the origin, with 4.5 mm pitch, so the outer coordinates are `+/-6.75 mm`.

## A1 calibration source

The 48 scalar calibration values come from:

- repository: `nguyenbh7/hall-effect-motion-tracking`
- branch: `a1-calibration`
- source artifact: `a1_calibration_results/comparison/sensor_scales.csv`
- calibration finalization commit: `26e89790aec7fe5ff921b3c1f26027f73e159abe`

The calibration artifact is indexed by physical sensor number. The live app derives the same physical sensor index from the sensor address, while the calibration analysis separately tracked the binary logger's raw-slot permutation.

Firmware board IDs map directly to calibration boards:

- `0` = Board A
- `1` = Board B
- `2` = Board C

Board C uses the XOR'd I2C address bit in firmware; the app normalizes that address before deriving the physical sensor index.

## Regression coverage

`app/src/live_fit.rs` contains tests for:

- KiCad sensor-index geometry
- finalized A1 scale lookup for Boards A/B/C
- Board C address normalization
- ignoring transmitted/stale `field.position`
- preserving raw logger XYZ through background capture/subtraction
- scalar normalization plus `[By, -Bx, Bz]` frame conversion after the background stage
- existing per-sensor/per-axis background subtraction

Run the app tests locally with the repository's Rust toolchain before flashing/using the branch for a new validation measurement:

```bash
cargo test -p app
```
