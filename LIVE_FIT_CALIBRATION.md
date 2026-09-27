# Live-fit calibration and coordinate convention

This note records the production live-fit preprocessing on the `gk_measurement` branch after the Bambu A1 calibration and the September 27 A1/Aug-14 coordinate audit.

## Production fit path

The raw Sensor Trace remains unchanged. Only the live pose fitter converts incoming measurements into the calibrated physical-board/model frame.

For board `b` and firmware/I2C address-derived sensor index `i`, the fit input is

```text
B_fit = T((B_raw - B_background) / s_address[b][i])
T([Bx, By, Bz]) = [By, -Bx, Bz]
```

Background capture and A/B companion background records remain in raw logger XYZ. Scalar normalization and the logger-to-board rotation are applied only after background subtraction, immediately before pose fitting.

## Canonical sensor identity and physical geometry

Sensor identity is the firmware/I2C address-derived index `0..15`. With JST connectors toward `-Y` / the front of the board, the resolved physical grid is

```text
             +Y / rear

 3    7   11   15
 2    6   10   14
 1    5    9   13
 0    4    8   12

             -Y / front / JST
 -X                  +X
```

The grid is 13.5 mm center-to-center across each axis with 4.5 mm pitch, so the outer coordinates are `+/-6.75 mm`.

The earlier implementation mirrored X because centered KiCad X was interpreted from the opposite PCB viewing side. The A1/Aug-14 audit resolved this by combining the empirical A1 raw-slot/target mapping, model-independent Maxwell checks, and an exact Aug-14 robust-baseline recovery.

The live fitter derives these positions locally and intentionally ignores transmitted `field.position` values. Firmware geometry is nevertheless kept consistent so raw metadata and fit geometry agree.

## A1 field transform

The canonical logger -> board/model transform remains

```text
[By, -Bx, Bz]
```

The independent full 3-D A1 Maxwell diagnostic tested all 48 signed axis permutations and ranked `[By, -Bx, Bz]` as the best proper rotation. No GK-specific field transform is required.

## A1 scalar gain source and re-indexing

The 48 scalar response values come from:

- repository: `nguyenbh7/hall-effect-motion-tracking`
- branch: `a1-calibration`
- source artifact: `a1_calibration_results/comparison/sensor_scales.csv`
- calibration finalization commit: `26e89790aec7fe5ff921b3c1f26027f73e159abe`

The source CSV is indexed by the **historical A1 commanded-target labels**, not by firmware/address identity. All three boards use the same saved relation:

```text
historical A1 label -> address index
0..3   -> 12..15
4..7   ->  8..11
8..11  ->  4..7
12..15 ->  0..3
```

Production therefore remaps the table before applying a sensor scale. Address index 0 receives historical label 12's scale; address index 12 receives historical label 0's scale, etc.

This correction preserves the useful response calibration while attaching it to the correct electrical sensor identity.

Firmware board IDs map directly to calibration boards:

- `0` = Board A
- `1` = Board B
- `2` = Board C

Board C uses the XOR'd I2C address bit in firmware; the app normalizes that bit before deriving address sensor index.

## Regression coverage

`app/src/live_fit.rs` contains tests for:

- resolved physical address-index geometry;
- historical-A1-label -> address-index scalar lookup for Boards A/B/C;
- Board C address normalization;
- ignoring transmitted/stale `field.position` in production fits;
- preserving raw logger XYZ through background capture/subtraction;
- remapped scalar normalization plus `[By, -Bx, Bz]` conversion after the background stage;
- existing per-sensor/per-axis background subtraction.

Run the app tests locally before flashing/using the branch for a new validation measurement:

```bash
cargo test -p app
```
