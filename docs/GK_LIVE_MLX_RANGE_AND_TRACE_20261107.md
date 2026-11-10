# Gamma Knife live rigid 6-DOF — sensor range and trace safety (2026-11-07)

**Implemented on `feature/live-rigid-body-fit`; the user reported an app test pass before the November 9 dotted-guide changes. The new code requires fresh local build/tests and has NOT been exercised against connected MLX90393 boards.**
These controls are experimental operator aids; they do not certify
sensor saturation, historical range, or clinical tracking accuracy.

## Motivation / actual Nov06 signal problem

Previous Nov06 offline original-source analysis found 11/16 raw board0
sensors with 66 >8 mT sensor/frame jumps during movement; the effective
logged count lattice strongly matched gain7/res0/HALLCONF0xC,
whose nominal *symmetric digital* half ranges are roughly
**±4.915 mT X/Y** and **±7.930 mT Z**. Thus a uniform 7mT
limit is unsafe for X/Y. An observed 22mm magnet–chip separation
is useful empirical operational experience, but the actual margin
depends on field orientation, magnet strengths, board geometry,
per-axis settings, and local backgrounds; it is not a universal
physical cutoff.

In the pre-change UI, the Live Tracking tab **already** showed raw
per-selected-sensor Bx/By/Bz traces, but only after lengthy
CT/calibration panels and rigid pose plots; the separate Both tab
also displayed traces. A single manually entered common component
full-scale value was used for a 90%-of-range warning across peak
histories, not per-axis digital ranges. This was too easy to miss.

## Implementation

1. **Near-top full-width monitoring (updated Nov 9):** Live Tracking in rigid 6DOF
   mode places the current XYZ/rotation/fit-quality readout and dotted-line
   instructions **above** the chart area. Rigid pose plots and selected
   Bx/By/Bz raw traces for each board are **stacked full-width**, rather than
   competing for two half-width columns. CT detector and calibration controls
   remain below the live plots. The **Both** tab also stacks full-width
   Live Tracking and Sensor Traces panels, without duplicating raw charts.
2. **Prominent red back-off warning:** Latest X/Y/Z absolute
   component magnitude is computed across ALL observed channels
   on each available board, **not only the selected trace**.
   At >=65% of the axis-specific *advisory* limit a caution
   appears; at >=80% a **BACK OFF MAGNET** banner appears,
   naming the most critical raw board, firmware sensor index,
   axis, measurement and limit. It is a real-time advisory,
   NOT a hardware-detected saturation flag.
3. **Explicit range provenance:** The app computes *nominal symmetric
   16-bit digital* half-ranges for RES0/RES1 from the
   existing `data_transfer/src/conversions.rs` sensitivity table.
   The `GetMlxSensitivity` firmware RPC returns the **cached
   Context setting**, NOT a readback of all 48 chips' registers.
   Its range banner is labeled accordingly. If status is unknown
   or RES2/3 does not provide a supported symmetric estimate,
   it displays a **conservative gain7/res0 fallback**, explicitly
   **UNVERIFIED**, not a presumed default.
4. **Analog guard:** The Melexis datasheet rev012 reports onset
   around **50 mT** for standard IMC `-011/-012/-013/-014`
   and **25 mT** for thin-IMC `-111`. With no physically verified
   option code, the alert uses the **lower of the digital half
   range and a 25mT conservative analog guard**. This is not
   a tested safe tracking threshold. The app may still warn too
   late for aliasing if the true physical field is already
   wrapping and the logged values look normal.
5. **Session-latched sudden-jump warning:** Each sensor's
   temporally adjacent raw samples are checked for any
   **>8mT component discontinuity within 0.25s**. This
   latches a `RAW STEP WARNING` and **BACK OFF MAGNET /
   REVIEW DATA** state for the acquisition until a new stream
   resets trace history. The event is **not** classified as
   proven wrap or automatically corrected. Even after
   moving back, a warning about contaminated logged data
   remains visible for that acquisition.
6. **Deliberate wider-range preset, NOT automatic:** New
   **Select wider symmetric range (gain 0, res 1)**
   populates the existing gain, resolution, and four-phase
   fields without writing to hardware. The operator must
   stop streaming/logging, review, and explicitly click
   **Apply to detected boards**. The handler now blocks
   setting changes during active streams/logging, and a
   changed/first-read sensitivity status clears previously
   captured Hall background/reference and invalidates
   experimental rigid calibration. Reacquire before fitting.
   Gain0/res1 nominal symmetric digital half-ranges are
   **±49.2175 mT X/Y and ±79.2986 mT Z**, but **the analog
   sensor can saturate sooner**, so this is NOT a claim of
   ±79mT usable physical field. Higher RES2/3 digital count
   ranges are not silently selected: TCMP-disabled
   status must be verified and RES3 encoding is asymmetric.
7. **No experimental truth altered:** Current source MLX
   configuration, firmware transport, original saved logs,
   magnetic rigid fitting algorithm, CT transform, sensor
   coordinate mapping and 8/10mT offline channel QC are
   otherwise unchanged. No automatic unwrapping or new
   clinical tracking-accuracy claim.

## Dynamic dotted trace limits (Nov 9, 2026; code committed, hardware unverified)

In both **Live Tracking → Rigid 6-DOF** and **Sensor Traces / Both**, the Bx/By/Bz raw field plot now draws **dotted positive and negative axis-specific ADVISORY limits**, using the EXACT same shared `live_hall_range_model` source as the red/yellow live-range banner. No magnet model or data values are adjusted. The canvas automatically includes the plotted guide levels in its y-axis, so a large change to a wider range can compress the visible raw magnetic traces; no line is secretly outside the view. The caption distinguishes selected Bx/By/Bz colors, displayed advisory limits, the larger nominal digital limits, and provenance. Equal X/Y thresholds coincide and are drawn once.

**Dynamic settings:** the plots use `MlxSensitivityStatus` returned by the *successful* firmware get/set RPC, not editable `mlx_gain`, `mlx_resolution` or `mlx_hall_conf` text fields. Simply choosing the wider-range preset or editing controls must **not** cause the active limits to grow before the operator applies them and receives an `ok=true` response. The response is only the firmware's **cached setting**, **not a live 48-chip register snapshot**. After a new valid get/set response the guides update immediately in both plot views, together with the range warning. A new port clears the previous status; unsupported RES2/3 or unknown settings **fall back to explicitly UNVERIFIED conservative gain7/res0** (they do not borrow old or claim symmetric large ranges). Per-chip or separate X/Y/Z resolutions are NOT yet read back, so true per-chip limit verification still needs hardware.

**Actual guide values and caution:** gain7/res0/HALLCONF0xC gives dotted **X/Y ±4.9152mT, Z ±7.929856mT**. Gain0/res1/HALLCONF0xC gives digital **X/Y ±49.217536mT, Z ±79.29856mT**, but the **dotted advisory guides are ±25mT on each component** because the as-installed IMC analog option is unknown. These guides are *not a hardware saturation guarantee*: an aliased/wrapped field might fall below the displayed guide. The 65%-caution, 80%-back-off and session-latched >8mT jump warnings remain separate, and evaluate **all available chips**, not only each plotted sensor. Old data, rigid fits, and MLX programming logic are unchanged.

**Offline regression (run on Windows after pulling; NOT YET EXECUTED on new commit):**

```powershell
cd F:\GitHub\magnetic_board_driver
git pull --ff-only origin feature/live-rigid-body-fit
cargo test -p app
cargo check -p app
```

New tests check plotted positive/negative limit bounds, invalid limits, fallback configuration, dynamic gain/resolution update after a firmware response, and prevention of false widening from unapplied settings. The user previously reported an app test pass, but these new changes have not yet been tested on that machine. GUI and live-board behavior remain to be inspected.

---

## Full-width plot layout (Nov 9, 2026; source-only GUI update)

In **Live Tracking / Rigid 6-DOF**, the dynamic signed dotted-guide instruction/caption now appears **above** the first plot. The selected-sensor or disconnected placeholder Board 0/1/2 raw traces now appear **first**, each using the full window width; the rigid XYZ/rotation/HDMM prediction plot column follows at full width. Latest all-sensor peaks, sensor selectors and raw-data warnings remain attached to their respective traces. The Iced canvas uses `Length::Fill`, so resizing the window expands the traces horizontally; no horizontal half-width split remains.

**Both** now places the Live Tracking panel above the Sensor Traces panel, with full width for each instead of two half-width columns. The separate Sensor Traces panel already places the dotted-line instructions above the raw charts. The source includes a structural regression test checking the order (caption → rigid plot → raw traces) and the full-width Both layout. This changes only GUI layout; range calculation, firmware state provenance, rigid fitter, original data and live warning thresholds stay unchanged.

After pulling, run `cargo test -p app` and `cargo check -p app`; the changes have NOT been compiled, visually inspected on a user's desktop, or checked on actual sensor boards by the assistant.

---

## Trace panel visibility without boards (Nov 9, 2026)

**Fixed:** previously `Live Tracking / Rigid 6-DOF` created a Bx/By/Bz plot only inside `for sensor_board in context.sensor_traces.board_summaries()`, so opening the app with no controller or sensor boards showed three rigid charts (displacement, rotation, HDMM prediction) **but no raw Hall plot**. That behavior obscured the new live-trace work during offline testing.

The revised live dashboard now displays the **raw Hall trace section FIRST, immediately below the live range banner and dotted-guide instructions**, before the displacement/rotation/HDMM charts. It always reserves three full-window-width trace canvases, `Board 0/1/2`, whether or not the boards are connected. An absent board is explicitly labeled **NOT CONNECTED (waiting for stream)** and its canvas says **waiting for live sensor data**. No fake measurements are generated; the plot simply draws its existing axes and *provisional* positive/negative dotted advisory limits. The separate **Sensor Traces** tab likewise displays three empty guides and the disconnected status when no boards are detected. Once each board is reported, the placeholder is replaced by that board's normal selectable sensor chart and all-sensor range/step warnings. All three rigid plots remain, below the raw traces, still full width.

The source includes a structural layout regression (guide caption → raw traces → rigid charts, including disconnected placeholders) and an empty-trace numerical guide-bounds test. **The newly committed code has not yet been compiled or GUI-inspected on the user's Windows machine.**

```powershell
cd F:\GitHub\magnetic_board_driver
git pull --ff-only origin feature/live-rigid-body-fit
cargo test -p app
cargo check -p app
cargo run -p app --release
```

---

## Hall and HDMM monitoring side by side (Nov 9, 2026; latest layout)

**User-facing experiment priority:** The Live Tracking / Rigid 6-DOF panel now places **three live Hall-board Bx/By/Bz sensor charts on the left (60% of available plot width)** and **Hall-predicted HDMM reflector displacement on the right (40%)**, in a common full-window-width row. These two monitors are simultaneously visible while moving the magnets and running the experimental rigid fit. The dynamic dotted-guide explanation, worst-all-sensor range/step warning, and pose/fit status remain ABOVE the combined row.

The HDMM chart is explicitly labeled **"Hall-predicted HDMM marker displacement (NOT measured HDMM)"**. It predicts displacement at an independently confirmed marker position *from the fitted rigid pose*; the app does **not** acquire the optical vendor HDMM stream. No predicted marker history is shown until its coordinate and fit-zero are established. The other **rigid XYZ displacement and rotation** plots are still available in the right column immediately **beneath HDMM**, not removed or mistaken for vendor truth.

Board0/Board1/Board2 sensor traces retain their own drop-down selector, all-chip/latest-peak labels, and mT axis-specific dotted advisory guides. Without connected hardware, the left column still displays three **NOT CONNECTED** empty plot placeholders (with dotted reference guides) rather than silently collapsing. The **Sensor Traces** and **Both** tabs remain usable; "Both" still stacks full-width independent panels to avoid duplicating traces inside its rigid view.

This replaces the November 9 "all charts full-width and vertically stacked" Live Tracking layout below. The 60/40 column split is a deliberate tradeoff to keep the two primary monitoring signals next to each other on a typical desktop monitor. If the window is too narrow, maximize it for legible traces. The production fitter, field conversions, range warning calculation, and hardware configuration behavior are **unchanged**.

**Validation:** A source-structure regression checks dotted-guide placement, 3-board placeholders, side-by-side columns, and HDMM-before-secondary-motion order. The new GUI and Rust tests **still need to be run locally**; no connected-board or optical-HDMM test has been performed for this layout.

```powershell
cd F:\GitHub\magnetic_board_driver
git pull --ff-only origin feature/live-rigid-body-fit
cargo test -p app
cargo check -p app
cargo run -p app --release
```

---

## User workflow for the next GK measurement

1. Bring up the app and select the correct board-controller COM port.
   While not streaming, use **Read firmware cached setting**
   to populate the UI, understanding this is **not chip
   register verification**.
2. For a wider *symmetric* digital range, press **Select wider
   symmetric range (gain 0, res 1)**, then **Apply to detected
   boards** only while the stream and any logging are stopped.
   Check the returned `ok=true` and displayed values.
   **Do not assume this confirms all 48 chip registers**:
   the Set endpoint itself performs per-chip readback checks
   when responding `ok=true`, whereas the Get endpoint
   reports Context cache.
3. Because the field conversion or first-read provenance changed,
   repeat the no-magnet background/reference measurement,
   recalibrate with the chosen settings, and start a **new**
   recording filename (original log cannot be overwritten).
4. During rigid-body live fitting use **Live Tracking**:
   raw Bx/By/Bz selected traces appear beside rigid
   displacement plots. The prominent banner reports the
   worst current axis across **all** sensors, and a >8mT
   sudden field step stays flagged for that stream.
   Change the selected trace sensor per board if an alarm
   indicates a channel not currently plotted.
5. If the banner indicates caution, stop approaching; if
   **BACK OFF** or **RAW STEP WARNING**, stop motion, back
   the magnet off, and treat the affected interval as
   invalid for motion-accuracy validation. A quiet trace
   afterward is NOT independent proof of the correct
   field branch. If appropriate restart a new acquisition
   after checking/calibrating again. Capture independent
   CT/HDMM endpoints and clocks for scientific accuracy.

## First Windows compile attempt and syntax correction (2026-11-08)

The user ran the original Windows command batch after pulling through
commit `f9234c3`. `cargo test -p data_transfer nominal_mlx_range_tests`
**PASSED 2/2**. Each of the `app` test and check commands then failed
**at compile time** on two newly introduced Rust syntax errors in
`app/src/main.rs`:

- `ratio >= .80` (Rust requires `0.80`).
- `ratio >= .65` (Rust requires `0.65`).

Those two errors were fixed and committed as `9659b1d9` on
`feature/live-rigid-body-fit`. The earlier warnings for non-camel-case
`uT`, unused imports, unused `pose_label`, etc. were *warnings*,
not the blocking compile errors. **No current app test or cargo check
pass is yet confirmed** after the syntax fix. Re-pull and rerun:

~~~powershell
cd F:\GitHub\magnetic_board_driver
git pull --ff-only origin feature/live-rigid-body-fit
cargo test -p app sensor_trace
cargo test -p app tracking_mode_ui_tests
cargo check -p app
~~~

## Local build/tests (NOT YET EXECUTED by assistant)

~~~powershell
cd F:\GitHub\magnetic_board_driver
git checkout feature/live-rigid-body-fit
git pull --ff-only origin feature/live-rigid-body-fit

cargo test -p data_transfer nominal_mlx_range_tests
cargo test -p app sensor_trace
cargo test -p app tracking_mode_ui_tests
cargo check -p app
~~~

This crate has an Iced 0.14 UI and hardware-dependent code; successful
unit tests and `cargo check` are prerequisites to on-device review.
Then verify with a connected GK controller **without magnets present**
that the button remains disabled during a stream, saved logs are not
replaced, and the displayed gain/resolution reflects intended settings.
Test axis caution and back-off behavior using synthetic tests / a
controlled benign field change before relying on it while positioning
the magnets.

**Scientific acceptance gate:** A live displayed range margin is not
a verified physical non-saturation range; independent register/TCMP
provenance and calibration remain necessary, especially if firmware
was previously flashed from a different source revision.

## References

- Melexis MLX90393 datasheet rev012 (Nov 2025), Table 17 and
  magnetic specification / option code table:
  https://media.melexis.com/-/media/files/documents/datasheets/mlx90393-datasheet-melexis.pdf
- Local firmware:
  `firmware/microcontroller/src/mlx90393/sensor.rs`,
  `firmware/microcontroller/src/handlers.rs`.
