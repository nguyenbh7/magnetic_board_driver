# MLX90393 4×4 Hall sensor PCB — source KiCad project

This directory is the designated version-controlled home for the **editable**
MLX90393 board project supplied by the lab. The original user-uploaded source
archive was `MLX90393_board4.zip` (ChatGPT Library upload 2026-09-25).
**Inside that ZIP the source project is still named `MLX90393_board3`.**
Preserve the filenames rather than renaming schematic references.

## Provenance and current upload state

Source ZIP SHA-256:
`32363b1b48f097b6289367b30ce883349a952ba2ddef2e18a702d23629b0fb12`.

A curated source-only ZIP has been prepared for local extraction into
`hardware/kicad/MLX90393_board4/`. It contains **21 source/library files**
(about 1.28 MB uncompressed, 265 KB compressed):
`MLX90393_board3.kicad_pro`,
`MLX90393_board3.kicad_pcb`,
`MLX90393_board3.kicad_sch`,
`MLX_array.kicad_sch`, `fp-lib-table`, `sym-lib-table`,
local `.kicad_sym` symbol files, `Footprints/`, `JLCPCB_lib/`
footprint/symbol/3D libraries, and `Addafruit.pretty/`.

**Important:** This documentation was committed directly to GitHub.
The **large KiCad source files themselves require the local Git push**
described below; do not assume they are already in Git because this README
exists. Do not commit the original archive's backups, `.kicad_prl`,
lock/autosave files, cache files, Gerbers, drill plots, or generated
manufacturing ZIPs. The editable project files are the source of truth.

### Install the source-only ZIP on the Windows development machine

1. Download `MLX90393_board4_git_source.zip` from the ChatGPT
   deliverable and save it to `$HOME\Downloads`.
2. From PowerShell:

```powershell
Set-Location 'F:\GitHub\magnetic_board_driver'
git switch feature/live-rigid-body-fit
git pull --ff-only
Expand-Archive -LiteralPath "$HOME\Downloads\MLX90393_board4_git_source.zip" -DestinationPath . -Force
git status --short -- hardware/kicad
git add hardware/kicad
git commit -m "Add editable MLX90393 board KiCad source and local footprint libraries"
git push
```

3. Open `hardware/kicad/MLX90393_board4/MLX90393_board3.kicad_pro`
   in KiCad 9. Review missing library/model warnings and compare
   against the **actual fabricated October 23 PCB revision**.
   The ZIP is selected source files, not a full backup of the
   original 10-MB archive.

## What the archived PCB supports

The populated `MLX90393_board3.kicad_pcb` has **16**
`MLX90393-01x` QFN-16 sensor footprints `M1–M16` on
**F.Cu**, all with `(at x y 180)`, on a regular
**4×4 array of 4.5-mm pitch**. Connector footprints
`J1–J3` are near the PCB's low-Y edge (native
KiCad coordinates approximately Y=82.9 mm), with
sensor centers around Y=118.4–131.9 mm. Four mounting
holes `H1–H4` are included. These are **KiCad layout
coordinates**, not patient DICOM coordinates or firmware
sensor-index evidence.

Current Rust `data_transfer/src/rpc.rs:SENSOR_POSITIONS_MM`
uses acquisition indices increasing by +Y first and then +X.
The source layout's reference designators `M1–M16` do not
automatically establish which footprint is firmware index 0;
trace the I²C addresses and design nets independently.

## Package axis orientation (provisional)

Melexis' **Figure 26** QFN top view shows
**pin 1 toward the upper-right**, with package **+X up,
+Y left, +Z outward**. Its separate **Figure 27**
pinout illustrates **pin 1 at bottom-left**, a view
rotated 180° in plane; do not transfer axis arrows
without rotating the view. The KiCad footprint's
**pad 1 is lower-left in the 0° footprint library**;
all 16 mounted footprints are rotated **180° on
F.Cu**, making pad 1 **upper-right in native KiCad
top view**, matching Figure 26.

Therefore the provisional **package axes in native
KiCad top view** are +X **toward decreasing PCB Y**
(toward JST), +Y **toward decreasing PCB X**,
and +Z **out of the F.Cu face**.

**Correction from the populated board and current firmware:**
`data_transfer/src/rpc.rs` explicitly diagrams
`0 4 8 12` on the **bottom row with JST down**,
i.e. firmware sensor 0 is **bottom-left**, not
bottom-right. Firmware +X points picture-right
(toward decreasing native KiCad X once viewed
with JST down), +Y away from JST (toward
increasing native KiCad Y), and +Z toward the
chip-facing viewer. The previous CT candidate
`gk_ct_sensor_order_proposal_v1` mistakenly
used sensor 0 at bottom-right and must be
replaced by the guarded `v2` mapping.

The KiCad footprint and manufacturer QFN axis
orientation then give the **provisional proper
chip-to-board field transform**:

```text
[B_grid_X, B_grid_Y, B_grid_Z] =
    [B_chip_Y, -B_chip_X, B_chip_Z]

M_chip_to_grid = [[0,1,0],[-1,0,0],[0,0,1]]
det(M_chip_to_grid) = +1
```

This agrees with the existing A1-style
`[By, -Bx, Bz]` transform in the Rust analysis
for this same chip/board convention. It remains
**unverified for actual October mounted
boards**, for the reasons below.

This matrix is **not yet validated against the actual
installed board, flashed firmware, or measured known
signed field**. For the CBCT application,
`B_LPS_candidate = R_grid_to_LPS @ M_chip_to_grid @ B_raw`,
with each board's scan-specific CT grid rotation.
Do **not** insert these candidate matrices into a
live `RigidProfile` as verified.

Melexis primary references: `MLX90393 Datasheet`
QFN-16 package/sensitive spot and pinout, figures 26–27;
`MLX90393 Getting Started Guide`, Figure 2
(MLX90393 QFN axis definition). Verify the actual
package designation before using this convention.

Further evidence and scan-specific geometry:
`hall-effect-motion-tracking/tools/cbct_bridge/HALL_AXIS_PROVENANCE.md`.
