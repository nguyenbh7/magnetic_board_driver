"""Apply synchronized A/B companion logging on top of local item-7 edits.

This is intentionally a local source patcher rather than a wholesale replacement of
``main.rs`` / ``live_fit.rs`` because the hardware checkout may contain uncommitted
item-7 UI changes.  The patch is transactional: both source files are transformed in
memory and are written only after every required edit succeeds.

Run from the magnetic_board_driver repository root:

    python3 scripts/apply_ab_companion_logging.py

Then run:

    cargo fmt -p app
    cargo test -p app
    cargo check -p app
"""

from __future__ import annotations

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "app" / "src" / "main.rs"
LIVE = ROOT / "app" / "src" / "live_fit.rs"


class PatchError(RuntimeError):
    pass


def sub_once(text: str, pattern: str, replacement: str, label: str, flags: int = 0) -> str:
    updated, count = re.subn(pattern, replacement, text, count=1, flags=flags)
    if count != 1:
        raise PatchError(f"{label}: expected exactly one match, found {count}")
    return updated


def patch_live_fit(text: str) -> str:
    if "pub struct LiveFitLogSnapshot" in text and "pub fn background_snapshots" in text:
        return text

    text = sub_once(
        text,
        r"(last_fit_time_us:\s*Option<u64>,)",
        r"\1\n    last_fit_frame_id: Option<u32>,",
        "live state frame id",
    )
    text = sub_once(
        text,
        r"(last_fit_time_us:\s*None,)",
        r"\1\n            last_fit_frame_id: None,",
        "live state default frame id",
    )

    point_pattern = re.compile(
        r"(#\[derive\(Debug, Clone\)\]\s*pub struct DisplacementPoint\s*\{\s*"
        r"pub time_s:\s*f64,\s*pub displacement_mm:\s*f64,\s*\})",
        re.MULTILINE,
    )
    match = point_pattern.search(text)
    if not match:
        raise PatchError("log structs: DisplacementPoint block not found")
    structs = r'''

#[derive(Debug, Clone)]
pub struct BackgroundSnapshot {
    pub board_id: u16,
    pub sensor_index: u8,
    pub field_m_t: [f64; 3],
}

#[derive(Debug, Clone)]
pub struct LiveFitLogSnapshot {
    pub board_id: u16,
    pub frame_id: u32,
    pub time_us: u64,
    pub x_mm: f64,
    pub y_mm: f64,
    pub z_mm: f64,
    pub theta_rad: f64,
    pub phi_rad: f64,
    pub residual_rms_m_t: f64,
    pub objective_score: f64,
    pub n_sensors: usize,
    pub target_moment_norm: f64,
    pub dx_mm: f64,
    pub dy_mm: f64,
    pub dz_mm: f64,
    pub displacement_mm: f64,
}
'''
    text = text[: match.end()] + structs + text[match.end() :]

    text = sub_once(
        text,
        r"(self\.last_fit_time_us\s*=\s*Some\(frame_mid_time_us\);)",
        r"\1\n            self.last_fit_frame_id = Some(frame.frame_id);",
        "record completed fit frame id",
    )

    # Clearing a stale fit-frame ID anywhere the fit timestamp is cleared is safe and
    # keeps logging from associating a previous solution with a later raw frame.
    text, count = re.subn(
        r"self\.last_fit_time_us\s*=\s*None;",
        "self.last_fit_time_us = None;\n            self.last_fit_frame_id = None;",
        text,
    )
    if count < 1:
        raise PatchError("clear fit frame id: no last_fit_time_us reset found")

    methods = r'''
    pub fn background_snapshots(&self) -> Vec<BackgroundSnapshot> {
        let mut snapshots = Vec::new();

        for (board_id, board) in &self.boards {
            let Some(background) = &board.background else {
                continue;
            };

            for (sensor_index, field_m_t) in background {
                snapshots.push(BackgroundSnapshot {
                    board_id: *board_id,
                    sensor_index: *sensor_index,
                    field_m_t: *field_m_t,
                });
            }
        }

        snapshots
    }

    pub fn log_snapshot_for_frame(
        &self,
        board_id: u16,
        frame_id: u32,
    ) -> Option<LiveFitLogSnapshot> {
        let board = self.boards.get(&board_id)?;
        if board.last_fit_frame_id != Some(frame_id) {
            return None;
        }

        let result = board.result.as_ref()?;
        let time_us = board.last_fit_time_us?;
        let origin = board.origin.unwrap_or(result.position);
        let dx_mm = result.position.0 - origin.0;
        let dy_mm = result.position.1 - origin.1;
        let dz_mm = result.position.2 - origin.2;
        let displacement_mm = (dx_mm * dx_mm + dy_mm * dy_mm + dz_mm * dz_mm).sqrt();

        let theta_rad = if result.moment_norm.is_finite() && result.moment_norm > 1.0e-12 {
            (result.moment.2 / result.moment_norm).clamp(-1.0, 1.0).acos()
        } else {
            f64::NAN
        };
        let phi_rad = result.moment.1.atan2(result.moment.0);

        let target_moment_norm = if board.use_known_magnet_prior {
            board.magnet_preset.moment_norm_app() * board.magnet_effective_scale
        } else {
            board.calibrated_moment_norm?
        };

        Some(LiveFitLogSnapshot {
            board_id,
            frame_id,
            time_us,
            x_mm: result.position.0,
            y_mm: result.position.1,
            z_mm: result.position.2,
            theta_rad,
            phi_rad,
            residual_rms_m_t: result.residual_rms,
            objective_score: result.objective_score,
            n_sensors: result.n_sensors,
            target_moment_norm,
            dx_mm,
            dy_mm,
            dz_mm,
            displacement_mm,
        })
    }

'''
    marker = re.search(r"\n\s*pub fn set_board_magnet_preset\s*\(", text)
    if not marker:
        raise PatchError("live logging methods: set_board_magnet_preset marker not found")
    text = text[: marker.start()] + "\n" + methods + text[marker.start() :]
    return text


def patch_main(text: str) -> str:
    if "mod ab_log;" in text and "ab_writer:" in text and "log_snapshot_for_frame" in text:
        return text

    text = sub_once(
        text,
        r"(mod live_fit;)",
        r"mod ab_log;\n\1",
        "ab_log module declaration",
    )

    text = sub_once(
        text,
        r"(file_writer:\s*Option<Arc<Mutex<BufWriter<File>>>>,)",
        r"\1\n    ab_writer: Option<Arc<Mutex<BufWriter<File>>>>,\n    ab_logged_frames: BTreeMap<u16, u32>,",
        "context A/B writers",
    )

    write_data_match = re.search(
        r"async fn write_data\(.*?\n\}\n",
        text,
        flags=re.DOTALL,
    )
    if not write_data_match:
        raise PatchError("A/B writer helper: write_data function not found")
    helper = r'''

async fn write_ab_fit(
    writer: &Mutex<BufWriter<File>>,
    record: &live_fit::LiveFitLogSnapshot,
) {
    let mut writer = writer.lock().await;
    if let Err(err) = ab_log::write_fit_record(&mut writer, record) {
        eprintln!("Failed to write A/B companion fit row: {err}");
    }
}
'''
    text = text[: write_data_match.end()] + helper + text[write_data_match.end() :]

    stream_start = re.search(
        r"(Message::RecievedStreamField\(sensor_field\)\s*=>\s*\{\s*"
        r"context\.live_fits\.update\(sensor_field\.clone\(\)\);)",
        text,
        flags=re.DOTALL,
    )
    if not stream_start:
        raise PatchError("stream logging: RecievedStreamField update marker not found")
    insertion = r'''

            let fit_log = context
                .live_fits
                .log_snapshot_for_frame(sensor_field.board_id, sensor_field.frame_id)
                .filter(|record| {
                    context.ab_logged_frames.get(&record.board_id).copied()
                        != Some(record.frame_id)
                });

            if let Some(record) = &fit_log {
                context
                    .ab_logged_frames
                    .insert(record.board_id, record.frame_id);
            }
'''
    text = text[: stream_start.end()] + insertion + text[stream_start.end() :]

    text = sub_once(
        text,
        r"(match\s*&context\.file_writer\s*\{\s*Some\(w\)\s*=>\s*\{\s*let wr = w\.clone\(\);)",
        r"let ab_writer = context.ab_writer.clone();\n            let fit_log_for_write = fit_log.clone();\n\n            \1",
        "stream companion captures",
        flags=re.DOTALL,
    )

    text = sub_once(
        text,
        r"(let _val = write_data\(&\*wr, &sensor_field\)\.await;)",
        r'''\1
                                if let (Some(ab_writer), Some(fit_record)) =
                                    (ab_writer, fit_log_for_write)
                                {
                                    write_ab_fit(&*ab_writer, &fit_record).await;
                                }''',
        "stream companion write",
    )

    file_open_pattern = re.compile(
        r"Message::FileOpened\(fh\)\s*=>\s*\{\s*"
        r"if let Ok\(fh\) = fh \{\s*"
        r"let path = fh\.path\(\);\s*"
        r"let file = File::create\(path\)\.unwrap\(\);\s*"
        r"let writer = BufWriter::new\(file\);\s*"
        r"let writer = Arc::new\(Mutex::new\(writer\)\);\s*"
        r"context\.file_writer = Some\(writer\);\s*"
        r"\}\s*Task::none\(\)\s*\},",
        re.DOTALL,
    )
    if not file_open_pattern.search(text):
        raise PatchError("file selection: FileOpened block not found")
    file_open_replacement = r'''Message::FileOpened(fh) => {
            if let Ok(fh) = fh {
                let path = fh.path().to_path_buf();
                let file = File::create(&path).unwrap();
                context.file_writer = Some(Arc::new(Mutex::new(BufWriter::new(file))));

                let backgrounds = context.live_fits.background_snapshots();
                match ab_log::create_companion(&path, &backgrounds) {
                    Ok(writer) => {
                        let companion_path = ab_log::companion_path(&path);
                        println!(
                            "A/B logging: raw={} companion={}",
                            path.display(),
                            companion_path.display()
                        );
                        context.ab_writer = Some(Arc::new(Mutex::new(writer)));
                    }
                    Err(err) => {
                        eprintln!("Failed to create A/B companion log: {err}");
                        context.ab_writer = None;
                    }
                }
                context.ab_logged_frames.clear();
            }
            Task::none()
        },'''
    text = file_open_pattern.sub(file_open_replacement, text, count=1)

    text = text.replace('button("Select File").on_press(Message::SelectFile)',
                        'button("Select raw + A/B log").on_press(Message::SelectFile)', 1)
    return text


def main() -> int:
    main_text = MAIN.read_text()
    live_text = LIVE.read_text()

    try:
        patched_live = patch_live_fit(live_text)
        patched_main = patch_main(main_text)
    except PatchError as exc:
        print(f"ABORTED without writing files: {exc}", file=sys.stderr)
        return 1

    if patched_live == live_text and patched_main == main_text:
        print("A/B companion logging already appears to be applied.")
        return 0

    # All transformations succeeded; only now mutate the working tree.
    LIVE.write_text(patched_live)
    MAIN.write_text(patched_main)
    print("Applied A/B companion logging to app/src/live_fit.rs and app/src/main.rs")
    print("Next: cargo fmt -p app && cargo test -p app && cargo check -p app")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
