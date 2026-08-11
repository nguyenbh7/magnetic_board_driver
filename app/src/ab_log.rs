use std::fs::File;
use std::io::{self, BufWriter, Write};
use std::path::{Path, PathBuf};

use crate::live_fit::{BackgroundSnapshot, LiveFitLogSnapshot};

const HEADER: &str = "record_type,board_id,frame_id,time_us,sensor_index,bg_x_mT,bg_y_mT,bg_z_mT,x_mm,y_mm,z_mm,theta_rad,phi_rad,residual_rms_mT,objective_score,n_sensors,target_moment_norm,dx_mm,dy_mm,dz_mm,disp_mm";

pub fn companion_path(raw_path: &Path) -> PathBuf {
    raw_path.with_extension("ab.csv")
}

pub fn create_companion(
    raw_path: &Path,
    backgrounds: &[BackgroundSnapshot],
) -> io::Result<BufWriter<File>> {
    let path = companion_path(raw_path);
    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);
    writeln!(writer, "{HEADER}")?;

    for background in backgrounds {
        write_background_record(&mut writer, background)?;
    }

    writer.flush()?;
    Ok(writer)
}

fn background_record_line(background: &BackgroundSnapshot) -> String {
    format!(
        "background,{},{},{},{},{:.17},{:.17},{:.17},,,,,,,,,,,,,",
        background.board_id,
        0,
        0,
        background.sensor_index,
        background.field_m_t[0],
        background.field_m_t[1],
        background.field_m_t[2],
    )
}

fn write_background_record(
    writer: &mut BufWriter<File>,
    background: &BackgroundSnapshot,
) -> io::Result<()> {
    writeln!(writer, "{}", background_record_line(background))
}

pub fn write_fit_record(
    writer: &mut BufWriter<File>,
    record: &LiveFitLogSnapshot,
) -> io::Result<()> {
    writeln!(
        writer,
        "fit,{},{},{},,,,,{:.17},{:.17},{:.17},{:.17},{:.17},{:.17},{:.17},{},{:.17},{:.17},{:.17},{:.17},{:.17}",
        record.board_id,
        record.frame_id,
        record.time_us,
        record.x_mm,
        record.y_mm,
        record.z_mm,
        record.theta_rad,
        record.phi_rad,
        record.residual_rms_m_t,
        record.objective_score,
        record.n_sensors,
        record.target_moment_norm,
        record.dx_mm,
        record.dy_mm,
        record.dz_mm,
        record.displacement_mm,
    )?;
    writer.flush()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn companion_extension_is_predictable() {
        let path = Path::new("stage_z.bin");
        assert_eq!(companion_path(path), PathBuf::from("stage_z.ab.csv"));
    }

    #[test]
    fn background_record_matches_header_width() {
        let background = BackgroundSnapshot {
            board_id: 1,
            sensor_index: 0,
            field_m_t: [0.1, -0.2, 0.3],
        };
        assert_eq!(
            background_record_line(&background).split(',').count(),
            HEADER.split(',').count()
        );
    }
}
