//! Built-in rigid tracking setups.
//! A1 V1 is a complete bundled calibration for a live demo. Gamma Knife
//! stage references are magnet centers only until a matching full calibrated
//! board/sensor/magnet geometry profile is loaded.
use std::fmt;

use crate::rigid_fit::RigidProfile;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MagnetLocationPreset {
    A1Demo,
    LinearStageCt,
    RotationStage,
}

impl MagnetLocationPreset {
    pub const ALL: [Self; 3] = [Self::A1Demo, Self::LinearStageCt, Self::RotationStage];

    pub fn title(self) -> &'static str {
        match self {
            Self::A1Demo => "A1 V1 — calibrated demo",
            Self::LinearStageCt => "Linear stage — May-15 CT",
            Self::RotationStage => "Rotation stage — measured centers",
        }
    }

    pub fn frame_key(self) -> &'static str {
        match self {
            Self::A1Demo => "a1_v1_rigid",
            Self::LinearStageCt => "gk_may15_ct",
            Self::RotationStage => "gk_rotation_stage",
        }
    }

    pub fn reference_frame(self) -> &'static str {
        match self {
            Self::A1Demo => "A1 V1 three-board common rigid-fit frame",
            Self::LinearStageCt => "CT world coordinates (May 15, 2026)",
            Self::RotationStage => "Rotational-stage frame, relative to stage rotation center",
        }
    }

    pub fn source(self) -> &'static str {
        match self {
            Self::A1Demo =>
                "hall-effect-motion-tracking/data/reference/a1_v1_rigid_constellation_2026-10-05.json",
            Self::LinearStageCt =>
                "sensor_position_data_05152026.xlsx, markers_051526; linear CT geometry",
            Self::RotationStage =>
                "hall-effect-motion-tracking/src/gk_analysis/rotation_stage.py, MEASURED_MAGNET_CENTERS_STAGE_MM",
        }
    }

    /// The index ordering must match the paired calibrated profile's magnets array.
    /// A1 order: Board 0, Board 1, Board 2.
    /// Linear CT order: Left/M1, Top/M2, Right/M3.
    /// Rotation stage order: Top, Left, Right.
    pub fn centers_mm(self) -> [[f64; 3]; 3] {
        match self {
            Self::A1Demo => [
                [-31.26555645542942, 55.12674842, -3.64219784],
                [-30.82923192542942, -55.50620025, -0.02221616],
                [65.13463900085884, 1.29267975, -1.30338343],
            ],
            Self::LinearStageCt => [
                [180.2, 114.4, 114.3],
                [104.1, 95.2, -14.3],
                [17.7, 105.0, 112.4],
            ],
            Self::RotationStage => [
                [126.89, 0.0, -90.59],
                [0.0, 82.03, -94.15],
                [0.0, -82.03, -94.15],
            ],
        }
    }

    pub fn labels(self) -> [&'static str; 3] {
        match self {
            Self::A1Demo => ["Board 0 / upper left", "Board 1 / lower left", "Board 2 / right"],
            Self::LinearStageCt => ["Left / M1", "Top / M2", "Right / M3"],
            Self::RotationStage => ["Top", "Left", "Right"],
        }
    }

    pub fn summary(self) -> String {
        let mut lines = vec![format!(
            "{} · {}",
            self.title(),
            self.reference_frame()
        )];
        for (name, center) in self.labels().into_iter().zip(self.centers_mm()) {
            lines.push(format!(
                "{}: ({:.2}, {:.2}, {:.2}) mm",
                name, center[0], center[1], center[2]
            ));
        }
        if self == Self::A1Demo {
            lines.push("Bundled 110 mm triangle calibration; capture all three no-magnet board backgrounds before demo".into());
            lines.push("A1 magnet centers are effective fitted offsets, not independently CT-measured".into());
        } else {
            lines.push("Reference centers only; fitting requires a matching calibrated profile".into());
        }
        lines.push(format!("Source: {}", self.source()));
        lines.join("\n")
    }

    /// A reference is identified only when its centers and explicit coordinate
    /// frame agree with the loaded full calibrated sensor and magnet profile.
    /// We never transfer numeric positions between CT and stage coordinate frames.
    pub fn matches_profile(self, profile: &RigidProfile) -> bool {
        profile.coordinate_frame.as_deref() == Some(self.frame_key())
            && profile.magnets.len() == 3 && self.centers_mm().iter()
            .zip(&profile.magnets)
            .all(|(expected, actual)| expected.iter().zip(actual.center_mm.iter())
                .all(|(a,b)| (a-b).abs() <= 0.05))
    }

    pub fn identify_profile(profile: &RigidProfile) -> Option<Self> {
        Self::ALL.into_iter().find(|preset| preset.matches_profile(profile))
    }
}

impl fmt::Display for MagnetLocationPreset {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.title())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn reference_coordinates_are_not_interchangeable() {
        assert_ne!(MagnetLocationPreset::LinearStageCt.centers_mm(),
                   MagnetLocationPreset::RotationStage.centers_mm());
    }

    #[test]
    fn builtin_a1_profile_is_selected_as_a1_not_the_gk_stages() {
        let a1 = RigidProfile::a1_reference().unwrap();
        assert_eq!(MagnetLocationPreset::identify_profile(&a1),
                   Some(MagnetLocationPreset::A1Demo));
        assert!(!MagnetLocationPreset::LinearStageCt.matches_profile(&a1));
        assert!(!MagnetLocationPreset::RotationStage.matches_profile(&a1));
    }

    #[test]
    fn stage_reference_recognition_requires_matching_centers_and_order() {
        let mut profile = RigidProfile::a1_reference().unwrap();
        for (magnet, center) in profile.magnets.iter_mut()
            .zip(MagnetLocationPreset::LinearStageCt.centers_mm()) {
            magnet.center_mm = center;
        }
        assert_eq!(MagnetLocationPreset::identify_profile(&profile), None);
        profile.coordinate_frame = Some("gk_may15_ct".to_string());
        assert_eq!(
            MagnetLocationPreset::identify_profile(&profile),
            Some(MagnetLocationPreset::LinearStageCt)
        );
        profile.magnets.swap(0, 1);
        assert_eq!(MagnetLocationPreset::identify_profile(&profile), None);
        profile.magnets.swap(0, 1);
        // A coincidental center set in an unlabelled or different frame
        // must also be rejected, even if its coordinates match.
        profile.coordinate_frame = None;
        assert_eq!(MagnetLocationPreset::identify_profile(&profile), None);
    }
}
