//! Magnet-center references for actual Gamma Knife motion-stage geometries.
//! These positions are not full Hall fitting profiles: sensor pose, field axes,
//! magnet orientations, strengths and background calibration are also required.
use std::fmt;

use crate::rigid_fit::RigidProfile;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MagnetLocationPreset {
    LinearStageCt,
    RotationStage,
}

impl MagnetLocationPreset {
    pub const ALL: [Self; 2] = [Self::LinearStageCt, Self::RotationStage];

    pub fn title(self) -> &'static str {
        match self {
            Self::LinearStageCt => "Linear stage — May-15 CT",
            Self::RotationStage => "Rotation stage — measured centers",
        }
    }

    pub fn reference_frame(self) -> &'static str {
        match self {
            Self::LinearStageCt => "CT world coordinates (May 15, 2026)",
            Self::RotationStage => "Rotational-stage frame, relative to stage rotation center",
        }
    }

    pub fn source(self) -> &'static str {
        match self {
            Self::LinearStageCt =>
                "sensor_position_data_05152026.xlsx, markers_051526; linear CT geometry",
            Self::RotationStage =>
                "hall-effect-motion-tracking/src/gk_analysis/rotation_stage.py, MEASURED_MAGNET_CENTERS_STAGE_MM",
        }
    }

    /// The index ordering must match the paired calibrated profile's magnets array.
    /// Linear CT order: Left/M1, Top/M2, Right/M3.
    /// Rotation stage order: Top, Left, Right.
    pub fn centers_mm(self) -> [[f64; 3]; 3] {
        match self {
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
        lines.push(format!("Source: {}", self.source()));
        lines.join("\n")
    }

    /// A magnet-location reference only authorizes live fitting when its centers
    /// agree with the same-frame, fully calibrated sensor and magnet profile.
    /// We never transfer numeric positions between CT and stage coordinate frames.
    pub fn matches_profile(self, profile: &RigidProfile) -> bool {
        profile.magnets.len() == 3 && self.centers_mm().iter()
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
    fn builtin_a1_profile_is_not_mistaken_for_either_stage() {
        let a1 = RigidProfile::a1_reference().unwrap();
        assert_eq!(MagnetLocationPreset::identify_profile(&a1), None);
    }

    #[test]
    fn stage_reference_recognition_requires_matching_centers_and_order() {
        let mut profile = RigidProfile::a1_reference().unwrap();
        for (magnet, center) in profile.magnets.iter_mut()
            .zip(MagnetLocationPreset::LinearStageCt.centers_mm()) {
            magnet.center_mm = center;
        }
        assert_eq!(
            MagnetLocationPreset::identify_profile(&profile),
            Some(MagnetLocationPreset::LinearStageCt)
        );
        profile.magnets.swap(0, 1);
        assert_eq!(MagnetLocationPreset::identify_profile(&profile), None);
    }
}
