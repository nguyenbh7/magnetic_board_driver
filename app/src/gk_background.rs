//! Portable, CT-independent 48-channel GK no-magnet background.
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GkBackgroundFile {
    format: String,
    units: String,
    sensor_order: String,
    raw_board_ids: [u16; 3],
    sensor_masks: [u16; 3],
    captured_at_unix_s: Option<u64>,
    background_raw_mt: [[[f64; 3]; 16]; 3],
    warning: String,
}

impl GkBackgroundFile {
    pub fn from_raw(raw: [[[f64; 3]; 16]; 3]) -> Result<Self, String> {
        let value=Self {
            format: "gk_raw_hall_background_v1".into(),
            units: "mT".into(),
            sensor_order: "raw_firmware_indices_0_to_15".into(),
            raw_board_ids: [0,1,2],
            sensor_masks: [0xffff;3],
            captured_at_unix_s: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH).ok().map(|d|d.as_secs()),
            background_raw_mt: raw,
            warning: "NO-MAGNET background only; verify same board mounting and Hall settings. This file does not approve CT mapping or enable tracking.".into(),
        };
        value.validate()?;
        Ok(value)
    }
    fn validate(&self) -> Result<(), String> {
        if self.format != "gk_raw_hall_background_v1" || self.units != "mT"
            || self.sensor_order != "raw_firmware_indices_0_to_15"
            || self.raw_board_ids != [0,1,2] || self.sensor_masks != [0xffff;3] {
            return Err("Wrong background format, units, board IDs, or sensor order".into());
        }
        if self.background_raw_mt.iter().flatten().flatten().any(|v|!v.is_finite()) {
            return Err("Background contains nonfinite field samples".into());
        }
        Ok(())
    }
    pub fn to_bytes(&self) -> Result<Vec<u8>, String> {
        self.validate()?;
        serde_json::to_vec_pretty(self).map_err(|e|e.to_string())
    }
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, String> {
        let result: Self=serde_json::from_slice(bytes).map_err(|e|format!("Background JSON invalid: {e}"))?;
        result.validate()?;
        Ok(result)
    }
    pub fn raw(&self) -> [[[f64;3];16];3] { self.background_raw_mt }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn independent_48_sensor_round_trip() {
        let mut fields=[[[0.0;3];16];3];
        fields[2][15]=[1.25,-0.45,0.001];
        let file=GkBackgroundFile::from_raw(fields).unwrap();
        let decoded=GkBackgroundFile::from_bytes(&file.to_bytes().unwrap()).unwrap();
        assert_eq!(decoded.raw(),fields);
    }
    #[test]
    fn reject_mismatched_hardware_and_nonfinite() {
        let mut fields=[[[0.0;3];16];3];
        let mut document:serde_json::Value=serde_json::from_slice(
            &GkBackgroundFile::from_raw(fields).unwrap().to_bytes().unwrap()).unwrap();
        document["raw_board_ids"]=serde_json::json!([2,1,0]);
        assert!(GkBackgroundFile::from_bytes(&serde_json::to_vec(&document).unwrap()).is_err());
        fields[0][7][1]=f64::NAN;
        assert!(GkBackgroundFile::from_raw(fields).is_err());
    }
}
