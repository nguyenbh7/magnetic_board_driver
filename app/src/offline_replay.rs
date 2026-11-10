//! Read-only MLXBF002 playback for offline inspection of the actual
//! Live Tracking/Sensor Traces pipeline. Recording bytes are NEVER rewritten.
//! This does not establish that the selected calibration or MLX settings
//! match the historical acquisition, nor is it live optical HDMM data.

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

use data_transfer::rpc::{BoardFrame, BoardPresence, MAX_SENSOR_BOARDS};
use postcard::experimental::max_size::MaxSize;

const MAGIC: &[u8; 8] = b"MLXBF002";
const MAX_FILE_BYTES: u64 = 256 * 1024 * 1024;
const MAX_FRAMES: usize = 250_000;
/// Bounded work per GUI tick; if the recording is denser than the requested
/// playback speed, the next tick catches up rather than starving the UI.
const MAX_FRAMES_PER_TICK: usize = 256;

#[derive(Debug, Clone)]
struct TimedFrame {
    elapsed_us: u64,
    raw: BoardFrame,
}

#[derive(Debug)]
pub struct OfflineRecording {
    path: PathBuf,
    frames: Vec<TimedFrame>,
    presence: BoardPresence,
    duration_us: u64,
    reordered_timestamps: u64,
}

impl OfflineRecording {
    /// Reads the original file without any writable handle or mutation.
    pub fn load(path: &Path) -> Result<Arc<Self>, String> {
        let meta = std::fs::metadata(path)
            .map_err(|err| format!("Cannot stat replay file: {err}"))?;
        if meta.len() > MAX_FILE_BYTES {
            return Err(format!(
                "Replay file is larger than {} MiB; refusing an unbounded memory load",
                MAX_FILE_BYTES / (1024 * 1024)
            ));
        }
        let bytes = std::fs::read(path)
            .map_err(|err| format!("Cannot read replay file: {err}"))?;
        Self::from_bytes(&bytes, path).map(Arc::new)
    }

    fn from_bytes(bytes: &[u8], path: &Path) -> Result<Self, String> {
        if bytes.len() < MAGIC.len() || &bytes[..MAGIC.len()] != MAGIC {
            return Err("Not an MLXBF002 compact board-frame recording".into());
        }
        let mut offset = MAGIC.len();
        let mut frames = Vec::new();
        let mut presence = BoardPresence::default();
        let mut first_time_us = None;
        let mut previous_raw_time_us: Option<u64> = None;
        let mut duration_us = 0_u64;
        let mut reordered_timestamps = 0_u64;

        while offset < bytes.len() {
            if bytes.len() - offset < 4 {
                return Err(format!("Truncated MLXBF002 frame length at byte {offset}"));
            }
            let n = u32::from_le_bytes(bytes[offset..offset + 4]
                .try_into().expect("four frame-length bytes")) as usize;
            offset += 4;
            if n == 0 || n > BoardFrame::POSTCARD_MAX_SIZE || n > bytes.len() - offset {
                return Err(format!("Truncated or invalid MLXBF002 frame at byte {}", offset - 4));
            }
            let raw: BoardFrame = postcard::from_bytes(&bytes[offset..offset + n])
                .map_err(|err| format!("Malformed Postcard board frame at byte {offset}: {err}"))?;
            offset += n;
            if usize::from(raw.board_id) >= MAX_SENSOR_BOARDS || raw.sensor_mask == 0 {
                return Err(format!("Invalid board ID or empty sensor mask at byte {}", offset - n));
            }
            // The shared live sensor conversion adds each sample's timestamp
            // offset; reject an untrusted file that would overflow that sum.
            if raw.samples.iter().enumerate().any(|(i, sample)|
                raw.sensor_mask & (1u16 << i) != 0 &&
                raw.base_time_us.checked_add(u64::from(sample.time_offset_us)).is_none()
            ) {
                return Err(format!("Sample timestamp overflows at byte {}", offset - n));
            }
            let begin = *first_time_us.get_or_insert(raw.base_time_us);
            let elapsed_us = raw.base_time_us.saturating_sub(begin);
            if previous_raw_time_us.is_some_and(|previous| raw.base_time_us < previous) {
                // Retain original frame order and its original timestamp.
                // Only the playback scheduler uses a nondecreasing clock.
                reordered_timestamps += 1;
            }
            previous_raw_time_us = Some(raw.base_time_us);
            duration_us = duration_us.max(elapsed_us);
            presence.board_mask |= 1u8 << raw.board_id;
            presence.sensor_masks[raw.board_id as usize] |= raw.sensor_mask;
            frames.push(TimedFrame {elapsed_us: duration_us, raw});
            if frames.len() > MAX_FRAMES {
                return Err(format!("Recording exceeds the {MAX_FRAMES} frame offline replay limit"));
            }
        }
        if frames.is_empty() {
            return Err("Recording has the MLXBF002 header but no frames".into());
        }
        Ok(Self {
            path: path.to_path_buf(), frames, presence,
            duration_us, reordered_timestamps,
        })
    }

    pub fn presence(&self) -> BoardPresence { self.presence.clone() }
    pub fn path(&self) -> &Path { &self.path }
    pub fn total_frames(&self) -> usize { self.frames.len() }
    pub fn duration_s(&self) -> f64 { self.duration_us as f64 / 1_000_000.0 }
    pub fn reordered_timestamps(&self) -> u64 { self.reordered_timestamps }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ReplaySpeed {
    #[default]
    One,
    Four,
    Sixteen,
}
impl ReplaySpeed {
    pub const ALL: [Self; 3] = [Self::One, Self::Four, Self::Sixteen];
    fn multiplier(self) -> u64 {
        match self { Self::One => 1, Self::Four => 4, Self::Sixteen => 16 }
    }
}
impl std::fmt::Display for ReplaySpeed {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}×", self.multiplier())
    }
}

#[derive(Debug)]
pub struct OfflineReplay {
    pub recording: Arc<OfflineRecording>,
    pub cursor: usize,
    pub playhead_us: u64,
    pub speed: ReplaySpeed,
    pub running: bool,
    last_tick: Option<Instant>,
}
impl OfflineReplay {
    pub fn new(recording: Arc<OfflineRecording>) -> Self {
        Self {recording, cursor: 0, playhead_us: 0, speed: ReplaySpeed::One,
              running: false, last_tick: None}
    }
    pub fn restart(&mut self) {
        self.cursor = 0;
        self.playhead_us = 0;
        self.running = false;
        self.last_tick = None;
    }
    pub fn play(&mut self) {
        if !self.finished() {
            self.running = true;
            self.last_tick = Some(Instant::now());
        }
    }
    pub fn pause(&mut self) {
        self.running = false;
        self.last_tick = None;
    }
    pub fn finished(&self) -> bool { self.cursor >= self.recording.frames.len() }
    pub fn progress(&self) -> String {
        format!(
            "{}/{} frames · {:.2}/{:.2} s · {} · {}",
            self.cursor, self.recording.total_frames(),
            self.playhead_us.min(self.recording.duration_us) as f64 / 1_000_000.0,
            self.recording.duration_s(), self.speed,
            if self.finished() { "COMPLETE" }
            else if self.running { "PLAYING" } else { "PAUSED" }
        )
    }
    pub fn tick(&mut self) -> Vec<BoardFrame> {
        if !self.running { return Vec::new(); }
        let now = Instant::now();
        let elapsed = self.last_tick.replace(now)
            .map(|prev| now.saturating_duration_since(prev))
            .unwrap_or(Duration::ZERO);
        self.advance(elapsed)
    }
    /// Testable playback clock. The record retains its original order and
    /// timestamps; only the wall-time delivery rate is adjusted.
    fn advance(&mut self, elapsed: Duration) -> Vec<BoardFrame> {
        if !self.running { return Vec::new(); }
        let scaled = (elapsed.as_micros() as u128)
            .saturating_mul(self.speed.multiplier() as u128)
            .min(u64::MAX as u128) as u64;
        self.playhead_us = self.playhead_us.saturating_add(scaled);
        let mut out = Vec::new();
        while out.len() < MAX_FRAMES_PER_TICK {
            let Some(entry) = self.recording.frames.get(self.cursor) else { break };
            if entry.elapsed_us > self.playhead_us { break; }
            out.push(entry.raw.clone());
            self.cursor += 1;
        }
        if self.finished() { self.pause(); }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn frame(board: u16, time_us: u64) -> BoardFrame {
        let mut f = BoardFrame::default();
        f.board_id = board;
        f.sensor_mask = 0xffff;
        f.base_time_us = time_us;
        for (i, sample) in f.samples.iter_mut().enumerate() {
            sample.bx_ut = 500.0 + i as f32;
            sample.by_ut = -200.0;
            sample.bz_ut = 4000.0;
        }
        f
    }
    fn encode(frames: &[BoardFrame]) -> Vec<u8> {
        let mut out = MAGIC.to_vec();
        for f in frames {
            let bytes = postcard::to_stdvec(f).unwrap();
            out.extend_from_slice(&(bytes.len() as u32).to_le_bytes());
            out.extend_from_slice(&bytes);
        }
        out
    }
    #[test]
    fn accepts_original_compact_frame_protocol_and_preserves_sensor_data() {
        let frames = [frame(0,1_000_000),frame(1,1_020_000),
                      frame(2,1_030_000),frame(0,1_100_000)];
        let rec = Arc::new(OfflineRecording::from_bytes(&encode(&frames), Path::new("saved.bin")).unwrap());
        assert_eq!(rec.total_frames(), 4);
        assert_eq!(rec.presence.board_mask, 0b111);
        assert_eq!(rec.presence.sensor_masks, [0xffff; 3]);
        assert!((rec.duration_s() - 0.1).abs() < 1e-10);
        let mut play = OfflineReplay::new(rec);
        play.play();
        let first = play.advance(Duration::ZERO);
        assert_eq!(first.len(), 1);
        assert_eq!(first[0], frames[0]);
        play.pause();
        assert!(play.advance(Duration::from_secs(10)).is_empty());
        assert_eq!(play.cursor,1);
        play.play();
        let rest = play.advance(Duration::from_millis(100));
        assert_eq!(rest, frames[1..].to_vec());
        assert!(play.finished());
        play.restart();
        assert_eq!(play.cursor, 0);
        assert_eq!(play.playhead_us, 0);
    }
    #[test]
    fn speed_changes_delivery_timing_not_original_field_values() {
        let frames = [frame(0,10_000),frame(1,1_010_000)];
        let rec=Arc::new(OfflineRecording::from_bytes(&encode(&frames),Path::new("test.bin")).unwrap());
        let mut p=OfflineReplay::new(rec);
        p.speed=ReplaySpeed::Four;
        p.play();
        assert_eq!(p.advance(Duration::ZERO).len(),1);
        assert_eq!(p.advance(Duration::from_millis(249)).len(),0);
        let end=p.advance(Duration::from_millis(1));
        assert_eq!(end.len(),1);
        assert_eq!(end[0].base_time_us,frames[1].base_time_us);
    }
    #[test]
    fn refuses_garbage_truncation_and_does_not_repair_source() {
        let f=frame(0,0);
        let original=encode(&[f]);
        assert!(OfflineRecording::from_bytes(b"MLXBF001", Path::new("invalid")).is_err());
        for n in [0, 1, 7, 8, 9, original.len()-1] {
            assert!(OfflineRecording::from_bytes(&original[..n],Path::new("truncated")).is_err());
        }
        assert!(OfflineRecording::from_bytes(b"MLXBF002",Path::new("empty")).is_err());
    }
    #[test]
    fn rejects_overflowed_sensor_timestamp_and_invalid_board() {
        let mut huge = frame(0, u64::MAX);
        huge.samples[0].time_offset_us = 1;
        assert!(OfflineRecording::from_bytes(&encode(&[huge]), Path::new("bad.bin"))
            .unwrap_err().contains("overflows"));
        let bad_board = frame(9, 100);
        assert!(OfflineRecording::from_bytes(&encode(&[bad_board]), Path::new("bad_board.bin"))
            .unwrap_err().contains("Invalid board"));
    }
    #[test]
    fn retains_original_order_when_device_timestamp_order_is_inconsistent() {
        let data = [frame(0,1_000_000), frame(1,900_000), frame(2,1_050_000)];
        let rec = OfflineRecording::from_bytes(&encode(&data),Path::new("reorder.bin")).unwrap();
        assert_eq!(rec.reordered_timestamps(),1);
        assert_eq!(rec.total_frames(),3);
        assert_eq!(rec.frames[1].raw.base_time_us,900_000);
        assert_eq!(rec.frames[1].elapsed_us,0);
        assert_eq!(rec.frames[2].elapsed_us,50_000);
    }

    #[test]
    fn caps_each_tick_and_keeps_all_frames_for_next_tick() {
        let frames=(0..300).map(|i|frame((i%3) as u16,1000+i as u64)).collect::<Vec<_>>();
        let rec=Arc::new(OfflineRecording::from_bytes(&encode(&frames),Path::new("dense.bin")).unwrap());
        let mut p=OfflineReplay::new(rec);
        p.speed=ReplaySpeed::Sixteen;
        p.play();
        assert_eq!(p.advance(Duration::from_secs(1)).len(),MAX_FRAMES_PER_TICK);
        assert!(!p.finished());
        assert_eq!(p.advance(Duration::ZERO).len(),300-MAX_FRAMES_PER_TICK);
        assert!(p.finished());
    }
}
