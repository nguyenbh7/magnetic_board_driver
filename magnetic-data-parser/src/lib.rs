use numpy::pyo3::prelude::*;

/// A Python module implemented in Rust.
#[pymodule]
mod magnetic_data_parser {
    use std::{
        fs::File,
        io::{BufReader, Read},
        ops::Deref,
        path::PathBuf,
    };

    use data_transfer::rpc::{BoardFrame, SensorField, SENSOR_POSITIONS_MM};
    use numpy::pyo3::prelude::*;
    use numpy::{array, Ix1, PyArray, ToPyArray};
    use postcard::experimental::max_size::MaxSize;
    use pyo3::exceptions::{PyRuntimeError, PyStopIteration};

    const MLXBF002_MAGIC: &[u8; 8] = b"MLXBF002";

    #[pyclass(sequence)]
    pub struct SensorBoardDataLog(Vec<SensorBoardSample>);

    #[pyclass(from_py_object)]
    #[derive(Debug, Clone)]
    pub struct SensorBoardSample(SensorField);

    #[pymethods]
    impl SensorBoardSample {
        #[getter]
        fn field<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray<f64, Ix1>> {
            let x = self.0.field.x.unwrap().value();
            let y = self.0.field.y.unwrap().value();
            let z = self.0.field.z.unwrap().value();
            array![x, y, z].to_pyarray(py)
        }

        #[getter]
        fn position<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray<f32, Ix1>> {
            let (x, y, z) = self.0.position;
            array![x, y, z].to_pyarray(py)
        }

        #[getter]
        fn time(&self) -> u64 {
            self.0.time
        }

        #[getter]
        fn board_id(&self) -> u16 {
            self.0.board_id
        }

        #[getter]
        fn address(&self) -> u8 {
            self.0.address
        }
    }

    impl SensorBoardDataLog {
        pub fn from_reader(reader: &mut impl Read) -> Self {
            let mut bytes: Vec<u8> = vec![];
            reader.read_to_end(&mut bytes).unwrap();
            let fields = bytes
                .as_chunks::<{ SensorField::POSTCARD_MAX_SIZE }>()
                .0
                .into_iter()
                .map(|x| postcard::from_bytes(x).unwrap())
                .map(SensorBoardSample)
                .collect();

            Self(fields)
        }
    }

    #[pymethods]
    impl SensorBoardDataLog {
        #[staticmethod]
        fn from_path(path: PathBuf) -> Self {
            let file = File::open(&path).unwrap();
            let mut reader = BufReader::new(file);
            Self::from_reader(&mut reader)
        }

        fn __getitem__(&self, i: usize) -> PyResult<SensorBoardSample> {
            self.0
                .get(i)
                .ok_or(PyStopIteration::new_err(()))
                .cloned()
        }

        fn __len__(&self) -> usize {
            self.0.len()
        }
    }

    #[pyclass(sequence)]
    pub struct CompactBoardFrameLog(Vec<CompactSensorSample>);

    #[pyclass(from_py_object)]
    #[derive(Debug, Clone)]
    pub struct CompactSensorSample {
        board_id: u16,
        frame_id: u32,
        sensor_index: u8,
        time: u64,
        bx_ut: f32,
        by_ut: f32,
        bz_ut: f32,
        temperature_c: f32,
        position: (f32, f32, f32),
    }

    #[pymethods]
    impl CompactSensorSample {
        #[getter]
        fn board_id(&self) -> u16 {
            self.board_id
        }

        #[getter]
        fn frame_id(&self) -> u32 {
            self.frame_id
        }

        #[getter]
        fn sensor_index(&self) -> u8 {
            self.sensor_index
        }

        #[getter]
        fn time(&self) -> u64 {
            self.time
        }

        #[getter]
        fn field<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray<f64, Ix1>> {
            array![self.bx_ut as f64, self.by_ut as f64, self.bz_ut as f64].to_pyarray(py)
        }

        #[getter]
        fn temperature_c(&self) -> f32 {
            self.temperature_c
        }

        #[getter]
        fn position<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray<f32, Ix1>> {
            let (x, y, z) = self.position;
            array![x, y, z].to_pyarray(py)
        }
    }

    impl CompactBoardFrameLog {
        fn from_reader(reader: &mut impl Read) -> PyResult<Self> {
            let mut magic = [0u8; 8];
            reader
                .read_exact(&mut magic)
                .map_err(|e| PyRuntimeError::new_err(format!("failed to read MLXBF002 magic: {e}")))?;

            if &magic != MLXBF002_MAGIC {
                return Err(PyRuntimeError::new_err(format!(
                    "unexpected file magic {:?}; expected {:?}",
                    magic, MLXBF002_MAGIC
                )));
            }

            let mut flattened = Vec::new();
            let mut frame_number = 0usize;

            loop {
                let mut len_bytes = [0u8; 4];
                match reader.read_exact(&mut len_bytes) {
                    Ok(()) => {}
                    Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => break,
                    Err(e) => {
                        return Err(PyRuntimeError::new_err(format!(
                            "failed reading frame length at frame {frame_number}: {e}"
                        )));
                    }
                }

                let encoded_len = u32::from_le_bytes(len_bytes) as usize;
                if encoded_len == 0 || encoded_len > BoardFrame::POSTCARD_MAX_SIZE {
                    return Err(PyRuntimeError::new_err(format!(
                        "invalid encoded BoardFrame length {encoded_len} at frame {frame_number}; max is {}",
                        BoardFrame::POSTCARD_MAX_SIZE
                    )));
                }

                let mut encoded = vec![0u8; encoded_len];
                reader.read_exact(&mut encoded).map_err(|e| {
                    PyRuntimeError::new_err(format!(
                        "truncated BoardFrame payload at frame {frame_number}: expected {encoded_len} bytes: {e}"
                    ))
                })?;

                let frame: BoardFrame = postcard::from_bytes(&encoded).map_err(|e| {
                    PyRuntimeError::new_err(format!(
                        "failed to decode BoardFrame {frame_number} ({encoded_len} bytes): {e}"
                    ))
                })?;

                for sensor_index in 0..frame.samples.len() {
                    if frame.sensor_mask & (1u16 << sensor_index) == 0 {
                        continue;
                    }

                    let sample = frame.samples[sensor_index];
                    flattened.push(CompactSensorSample {
                        board_id: frame.board_id,
                        frame_id: frame.frame_id,
                        sensor_index: sensor_index as u8,
                        time: frame.base_time_us + u64::from(sample.time_offset_us),
                        bx_ut: sample.bx_ut,
                        by_ut: sample.by_ut,
                        bz_ut: sample.bz_ut,
                        temperature_c: sample.temperature_c,
                        position: SENSOR_POSITIONS_MM[sensor_index],
                    });
                }

                frame_number += 1;
            }

            Ok(Self(flattened))
        }
    }

    #[pymethods]
    impl CompactBoardFrameLog {
        #[staticmethod]
        fn from_path(path: PathBuf) -> PyResult<Self> {
            let file = File::open(&path).map_err(|e| {
                PyRuntimeError::new_err(format!("failed to open {}: {e}", path.display()))
            })?;
            let mut reader = BufReader::new(file);
            Self::from_reader(&mut reader)
        }

        fn __getitem__(&self, i: usize) -> PyResult<CompactSensorSample> {
            self.0
                .get(i)
                .ok_or(PyStopIteration::new_err(()))
                .cloned()
        }

        fn __len__(&self) -> usize {
            self.0.len()
        }
    }
}
