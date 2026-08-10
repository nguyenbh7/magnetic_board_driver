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

    use data_transfer::rpc::{SensorField, StopFieldStream};
    use numpy::pyo3::{prelude::*, types::PyType};
    use numpy::{
        Ix1, PyArrayLike1,
        ndarray::{Dim, array},
    };
    use numpy::{PyArray, PyArrayMethods, ToPyArray};
    use postcard::experimental::max_size::MaxSize;
    use pyo3::exceptions::PyStopIteration;

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
            reader.read_to_end(&mut bytes);
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
            let mut file = File::open(&path).unwrap();
            let mut reader = BufReader::new(file);

            Self::from_reader(&mut reader)
        }

        fn __getitem__(&self, i: usize) -> PyResult<SensorBoardSample> {
            self.0.get(i).ok_or(PyStopIteration::new_err(())).cloned()
        }

        fn __len__(&self) -> usize {
            self.0.len()
        }
    }
}
