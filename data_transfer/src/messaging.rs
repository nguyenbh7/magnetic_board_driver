#[cfg(feature = "use-defmt")]
use defmt::write;

use postcard::experimental::max_size::MaxSize;

use serde::{Deserialize, Serialize};

use crate::conversions::MagneticField;

#[cfg(feature = "alloc")]
extern crate alloc;

#[cfg(feature = "use-std")]
extern crate std;

#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
#[derive(Debug)]
pub enum Error {
    FailedCRCSerialization,
    FailedCOBSSerialization,
    FailedCRCDeserialization,
    FailedCOBSDeserialization,
    FailedWrite,
    FailedRead,
    DeserializeUnexpectedEnd,
    FailedParse(PostcardError),
}

#[derive(Debug)]
pub struct PostcardError(postcard::Error);

#[cfg(feature = "use-defmt")]
impl defmt::Format for PostcardError {
    fn format(&self, fmt: defmt::Formatter) {
        let Self(err) = self;
        match err {
            postcard::Error::WontImplement => write!(fmt, "Won't implmenet"),
            postcard::Error::NotYetImplemented => write!(fmt, "Not yet implmented"),
            postcard::Error::SerializeBufferFull => write!(fmt, "Serialize buffer full"),
            postcard::Error::SerializeSeqLengthUnknown => {
                write!(fmt, "Serialize Seq length unknown")
            }
            postcard::Error::DeserializeUnexpectedEnd => write!(fmt, "Deserialize unexpected end"),
            postcard::Error::DeserializeBadVarint => write!(fmt, "Deserialize bad varint"),
            postcard::Error::DeserializeBadBool => write!(fmt, "Deserialize bad bool"),
            postcard::Error::DeserializeBadChar => write!(fmt, "Deserialize bad char"),
            postcard::Error::DeserializeBadUtf8 => write!(fmt, "Deserialize bad utf8"),
            postcard::Error::DeserializeBadOption => write!(fmt, "Deserialize bad option"),
            postcard::Error::DeserializeBadEnum => write!(fmt, "Deserialize bad enum"),
            postcard::Error::DeserializeBadEncoding => write!(fmt, "Deserialize bad encoding"),
            postcard::Error::DeserializeBadCrc => write!(fmt, "Deserialize bad crc"),
            postcard::Error::SerdeSerCustom => write!(fmt, "Serde ser custom"),
            postcard::Error::SerdeDeCustom => write!(fmt, "Serde deser custom"),
            postcard::Error::CollectStrError => write!(fmt, "Collect str error"),
            _ => write!(fmt, "Unknown error"),
        }
    }
}

pub fn take_from_slice<'a, T>(buf: &'a mut [u8]) -> (&'a mut [u8], Result<T, postcard::Error>)
where
    T: Deserialize<'a>,
{
    let Some(zero_pos) = buf.iter().position(|&b| b == 0) else {
        return (buf, Err(postcard::Error::DeserializeUnexpectedEnd));
    };

    let (leading, trailing) = buf.split_at_mut(zero_pos + 1);

    (trailing, postcard::from_bytes_cobs(leading))
}

impl From<postcard::Error> for Error {
    fn from(value: postcard::Error) -> Self {
        Self::FailedParse(PostcardError(value))
    }
}
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
#[derive(Debug, Serialize, Deserialize, MaxSize)]
pub struct Message {
    pub field: MagneticField,
    pub position: (f32, f32, f32),
    pub address: u8,
    pub time: u64,
}

impl Message {
    pub fn new(field: MagneticField, position: (f32, f32, f32), address: u8, time: u64) -> Self {
        Self {
            field,
            position,
            address,
            time,
        }
    }

    #[cfg(feature = "use-std")]
    pub fn read<T: std::io::Read>(reader: &mut T) -> Result<Self, Error> {
        let mut buffer = [0; 128];
        let _ = reader.read_exact(&mut buffer);
        let mut slice = buffer.split_mut(|val| *val == 0x00);
        let _ = slice.next();
        let msg = slice.next().ok_or(Error::FailedRead)?;

        let cobs = postcard::take_from_bytes_cobs::<Message>(msg)?;
        Ok(cobs.0)
    }

    #[cfg(feature = "use-std")]
    pub fn read_all<T: std::io::Read>(reader: &mut T) -> std::vec::Vec<Result<Self, Error>> {
        let mut buffer = std::vec![];
        let _ = reader.read_to_end(&mut buffer);
        let slice = buffer.split_inclusive_mut(|val| *val == 0x00);

        let values = slice
            .map(|buf| {
                let cobs = postcard::take_from_bytes_cobs::<Message>(buf)?;
                Ok(cobs)
            })
            .filter(|cobs| cobs.as_ref().is_ok_and(|c| c.1.is_empty()))
            .map(|x| x.map(|y| y.0))
            .collect();

        values
    }

    #[cfg(feature = "use-std")]
    pub fn read_all_buf(buffer: &mut [u8]) -> (std::vec::Vec<Result<Self, Error>>, &mut [u8]) {
        let mut values = std::vec![];
        let mut window = buffer;
        while !window.is_empty() {
            let (remaining, val) = take_from_slice::<Message>(window);
            window = remaining;
            if let Err(postcard::Error::DeserializeUnexpectedEnd) = val {
                break;
            }
            values.push(val.map_err(Error::from));
        }
        (values, window)
    }
}

pub trait Writable {
    async fn write_to<T: embedded_io_async::Write>(&self, writer: &mut T) -> Result<(), Error>;
}

impl Writable for Message {
    async fn write_to<T: embedded_io_async::Write>(&self, writer: &mut T) -> Result<(), Error> {
        //let mut cobs_buffer = heapless::Vec::<u8, { Message::POSTCARD_MAX_SIZE + 1 }>::new();
        //let c = crc::Crc::<u32>::new(&crc::CRC_32_CKSUM);
        //let crc_serialized = postcard::to_slice_crc32(self, &mut crc_buffer, digest)
        //   .map_err(|_| Error::FailedCRCSerialization)?;
        let cobs_serialized = postcard::to_vec_cobs::<_, { Message::POSTCARD_MAX_SIZE + 1 }>(self)
            .map_err(|_| Error::FailedCOBSSerialization)?;
        //let _ = postcard::to_eio(cobs_serialized, writer).map_err(|_| Error::FailedWrite)?;
        writer
            .write_all(&cobs_serialized)
            .await
            .map_err(|_| Error::FailedWrite)?;
        Ok(())
    }
}

#[cfg(feature = "use-std")]
pub struct MessageReader {
    buf: std::vec::Vec<u8>,
}

#[cfg(feature = "use-std")]
impl Default for MessageReader {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(feature = "use-std")]
impl MessageReader {
    pub fn new() -> Self {
        Self {
            buf: std::vec::Vec::new(),
        }
    }

    pub fn read_all<T: std::io::Read>(
        &mut self,
        reader: &mut T,
    ) -> std::vec::Vec<Result<Message, Error>> {
        let mut buffer = std::vec![];
        let _ = reader.read_to_end(&mut buffer);
        self.buf.extend(buffer);
        let (values, remaining) = Message::read_all_buf(&mut self.buf);
        self.buf = remaining.to_vec();
        values
    }
}
