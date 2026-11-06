//! Real serial port.
//!
//! Line settings follow GMC [GMC] and the spec [SPEC p.6]: 115200 8-N-1,
//! no flow control, DTR and RTS asserted.

use std::io::{self, Read, Write};
use std::time::Duration;

use serialport::{DataBits, FlowControl, Parity, StopBits};

use crate::{Link, Reader, Writer};

pub const DEFAULT_BAUD: u32 = 115_200;

#[derive(Debug, Clone)]
pub struct SerialConfig {
    pub path: String,
    pub baud: u32,
    /// Upper bound for a single `read` call; the controller layer applies its own deadlines.
    pub read_timeout: Duration,
}

impl SerialConfig {
    pub fn new(path: impl Into<String>) -> Self {
        Self {
            path: path.into(),
            baud: DEFAULT_BAUD,
            read_timeout: Duration::from_millis(20),
        }
    }
}

/// Open the port and split it into independent reader and writer halves.
pub fn open(cfg: &SerialConfig) -> serialport::Result<Link> {
    let mut port = serialport::new(&cfg.path, cfg.baud)
        .data_bits(DataBits::Eight)
        .parity(Parity::None)
        .stop_bits(StopBits::One)
        .flow_control(FlowControl::None)
        .timeout(cfg.read_timeout)
        .open()?;
    port.write_data_terminal_ready(true)?;
    port.write_request_to_send(true)?;
    let reader = port.try_clone()?;
    Ok(Link {
        name: cfg.path.clone(),
        reader: Box::new(SerialReader(reader)),
        writer: Box::new(SerialWriter(port)),
    })
}

/// A serial port the OS offers.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PortInfo {
    /// The name to open: `/dev/cu.usbserial-1420`, `/dev/ttyUSB0`, `COM4`, …
    pub name: String,
    /// A USB device, such as a USB-serial adapter.
    pub usb: bool,
}

/// The serial ports this OS offers, as it reports them (macOS lists each device twice, as
/// `/dev/cu.*` and `/dev/tty.*`).
pub fn available() -> serialport::Result<Vec<PortInfo>> {
    Ok(serialport::available_ports()?
        .into_iter()
        .map(|p| PortInfo {
            usb: matches!(p.port_type, serialport::SerialPortType::UsbPort(_)),
            name: p.port_name,
        })
        .collect())
}

struct SerialReader(Box<dyn serialport::SerialPort>);

impl Reader for SerialReader {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self.0.read(buf) {
            Err(e) if e.kind() == io::ErrorKind::TimedOut => Ok(0),
            other => other,
        }
    }
}

struct SerialWriter(Box<dyn serialport::SerialPort>);

impl Writer for SerialWriter {
    fn write_all(&mut self, bytes: &[u8]) -> io::Result<()> {
        self.0.write_all(bytes)?;
        self.0.flush()
    }
}
