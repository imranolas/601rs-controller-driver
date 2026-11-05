//! Real serial port.
//!
//! Line settings follow GMC [GMC] and the spec [SPEC p.6]: 115200 8-N-1,
//! no flow control, DTR and RTS asserted.

use std::io::{self, Read, Write};
use std::time::Duration;

use serialport::{DataBits, FlowControl, Parity, StopBits};

use crate::Port;

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
        Self { path: path.into(), baud: DEFAULT_BAUD, read_timeout: Duration::from_millis(20) }
    }
}

pub struct SerialPort {
    inner: Box<dyn serialport::SerialPort>,
}

impl std::fmt::Debug for SerialPort {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SerialPort").field("name", &self.inner.name()).finish()
    }
}

impl SerialPort {
    pub fn open(cfg: &SerialConfig) -> serialport::Result<Self> {
        let mut inner = serialport::new(&cfg.path, cfg.baud)
            .data_bits(DataBits::Eight)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .flow_control(FlowControl::None)
            .timeout(cfg.read_timeout)
            .open()?;
        inner.write_data_terminal_ready(true)?;
        inner.write_request_to_send(true)?;
        Ok(Self { inner })
    }

    /// Candidate port names on this OS (e.g. `/dev/cu.usbserial-*`, `COM4`).
    pub fn available() -> Vec<String> {
        serialport::available_ports()
            .map(|ports| ports.into_iter().map(|p| p.port_name).collect())
            .unwrap_or_default()
    }
}

impl Port for SerialPort {
    fn write_all(&mut self, bytes: &[u8]) -> io::Result<()> {
        self.inner.write_all(bytes)?;
        self.inner.flush()
    }

    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self.inner.read(buf) {
            Err(e) if e.kind() == io::ErrorKind::TimedOut => Ok(0),
            other => other,
        }
    }
}
