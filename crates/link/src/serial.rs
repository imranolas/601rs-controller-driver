//! Real serial port.
//!
//! Line settings follow GMC [GMC] and the spec [SPEC p.6]: 115200 8-N-1,
//! no flow control, DTR and RTS asserted.

use std::io::{self, Read, Write};
use std::time::Duration;

use serialport::{DataBits, FlowControl, Parity, StopBits};

use crate::{Link, Paced, Reader, Writer};

pub const DEFAULT_BAUD: u32 = 115_200;

/// How many bytes a write may leave waiting to be sent: ~5.6 ms at 115200 baud, a few
/// vectors, so a stop written next goes out at once.
const LEAD_BYTES: usize = 64;

/// The pace writes are held to, as a share of the line's nominal rate (8-N-1 sends ten bits a
/// byte). A little under the line, so that a slow clock or gaps between bytes never let the
/// bytes the model thinks are sent pile up in the adapter.
const PACE_SHARE: f64 = 0.97;

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
    let builder = serialport::new(&cfg.path, cfg.baud)
        .data_bits(DataBits::Eight)
        .parity(Parity::None)
        .stop_bits(StopBits::One)
        .flow_control(FlowControl::None)
        .timeout(cfg.read_timeout);
    #[cfg(target_os = "macos")]
    let mut port: Box<dyn serialport::SerialPort> = {
        let port = builder.open_native()?;
        set_speed(&port, cfg.baud)?;
        Box::new(port)
    };
    #[cfg(not(target_os = "macos"))]
    let mut port = builder.open()?;
    port.write_data_terminal_ready(true)?;
    port.write_request_to_send(true)?;
    let reader = port.try_clone()?;
    Ok(Link {
        name: cfg.path.clone(),
        reader: Box::new(SerialReader(reader)),
        writer: Box::new(Paced::new(
            SerialWriter(port),
            f64::from(cfg.baud) / 10.0 * PACE_SHARE,
            LEAD_BYTES,
        )),
    })
}

/// Set the speed through termios as well. On macOS serialport writes the termios with 9600 and
/// then sets the speed with `IOSSIOSPEED`, which reads back as set but leaves the line at the
/// old speed on some drivers: with the Prolific PL2303 adapter at the 601 the box heard nothing
/// it could read, while a termios write at 115200 works.
#[cfg(target_os = "macos")]
fn set_speed(port: &serialport::TTYPort, baud: u32) -> io::Result<()> {
    use nix::sys::termios::{SetArg, cfsetspeed, tcgetattr, tcsetattr};
    use std::os::fd::AsRawFd;
    let fd = port.as_raw_fd();
    let mut t = tcgetattr(fd)?;
    cfsetspeed(&mut t, baud)?;
    tcsetattr(fd, SetArg::TCSANOW, &t)?;
    Ok(())
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
    /// `flush` asks the OS to drain (`tcdrain`), yet with the PL2303 adapter at the 601 a stop
    /// written after a burst still crossed the line ~1 s later. [`Paced`] keeps the real pace.
    fn write_all(&mut self, bytes: &[u8]) -> io::Result<()> {
        self.0.write_all(bytes)?;
        self.0.flush()
    }
}
