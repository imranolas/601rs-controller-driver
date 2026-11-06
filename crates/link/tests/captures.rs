//! Runs every GMC capture in `testdata/captures/` through the capture reader and the
//! protocol decoder. These are the first replay tests: the decoder must understand every
//! byte the real controller ever sent us.

use std::path::{Path, PathBuf};

use link::Direction;
use link::capture;
use protocol::{Decoder, Frame, Reply};

fn captures() -> Vec<PathBuf> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/captures");
    let mut files: Vec<PathBuf> = std::fs::read_dir(&root)
        .expect("testdata/captures directory")
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| {
            p.file_name()
                .and_then(|n| n.to_str())
                .is_some_and(|n| n.starts_with("g_") && n.ends_with(".txt"))
        })
        .collect();
    files.sort();
    assert!(
        !files.is_empty(),
        "no g_*.txt captures found in {}",
        root.display()
    );
    files
}

#[test]
fn every_capture_parses_and_decodes() {
    for path in captures() {
        let name = path
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_default();
        let transfers = capture::read_file(&path).unwrap_or_else(|e| panic!("{name}: {e}"));
        assert!(!transfers.is_empty(), "{name}: no transfers");

        let mut decoder = Decoder::new();
        let mut lines = 0;
        for t in transfers
            .iter()
            .filter(|t| t.direction == Direction::ControllerToHost)
        {
            for frame in decoder.push(&t.bytes) {
                match frame {
                    Frame::Unknown(b) => panic!("{name}: unknown control byte {b:#04x}"),
                    Frame::Garbled(b) => panic!("{name}: {} bytes without a CR", b.len()),
                    Frame::Line(l) => {
                        lines += 1;
                        // Anything that looks like a status reply must parse as one.
                        if l.starts_with("@X") {
                            assert!(matches!(Reply::parse(&l), Reply::Status(_)), "{name}: {l}");
                        }
                    }
                    Frame::Signal(_) => {}
                }
            }
        }
        assert!(
            decoder.pending().is_empty(),
            "{name}: trailing partial line"
        );
        assert!(lines > 0, "{name}: no reply lines");
    }
}

#[test]
fn init_capture_starts_with_version_query() {
    let path = captures()
        .into_iter()
        .find(|p| p.ends_with("g_init.txt"))
        .expect("g_init.txt");
    let t = capture::read_file(path).expect("parse");
    assert_eq!(t[0].direction, Direction::HostToController);
    assert_eq!(t[0].bytes, b"@V\r");
    assert_eq!(t[1].bytes, b"@V2.00/55W 19.10.10 2807001\r");
}
