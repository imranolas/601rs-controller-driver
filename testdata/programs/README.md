# GMC's jobs as programs

Each file is the cuts GMC made in `testdata/captures/<name>.txt`, as a program
(`PROGRAM_FORMAT.md`) in board millimetres: GMC's paths, overcut included, in GMC's order,
with the material GMC used.

`crates/machine/tests/golden.rs` writes them from the captures (run it with `BLESS=1` to write
them again) and then requires that lowering each one reproduces what GMC sent: every motion
with the same vectors and the same controller settings and outputs. They also serve as known
programs for the runner and the simulator.
