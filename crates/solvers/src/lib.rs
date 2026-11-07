//! WebAssembly exports of the [`route`] optimiser and the [`nest`]er: one module for the
//! browser. The page asks for buffers of f64 in the module's memory (`solvers_alloc`), fills
//! the input (each solver's flat encoding: [`route::solve_flat`], [`nest::solve_flat`]), calls
//! `route_solve` or `nest_solve`, reads the output and frees both (`solvers_free`).
//!
//! The only unsafe code is here: turning a pointer and a length from JavaScript into a slice.

/// A zeroed buffer of `len` numbers in the module's memory.
#[allow(unsafe_code)]
#[unsafe(no_mangle)]
pub extern "C" fn solvers_alloc(len: usize) -> *mut f64 {
    Box::into_raw(vec![0.0f64; len].into_boxed_slice()).cast::<f64>()
}

/// Give back a buffer.
///
/// # Safety
/// `ptr` and `len` must come from one `solvers_alloc` call, not yet freed.
#[allow(unsafe_code)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn solvers_free(ptr: *mut f64, len: usize) {
    drop(unsafe { Box::from_raw(std::ptr::slice_from_raw_parts_mut(ptr, len)) });
}

/// Run `solve` on the input buffer and copy its result into the output buffer. Returns how many
/// numbers were written, -1 if the output is too small, or minus the solver's error code.
///
/// # Safety
/// Both buffers must come from `solvers_alloc` with these lengths, and be distinct.
#[allow(unsafe_code)]
unsafe fn run(
    input: *const f64,
    input_len: usize,
    output: *mut f64,
    output_len: usize,
    solve: impl FnOnce(&[f64]) -> Result<Vec<f64>, i32>,
) -> i32 {
    let input = unsafe { std::slice::from_raw_parts(input, input_len) };
    let output = unsafe { std::slice::from_raw_parts_mut(output, output_len) };
    match solve(input) {
        Ok(v) if v.len() <= output.len() => {
            output[..v.len()].copy_from_slice(&v);
            i32::try_from(v.len()).unwrap_or(-1)
        }
        Ok(_) => -1,
        Err(code) => -code,
    }
}

/// Order a sheet's cuts (see [`route::solve_flat`]).
///
/// # Safety
/// Both buffers must come from `solvers_alloc` with these lengths, and be distinct.
#[allow(unsafe_code)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn route_solve(input: *const f64, input_len: usize, output: *mut f64, output_len: usize) -> i32 {
    unsafe { run(input, input_len, output, output_len, |i| route::solve_flat(i).map_err(|e| e.code())) }
}

/// Nest mats on sheets or in drop-outs (see [`nest::solve_flat`]).
///
/// # Safety
/// Both buffers must come from `solvers_alloc` with these lengths, and be distinct.
#[allow(unsafe_code)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nest_solve(input: *const f64, input_len: usize, output: *mut f64, output_len: usize) -> i32 {
    unsafe { run(input, input_len, output, output_len, |i| nest::solve_flat(i).map_err(|e| e.code())) }
}
