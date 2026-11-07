//! The [`route`] optimiser and the [`nest`]er for the browser: one WebAssembly module, and the
//! formats it speaks. Each solver takes one array of numbers and answers with another
//! ([`route_flat`], [`nest_flat`]), so the page needs no glue beyond copying arrays: it asks
//! for buffers of f64 in the module's memory (`solvers_alloc`), fills the input, calls
//! `route_solve` or `nest_solve`, reads the output and frees both (`solvers_free`). A refusal
//! comes back as a negative [`Error::code`].
//!
//! `route` and `nest` stay plain typed libraries: the formats, their versions and their error
//! codes live here only. The only unsafe code is here too: turning a pointer and a length from
//! JavaScript into a slice.

mod format;
pub mod nest_flat;
pub mod route_flat;

pub use format::Error;

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
/// numbers were written, -1 if the output is too small, or minus the error's code.
///
/// # Safety
/// Both buffers must come from `solvers_alloc` with these lengths, and be distinct.
#[allow(unsafe_code)]
unsafe fn run(
    input: *const f64,
    input_len: usize,
    output: *mut f64,
    output_len: usize,
    solve: impl FnOnce(&[f64]) -> Result<Vec<f64>, Error>,
) -> i32 {
    let input = unsafe { std::slice::from_raw_parts(input, input_len) };
    let output = unsafe { std::slice::from_raw_parts_mut(output, output_len) };
    match solve(input) {
        Ok(v) if v.len() <= output.len() => {
            output[..v.len()].copy_from_slice(&v);
            i32::try_from(v.len()).unwrap_or(-1)
        }
        Ok(_) => -1,
        Err(e) => -e.code(),
    }
}

/// Order a sheet's cuts ([`route_flat`]).
///
/// # Safety
/// Both buffers must come from `solvers_alloc` with these lengths, and be distinct.
#[allow(unsafe_code)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn route_solve(
    input: *const f64,
    input_len: usize,
    output: *mut f64,
    output_len: usize,
) -> i32 {
    unsafe { run(input, input_len, output, output_len, route_flat::solve) }
}

/// Nest mats on sheets or in drop-outs ([`nest_flat`]).
///
/// # Safety
/// Both buffers must come from `solvers_alloc` with these lengths, and be distinct.
#[allow(unsafe_code)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nest_solve(
    input: *const f64,
    input_len: usize,
    output: *mut f64,
    output_len: usize,
) -> i32 {
    unsafe { run(input, input_len, output, output_len, nest_flat::solve) }
}

#[cfg(test)]
mod tests;
