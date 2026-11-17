#!/bin/sh
# Everything a commit must pass (AGENTS.md). Run it from anywhere.
set -eu
cd "$(dirname "$0")/.."

cargo fmt --all --check
cargo clippy --workspace --all-targets -- -D warnings
# Apps run these in the browser (AGENTS.md, "Pure crates stay pure"). Needs, once:
# rustup target add wasm32-unknown-unknown
cargo clippy --target wasm32-unknown-unknown \
  -p protocol -p program -p machine -p route -p nest -p compiler -p calibrate -- -D warnings
cargo test --workspace

echo "check: all passed"
