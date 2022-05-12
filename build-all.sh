#!/bin/bash

set -euxo pipefail

TGT=thumbv6m-none-eabi

CHIPS="w25q080 gd25q64"

mkdir -p bin/

for chip in $CHIPS; do
    cargo build --release --target $TGT --features chip-${chip}
    cargo run -p bootcrc -- target/$TGT/release/rp2040-rustboot bin/rustboot-${chip}.bin
done
