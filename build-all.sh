#!/bin/bash

set -euxo pipefail

TGT=thumbv6m-none-eabi

CHIPS="w25q080 gd25q64 at25sf128a"

mkdir -p bin/ elf/

for chip in $CHIPS; do
    cargo build --release --target $TGT --features chip-${chip}
    cp target/$TGT/release/rp2040-rustboot elf/rustboot-${chip}
    cargo run -p bootcrc -- --update elf/rustboot-${chip} bin/rustboot-${chip}.bin
done
