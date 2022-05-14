// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! A bootloader for running programs out of QSPI Flash on the RP2040.
//!
//! The RP2040 boot ROM loads the first 256 bytes from an attached flash chip
//! using very conservative settings. It then checks a CRC and, if valid, jumps
//! to the start of the block. The program in those 256 bytes is responsible for
//! reconfiguring the flash chip as desired for higher performance, and then
//! jumping into the _real_ application (which normally starts at offset 256
//! into the flash).
//!
//! The official bootloaders are written in assembly and rely on the C SDK. This
//! one is written in Rust and does not. I've also analyzed the assembly
//! bootloaders (of which there are several, copies of one another) and
//! extracted their differences into Cargo features.

#![no_std]
#![no_main]

// needed for the cheap reset facade below.
#![feature(naked_functions)]

use panic_halt as _;

///////////////////////////////////////////////////////////////////////////
// Tunables.

const PICO_FLASH_SPI_CLKDIV: u16 = 4;

///////////////////////////////////////////////////////////////////////////
// Flash configuration.

cfg_if::cfg_if! {
    if #[cfg(any(feature = "chip-w25q080", feature = "chip-at25sf128a"))] {
        const CMD_READ: u32 = 0xeb;
        const WAIT_CYCLES: u8 = 4;
    } else if #[cfg(feature = "chip-gd25q64")] {
        const CMD_READ: u32 = 0xe7;
        const WAIT_CYCLES: u8 = 2;
    } else {
        compile_error!("must define a chip-* feature");
    }
}

const CMD_WRITE_STATUS: u32 = 0x01;
const CMD_WRITE_STATUS2: u32 = 0x31;
const CMD_READ_STATUS: u32 = 0x05;
const CMD_WRITE_ENABLE: u32 = 0x06;
const CMD_READ_STATUS2: u32 = 0x35;
const STATUS2_VALUE: u32 = 0x02;
const MODE_CONTINUOUS_READ: u8 = 0xA0;

// Number of address + mode bits.
const ADDR_MODE_BIT_COUNT: u8 = 32;
const ADDR_L: u8 = ADDR_MODE_BIT_COUNT / 4;

///////////////////////////////////////////////////////////////////////////
// Code

/// Reset façade. This is the first thing executed after ROM, and it's
/// responsible for forwarding execution into `safe_reset` below and then
/// jumping into the app.
///
/// Jumping into the app merits some additional discussion. The ROM passes us
/// the app information in LR, effectively acting as the return address; we're
/// expected to do one of two things:
///
/// 1. If LR is 0: read the Flash vector table that immediately follows our code
///    and jump into it.
///
/// 2. If LR is not zero: jump where it points instead.
///
/// Either way, that jump will be our final act.
#[link_section = ".reset_handler"]
#[no_mangle]
#[naked]
pub unsafe extern "C" fn reset_handler() -> ! {
    core::arch::asm!(
        "
            mov r4, lr      @ back up return address in caller-save reg
            bl safe_reset   @ do the Rust part below
            cmp r4, #0      @ was the return address zero?
            bne 1f          @ if not, jump ahead to branch to it.

            ldr r0, =0x10000100     @ address of code after bootloader in XIP
            ldr r1, =0xE000ED08     @ address of VTOR in SCS
            str r0, [r1]            @ set vector table to start of code
            ldmia r0!, {{r3, r4}}   @ read initial SP, PC
            mov sp, r3              @ configure initial SP

        1:  bx r4                   @ and jump to initial PC
        ",
        options(noreturn),
    )
}

#[no_mangle]
extern "C" fn safe_reset() {
    // This is like Peripherals::steal, but that function insists on using RAM.
    // This doesn't.
    let p: rp2040_pac::Peripherals = unsafe {
        core::mem::transmute(())
    };

    // Reconfigure our pads to allow faster operation.
    //
    // On SCK, we want more drive current (8mA) and fast slew rate.
    p.PADS_QSPI.gpio_qspi_sclk.write(|w| {
        w.drive()._8m_a()
            .slewfast().set_bit()
    });

    // On the four data lines, we just want to disable the Schmitt trigger to
    // reduce propagation delay on inputs. It seems we trust the QSPI Flash to
    // drive the outputs firmly in one direction or the other.
    p.PADS_QSPI.gpio_qspi_sd0.write(|w| w.schmitt().clear_bit());
    p.PADS_QSPI.gpio_qspi_sd1.write(|w| w.schmitt().clear_bit());
    p.PADS_QSPI.gpio_qspi_sd2.write(|w| w.schmitt().clear_bit());
    p.PADS_QSPI.gpio_qspi_sd3.write(|w| w.schmitt().clear_bit());

    // Disable the SSI as a prerequisite for changing most of its config
    // registers.
    disable_ssi(&p.XIP_SSI);

    // Set the SSI clock divider to our chosen value.
    p.XIP_SSI.baudr.write(|w| unsafe {
        w.sckdv().bits(PICO_FLASH_SPI_CLKDIV)
    });

    // Configure for one SCK cycle delay on RXD sampling. The original
    // bootloader asserts that this is important if PICO_FLASH_SPI_CLKDIV is 2
    // to try and maintain the hold times for RXD; and that it has no effect
    // otherwise. I haven't tested this.
    p.XIP_SSI.rx_sample_dly.write(|w| unsafe { w.rsd().bits(1) });

    // Set the data frame size in 32-bit mode to 8 bits, and configure to both
    // transmit and receive.
    //
    // Note that this implicitly selects 1-SPI mode by setting the FRF field to
    // its reset value.
    p.XIP_SSI.ctrlr0.write(|w| unsafe {
        w.dfs_32().bits(7)
            .tmod().tx_and_rx()
    });
    // Turn the SSI back on so we can use it to poke the Flash chip by reading
    // its status registers.
    enable_ssi(&p.XIP_SSI);
    let sreg = read_flash_sreg(&p.XIP_SSI, CMD_READ_STATUS2);

    // Check that the status register indicates QSPI mode. Reprogram it if not.
    if sreg != STATUS2_VALUE {
        p.XIP_SSI.dr0.write(|w| unsafe { w.bits(CMD_WRITE_ENABLE) });
        wait_ssi_ready(&p.XIP_SSI);
        p.XIP_SSI.dr0.read();

        if cfg!(feature = "supports-write-status2") {
            p.XIP_SSI.dr0.write(|w| unsafe { w.bits(CMD_WRITE_STATUS2) });
        } else {
            // Issue a two-byte write to STATUS, which overwrites more than
            // we'd like, but the W25Q080 (at least) doesn't support
            // single-byte writes to STATUS2 only.
            p.XIP_SSI.dr0.write(|w| unsafe { w.bits(CMD_WRITE_STATUS) });
            // Dummy extra byte
            p.XIP_SSI.dr0.write(|w| unsafe { w.bits(0) });
        }
        // Write the SREG STATUS2 data regardless.
        p.XIP_SSI.dr0.write(|w| unsafe { w.bits(STATUS2_VALUE) });

        wait_ssi_ready(&p.XIP_SSI);
        p.XIP_SSI.dr0.read();
        p.XIP_SSI.dr0.read();
        p.XIP_SSI.dr0.read();

        while read_flash_sreg(&p.XIP_SSI, CMD_READ_STATUS) & 1 != 0 {}
    }

    // Turn SSI back off so we can reconfigure things again.
    disable_ssi(&p.XIP_SSI);

    // Reconfigure for
    // - QSPI mode
    // - 32 bit frames
    // - TX-then-RX mode ("eeprom read")
    p.XIP_SSI.ctrlr0.write(|w| unsafe {
        w.spi_frf().quad()
            .dfs_32().bits(31)
            .tmod().eeprom_read()
    });
    // Configure for zero(?) control frames before we turn the bus around. (I
    // assume this is an undocumented N-1 field and this actually means 1
    // control frame.)
    p.XIP_SSI.ctrlr1.write(|w| unsafe { w.ndf().bits(0) });

    // Configure transaction shape: address/mode bits, wait cycles, instruction
    // length (8 bit), and transaction type (1SPI command followed by QSPI
    // address).
    p.XIP_SSI.spi_ctrlr0.write(|w| unsafe {
        w.addr_l().bits(ADDR_L)
            .wait_cycles().bits(WAIT_CYCLES)
            .inst_l()._8b()
            .trans_type()._1c2a()
    });

    // Aight, turn it back on so we can talk to the Flash.
    enable_ssi(&p.XIP_SSI);

    // Enqueue a READ command followed by the continuous-read mode bits. This
    // lets us issue more reads without issuing a command byte.
    p.XIP_SSI.dr0.write(|w| unsafe { w.bits(CMD_READ) });
    p.XIP_SSI.dr0.write(|w| unsafe { w.bits(MODE_CONTINUOUS_READ as u32) });

    wait_ssi_ready(&p.XIP_SSI);

    // Okay, with the Flash waiting in continuous-read mode, we want to
    // reconfigure the SSI _again._ Last time, I promise.
    disable_ssi(&p.XIP_SSI);

    // Turn on XIP / memory mapping, turn off instructions, and ensure that
    // everything happens in QSPI mode.
    p.XIP_SSI.spi_ctrlr0.write(|w| unsafe {
        w.xip_cmd().bits(MODE_CONTINUOUS_READ)
            .addr_l().bits(ADDR_L)
            .wait_cycles().bits(WAIT_CYCLES)
            .inst_l().none()
            .trans_type()._2c2a()
    });

    // And, back on! We can now address the flash.
    enable_ssi(&p.XIP_SSI);
}

/// Sends `status_read_cmd` and collects a result. This must be issued while
/// we're in "eeprom mode" with CTRLR1.NDF configured correctly.
fn read_flash_sreg(ssi: &rp2040_pac::XIP_SSI, status_read_cmd: u32) -> u32 {
    ssi.dr0.write(|w| unsafe { w.bits(status_read_cmd) });
    // Dummy byte:
    ssi.dr0.write(|w| unsafe { w.bits(status_read_cmd) });

    wait_ssi_ready(ssi);

    // Discard first byte
    ssi.dr0.read();
    ssi.dr0.read().bits()
}

/// Poll SSI SR until (1) Transmit Fifo Empty is set and (2) Busy is not set.
fn wait_ssi_ready(ssi: &rp2040_pac::XIP_SSI) {
    while {
        let sr = ssi.sr.read();
        !sr.tfe().bit() || sr.busy().bit()
    } {}
}

#[inline(always)]
fn disable_ssi(ssi: &rp2040_pac::XIP_SSI) {
    ssi.ssienr.write(|w| w.ssi_en().clear_bit());
}

#[inline(always)]
fn enable_ssi(ssi: &rp2040_pac::XIP_SSI) {
    ssi.ssienr.write(|w| w.ssi_en().set_bit());
}
