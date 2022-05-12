#![no_std]
#![no_main]

// asm isn't stable yet on the older toolchain I'm targeting here
#![feature(asm)]

// needed for the cheap reset facade below.
#![feature(naked_functions)]

use panic_halt as _;

///////////////////////////////////////////////////////////////////////////
// Tunables.

const PICO_FLASH_SPI_CLKDIV: u16 = 4;

///////////////////////////////////////////////////////////////////////////
// Flash configuration.

const CMD_WRITE_STATUS: u32 = 0x01;
const CMD_READ_STATUS: u32 = 0x05;
const CMD_WRITE_ENABLE: u32 = 0x06;
const CMD_READ_STATUS2: u32 = 0x35;
const CMD_READ: u32 = 0xeb;
const SREG_DATA: u32 = 0x02;
const MODE_CONTINUOUS_READ: u8 = 0xA0;
const WAIT_CYCLES: u8 = 4;

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
#[link_section = ".Reset"]
#[no_mangle]
#[naked]
pub unsafe extern "C" fn Reset() -> ! {
    asm!(
        "
            push {{lr}}
            bl safe_reset
            pop {{r0}}
            cmp r0, #0
            beq 1f
            bx r0

        1:
            ldr r0, =0x10000100
            ldr r1, =0xE000ED08
            str r0, [r1]
            ldmia r0, {{r0, r1}}
            msr msp, r0
            bx r1
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
    if sreg != SREG_DATA {
        p.XIP_SSI.dr0.write(|w| unsafe { w.bits(CMD_WRITE_ENABLE) });
        wait_ssi_ready(&p.XIP_SSI);
        p.XIP_SSI.dr0.read();

        p.XIP_SSI.dr0.write(|w| unsafe { w.bits(CMD_WRITE_STATUS) });
        p.XIP_SSI.dr0.write(|w| unsafe { w.bits(0) });
        p.XIP_SSI.dr0.write(|w| unsafe { w.bits(SREG_DATA) });

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
