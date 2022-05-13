use clap::Parser;
use std::path::PathBuf;
use std::io::{Seek, SeekFrom, Write};

#[derive(Parser)]
struct Bootcrc {
    #[clap(short, long)]
    update: bool,

    input: PathBuf,
    output: PathBuf,
}

const BOGUS_CRC: &[u8] = &[0xDE, 0xAD, 0xBE, 0xEF];

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Bootcrc::parse();

    let elf_image = std::fs::read(&args.input)?;

    let elf = goblin::elf::Elf::parse(&elf_image)?;

    let mut load_phdrs = elf.program_headers.iter()
        .filter(|phdr| phdr.p_type == goblin::elf::program_header::PT_LOAD);

    assert_eq!(load_phdrs.clone().count(), 1, "bootloader image should have one PHDR.");

    let phdr = load_phdrs.next().unwrap();

    let offset = usize::try_from(phdr.p_offset)?;
    let size = usize::try_from(phdr.p_filesz)?;
    let data = &elf_image[offset..offset + size];

    println!("bootloader length before padding: {}", data.len());

    let mut data = data.to_vec();

    let room_for_update = if data.len() == 256 && &data[252..256] == BOGUS_CRC {
        // Shave off the last four bytes.
        data.resize(252, 0);
        true
    } else {
        false
    };

    if data.len() > 252 {
        panic!("bootloader too large");
    } else if data.len() < 252 {
        // Pad with zeros to length.
        data.resize(252_usize, 0_u8);
    }

    let crc = {
        let mut c = crc_any::CRCu32::crc32mpeg2();
        c.digest(&data);
        c.get_crc()
    };

    data.extend(crc.to_le_bytes());

    if args.update {
        if room_for_update {
            println!("updating {}", args.input.display());
            let mut upd = std::fs::OpenOptions::new()
                .write(true)
                .create(false)
                .open(&args.input)?;
            upd.seek(SeekFrom::Start(offset as u64 + 252))?;
            upd.write_all(&data[252..256])?;
            drop(upd);
        } else {
            panic!("can't update ELF file: section too small");
        }
    }

    std::fs::write(args.output, data)?;

    println!("success, crc = 0x{:08x}", crc);

    Ok(())
}
