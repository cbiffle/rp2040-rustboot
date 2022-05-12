use clap::Parser;
use std::path::PathBuf;

#[derive(Parser)]
struct Bootcrc {
    input: PathBuf,
    output: PathBuf,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Bootcrc::parse();

    let elf_image = std::fs::read(args.input)?;

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
    if data.len() < 252 {
        data.resize(252_usize, 0_u8);
    } else if data.len() > 252 {
        panic!("bootloader too large");
    }

    let crc = {
        let mut c = crc_any::CRCu32::crc32mpeg2();
        c.digest(&data);
        c.get_crc()
    };

    data.extend(crc.to_le_bytes());

    std::fs::write(args.output, data)?;

    println!("success, crc = 0x{:08x}", crc);

    Ok(())
}
