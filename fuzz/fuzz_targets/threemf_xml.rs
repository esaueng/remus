//! 3MF model XML behind a valid archive, then `import_mesh`.
//!
//! `threemf_reader` mutates raw ZIP bytes, so CRC and deflate checks refuse
//! almost every input before the model XML parser runs. This target treats
//! the input as the `3D/3dmodel.model` part and wraps it in a well-formed,
//! Stored (uncompressed) single-entry ZIP, so every input reaches the XML
//! parser, component/build resolution and transform flattening. Accepted
//! meshes then go through the same `import_mesh` step the wasm import uses.

#![no_main]

use libfuzzer_sys::fuzz_target;

mod common;

const MODEL_PATH: &[u8] = b"3D/3dmodel.model";

/// Bitwise CRC-32 (IEEE, reflected): inputs are small, so no table.
fn crc32(data: &[u8]) -> u32 {
    let mut crc = !0u32;
    for &byte in data {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

/// A single-entry Stored ZIP holding `model` at [`MODEL_PATH`].
fn stored_zip(model: &[u8]) -> Option<Vec<u8>> {
    let size = u32::try_from(model.len()).ok()?;
    let crc = crc32(model);
    let name_len = u16::try_from(MODEL_PATH.len()).ok()?;
    // Version 2.0, no flags, method 0 (Stored), DOS date 1980-01-01.
    let common = |out: &mut Vec<u8>| {
        out.extend_from_slice(&20u16.to_le_bytes());
        out.extend_from_slice(&0u16.to_le_bytes());
        out.extend_from_slice(&0u16.to_le_bytes());
        out.extend_from_slice(&0u16.to_le_bytes());
        out.extend_from_slice(&0x0021u16.to_le_bytes());
        out.extend_from_slice(&crc.to_le_bytes());
        out.extend_from_slice(&size.to_le_bytes());
        out.extend_from_slice(&size.to_le_bytes());
        out.extend_from_slice(&name_len.to_le_bytes());
        out.extend_from_slice(&0u16.to_le_bytes());
    };

    let mut zip = Vec::with_capacity(model.len() + 128);
    zip.extend_from_slice(&0x0403_4b50u32.to_le_bytes());
    common(&mut zip);
    zip.extend_from_slice(MODEL_PATH);
    zip.extend_from_slice(model);

    let directory_offset = u32::try_from(zip.len()).ok()?;
    zip.extend_from_slice(&0x0201_4b50u32.to_le_bytes());
    zip.extend_from_slice(&20u16.to_le_bytes()); // version made by
    common(&mut zip);
    zip.extend_from_slice(&0u16.to_le_bytes()); // comment length
    zip.extend_from_slice(&0u16.to_le_bytes()); // disk number start
    zip.extend_from_slice(&0u16.to_le_bytes()); // internal attributes
    zip.extend_from_slice(&0u32.to_le_bytes()); // external attributes
    zip.extend_from_slice(&0u32.to_le_bytes()); // local header offset
    zip.extend_from_slice(MODEL_PATH);
    let directory_size = u32::try_from(zip.len()).ok()? - directory_offset;

    zip.extend_from_slice(&0x0605_4b50u32.to_le_bytes());
    zip.extend_from_slice(&0u16.to_le_bytes()); // this disk
    zip.extend_from_slice(&0u16.to_le_bytes()); // directory disk
    zip.extend_from_slice(&1u16.to_le_bytes()); // entries on this disk
    zip.extend_from_slice(&1u16.to_le_bytes()); // entries in total
    zip.extend_from_slice(&directory_size.to_le_bytes());
    zip.extend_from_slice(&directory_offset.to_le_bytes());
    zip.extend_from_slice(&0u16.to_le_bytes()); // comment length
    Some(zip)
}

fuzz_target!(|data: &[u8]| {
    // The reader decodes the part as UTF-8 text; other bytes never reach XML.
    if std::str::from_utf8(data).is_err() {
        return;
    }
    let Some(archive) = stored_zip(data) else {
        return;
    };
    match remus_io::threemf::read_threemf_with_limits(&archive, common::limits()) {
        Ok(meshes) => common::import_meshes(&meshes),
        Err(remus_io::IoError::Zip(error)) => {
            panic!("the wrapper produced an archive the reader refused: {error}")
        }
        Err(_) => {}
    }
});
