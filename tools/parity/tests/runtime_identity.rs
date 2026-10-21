//! The host proof must identify the actual executable without reading a case.

use std::process::{Command, Stdio};

#[test]
fn both_native_runners_identify_their_compiled_host_before_reading_stdin()
-> Result<(), Box<dyn std::error::Error>> {
    for (path, name) in [
        (
            env!("CARGO_BIN_EXE_remus-parity-native"),
            "remus-parity-native",
        ),
        (
            env!("CARGO_BIN_EXE_remus-split-io-native"),
            "remus-split-io-native",
        ),
    ] {
        let output = Command::new(path)
            .arg("--runtime-identity")
            .stdin(Stdio::null())
            .output()?;
        assert!(output.status.success(), "{name}: {output:?}");
        let identity: serde_json::Value = serde_json::from_slice(&output.stdout)?;
        assert_eq!(
            identity,
            serde_json::json!({
                "schemaVersion": 1,
                "binary": name,
                "os": std::env::consts::OS,
                "arch": std::env::consts::ARCH,
                "pointerWidth": usize::BITS,
            })
        );
    }
    Ok(())
}
