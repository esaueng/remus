//! CI guard that prevents renderer coverage from silently self-skipping.

#[test]
fn required_wgpu_adapter_is_available() {
    if std::env::var_os("BREPKIT_REQUIRE_WGPU_ADAPTER").is_some() {
        assert!(
            brepkit_render::probe_adapter().is_some(),
            "BREPKIT_REQUIRE_WGPU_ADAPTER is set, but no GPU or software adapter is available"
        );
    }
}
