//! CI guard that prevents renderer coverage from silently self-skipping.

#[test]
fn required_wgpu_adapter_is_available() {
    if std::env::var_os("REMUS_REQUIRE_WGPU_ADAPTER").is_some() {
        assert!(
            remus_render::probe_adapter().is_some(),
            "REMUS_REQUIRE_WGPU_ADAPTER is set, but no GPU or software adapter is available"
        );
    }
}
