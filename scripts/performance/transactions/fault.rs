//! Diagnostic-only failure injection into real boolean commit boundaries.
use std::cell::Cell;
use remus_topology::SolidId;
thread_local! {
    static STAGE: Cell<u8> = const { Cell::new(0) };
    static FAILED: Cell<Option<SolidId>> = const { Cell::new(None) };
}
pub fn set(stage: u8) { STAGE.set(stage); FAILED.set(None); }
pub fn failed() -> Option<SolidId> { FAILED.get() }
pub fn check(stage: u8, solid: SolidId) -> Result<(), crate::OperationsError> {
    if STAGE.get() == stage {
        STAGE.set(0);
        FAILED.set(Some(solid));
        return Err(crate::OperationsError::InvalidInput { reason: format!("PERF-T01 injected at stage {stage}") });
    }
    Ok(())
}
