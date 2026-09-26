//! Shared clockwise-predecessor rule from `wire_builder`'s DCEL tracer.

/// Each rotation lists all departures at a vertex counterclockwise. The
/// callback lets the arrangement charge work and poll cancellation.
pub(super) fn successors<E>(
    rotations: &[Vec<usize>],
    twins: &[usize],
    step: &mut impl FnMut() -> Result<(), E>,
) -> Result<Option<Vec<usize>>, E> {
    let mut next = vec![usize::MAX; twins.len()];
    for rotation in rotations {
        if rotation.is_empty() {
            return Ok(None);
        }
        for (position, &outgoing) in rotation.iter().enumerate() {
            step()?;
            let Some(&incoming) = twins.get(outgoing) else {
                return Ok(None);
            };
            if twins.get(incoming) != Some(&outgoing) || next[incoming] != usize::MAX {
                return Ok(None);
            }
            next[incoming] = rotation[(position + rotation.len() - 1) % rotation.len()];
        }
    }
    Ok((!next.contains(&usize::MAX)).then_some(next))
}
