//! Solid — a volume bounded by closed shells.

use crate::arena;
use crate::shell::ShellId;

/// Typed handle for a [`Solid`] stored in an [`Arena`](crate::Arena).
pub type SolidId = arena::Id<Solid>;

/// A topological solid: a volume bounded by one or more shells.
///
/// The outer shell defines the exterior boundary. Inner shells
/// represent voids (cavities) within the solid.
#[derive(Debug, Clone)]
pub struct Solid {
    /// The outer bounding shell of the solid.
    outer_shell: ShellId,
    /// Inner shells representing voids inside the solid.
    inner_shells: Vec<ShellId>,
}

impl Solid {
    /// Creates a new solid with the given outer shell and optional inner shells.
    #[must_use]
    pub const fn new(outer_shell: ShellId, inner_shells: Vec<ShellId>) -> Self {
        Self {
            outer_shell,
            inner_shells,
        }
    }

    /// Returns the outer bounding shell of this solid.
    #[must_use]
    pub const fn outer_shell(&self) -> ShellId {
        self.outer_shell
    }

    /// Sets the outer bounding shell of this solid.
    pub fn set_outer_shell(&mut self, shell_id: ShellId) {
        self.outer_shell = shell_id;
    }

    /// Returns the inner shells (voids) of this solid.
    #[must_use]
    pub fn inner_shells(&self) -> &[ShellId] {
        &self.inner_shells
    }

    /// Adds an inner shell (void/cavity) to this solid.
    pub fn add_inner_shell(&mut self, shell_id: ShellId) {
        self.inner_shells.push(shell_id);
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use crate::shell::Shell;
    use crate::topology::Topology;

    use super::*;

    /// Three distinct live shell handles.
    fn three_shells() -> (Topology, ShellId, ShellId, ShellId) {
        let mut topo = Topology::new();
        let s0 = topo.add_shell(Shell::empty());
        let s1 = topo.add_shell(Shell::empty());
        let s2 = topo.add_shell(Shell::empty());
        (topo, s0, s1, s2)
    }

    #[test]
    fn set_outer_shell_replaces_the_stored_boundary() {
        let (_topo, s0, s1, _s2) = three_shells();
        let mut solid = Solid::new(s0, vec![]);
        assert_eq!(solid.outer_shell(), s0);

        solid.set_outer_shell(s1);
        assert_eq!(solid.outer_shell(), s1, "the new outer shell is stored");
        assert!(
            solid.inner_shells().is_empty(),
            "replacing the outer shell does not invent cavities"
        );
    }

    #[test]
    fn add_inner_shell_appends_without_disturbing_the_outer_shell() {
        let (_topo, s0, s1, s2) = three_shells();
        let mut solid = Solid::new(s0, vec![]);
        assert!(solid.inner_shells().is_empty());

        solid.add_inner_shell(s1);
        assert_eq!(solid.inner_shells().len(), 1);
        assert_eq!(solid.inner_shells(), &[s1]);

        solid.add_inner_shell(s2);
        assert_eq!(solid.inner_shells().len(), 2, "the cavity count grows");
        assert_eq!(solid.inner_shells(), &[s1, s2]);
        assert_eq!(
            solid.outer_shell(),
            s0,
            "adding a cavity leaves the outer shell untouched"
        );
    }
}
