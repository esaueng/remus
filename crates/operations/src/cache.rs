//! The thread's default cache scope (RFC 0009).
//!
//! Content caches are parts of a [`CacheScope`] that an owner holds. Until
//! every owner passes its scope explicitly, a thread without one gets a
//! default scope on first use: the thread-wide `*_thread_*` switches act on
//! it, and a kernel session constructed on the thread shares it, so
//! kernels on one thread keep sharing content-keyed hits as before.

use remus_math::context::CacheScope;

/// The scope installed on this thread, creating and installing an empty one
/// when there is none.
#[must_use]
pub fn thread_default_scope() -> CacheScope {
    CacheScope::ambient().unwrap_or_else(|| {
        let scope = CacheScope::new();
        // Nothing was installed, so nothing is replaced.
        drop(CacheScope::install(Some(scope.clone())));
        scope
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_default_scope_is_created_once_per_thread() {
        let first = thread_default_scope();
        assert_eq!(thread_default_scope(), first);
        let other = std::thread::spawn(thread_default_scope).join();
        assert!(other.is_ok_and(|scope| scope != first));
    }

    #[test]
    fn an_entered_scope_is_the_default_while_it_is_entered() {
        let entered = CacheScope::new();
        entered.enter(|| assert_eq!(thread_default_scope(), entered));
    }
}
