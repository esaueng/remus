//! Owned cache scopes (RFC 0009).
//!
//! The kernel's content caches belong to an owner (a kernel session, a native
//! model) instead of to the thread. This layer cannot name them: they are
//! defined in the crates above it. A [`CacheScope`] is a cloneable handle to
//! a set of cache *parts*, one per type. Each layer defines the part it owns
//! and fetches it by type, once per operation, without naming the parts of
//! the layers above it.
//!
//! Context-free entry points reach the scope through the thread's *ambient*
//! slot: [`CacheScope::enter`] installs a scope for the duration of a call,
//! and [`CacheScope::ambient`] reads it. Outside any scope the slot is empty
//! and every cache is off, which is the library default. The slot is a
//! migration bridge for entry points that do not take an
//! [`super::OperationContext`] yet.
//!
//! The ambient slot does not follow work onto other threads: caches are
//! consulted on the thread that owns the operation. A part itself is shared
//! by every clone of its scope, so it must be `Send + Sync`.

use std::any::{Any, TypeId};
use std::cell::RefCell;
use std::sync::{Arc, Mutex, PoisonError};

type Part = Arc<dyn Any + Send + Sync>;

/// A shared handle to one owner's cache parts.
///
/// Clones share the parts. Two scopes are equal when they are clones of one
/// another.
#[derive(Clone, Debug, Default)]
pub struct CacheScope(Arc<Parts>);

#[derive(Debug, Default)]
struct Parts(Mutex<Vec<(TypeId, Part)>>);

impl CacheScope {
    /// A scope holding no parts yet.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// This scope's part of type `T`, created with `T::default()` the first
    /// time it is asked for.
    #[must_use]
    pub fn part<T: Any + Send + Sync + Default>(&self) -> Arc<T> {
        let mut parts = self.0.0.lock().unwrap_or_else(PoisonError::into_inner);
        if let Some(part) = find::<T>(&parts) {
            return part;
        }
        let part = Arc::new(T::default());
        parts.push((TypeId::of::<T>(), part.clone()));
        part
    }

    /// This scope's part of type `T` if it was ever created.
    #[must_use]
    pub fn existing<T: Any + Send + Sync>(&self) -> Option<Arc<T>> {
        find(&self.0.0.lock().unwrap_or_else(PoisonError::into_inner))
    }

    /// The scope installed on this thread, if any.
    #[must_use]
    pub fn ambient() -> Option<Self> {
        AMBIENT.with(|slot| slot.try_borrow().ok().and_then(|scope| scope.clone()))
    }

    /// Install `scope` on this thread until the next [`CacheScope::install`],
    /// returning the scope it replaces. An owner that cannot bracket each
    /// call (a long-lived session) installs its scope once; everything else
    /// uses [`CacheScope::enter`].
    #[must_use = "the replaced scope is dropped; bind it to keep or restore it"]
    pub fn install(scope: Option<Self>) -> Option<Self> {
        AMBIENT.with(|slot| {
            slot.try_borrow_mut()
                .map_or(None, |mut current| std::mem::replace(&mut *current, scope))
        })
    }

    /// Run `f` with this scope installed on the thread, then put back the
    /// scope it replaced, also when `f` unwinds. Scopes nest.
    pub fn enter<R>(&self, f: impl FnOnce() -> R) -> R {
        struct Restore(Option<CacheScope>);
        impl Drop for Restore {
            fn drop(&mut self) {
                drop(CacheScope::install(self.0.take()));
            }
        }
        let _restore = Restore(Self::install(Some(self.clone())));
        f()
    }
}

fn find<T: Any + Send + Sync>(parts: &[(TypeId, Part)]) -> Option<Arc<T>> {
    let id = TypeId::of::<T>();
    let (_, part) = parts.iter().find(|(key, _)| *key == id)?;
    part.clone().downcast().ok()
}

impl PartialEq for CacheScope {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }
}

impl Eq for CacheScope {}

thread_local! {
    static AMBIENT: RefCell<Option<CacheScope>> = const { RefCell::new(None) };
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::panic)]

    use std::sync::atomic::{AtomicUsize, Ordering};

    use super::*;

    #[derive(Debug, Default)]
    struct Counter(AtomicUsize);

    #[derive(Debug, Default)]
    struct Other(AtomicUsize);

    #[test]
    fn parts_are_created_once_per_type() {
        let s = CacheScope::new();
        assert!(s.existing::<Counter>().is_none());
        s.part::<Counter>().0.fetch_add(1, Ordering::Relaxed);
        s.part::<Counter>().0.fetch_add(1, Ordering::Relaxed);
        assert_eq!(
            s.existing::<Counter>().unwrap().0.load(Ordering::Relaxed),
            2
        );
        assert_eq!(s.part::<Other>().0.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn clones_share_parts_and_compare_equal() {
        let a = CacheScope::new();
        let b = a.clone();
        b.part::<Counter>().0.fetch_add(1, Ordering::Relaxed);
        assert_eq!(a.part::<Counter>().0.load(Ordering::Relaxed), 1);
        assert_eq!(a, b);
        let c = CacheScope::new();
        assert_ne!(a, c);
        assert_eq!(c.part::<Counter>().0.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn enter_nests_and_restores() {
        assert_eq!(CacheScope::ambient(), None);
        let outer = CacheScope::new();
        let inner = CacheScope::new();
        outer.enter(|| {
            assert_eq!(CacheScope::ambient().as_ref(), Some(&outer));
            inner.enter(|| assert_eq!(CacheScope::ambient().as_ref(), Some(&inner)));
            assert_eq!(CacheScope::ambient().as_ref(), Some(&outer));
        });
        assert_eq!(CacheScope::ambient(), None);
    }

    #[test]
    fn enter_restores_on_unwind() {
        let s = CacheScope::new();
        let caught = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            s.enter(|| panic!("inside the scope"));
        }));
        assert!(caught.is_err());
        assert_eq!(CacheScope::ambient(), None);
    }

    #[test]
    fn install_replaces_and_returns_the_previous_scope() {
        let a = CacheScope::new();
        assert_eq!(CacheScope::install(Some(a.clone())), None);
        let b = CacheScope::new();
        assert_eq!(CacheScope::install(Some(b.clone())), Some(a));
        assert_eq!(CacheScope::install(None), Some(b));
        assert_eq!(CacheScope::ambient(), None);
    }

    #[test]
    fn the_ambient_scope_does_not_cross_threads_but_parts_do() {
        let s = CacheScope::new();
        let part = s.part::<Counter>();
        s.enter(|| {
            std::thread::spawn(move || {
                assert_eq!(CacheScope::ambient(), None);
                part.0.fetch_add(1, Ordering::Relaxed);
            })
            .join()
            .unwrap();
        });
        assert_eq!(s.part::<Counter>().0.load(Ordering::Relaxed), 1);
    }
}
