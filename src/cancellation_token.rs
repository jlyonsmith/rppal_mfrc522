use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

/// The simplest implementation of an inter-thread cancelation token possible.
#[derive(Clone)]
pub struct CancellationToken {
    canceled: Arc<AtomicBool>,
}

impl CancellationToken {
    /// Create a new cancelation token.  Clone it to pass it to another thread
    pub fn new() -> CancellationToken {
        let canceled = Arc::new(AtomicBool::new(false));

        CancellationToken { canceled }
    }

    /// Flips the state of the token to canceled
    #[inline]
    pub fn cancel(&self) {
        self.canceled.store(true, Ordering::Release);
    }

    /// Checks if the token has been canceled
    #[inline]
    pub fn is_canceled(&self) -> bool {
        self.canceled.load(Ordering::Acquire)
    }
}
