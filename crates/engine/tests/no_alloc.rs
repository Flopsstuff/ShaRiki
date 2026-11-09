//! Real-time contract: `update` and `render` must not allocate.
//!
//! A counting global allocator records allocations made on the current thread while armed.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use shariki_engine::render::ShakeGesture;

struct Counting;

thread_local! {
    static ARMED: Cell<bool> = const { Cell::new(false) };
    static COUNT: Cell<usize> = const { Cell::new(0) };
}

unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let _ = ARMED.try_with(|a| {
            if a.get() {
                COUNT.with(|c| c.set(c.get() + 1));
            }
        });
        unsafe { System.alloc(layout) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

#[test]
fn update_and_render_do_not_allocate() {
    let g = ShakeGesture::default();
    let states = g.states();
    let mut block = vec![0.0f32; 256];
    for info in shariki_engine::catalog() {
        let mut inst = shariki_engine::create(info.id, 48_000.0).unwrap();
        COUNT.with(|c| c.set(0));
        ARMED.with(|a| a.set(true));
        for chunk in states.chunks(4) {
            for s in chunk {
                inst.update(s);
            }
            inst.render(&mut block);
            inst.render(&mut block);
        }
        ARMED.with(|a| a.set(false));
        assert_eq!(
            COUNT.with(|c| c.get()),
            0,
            "{} allocated on the audio path",
            info.id
        );
    }
}
