//! macOS: make this process "responsible for itself" before touching Bluetooth.
//!
//! TCC (the privacy subsystem) attributes Bluetooth access to the *responsible* process —
//! normally the terminal app that launched us. If that app's Info.plist lacks
//! `NSBluetoothAlwaysUsageDescription` (Warp, Terminal.app, IDE terminals…), macOS kills us
//! with `Abort trap: 6` before it ever asks the user. So we re-spawn ourselves with the
//! (undocumented but long-stable) `responsibility_spawnattrs_setdisclaim` flag: the child
//! becomes its own responsible process, TCC reads the Info.plist embedded in this binary by
//! `build.rs`, and the user gets a normal "shariki would like to use Bluetooth" prompt.
//! The parent just waits and forwards the exit status.

use std::ffi::{CString, OsStr};
use std::os::unix::ffi::OsStrExt;
use std::ptr;
use std::sync::atomic::{AtomicI32, Ordering};

use anyhow::{Context, Result, bail};

const MARKER: &str = "SHARIKI_SELF_RESPONSIBLE";

/// Pid of the re-spawned child, for the signal forwarder.
static CHILD: AtomicI32 = AtomicI32::new(0);

extern "C" fn forward(sig: libc::c_int) {
    let pid = CHILD.load(Ordering::SeqCst);
    if pid > 0 {
        // SAFETY: kill(2) is async-signal-safe.
        unsafe { libc::kill(pid, sig) };
    }
}

unsafe extern "C" {
    fn responsibility_spawnattrs_setdisclaim(
        attrs: *mut libc::posix_spawnattr_t,
        disclaim: libc::c_int,
    ) -> libc::c_int;
}

fn cstring(bytes: &[u8]) -> Result<CString> {
    CString::new(bytes).context("argument or environment contains a NUL byte")
}

/// Returns immediately in the re-spawned child; in the original process it spawns the
/// child, waits for it and exits with its status (never returns).
pub fn ensure_self_responsible() -> Result<()> {
    if std::env::var_os(MARKER).is_some() {
        return Ok(());
    }

    let exe = std::env::current_exe().context("cannot locate the shariki executable")?;
    let exe = cstring(exe.as_os_str().as_bytes())?;
    let args = std::env::args_os()
        .map(|a| cstring(a.as_bytes()))
        .collect::<Result<Vec<_>>>()?;
    let mut envs = std::env::vars_os()
        .map(|(k, v)| cstring(&[k.as_bytes(), b"=", v.as_bytes()].concat()))
        .collect::<Result<Vec<_>>>()?;
    envs.push(cstring(&[OsStr::new(MARKER).as_bytes(), b"=1"].concat())?);

    let argv: Vec<*mut libc::c_char> = args
        .iter()
        .map(|a| a.as_ptr() as *mut _)
        .chain(std::iter::once(ptr::null_mut()))
        .collect();
    let envp: Vec<*mut libc::c_char> = envs
        .iter()
        .map(|e| e.as_ptr() as *mut _)
        .chain(std::iter::once(ptr::null_mut()))
        .collect();

    // SAFETY: plain libc calls; every pointer passed outlives the call, argv/envp are
    // NULL-terminated arrays of valid C strings.
    unsafe {
        let mut attr: libc::posix_spawnattr_t = ptr::null_mut();
        if libc::posix_spawnattr_init(&mut attr) != 0 {
            bail!("posix_spawnattr_init failed");
        }
        responsibility_spawnattrs_setdisclaim(&mut attr, 1);
        let mut pid: libc::pid_t = 0;
        let rc = libc::posix_spawn(
            &mut pid,
            exe.as_ptr(),
            ptr::null(),
            &attr,
            argv.as_ptr(),
            envp.as_ptr(),
        );
        libc::posix_spawnattr_destroy(&mut attr);
        if rc != 0 {
            return Err(std::io::Error::from_raw_os_error(rc)).context("cannot re-spawn shariki");
        }

        // Forward termination signals to the child; the parent itself must survive them to
        // collect the child's status. (Ctrl-C from a terminal reaches the child directly as
        // well, since it shares our process group — a duplicate SIGINT is harmless.)
        CHILD.store(pid, Ordering::SeqCst);
        for sig in [libc::SIGINT, libc::SIGTERM, libc::SIGHUP] {
            libc::signal(
                sig,
                forward as extern "C" fn(libc::c_int) as libc::sighandler_t,
            );
        }

        let mut status: libc::c_int = 0;
        loop {
            if libc::waitpid(pid, &mut status, 0) != -1 {
                break;
            }
            if std::io::Error::last_os_error().raw_os_error() != Some(libc::EINTR) {
                bail!("waitpid failed: {}", std::io::Error::last_os_error());
            }
        }
        let code = if libc::WIFEXITED(status) {
            libc::WEXITSTATUS(status)
        } else if libc::WIFSIGNALED(status) {
            128 + libc::WTERMSIG(status)
        } else {
            1
        };
        std::process::exit(code);
    }
}
