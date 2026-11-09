//! BLE driver for the Żabka Triki motion token (nRF52810 + LSM6DSL IMU).
//!
//! Built on [`btleplug`] and [`tokio`]; all async functions must run inside a tokio
//! runtime with the time driver enabled. macOS (CoreBluetooth) and Linux (BlueZ) are the
//! primary targets; Windows (WinRT) should work through btleplug as well.
//!
//! ```no_run
//! use futures::StreamExt;
//! use triki_ble::{ConnectOptions, Triki};
//!
//! # async fn run() -> Result<(), triki_ble::TrikiError> {
//! let triki = Triki::connect(&ConnectOptions::default()).await?;
//! let mut imu = triki.start_imu(104).await?;
//! while let Some(frame) = imu.next().await {
//!     println!("{:.3}s accel={:?}", frame.t, frame.accel_g);
//! }
//! // The stream ended: the link dropped.
//! # Ok(()) }
//! ```
//!
//! # Platform notes
//!
//! - **macOS** hides MAC addresses: peripheral ids are CoreBluetooth UUIDs, stable per
//!   host but different on every Mac. The process needs Bluetooth permission (System
//!   Settings → Privacy & Security → Bluetooth, for the terminal app running it);
//!   without it [`TrikiError::PermissionDenied`] is returned or the adapter never powers
//!   on ([`TrikiError::AdapterNotReady`]).
//! - **Linux** needs BlueZ running (`bluetoothd`) and access to the system D-Bus; ids are
//!   MAC addresses (`AA:BB:CC:DD:EE:FF`).
//!
//! The pure protocol (UUIDs, commands, frame parsing, scaling) lives in [`protocol`] and
//! is usable without BLE.

pub mod protocol;

use std::collections::VecDeque;
use std::pin::Pin;
use std::sync::Mutex;
use std::task::{Context, Poll};
use std::time::Duration;

use btleplug::api::{
    BDAddr, Central, CentralEvent, CentralState, CharPropFlags, Characteristic, Manager as _,
    Peripheral as _, ScanFilter, ValueNotification, WriteType,
};
use btleplug::platform::{Adapter, Manager, Peripheral, PeripheralId};
use futures::channel::oneshot;
use futures::{FutureExt, Stream, StreamExt};
use tokio::time::{Instant, timeout, timeout_at};

use protocol::{
    BATTERY_LEVEL, DEFAULT_SKIP_FRAMES, DEVICE_NAME_PREFIX, ImuDecoder, NUS_CTRL, NUS_RX, NUS_TX,
};

pub use shariki_core::ImuFrame;

/// Budget for connecting to a found token and discovering its services.
const LINK_TIMEOUT: Duration = Duration::from_secs(15);
/// Budget for a single GATT operation (write, read, subscribe). btleplug may never
/// settle an operation on a link that dropped mid-flight.
const GATT_TIMEOUT: Duration = Duration::from_secs(10);
/// How often the adapter state is re-checked while waiting for it to power on.
const POWER_POLL: Duration = Duration::from_millis(100);

/// Errors from the Triki driver.
#[derive(Debug, thiserror::Error)]
pub enum TrikiError {
    /// No Bluetooth adapter on this host.
    #[error("no Bluetooth adapter found")]
    NoAdapter,
    /// The OS denied Bluetooth access.
    #[error(
        "Bluetooth permission denied (macOS: allow your terminal app in System Settings → \
         Privacy & Security → Bluetooth)"
    )]
    PermissionDenied,
    /// The adapter did not reach the powered-on state in time.
    #[error(
        "Bluetooth adapter is not powered on (state: {0}); is Bluetooth enabled and, on \
         macOS, permitted for this app?"
    )]
    AdapterNotReady(String),
    /// No matching token was seen during the scan.
    #[error("no Triki token matching {target} found within {timeout:?}")]
    NotFound {
        /// What was looked for (name prefix or id).
        target: String,
        /// How long the scan ran.
        timeout: Duration,
    },
    /// A BLE operation did not complete in time.
    #[error("timed out: {0}")]
    Timeout(&'static str),
    /// The requested IMU rate is not one of [`protocol::SUPPORTED_RATES_HZ`].
    #[error("unsupported IMU rate {0} Hz (supported: 26, 52, 104, 208, 416)")]
    UnsupportedRate(u16),
    /// The token lacks a characteristic this operation needs.
    #[error("characteristic {0} not found on the token")]
    MissingCharacteristic(uuid::Uuid),
    /// The link to the token is down.
    #[error("not connected")]
    NotConnected,
    /// The token answered with something unexpected.
    #[error("protocol error: {0}")]
    Protocol(String),
    /// Any other BLE stack error.
    #[error("BLE error: {0}")]
    Ble(btleplug::Error),
}

impl From<btleplug::Error> for TrikiError {
    fn from(e: btleplug::Error) -> Self {
        match e {
            btleplug::Error::PermissionDenied => Self::PermissionDenied,
            btleplug::Error::NoAdapterAvailable => Self::NoAdapter,
            btleplug::Error::NotConnected => Self::NotConnected,
            other => Self::Ble(other),
        }
    }
}

/// How [`Triki::connect`] finds the token.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConnectOptions {
    /// Match devices whose advertised name starts with this prefix (ASCII
    /// case-insensitive). Default `"TRIKI"`. Ignored when [`id`](Self::id) is set.
    pub name_prefix: String,
    /// Connect to this exact device instead of matching by name: a [`Discovered::id`]
    /// (CoreBluetooth UUID on macOS, MAC address on Linux/Windows), case-insensitive.
    pub id: Option<String>,
    /// How long to scan for a matching token. Default 15 s. Connecting and discovering
    /// services afterwards has its own budget.
    pub scan_timeout: Duration,
}

impl Default for ConnectOptions {
    fn default() -> Self {
        Self {
            name_prefix: DEVICE_NAME_PREFIX.to_owned(),
            id: None,
            scan_timeout: Duration::from_secs(15),
        }
    }
}

/// A token seen during [`scan`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Discovered {
    /// Advertised name, e.g. `TRIKI 1234`.
    pub name: String,
    /// Id to pass as [`ConnectOptions::id`]: CoreBluetooth UUID on macOS, MAC address on
    /// Linux/Windows.
    pub id: String,
    /// Last received signal strength, dBm.
    pub rssi: Option<i16>,
}

/// Scan for `timeout` and list every token whose name starts with `TRIKI`
/// (case-insensitive), strongest signal first.
pub async fn scan(timeout: Duration) -> Result<Vec<Discovered>, TrikiError> {
    let adapter = first_adapter().await?;
    wait_powered_on(
        &adapter,
        Instant::now() + timeout.max(Duration::from_secs(5)),
    )
    .await?;
    adapter.start_scan(ScanFilter::default()).await?;
    tokio::time::sleep(timeout).await;
    let peripherals = adapter.peripherals().await;
    let _ = adapter.stop_scan().await;

    let mut found = Vec::new();
    for p in peripherals? {
        let Ok(Some(props)) = p.properties().await else {
            continue;
        };
        let Some(name) = advertised_name(&props) else {
            continue;
        };
        if protocol::name_matches(&name, DEVICE_NAME_PREFIX) {
            found.push(Discovered {
                name,
                id: display_id(&p, props.address),
                rssi: props.rssi,
            });
        }
    }
    found.sort_by_key(|d| std::cmp::Reverse(d.rssi.unwrap_or(i16::MIN)));
    Ok(found)
}

/// A connected Triki token.
///
/// Dropping it does not disconnect the link; call [`Triki::disconnect`].
pub struct Triki {
    adapter: Adapter,
    peripheral: Peripheral,
    name: String,
    id: String,
    rx: Characteristic,
    tx: Characteristic,
    ctrl: Option<Characteristic>,
    battery: Option<Characteristic>,
    /// Close signals of the streams started on this connection.
    closers: Mutex<Vec<oneshot::Sender<()>>>,
}

impl std::fmt::Debug for Triki {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Triki")
            .field("name", &self.name)
            .field("id", &self.id)
            .finish()
    }
}

impl Triki {
    /// Scan for a token matching `opts`, connect, and discover its services.
    pub async fn connect(opts: &ConnectOptions) -> Result<Triki, TrikiError> {
        let adapter = first_adapter().await?;
        let scan_deadline = Instant::now() + opts.scan_timeout;
        wait_powered_on(&adapter, scan_deadline).await?;

        let (peripheral, name) = match timeout_at(scan_deadline, find(&adapter, opts)).await {
            Ok(found) => found,
            Err(_) => {
                let _ = adapter.stop_scan().await;
                let target = match &opts.id {
                    Some(id) => format!("id {id:?}"),
                    None => format!("name prefix {:?}", opts.name_prefix),
                };
                return Err(TrikiError::NotFound {
                    target,
                    timeout: opts.scan_timeout,
                });
            }
        }?;

        match timeout(LINK_TIMEOUT, Self::open(adapter, peripheral.clone(), name)).await {
            Ok(Ok(triki)) => Ok(triki),
            Ok(Err(e)) => {
                let _ = peripheral.disconnect().await;
                Err(e)
            }
            Err(_) => {
                let _ = peripheral.disconnect().await;
                Err(TrikiError::Timeout("connecting to the token"))
            }
        }
    }

    async fn open(
        adapter: Adapter,
        peripheral: Peripheral,
        name: String,
    ) -> Result<Triki, TrikiError> {
        if !peripheral.is_connected().await? {
            peripheral.connect().await?;
        }
        peripheral.discover_services().await?;
        let chars = peripheral.characteristics();
        let find = |uuid| chars.iter().find(|c| c.uuid == uuid).cloned();
        let rx = find(NUS_RX).ok_or(TrikiError::MissingCharacteristic(NUS_RX))?;
        let tx = find(NUS_TX).ok_or(TrikiError::MissingCharacteristic(NUS_TX))?;
        let address = peripheral.address();
        Ok(Triki {
            id: display_id(&peripheral, address),
            ctrl: find(NUS_CTRL),
            battery: find(BATTERY_LEVEL),
            adapter,
            peripheral,
            name,
            rx,
            tx,
            closers: Mutex::new(Vec::new()),
        })
    }

    /// Advertised name, e.g. `TRIKI 1234`.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Device id, usable as [`ConnectOptions::id`] to reconnect to this exact token.
    pub fn id(&self) -> &str {
        &self.id
    }

    /// Whether the link is currently up.
    pub async fn is_connected(&self) -> Result<bool, TrikiError> {
        Ok(self.peripheral.is_connected().await?)
    }

    /// Send START at `rate_hz` and subscribe to TX.
    ///
    /// The stream yields scaled [`ImuFrame`]s with `t`/`index` reconstructed from the
    /// nominal rate; the first [`DEFAULT_SKIP_FRAMES`](protocol::DEFAULT_SKIP_FRAMES)
    /// frames (start-up noise) are dropped. It ends (yields `None`) when the link drops
    /// or [`Triki::disconnect`] is called. `rate_hz` must be one of
    /// [`SUPPORTED_RATES_HZ`](protocol::SUPPORTED_RATES_HZ).
    pub async fn start_imu(&self, rate_hz: u16) -> Result<ImuStream, TrikiError> {
        self.start_imu_with(rate_hz, DEFAULT_SKIP_FRAMES).await
    }

    /// Like [`Triki::start_imu`], dropping `skip_frames` initial frames instead of the
    /// default.
    ///
    /// Calling it again while a stream is running restarts the token's stream at the new
    /// rate; earlier streams keep receiving frames but their timestamps assume the old
    /// rate, so drop them.
    pub async fn start_imu_with(
        &self,
        rate_hz: u16,
        skip_frames: u64,
    ) -> Result<ImuStream, TrikiError> {
        if !protocol::is_supported_rate(rate_hz) {
            return Err(TrikiError::UnsupportedRate(rate_hz));
        }
        // Open both streams before START so no early frame or disconnect is missed.
        let events = self.adapter.events().await?;
        let notifications = self.peripheral.notifications().await?;
        gatt(self.peripheral.subscribe(&self.tx), "subscribing to TX").await?;
        let write_type = if self
            .rx
            .properties
            .contains(CharPropFlags::WRITE_WITHOUT_RESPONSE)
        {
            WriteType::WithoutResponse
        } else {
            WriteType::WithResponse
        };
        gatt(
            self.peripheral
                .write(&self.rx, &protocol::start_cmd(rate_hz), write_type),
            "writing START",
        )
        .await?;

        let (close_tx, close_rx) = oneshot::channel();
        let mut closers = self.closers.lock().unwrap_or_else(|e| e.into_inner());
        closers.retain(|c| !c.is_canceled());
        closers.push(close_tx);

        Ok(ImuStream {
            notifications,
            events,
            close: Some(close_rx),
            peripheral_id: self.peripheral.id(),
            decoder: ImuDecoder::new(rate_hz, skip_frames),
            pending: VecDeque::new(),
            done: false,
        })
    }

    /// Turn the green LED on or off.
    pub async fn set_led(&self, on: bool) -> Result<(), TrikiError> {
        let ctrl = self
            .ctrl
            .as_ref()
            .ok_or(TrikiError::MissingCharacteristic(NUS_CTRL))?;
        gatt(
            self.peripheral
                .write(ctrl, &protocol::led_cmd(on), WriteType::WithResponse),
            "writing LED",
        )
        .await
    }

    /// Read the battery level in percent; `None` if the token has no Battery service.
    pub async fn battery(&self) -> Result<Option<u8>, TrikiError> {
        let Some(batt) = &self.battery else {
            return Ok(None);
        };
        let value = gatt(self.peripheral.read(batt), "reading battery").await?;
        match value.first() {
            Some(&pct) => Ok(Some(pct)),
            None => Err(TrikiError::Protocol("empty battery level".into())),
        }
    }

    /// End all IMU streams of this connection and drop the link. Idempotent.
    pub async fn disconnect(&self) -> Result<(), TrikiError> {
        for closer in self
            .closers
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .drain(..)
        {
            let _ = closer.send(());
        }
        match timeout(GATT_TIMEOUT, self.peripheral.disconnect()).await {
            Ok(Ok(())) | Ok(Err(btleplug::Error::NotConnected)) => Ok(()),
            Ok(Err(e)) => Err(e.into()),
            Err(_) => Err(TrikiError::Timeout("disconnecting")),
        }
    }
}

/// Stream of IMU samples from one [`Triki::start_imu`] call.
///
/// Owns its own frame parser. Ends when the link drops or [`Triki::disconnect`] is
/// called. Frames are buffered by the BLE stack while not polled, but a consumer that
/// stalls for long may lose frames (the stack's queue is bounded); poll it continuously.
pub struct ImuStream {
    notifications: Pin<Box<dyn Stream<Item = ValueNotification> + Send>>,
    events: Pin<Box<dyn Stream<Item = CentralEvent> + Send>>,
    close: Option<oneshot::Receiver<()>>,
    peripheral_id: PeripheralId,
    decoder: ImuDecoder,
    pending: VecDeque<ImuFrame>,
    done: bool,
}

impl std::fmt::Debug for ImuStream {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ImuStream")
            .field("rate_hz", &self.decoder.rate_hz())
            .field("done", &self.done)
            .finish_non_exhaustive()
    }
}

impl ImuStream {
    /// Nominal sample rate of this stream in Hz.
    pub fn rate_hz(&self) -> f64 {
        self.decoder.rate_hz()
    }

    /// Whether the end of the link has been observed (buffered frames may remain).
    fn link_closed(&mut self, cx: &mut Context<'_>) -> bool {
        if let Some(close) = &mut self.close {
            match close.poll_unpin(cx) {
                Poll::Ready(Ok(())) => return true,
                // The Triki was dropped without disconnecting: the link may live on.
                Poll::Ready(Err(oneshot::Canceled)) => self.close = None,
                Poll::Pending => {}
            }
        }
        loop {
            match self.events.poll_next_unpin(cx) {
                Poll::Ready(Some(CentralEvent::DeviceDisconnected(id)))
                    if id == self.peripheral_id =>
                {
                    return true;
                }
                Poll::Ready(Some(_)) => {}
                Poll::Ready(None) => return true,
                Poll::Pending => return false,
            }
        }
    }
}

impl Stream for ImuStream {
    type Item = ImuFrame;

    fn poll_next(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<ImuFrame>> {
        let this = &mut *self;
        loop {
            if let Some(frame) = this.pending.pop_front() {
                return Poll::Ready(Some(frame));
            }
            if this.done || this.link_closed(cx) {
                this.done = true;
                return Poll::Ready(None);
            }
            match this.notifications.poll_next_unpin(cx) {
                Poll::Ready(Some(n)) if n.uuid == NUS_TX => {
                    let pending = &mut this.pending;
                    this.decoder.push_with(&n.value, |f| pending.push_back(f));
                }
                Poll::Ready(Some(_)) => {}
                Poll::Ready(None) => {
                    this.done = true;
                    return Poll::Ready(None);
                }
                Poll::Pending => return Poll::Pending,
            }
        }
    }
}

/// Run a GATT operation with [`GATT_TIMEOUT`].
async fn gatt<T>(
    op: impl Future<Output = btleplug::Result<T>>,
    what: &'static str,
) -> Result<T, TrikiError> {
    match timeout(GATT_TIMEOUT, op).await {
        Ok(r) => Ok(r?),
        Err(_) => Err(TrikiError::Timeout(what)),
    }
}

async fn first_adapter() -> Result<Adapter, TrikiError> {
    let manager = Manager::new().await?;
    manager
        .adapters()
        .await?
        .into_iter()
        .next()
        .ok_or(TrikiError::NoAdapter)
}

/// Wait until the adapter reports powered-on. CoreBluetooth starts in an unknown state
/// and ignores scans issued before it powers on.
async fn wait_powered_on(adapter: &Adapter, deadline: Instant) -> Result<(), TrikiError> {
    loop {
        let state = adapter.adapter_state().await?;
        if state == CentralState::PoweredOn {
            return Ok(());
        }
        if Instant::now() >= deadline {
            return Err(TrikiError::AdapterNotReady(format!("{state:?}")));
        }
        tokio::time::sleep(POWER_POLL).await;
    }
}

/// Scan until a peripheral matches `opts`. Runs until cancelled by the caller's timeout.
async fn find(
    adapter: &Adapter,
    opts: &ConnectOptions,
) -> Result<(Peripheral, String), TrikiError> {
    let mut events = adapter.events().await?;
    adapter.start_scan(ScanFilter::default()).await?;
    // Already-known peripherals produce no new discovery event on some backends.
    for p in adapter.peripherals().await? {
        if let Some(name) = matches(&p, opts).await {
            let _ = adapter.stop_scan().await;
            return Ok((p, name));
        }
    }
    while let Some(event) = events.next().await {
        let (CentralEvent::DeviceDiscovered(id) | CentralEvent::DeviceUpdated(id)) = event else {
            continue;
        };
        let Ok(p) = adapter.peripheral(&id).await else {
            continue;
        };
        if let Some(name) = matches(&p, opts).await {
            // Stop scanning before connecting: BlueZ may reject a connect during a scan.
            let _ = adapter.stop_scan().await;
            return Ok((p, name));
        }
    }
    Err(TrikiError::Protocol(
        "adapter event stream ended during scan".into(),
    ))
}

/// If `p` matches `opts`, its display name.
async fn matches(p: &Peripheral, opts: &ConnectOptions) -> Option<String> {
    let props = p.properties().await.ok().flatten()?;
    let name = advertised_name(&props);
    match &opts.id {
        Some(want) => {
            let hit = p.id().to_string().eq_ignore_ascii_case(want)
                || (props.address != BDAddr::default()
                    && props.address.to_string().eq_ignore_ascii_case(want));
            hit.then(|| name.unwrap_or_else(|| want.clone()))
        }
        None => name.filter(|n| protocol::name_matches(n, &opts.name_prefix)),
    }
}

fn advertised_name(props: &btleplug::api::PeripheralProperties) -> Option<String> {
    props
        .local_name
        .clone()
        .or_else(|| props.advertisement_name.clone())
}

/// User-facing id: the MAC address where the OS exposes one, else the platform id
/// (CoreBluetooth UUID on macOS).
fn display_id(p: &Peripheral, address: BDAddr) -> String {
    if address == BDAddr::default() {
        p.id().to_string()
    } else {
        address.to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn public_types_are_thread_safe() {
        fn stream<T: Stream<Item = ImuFrame> + Send + Unpin>() {}
        fn shared<T: Send + Sync>() {}
        stream::<ImuStream>();
        shared::<Triki>();
        shared::<TrikiError>();
    }

    #[test]
    fn default_options() {
        let o = ConnectOptions::default();
        assert_eq!(o.name_prefix, "TRIKI");
        assert_eq!(o.id, None);
        assert_eq!(o.scan_timeout, Duration::from_secs(15));
    }
}
