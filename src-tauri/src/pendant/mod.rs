//! Opt-in, process-local LAN bridge for the simulation pendant. This module
//! knows no robot solver, filesystem paths, or Tauri APIs. Only validated input
//! reaches its callback; desktop motion arbitration stays in the workspace.
mod listener;
mod smoke;
mod types;
use axum::{
    extract::{
        ws::{Message, WebSocket, WebSocketUpgrade},
        State,
    },
    http::{header, HeaderMap, StatusCode},
    response::{IntoResponse, Response},
    routing::get,
    Router,
};
use futures_util::{SinkExt, StreamExt};
use serde_json::json;
use std::{
    collections::BTreeSet,
    net::Ipv4Addr,
    sync::{
        atomic::{AtomicBool, AtomicU64, Ordering},
        Arc, Mutex,
    },
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};
use tokio::{
    net::TcpListener,
    sync::{watch, Semaphore},
    task::JoinHandle,
};
use types::{valid_input, ClientMessage};
pub use types::{
    Mode, PendantAddress, PendantInput, PendantPose, PendantSession, PendantState, Space,
};

pub type InputCallback = Arc<dyn Fn(PendantInput) + Send + Sync + 'static>;
const INPUT_TIMEOUT: Duration = Duration::from_millis(350);
const DESKTOP_TIMEOUT: Duration = Duration::from_millis(800);
const AUTH_TIMEOUT: Duration = Duration::from_secs(3);
const PAIR_LIFETIME: Duration = Duration::from_secs(10 * 60);
const SESSION_LIFETIME: Duration = Duration::from_secs(8 * 60 * 60);
const WRITE_TIMEOUT: Duration = Duration::from_millis(250);
const MAX_MESSAGE: usize = 2048;

fn now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis()
        .min(u64::MAX as u128) as u64
}
fn random_hex(size: usize) -> Result<String, String> {
    let mut bytes = vec![0; size];
    getrandom::fill(&mut bytes).map_err(|_| "无法生成示教器配对凭据")?;
    Ok(bytes.iter().map(|byte| format!("{byte:02x}")).collect())
}
fn token_matches(a: &str, b: &str) -> bool {
    a.len() == b.len()
        && a.bytes()
            .zip(b.bytes())
            .fold(0u8, |sum, (a, b)| sum | (a ^ b))
            == 0
}

#[derive(Clone)]
struct DesktopSnapshot {
    state: PendantState,
    updated: Option<Instant>,
}
impl DesktopSnapshot {
    fn ready(&self) -> bool {
        self.state.available
            && self
                .updated
                .is_some_and(|time| time.elapsed() <= DESKTOP_TIMEOUT)
    }
    fn presented(&self) -> PendantState {
        if self
            .updated
            .is_some_and(|time| time.elapsed() <= DESKTOP_TIMEOUT)
        {
            return self.state.clone();
        }
        PendantState {
            available: false,
            moving: false,
            reason: Some("电脑端连接已暂停".into()),
            ..self.state.clone()
        }
    }
}
struct Session {
    id: String,
    token: String,
    addresses: Vec<PendantAddress>,
    hosts: BTreeSet<String>,
    port: u16,
    created: Instant,
    expires_at_unix_ms: u64,
    callback: InputCallback,
    connected: AtomicBool,
    running: AtomicBool,
    epoch: AtomicU64,
    last_stop: Mutex<String>,
    desktop: watch::Sender<DesktopSnapshot>,
    shutdown: watch::Sender<bool>,
    task: Mutex<Option<JoinHandle<()>>>,
    connections: Arc<Semaphore>,
}
impl Session {
    fn info(&self) -> PendantSession {
        PendantSession {
            session_id: self.id.clone(),
            port: self.port,
            urls: self.addresses.clone(),
            connected: self.connected.load(Ordering::SeqCst),
            expires_at_unix_ms: self.expires_at_unix_ms,
        }
    }
    fn stop_input(&self, reason: &str) -> u64 {
        *self.last_stop.lock().unwrap() = reason.into();
        let epoch = self.epoch.fetch_add(1, Ordering::SeqCst) + 1;
        (self.callback)(PendantInput {
            session_id: self.id.clone(),
            epoch,
            sequence: 0,
            received_at_ms: now_ms(),
            mode: Mode::Linear,
            space: Space::World,
            axes: [0.; 6],
            speed: 0.,
            active: false,
            reason: Some(reason.into()),
        });
        epoch
    }
}

/// Managed by the desktop application; creating the object opens no listener.
pub struct PendantServer {
    active: Mutex<Option<Arc<Session>>>,
    lifecycle: tokio::sync::Mutex<()>,
}
impl Default for PendantServer {
    fn default() -> Self {
        Self::new()
    }
}
impl PendantServer {
    pub fn new() -> Self {
        Self {
            active: Mutex::new(None),
            lifecycle: tokio::sync::Mutex::new(()),
        }
    }
    pub async fn start(
        self: &Arc<Self>,
        callback: InputCallback,
    ) -> Result<PendantSession, String> {
        let _guard = self.lifecycle.lock().await;
        self.stop_active().await;
        let listener = TcpListener::bind((Ipv4Addr::UNSPECIFIED, 0))
            .await
            .map_err(|_| "无法启动局域网示教器，请检查网络权限")?;
        let port = listener
            .local_addr()
            .map_err(|_| "无法读取示教器端口")?
            .port();
        let token = random_hex(32)?;
        let mut addresses = Vec::new();
        let mut hosts = BTreeSet::new();
        hosts.insert(format!("127.0.0.1:{port}"));
        for interface in if_addrs::get_if_addrs().map_err(|_| "无法读取本机局域网地址")?
        {
            let std::net::IpAddr::V4(ip) = interface.ip() else {
                continue;
            };
            if ip.is_loopback() || ip.is_unspecified() || ip.is_multicast() || ip.is_broadcast() {
                continue;
            }
            if hosts.insert(format!("{ip}:{port}")) {
                addresses.push(PendantAddress {
                    name: format!("{} · {}", interface.name, ip),
                    url: format!("http://{ip}:{port}/#token={token}"),
                });
            }
        }
        if addresses.is_empty() {
            addresses.push(PendantAddress {
                name: "本机回环（需连接局域网后重新配对）".into(),
                url: format!("http://127.0.0.1:{port}/#token={token}"),
            });
        }
        let (desktop, _) = watch::channel(DesktopSnapshot {
            state: PendantState::default(),
            updated: None,
        });
        let (shutdown, _) = watch::channel(false);
        let session = Arc::new(Session {
            id: random_hex(16)?,
            token,
            addresses,
            hosts,
            port,
            created: Instant::now(),
            expires_at_unix_ms: now_ms() + PAIR_LIFETIME.as_millis() as u64,
            callback,
            connected: AtomicBool::new(false),
            running: AtomicBool::new(true),
            epoch: AtomicU64::new(0),
            last_stop: Mutex::new(String::new()),
            desktop,
            shutdown,
            task: Mutex::new(None),
            connections: Arc::new(Semaphore::new(4)),
        });
        let router = Router::new()
            .route("/", get(index))
            .route("/controller.js", get(script))
            .route("/styles.css", get(styles))
            .route("/ws", get(websocket))
            .fallback(not_found)
            .with_state(session.clone());
        let mut shutdown = session.shutdown.subscribe();
        let weak_session = Arc::downgrade(&session);
        let task = tokio::spawn(async move {
            let _ = axum::serve(listener::BoundedListener::new(listener), router)
                .with_graceful_shutdown(async move {
                    let _ = shutdown.changed().await;
                })
                .await;
            if let Some(session) = weak_session.upgrade() {
                if session.running.swap(false, Ordering::SeqCst) {
                    session.stop_input("示教器连接服务已停止");
                }
            }
        });
        *session.task.lock().unwrap() = Some(task);
        *self.active.lock().unwrap() = Some(session.clone());
        Ok(session.info())
    }
    pub fn status(&self) -> Option<PendantSession> {
        self.active
            .lock()
            .unwrap()
            .as_ref()
            .filter(|session| session.running.load(Ordering::SeqCst))
            .map(|session| session.info())
    }
    pub fn publish_state(&self, session_id: &str, state: PendantState) -> Result<(), String> {
        state.validate()?;
        let session = self.active.lock().unwrap().clone().ok_or("示教器未启动")?;
        if session.id != session_id || !session.running.load(Ordering::SeqCst) {
            return Err("示教器会话已失效".into());
        }
        // Revocation is synchronous, before a watch-channel update can be
        // coalesced with a subsequent available state. Even a short busy phase
        // invalidates the controller epoch and any old held input.
        if !state.available && session.desktop.borrow().state.available {
            session.stop_input(state.reason.as_deref().unwrap_or("电脑端已暂停控制"));
        }
        session.desktop.send_replace(DesktopSnapshot {
            state,
            updated: Some(Instant::now()),
        });
        Ok(())
    }
    /// Only expose through a desktop command explicitly gated by --ui-smoke-test.
    pub async fn smoke_drive(&self, mode: Mode) -> Result<(), String> {
        let session = self.active.lock().unwrap().clone().ok_or("示教器未启动")?;
        tokio::time::timeout(Duration::from_secs(5), smoke::drive(session, mode))
            .await
            .map_err(|_| "Installed pendant socket check timed out".to_string())?
    }
    pub async fn stop(&self) {
        let _guard = self.lifecycle.lock().await;
        self.stop_active().await;
    }
    async fn stop_active(&self) {
        let session = self.active.lock().unwrap().take();
        if let Some(session) = session {
            session.running.store(false, Ordering::SeqCst);
            session.stop_input("电脑端已关闭示教器");
            session.shutdown.send_replace(true);
            let task = session.task.lock().unwrap().take();
            if let Some(mut task) = task {
                if tokio::time::timeout(Duration::from_secs(1), &mut task)
                    .await
                    .is_err()
                {
                    task.abort();
                }
            }
        }
    }
}
impl Drop for PendantServer {
    fn drop(&mut self) {
        if let Some(session) = self.active.get_mut().unwrap().take() {
            session.running.store(false, Ordering::SeqCst);
            session.stop_input("示教器已退出");
            session.shutdown.send_replace(true);
            if let Some(task) = session.task.lock().unwrap().take() {
                task.abort();
            }
        }
    }
}

fn valid_host(headers: &HeaderMap, session: &Session) -> bool {
    headers
        .get(header::HOST)
        .and_then(|value| value.to_str().ok())
        .is_some_and(|host| session.hosts.contains(host))
}
fn asset(
    headers: HeaderMap,
    session: &Session,
    content_type: &'static str,
    body: &'static str,
) -> Response {
    if !session.running.load(Ordering::SeqCst) || !valid_host(&headers, session) {
        return StatusCode::FORBIDDEN.into_response();
    }
    let host = headers
        .get(header::HOST)
        .and_then(|value| value.to_str().ok())
        .unwrap();
    // WebKit does not consistently treat ws: as matching connect-src 'self'.
    // Only the already validated, numeric local authority is allowed here.
    let policy = format!("default-src 'none'; script-src 'self'; style-src 'self'; connect-src 'self' ws://{host}; img-src 'self' data:; base-uri 'none'; frame-ancestors 'none'; form-action 'none'");
    let mut response = (
        [
            (header::CONTENT_TYPE, content_type),
            (header::CACHE_CONTROL, "no-store"),
            (header::X_CONTENT_TYPE_OPTIONS, "nosniff"),
            (header::REFERRER_POLICY, "no-referrer"),
        ],
        body,
    )
        .into_response();
    response.headers_mut().insert(
        header::CONTENT_SECURITY_POLICY,
        policy
            .parse()
            .expect("validated numeric host produces a valid policy"),
    );
    response
}
async fn index(State(session): State<Arc<Session>>, headers: HeaderMap) -> Response {
    asset(
        headers,
        &session,
        "text/html; charset=utf-8",
        include_str!("../../../src/pendant/mobile/index.html"),
    )
}
async fn script(State(session): State<Arc<Session>>, headers: HeaderMap) -> Response {
    asset(
        headers,
        &session,
        "text/javascript; charset=utf-8",
        include_str!("../../../src/pendant/mobile/controller.js"),
    )
}
async fn styles(State(session): State<Arc<Session>>, headers: HeaderMap) -> Response {
    asset(
        headers,
        &session,
        "text/css; charset=utf-8",
        include_str!("../../../src/pendant/mobile/styles.css"),
    )
}
async fn not_found() -> StatusCode {
    StatusCode::NOT_FOUND
}
async fn websocket(
    State(session): State<Arc<Session>>,
    headers: HeaderMap,
    ws: WebSocketUpgrade,
) -> Response {
    let origin_ok = headers
        .get(header::ORIGIN)
        .and_then(|value| value.to_str().ok())
        .and_then(|value| value.strip_prefix("http://"))
        .is_some_and(|host| {
            session.hosts.contains(host)
                && headers.get(header::HOST).is_some_and(|value| value == host)
        });
    if !session.running.load(Ordering::SeqCst) || !valid_host(&headers, &session) || !origin_ok {
        return StatusCode::FORBIDDEN.into_response();
    }
    let Ok(permit) = session.connections.clone().try_acquire_owned() else {
        return StatusCode::SERVICE_UNAVAILABLE.into_response();
    };
    ws.max_message_size(MAX_MESSAGE)
        .max_frame_size(MAX_MESSAGE)
        .on_upgrade(move |socket| async move {
            let _permit = permit;
            connection(socket, session).await;
        })
        .into_response()
}
async fn send(socket: &mut WebSocket, value: serde_json::Value) -> bool {
    tokio::time::timeout(
        WRITE_TIMEOUT,
        socket.send(Message::Text(value.to_string().into())),
    )
    .await
    .is_ok_and(|result| result.is_ok())
}
struct ControllerLease(Arc<Session>);
impl Drop for ControllerLease {
    fn drop(&mut self) {
        self.0.stop_input("手机连接已断开");
        self.0.connected.store(false, Ordering::SeqCst);
    }
}
async fn connection(mut socket: WebSocket, session: Arc<Session>) {
    let first = tokio::time::timeout(AUTH_TIMEOUT, socket.next()).await;
    let authenticated = match first {
        Ok(Some(Ok(Message::Text(text)))) => match serde_json::from_str::<ClientMessage>(&text) {
            Ok(ClientMessage::Auth { token }) => {
                session.created.elapsed() <= PAIR_LIFETIME && token_matches(&token, &session.token)
            }
            _ => false,
        },
        _ => false,
    };
    if !authenticated || !session.running.load(Ordering::SeqCst) {
        send(
            &mut socket,
            json!({"type":"error","message":"配对已失效，请重新扫描电脑端二维码"}),
        )
        .await;
        return;
    }
    if session
        .connected
        .compare_exchange(false, true, Ordering::SeqCst, Ordering::SeqCst)
        .is_err()
    {
        send(
            &mut socket,
            json!({"type":"error","message":"已有手机连接，请先在电脑端断开"}),
        )
        .await;
        return;
    }
    let _lease = ControllerLease(session.clone());
    let mut epoch = session.stop_input("手机已连接，请松开摇杆后开始");
    let mut desktop = session.desktop.subscribe();
    let mut shutdown = session.shutdown.subscribe();
    let mut last_sequence = 0;
    let mut armed = false;
    let mut active = false;
    let mut last_mode = Mode::Linear;
    let mut last_space = Space::World;
    let mut last_input = Instant::now();
    let mut rate_start = Instant::now();
    let mut frame_count = 0u32;
    let mut blocked = !desktop.borrow().ready();
    if !send(
        &mut socket,
        json!({"type":"ready","sessionId":session.id,"epoch":epoch}),
    )
    .await
    {
        return;
    }
    let initial_state = desktop.borrow().presented();
    if !send(&mut socket, json!({"type":"state","state":initial_state})).await {
        return;
    }
    let mut tick = tokio::time::interval(Duration::from_millis(50));
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    loop {
        let current_epoch = session.epoch.load(Ordering::SeqCst);
        if current_epoch != epoch {
            epoch = current_epoch;
            armed = false;
            active = false;
            blocked = !desktop.borrow().ready();
            let reason = session.last_stop.lock().unwrap().clone();
            if !send(
                &mut socket,
                json!({"type":"stop","epoch":epoch,"message":reason}),
            )
            .await
            {
                break;
            }
        }
        tokio::select! {
            biased;
            _ = shutdown.changed() => { break; }
            _ = tick.tick() => {
                if !session.running.load(Ordering::SeqCst) || session.created.elapsed() > SESSION_LIFETIME { break; }
                let now_blocked = !desktop.borrow().ready();
                let reason = if now_blocked && !blocked { Some("电脑端已暂停控制") }
                    else if active && last_input.elapsed() > INPUT_TIMEOUT { Some("摇杆信号超时，已停止；请松开后重新操作") } else { None };
                blocked = now_blocked;
                if let Some(reason) = reason {
                    epoch = session.stop_input(reason); armed = false; active = false;
                    if !send(&mut socket, json!({"type":"stop","epoch":epoch,"message":reason})).await { break; }
                    let state = desktop.borrow().presented();
                    if !send(&mut socket, json!({"type":"state","state":state})).await { break; }
                }
            }
            result = desktop.changed() => {
                if result.is_err() { break; }
                let state = desktop.borrow_and_update().presented();
                if !send(&mut socket, json!({"type":"state","state":state})).await { break; }
            }
            frame = socket.next() => {
                let Some(Ok(frame)) = frame else { break; };
                if rate_start.elapsed() >= Duration::from_secs(1) { rate_start = Instant::now(); frame_count = 0; }
                frame_count += 1;
                if frame_count > 60 { send(&mut socket, json!({"type":"error","message":"摇杆发送频率过高，已断开"})).await; break; }
                let Message::Text(text) = frame else { if matches!(frame, Message::Close(_)) { break; } else { continue; } };
                let Ok(ClientMessage::Input { epoch: input_epoch, sequence, mode, space, axes, speed, active: requested_active }) = serde_json::from_str::<ClientMessage>(&text) else {
                    send(&mut socket, json!({"type":"error","message":"无效的示教器消息"})).await; break;
                };
                if !valid_input(input_epoch, sequence, mode, &axes, speed, requested_active) || sequence <= last_sequence {
                    send(&mut socket, json!({"type":"error","message":"示教器消息顺序或数值无效"})).await; break;
                }
                last_sequence = sequence;
                if input_epoch != epoch || epoch != session.epoch.load(Ordering::SeqCst) || !session.running.load(Ordering::SeqCst) { continue; }
                if !requested_active {
                    armed = true; active = false; last_input = Instant::now();
                } else if !armed || !desktop.borrow().ready() { continue; }
                else if active && (mode != last_mode || space != last_space) {
                    epoch = session.stop_input("控制模式已改变，请松开后重新操作"); armed = false; active = false;
                    if !send(&mut socket, json!({"type":"stop","epoch":epoch,"message":"控制模式已改变，请松开后重新操作"})).await { break; }
                    continue;
                } else { active = true; last_input = Instant::now(); }
                last_mode = mode; last_space = space;
                (session.callback)(PendantInput { session_id: session.id.clone(), epoch, sequence, received_at_ms: now_ms(), mode, space, axes, speed, active, reason: None });
            }
        }
    }
    let _ = tokio::time::timeout(WRITE_TIMEOUT, socket.close()).await;
}

#[cfg(test)]
mod tests;
