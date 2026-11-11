use super::*;
use futures_util::{SinkExt, StreamExt};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    net::TcpStream,
    sync::mpsc,
};
use tokio_tungstenite::{
    connect_async,
    tungstenite::{client::IntoClientRequest, Message as ClientFrame},
    MaybeTlsStream, WebSocketStream,
};
type Client = WebSocketStream<MaybeTlsStream<TcpStream>>;

fn state() -> PendantState {
    PendantState {
        robot_name: "ABB IRB1100".into(),
        model_id: "irb1100-4-0.58".into(),
        joints: [0., 20., -30., 0., 40., 0.],
        tcp: Some(PendantPose {
            position: [300., 0., 500.],
            orientation: [1., 0., 0., 0.],
        }),
        available: true,
        reason: None,
        moving: false,
    }
}
async fn setup() -> (
    Arc<PendantServer>,
    PendantSession,
    mpsc::UnboundedReceiver<PendantInput>,
) {
    let server = Arc::new(PendantServer::new());
    let (tx, rx) = mpsc::unbounded_channel();
    let info = server
        .start(Arc::new(move |input| {
            let _ = tx.send(input);
        }))
        .await
        .unwrap();
    server.publish_state(&info.session_id, state()).unwrap();
    (server, info, rx)
}
fn token(info: &PendantSession) -> &str {
    info.urls[0].url.split("#token=").nth(1).unwrap()
}
async fn connect(info: &PendantSession) -> Client {
    let mut request = format!("ws://127.0.0.1:{}/ws", info.port)
        .into_client_request()
        .unwrap();
    request.headers_mut().insert(
        "Origin",
        format!("http://127.0.0.1:{}", info.port).parse().unwrap(),
    );
    connect_async(request).await.unwrap().0
}
async fn send_client(client: &mut Client, value: serde_json::Value) {
    client
        .send(ClientFrame::Text(value.to_string().into()))
        .await
        .unwrap();
}
async fn receive(client: &mut Client, wanted: &str) -> serde_json::Value {
    tokio::time::timeout(Duration::from_secs(2), async {
        loop {
            match client.next().await.unwrap().unwrap() {
                ClientFrame::Text(text) => {
                    let value: serde_json::Value = serde_json::from_str(&text).unwrap();
                    if value["type"] == wanted {
                        break value;
                    }
                }
                other => panic!("unexpected frame {other:?}"),
            }
        }
    })
    .await
    .expect("expected protocol message")
}
async fn authenticate(client: &mut Client, info: &PendantSession) -> u64 {
    send_client(client, json!({"type":"auth","token":token(info)})).await;
    let value = receive(client, "ready").await;
    assert_eq!(value["sessionId"], info.session_id);
    value["epoch"].as_u64().unwrap()
}
async fn input(client: &mut Client, epoch: u64, sequence: u64, active: bool) {
    send_client(
        client,
        json!({"type":"input","epoch":epoch,"sequence":sequence,"mode":"linear","space":"world",
        "axes":[if active {0.5} else {0.0},0,0,0,0,0],"speed":0.25,"active":active}),
    )
    .await;
}
async fn event(rx: &mut mpsc::UnboundedReceiver<PendantInput>, active: bool) -> PendantInput {
    tokio::time::timeout(Duration::from_secs(2), async {
        loop {
            let value = rx.recv().await.unwrap();
            if value.active == active {
                break value;
            }
        }
    })
    .await
    .unwrap()
}

#[tokio::test]
async fn opt_in_assets_origin_and_no_file_or_command_routes() {
    let server = Arc::new(PendantServer::new());
    assert!(server.status().is_none());
    let info = server.start(Arc::new(|_| {})).await.unwrap();
    for (path, expected) in [
        ("/", "200 OK"),
        ("/controller.js", "200 OK"),
        ("/styles.css", "200 OK"),
        ("/etc/passwd", "404 Not Found"),
        ("/api/execute", "404 Not Found"),
    ] {
        let mut socket = TcpStream::connect((Ipv4Addr::LOCALHOST, info.port))
            .await
            .unwrap();
        socket
            .write_all(
                format!(
                    "GET {path} HTTP/1.1\r\nHost: 127.0.0.1:{}\r\nConnection: close\r\n\r\n",
                    info.port
                )
                .as_bytes(),
            )
            .await
            .unwrap();
        let mut response = Vec::new();
        socket
            .take(512 * 1024)
            .read_to_end(&mut response)
            .await
            .unwrap();
        let text = String::from_utf8_lossy(&response);
        assert!(text.starts_with(&format!("HTTP/1.1 {expected}")));
        if path == "/" {
            assert!(text.contains("no-store"));
            assert!(text.contains("frame-ancestors 'none'"));
            assert!(!text.contains(token(&info)));
        }
    }
    for (origin, host) in [
        (
            "http://attacker.example".to_string(),
            format!("127.0.0.1:{}", info.port),
        ),
        (
            format!("http://127.0.0.1:{}", info.port),
            "attacker.example".to_string(),
        ),
    ] {
        let mut request = format!("ws://127.0.0.1:{}/ws", info.port)
            .into_client_request()
            .unwrap();
        request
            .headers_mut()
            .insert("Origin", origin.parse().unwrap());
        request.headers_mut().insert("Host", host.parse().unwrap());
        let error = connect_async(request).await.unwrap_err();
        assert!(error.to_string().contains("403"));
    }
    server.stop().await;
    assert!(server.status().is_none());
    assert!(TcpStream::connect((Ipv4Addr::LOCALHOST, info.port))
        .await
        .is_err());
}

#[tokio::test]
async fn pairing_is_exclusive_and_requires_zero_before_motion() {
    let (server, info, mut rx) = setup().await;
    let mut wrong = connect(&info).await;
    send_client(&mut wrong, json!({"type":"auth","token":"wrong"})).await;
    receive(&mut wrong, "error").await;
    assert!(!server.status().unwrap().connected);
    let mut client = connect(&info).await;
    let epoch = authenticate(&mut client, &info).await;
    assert!(server.status().unwrap().connected);
    event(&mut rx, false).await;
    let mut second = connect(&info).await;
    send_client(&mut second, json!({"type":"auth","token":token(&info)})).await;
    receive(&mut second, "error").await;
    input(&mut client, epoch, 1, true).await;
    assert!(tokio::time::timeout(Duration::from_millis(60), rx.recv())
        .await
        .is_err());
    input(&mut client, epoch, 2, false).await;
    let released = event(&mut rx, false).await;
    assert_eq!(released.sequence, 2);
    input(&mut client, epoch, 3, true).await;
    let moving = event(&mut rx, true).await;
    assert_eq!(moving.axes, [0.5, 0., 0., 0., 0., 0.]);
    assert_eq!(moving.epoch, epoch);
    assert_eq!(moving.session_id, info.session_id);
    assert!(now_ms().abs_diff(moving.received_at_ms) < 100);
    input(&mut client, epoch, 4, false).await;
    assert!(!event(&mut rx, false).await.active);
    server.stop().await;
}

#[tokio::test]
async fn deadman_rotates_epoch_and_old_held_packets_cannot_resume() {
    let (server, info, mut rx) = setup().await;
    let mut client = connect(&info).await;
    let epoch = authenticate(&mut client, &info).await;
    event(&mut rx, false).await;
    input(&mut client, epoch, 1, false).await;
    event(&mut rx, false).await;
    input(&mut client, epoch, 2, true).await;
    event(&mut rx, true).await;
    let stopped = receive(&mut client, "stop").await;
    let new_epoch = stopped["epoch"].as_u64().unwrap();
    assert!(new_epoch > epoch);
    assert_eq!(event(&mut rx, false).await.epoch, new_epoch);
    server.publish_state(&info.session_id, state()).unwrap();
    input(&mut client, epoch, 3, true).await;
    input(&mut client, new_epoch, 4, true).await;
    assert!(tokio::time::timeout(Duration::from_millis(60), rx.recv())
        .await
        .is_err());
    input(&mut client, new_epoch, 5, false).await;
    event(&mut rx, false).await;
    input(&mut client, new_epoch, 6, true).await;
    assert_eq!(event(&mut rx, true).await.epoch, new_epoch);
    client.close(None).await.unwrap();
    let disconnected = event(&mut rx, false).await;
    assert!(disconnected.reason.unwrap().contains("断开"));
    server.stop().await;
}

#[tokio::test]
async fn lost_desktop_stops_even_with_fresh_mobile_inputs() {
    let (server, info, mut rx) = setup().await;
    let mut client = connect(&info).await;
    let epoch = authenticate(&mut client, &info).await;
    event(&mut rx, false).await;
    input(&mut client, epoch, 1, false).await;
    event(&mut rx, false).await;
    for sequence in 2..=20 {
        input(&mut client, epoch, sequence, true).await;
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
    let stopped = receive(&mut client, "stop").await;
    assert!(stopped["epoch"].as_u64().unwrap() > epoch);
    let stop = event(&mut rx, false).await;
    assert!(stop.reason.unwrap().contains("电脑"));
    while let Ok(value) = rx.try_recv() {
        assert!(!value.active, "no movement after the stop event");
    }
    server.stop().await;
}

#[tokio::test]
async fn unavailable_state_revokes_held_input_and_replacement_revokes_old_token() {
    let (server, info, mut rx) = setup().await;
    let mut client = connect(&info).await;
    let epoch = authenticate(&mut client, &info).await;
    event(&mut rx, false).await;
    input(&mut client, epoch, 1, false).await;
    event(&mut rx, false).await;
    input(&mut client, epoch, 2, true).await;
    event(&mut rx, true).await;
    server
        .publish_state(
            &info.session_id,
            PendantState {
                available: false,
                reason: Some("程序正在运行".into()),
                ..state()
            },
        )
        .unwrap();
    receive(&mut client, "stop").await;
    event(&mut rx, false).await;
    let next = server.start(Arc::new(|_| {})).await.unwrap();
    assert_ne!(next.session_id, info.session_id);
    assert_ne!(token(&next), token(&info));
    assert!(server.publish_state(&info.session_id, state()).is_err());
    let mut stale = connect(&next).await;
    send_client(&mut stale, json!({"type":"auth","token":token(&info)})).await;
    receive(&mut stale, "error").await;
    server.stop().await;
}

#[tokio::test]
async fn replay_or_malformed_control_disconnects_and_stops() {
    for malformed in [false, true] {
        let (server, info, mut rx) = setup().await;
        let mut client = connect(&info).await;
        let epoch = authenticate(&mut client, &info).await;
        event(&mut rx, false).await;
        input(&mut client, epoch, 1, false).await;
        event(&mut rx, false).await;
        input(&mut client, epoch, 2, true).await;
        event(&mut rx, true).await;
        if malformed {
            send_client(&mut client, json!({"type":"input","epoch":epoch,"sequence":3,"mode":"joint","space":"world","axes":[2,0,0,0,0,0],"speed":1,"active":true})).await;
        } else {
            input(&mut client, epoch, 2, true).await;
        }
        receive(&mut client, "error").await;
        event(&mut rx, false).await;
        server.stop().await;
    }
}

#[test]
fn rejects_nonfinite_state_and_invalid_protocol_vectors() {
    let mut bad = state();
    bad.joints[1] = f64::NAN;
    assert!(bad.validate().is_err());
    let mut bad = state();
    bad.tcp.as_mut().unwrap().orientation = [0.; 4];
    assert!(bad.validate().is_err());
    assert!(!valid_input(
        1,
        1,
        Mode::Linear,
        &[0., 0., 0., 1., 0., 0.],
        1.,
        true
    ));
    assert!(!valid_input(
        1,
        1,
        Mode::Joint,
        &[1., 0., 0., 0., 0., 0.],
        1.,
        false
    ));
    assert!(!valid_input(
        1,
        1,
        Mode::Joint,
        &[0.; 6],
        f64::INFINITY,
        true
    ));
    assert!(!valid_input(1, 0, Mode::Joint, &[0.; 6], 1., false));
    assert!(!token_matches("a", "aa"));
    assert!(!token_matches("ab", "aa"));
    assert!(token_matches("aa", "aa"));
}

#[tokio::test]
async fn brief_desktop_unavailability_cannot_be_coalesced_away() {
    let (server, info, mut rx) = setup().await;
    let mut client = connect(&info).await;
    let epoch = authenticate(&mut client, &info).await;
    event(&mut rx, false).await;
    input(&mut client, epoch, 1, false).await;
    event(&mut rx, false).await;
    input(&mut client, epoch, 2, true).await;
    event(&mut rx, true).await;
    server
        .publish_state(
            &info.session_id,
            PendantState {
                available: false,
                ..state()
            },
        )
        .unwrap();
    server.publish_state(&info.session_id, state()).unwrap();
    let stopped = receive(&mut client, "stop").await;
    let new_epoch = stopped["epoch"].as_u64().unwrap();
    assert!(new_epoch > epoch);
    assert_eq!(event(&mut rx, false).await.epoch, new_epoch);
    input(&mut client, epoch, 3, true).await;
    input(&mut client, new_epoch, 4, true).await;
    assert!(tokio::time::timeout(Duration::from_millis(60), rx.recv())
        .await
        .is_err());
    input(&mut client, new_epoch, 5, false).await;
    event(&mut rx, false).await;
    input(&mut client, new_epoch, 6, true).await;
    assert_eq!(event(&mut rx, true).await.epoch, new_epoch);
    server.stop().await;
}

#[tokio::test]
async fn incomplete_http_connections_are_admitted_with_a_fixed_bound() {
    let (server, info, _) = setup().await;
    let mut held = Vec::new();
    for _ in 0..16 {
        held.push(
            TcpStream::connect((Ipv4Addr::LOCALHOST, info.port))
                .await
                .unwrap(),
        );
    }
    let mut queued = TcpStream::connect((Ipv4Addr::LOCALHOST, info.port))
        .await
        .unwrap();
    queued
        .write_all(
            format!(
                "GET / HTTP/1.1\r\nHost: 127.0.0.1:{}\r\nConnection: close\r\n\r\n",
                info.port
            )
            .as_bytes(),
        )
        .await
        .unwrap();
    let mut response = [0; 64];
    assert!(
        tokio::time::timeout(Duration::from_millis(100), queued.read(&mut response))
            .await
            .is_err()
    );
    drop(held.pop());
    let count = tokio::time::timeout(Duration::from_secs(1), queued.read(&mut response))
        .await
        .unwrap()
        .unwrap();
    assert!(String::from_utf8_lossy(&response[..count]).starts_with("HTTP/1.1 200 OK"));
    drop(held);
    server.stop().await;
}
