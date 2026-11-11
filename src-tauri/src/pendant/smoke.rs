//! Installed-client diagnostic: actual local WebSocket inputs travel through
//! authentication, the Tauri event bridge and the workspace's native Jog solver.
use super::{Mode, Session};
use futures_util::{SinkExt, StreamExt};
use serde_json::{json, Value};
use std::{sync::Arc, time::Duration};
use tokio_tungstenite::{
    connect_async,
    tungstenite::{client::IntoClientRequest, Message},
};

fn checked_frame(value: Value) -> Result<Value, String> {
    if matches!(value["type"].as_str(), Some("error" | "stop")) {
        return Err("Installed pendant connection rejected or stopped the test gesture".into());
    }
    Ok(value)
}
pub(super) async fn drive(session: Arc<Session>, mode: Mode) -> Result<(), String> {
    let mut request = format!("ws://127.0.0.1:{}/ws", session.port)
        .into_client_request()
        .map_err(|_| "Invalid pendant test address")?;
    request.headers_mut().insert(
        "Origin",
        format!("http://127.0.0.1:{}", session.port)
            .parse()
            .map_err(|_| "Invalid pendant test origin")?,
    );
    let (mut socket, _) = connect_async(request)
        .await
        .map_err(|_| "Installed pendant WebSocket connection failed")?;
    socket
        .send(Message::Text(
            json!({"type":"auth","token":session.token})
                .to_string()
                .into(),
        ))
        .await
        .map_err(|_| "Installed pendant authentication send failed")?;
    let mut epoch = None;
    let mut available = false;
    while epoch.is_none() || !available {
        let frame = socket
            .next()
            .await
            .ok_or("Installed pendant closed before ready")?
            .map_err(|_| "Installed pendant handshake read failed")?;
        if let Message::Text(text) = frame {
            let value = checked_frame(
                serde_json::from_str(&text)
                    .map_err(|_| "Installed pendant returned invalid JSON")?,
            )?;
            if value["type"] == "ready" {
                epoch = value["epoch"].as_u64();
            }
            if value["type"] == "state" {
                available = value["state"]["available"] == true;
            }
        }
    }
    let epoch = epoch.ok_or("Installed pendant missing controller epoch")?;
    for sequence in 1..=22 {
        let active = sequence > 1 && sequence < 22;
        let packet = json!({"type":"input","epoch":epoch,"sequence":sequence,"mode":mode,"space":"world",
            "axes":[if active {1.0} else {0.0},0,0,0,0,0],"speed":0.1,"active":active});
        socket
            .send(Message::Text(packet.to_string().into()))
            .await
            .map_err(|_| "Installed pendant input send failed")?;
        let delay = tokio::time::sleep(Duration::from_millis(50));
        tokio::pin!(delay);
        loop {
            tokio::select! {
                _ = &mut delay => break,
                frame = socket.next() => {
                    let frame = frame.ok_or("Installed pendant closed during movement")?.map_err(|_| "Installed pendant input read failed")?;
                    if let Message::Text(text) = frame { checked_frame(serde_json::from_str(&text).map_err(|_| "Installed pendant returned invalid JSON")?)?; }
                }
            }
        }
    }
    socket
        .close(None)
        .await
        .map_err(|_| "Installed pendant close failed")?;
    Ok(())
}
