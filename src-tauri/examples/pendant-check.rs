//! Explicit local integration harness. The pairing capability is written only
//! to a caller-owned temporary file; stdout contains input events, never tokens.
//! Feed {"type":"state","sessionId":"...","state":{...}} lines on stdin.
use emulation_desktop::pendant::{PendantServer, PendantState};
use serde::Deserialize;
use std::{
    io::{BufRead, Write},
    sync::Arc,
};

#[derive(Deserialize)]
#[serde(tag = "type", rename_all = "lowercase", deny_unknown_fields)]
enum Command {
    State {
        #[serde(rename = "sessionId")]
        session_id: String,
        state: PendantState,
    },
    Stop,
}
#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let path = std::path::PathBuf::from(
        args.next()
            .ok_or("usage: pendant-check <new-private-session-file>")?,
    );
    if args.next().is_some() {
        return Err("unexpected arguments".into());
    }
    let server = Arc::new(PendantServer::new());
    let info = server
        .start(Arc::new(|input| {
            let mut output = std::io::stdout().lock();
            let _ = writeln!(
                output,
                "{}",
                serde_json::json!({"type":"input","input":input})
            );
        }))
        .await?;
    let mut options = std::fs::OpenOptions::new();
    options.write(true).create_new(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        options.mode(0o600);
    }
    let mut file = options.open(&path)?;
    file.write_all(serde_json::to_string(&info)?.as_bytes())?;
    println!("{{\"type\":\"ready\"}}");
    let (tx, mut rx) = tokio::sync::mpsc::channel(32);
    std::thread::spawn(move || {
        for line in std::io::stdin().lock().lines() {
            let Ok(line) = line else {
                break;
            };
            if line.len() > 8192 || tx.blocking_send(line).is_err() {
                break;
            }
        }
    });
    while let Some(line) = rx.recv().await {
        match serde_json::from_str::<Command>(&line) {
            Ok(Command::State { session_id, state }) => {
                if server.publish_state(&session_id, state).is_err() {
                    eprintln!("Invalid or stale desktop state");
                }
            }
            Ok(Command::Stop) => break,
            Err(_) => eprintln!("Invalid harness command"),
        }
    }
    server.stop().await;
    std::fs::remove_file(path)?;
    Ok(())
}
