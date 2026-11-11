//! Limit accepted TCP connections as well as authenticated WebSockets. Idle
//! HTTP/upgrade connections cannot retain a slot indefinitely.
use axum::serve::Listener;
use std::{
    future::Future,
    io,
    net::SocketAddr,
    pin::Pin,
    sync::Arc,
    task::{Context, Poll},
    time::Duration,
};
use tokio::{
    io::{AsyncRead, AsyncWrite, ReadBuf},
    net::{TcpListener, TcpStream},
    sync::{OwnedSemaphorePermit, Semaphore},
    time::{Instant, Sleep},
};
const IDLE_TIMEOUT: Duration = Duration::from_secs(5);
pub(super) struct BoundedListener {
    listener: TcpListener,
    capacity: Arc<Semaphore>,
}
impl BoundedListener {
    pub fn new(listener: TcpListener) -> Self {
        Self {
            listener,
            capacity: Arc::new(Semaphore::new(16)),
        }
    }
}
pub(super) struct BoundedStream {
    stream: TcpStream,
    _permit: OwnedSemaphorePermit,
    deadline: Pin<Box<Sleep>>,
}
impl BoundedStream {
    fn check(&mut self, cx: &mut Context<'_>) -> io::Result<()> {
        if self.deadline.as_mut().poll(cx).is_ready() {
            return Err(io::Error::new(
                io::ErrorKind::TimedOut,
                "idle pendant connection",
            ));
        }
        Ok(())
    }
    fn touch(&mut self) {
        self.deadline.as_mut().reset(Instant::now() + IDLE_TIMEOUT);
    }
}
impl AsyncRead for BoundedStream {
    fn poll_read(
        mut self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buffer: &mut ReadBuf<'_>,
    ) -> Poll<io::Result<()>> {
        if let Err(error) = self.check(cx) {
            return Poll::Ready(Err(error));
        }
        let before = buffer.filled().len();
        let result = Pin::new(&mut self.stream).poll_read(cx, buffer);
        if matches!(result, Poll::Ready(Ok(()))) && buffer.filled().len() > before {
            self.touch();
        }
        result
    }
}
impl AsyncWrite for BoundedStream {
    fn poll_write(
        mut self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buffer: &[u8],
    ) -> Poll<io::Result<usize>> {
        if let Err(error) = self.check(cx) {
            return Poll::Ready(Err(error));
        }
        let result = Pin::new(&mut self.stream).poll_write(cx, buffer);
        if matches!(result, Poll::Ready(Ok(count)) if count > 0) {
            self.touch();
        }
        result
    }
    fn poll_flush(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<io::Result<()>> {
        Pin::new(&mut self.stream).poll_flush(cx)
    }
    fn poll_shutdown(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<io::Result<()>> {
        Pin::new(&mut self.stream).poll_shutdown(cx)
    }
}
impl Listener for BoundedListener {
    type Io = BoundedStream;
    type Addr = SocketAddr;
    async fn accept(&mut self) -> (Self::Io, Self::Addr) {
        loop {
            let permit = self
                .capacity
                .clone()
                .acquire_owned()
                .await
                .expect("listener capacity remains open");
            match self.listener.accept().await {
                Ok((stream, address)) => {
                    let _ = stream.set_nodelay(true);
                    return (
                        BoundedStream {
                            stream,
                            _permit: permit,
                            deadline: Box::pin(tokio::time::sleep(IDLE_TIMEOUT)),
                        },
                        address,
                    );
                }
                Err(_) => {
                    drop(permit);
                    tokio::time::sleep(Duration::from_millis(50)).await;
                }
            }
        }
    }
    fn local_addr(&self) -> io::Result<Self::Addr> {
        self.listener.local_addr()
    }
}
