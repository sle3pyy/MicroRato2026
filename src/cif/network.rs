use std::net::UdpSocket;
use std::time::Duration;

pub struct NetworkHandler {
    socket: UdpSocket,
}

impl NetworkHandler {
    pub fn new() -> std::io::Result<Self> {
        let socket = UdpSocket::bind("0.0.0.0:0")?;
        socket.set_read_timeout(Some(Duration::from_millis(2000)))?;
        Ok(Self { socket })
    }

    pub fn send_to(&self, data: &[u8], host: &str, port: u16) -> std::io::Result<usize> {
        self.socket.send_to(data, (host, port))
    }

    pub fn send_str(&self, data: &str, host: &str, port: u16) -> std::io::Result<usize> {
        self.send_to(data.as_bytes(), host, port)
    }

    pub fn receive(&self, buf: &mut [u8]) -> std::io::Result<(usize, std::net::SocketAddr)> {
        self.socket.recv_from(buf)
    }
}
