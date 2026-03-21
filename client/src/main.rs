use std::fs;
use std::io::{self, Write};
use std::sync::mpsc::{self, Receiver};
use std::time::Duration;

use log::{error, info, warn};

const CMD_IDENTIFY: u8  = 0x01;
const CMD_SET_ID: u8    = 0x02;
const CMD_DISCOVER: u8  = 0x04;
const CMD_OTA_BEGIN: u8 = 0x05;
const CMD_OTA_DATA: u8  = 0x06;
const CMD_OTA_END: u8   = 0x07;

const MSG_RESPONSE: u8  = 0x10;
const MSG_DISCOVERY: u8 = 0x11;
const MSG_OTA_ACK: u8   = 0x12;

const OTA_UART_CHUNK: usize = 128;

#[derive(Debug, Clone)]
struct DeviceInfo {
    can_id: u8,
    module_type: u8,
    fw_version: u8,
}

#[derive(Debug)]
enum RxEvent {
    Response { msg_type: u8, source_device: u8, data: Vec<u8> },
    Raw(Vec<u8>),
}

fn spawn_reader(mut port: Box<dyn serialport::SerialPort>, tx: mpsc::Sender<RxEvent>) {
    std::thread::spawn(move || {
        let mut buf = [0u8; 256];
        loop {
            match port.read(&mut buf) {
                Ok(0) => {}
                Ok(n) => {
                    let bytes = &buf[..n];
                    let msg_type = bytes[0];
                    if n >= 2 && matches!(msg_type, m if m == MSG_RESPONSE || m == MSG_DISCOVERY || m == MSG_OTA_ACK) {
                        let _ = tx.send(RxEvent::Response {
                            msg_type,
                            source_device: bytes[1],
                            data: bytes[2..].to_vec(),
                        });
                    } else {
                        let _ = tx.send(RxEvent::Raw(bytes.to_vec()));
                    }
                }
                Err(ref e) if e.kind() == io::ErrorKind::TimedOut => {}
                Err(e) => { error!("[reader] {}", e); break; }
            }
        }
    });
}

struct Client {
    port: Box<dyn serialport::SerialPort>,
    rx: Receiver<RxEvent>,
}

impl Client {
    fn new(port_name: &str, baud_rate: u32) -> Result<Self, Box<dyn std::error::Error>> {
        let port = serialport::new(port_name, baud_rate)
            .timeout(Duration::from_millis(50))
            .data_bits(serialport::DataBits::Eight)
            .parity(serialport::Parity::None)
            .stop_bits(serialport::StopBits::One)
            .flow_control(serialport::FlowControl::None)
            .open()?;

        let read_port = port.try_clone()?;
        let (tx, rx) = mpsc::channel();
        spawn_reader(read_port, tx);

        Ok(Client { port, rx })
    }

    fn send(&mut self, target: u8, command: u8, payload: &[u8]) -> io::Result<()> {
        let mut buf = vec![target, command];
        buf.extend_from_slice(payload);
        self.port.write_all(&buf)?;
        self.port.flush()
    }

    fn wait_for(&self, filter: impl Fn(&RxEvent) -> bool, timeout_ms: u64) -> Option<RxEvent> {
        let deadline = std::time::Instant::now() + Duration::from_millis(timeout_ms);
        loop {
            let remaining = deadline.saturating_duration_since(std::time::Instant::now());
            if remaining.is_zero() { return None; }

            match self.rx.recv_timeout(remaining.min(Duration::from_millis(20))) {
                Ok(RxEvent::Raw(_)) => {}
                Ok(evt) if filter(&evt) => return Some(evt),
                Ok(_) => {}
                Err(_) => {}
            }
        }
    }

    fn wait_ota_ack(&self, expected_seq: u16, timeout_ms: u64) -> bool {
        match self.wait_for(|e| matches!(e, RxEvent::Response { msg_type, .. } if *msg_type == MSG_OTA_ACK), timeout_ms) {
            Some(RxEvent::Response { data, .. }) if data.len() >= 2 => {
                let seq = (data[0] as u16) | ((data[1] as u16) << 8);
                seq == expected_seq
            }
            _ => false,
        }
    }

    fn flash(&mut self, target: u8, path: &str) -> Result<(), Box<dyn std::error::Error>> {
        let firmware = fs::read(path)?;
        let total = firmware.len() as u32;
        info!("Flashing {} ({} bytes) -> 0x{:02X}", path, total, target);

        self.send(target, CMD_OTA_BEGIN, &total.to_le_bytes())?;

        match self.wait_for(|e| matches!(e, RxEvent::Response { msg_type, .. } if *msg_type == MSG_OTA_ACK), 3000) {
            None => return Err("OTA_BEGIN timeout".into()),
            Some(RxEvent::Response { data, .. }) if data.len() >= 2 => {
                if (data[0] as u16) | ((data[1] as u16) << 8) == 0xFFFF {
                    return Err("Device rejected OTA_BEGIN (no OTA partition?)".into());
                }
            }
            _ => return Err("Malformed OTA_BEGIN response".into()),
        }

        let mut seq: u16 = 0;
        for chunk in firmware.chunks(OTA_UART_CHUNK) {
            let mut payload = vec![(seq & 0xFF) as u8, (seq >> 8) as u8];
            payload.extend_from_slice(chunk);
            self.send(target, CMD_OTA_DATA, &payload)?;

            if !self.wait_ota_ack(seq, 2000) {
                let _ = self.send(target, CMD_OTA_END, &[0x01]);
                return Err(format!("OTA stalled at seq {}", seq).into());
            }

            seq = seq.wrapping_add(1);
            if seq % 16 == 0 {
                info!("  {}/{} bytes", (seq as usize) * OTA_UART_CHUNK, total);
            }
        }

        self.send(target, CMD_OTA_END, &[0x00])?;
        info!("OTA complete — device will reboot");
        Ok(())
    }

    fn discover(&mut self) -> Result<Vec<DeviceInfo>, Box<dyn std::error::Error>> {
        self.send(0x00, CMD_DISCOVER, &[])?;

        let mut devices = Vec::new();
        let deadline = std::time::Instant::now() + Duration::from_millis(1000);

        loop {
            let remaining = deadline.saturating_duration_since(std::time::Instant::now());
            if remaining.is_zero() { break; }

            match self.rx.recv_timeout(remaining.min(Duration::from_millis(20))) {
                Ok(RxEvent::Response { msg_type: t, source_device, data })
                if t == MSG_DISCOVERY && data.len() >= 2 =>
                    {
                        devices.push(DeviceInfo { can_id: source_device, module_type: data[0], fw_version: data[1] });
                    }
                _ => {}
            }
        }

        Ok(devices)
    }
}

fn parse_hex(s: &str) -> Result<u8, std::num::ParseIntError> {
    u8::from_str_radix(s.trim_start_matches("0x").trim_start_matches("0X"), 16)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt::init();

    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: client <port>");
        return Ok(());
    }

    let mut client = Client::new(&args[1], 115200)?;

    loop {
        println!("\n=== Devices ===");
        match client.discover() {
            Ok(ref d) if d.is_empty() => warn!("None found"),
            Ok(devices) => {
                for d in &devices {
                    println!("  0x{:02X}  type=0x{:02X}  fw=0x{:02X}", d.can_id, d.module_type, d.fw_version);
                }
            }
            Err(e) => error!("{}", e),
        }

        println!("\n  d              — discover");
        println!("  i <id>         — identify");
        println!("  s <id> <new>   — set ID");
        println!("  f <id> <file>  — flash OTA");
        println!("  q              — quit");
        print!("> ");
        io::stdout().flush()?;

        let mut input = String::new();
        io::stdin().read_line(&mut input)?;
        let parts: Vec<&str> = input.split_whitespace().collect();
        if parts.is_empty() { continue; }

        match parts[0] {
            "q" | "quit" => break,
            "d" | "discover" => {}

            "i" | "identify" => {
                if parts.len() < 2 { eprintln!("Usage: i <id>"); continue; }
                if let Ok(id) = parse_hex(parts[1]) {
                    let _ = client.send(id, CMD_IDENTIFY, &[]);
                }
            }

            "s" | "set" => {
                if parts.len() < 3 { eprintln!("Usage: s <id> <new_id>"); continue; }
                if let (Ok(id), Ok(new_id)) = (parse_hex(parts[1]), parse_hex(parts[2])) {
                    let _ = client.send(id, CMD_SET_ID, &[new_id]);
                }
            }

            "f" | "flash" => {
                if parts.len() < 3 { eprintln!("Usage: f <id> <file>"); continue; }
                if let Ok(id) = parse_hex(parts[1]) {
                    if let Err(e) = client.flash(id, parts[2]) {
                        error!("Flash failed: {}", e);
                    }
                }
            }

            _ => eprintln!("Unknown command"),
        }

        std::thread::sleep(Duration::from_millis(200));
    }

    Ok(())
}
