use std::io::{self, Read, Write};
use std::time::Duration;

use log::{error, info, warn};

// Commands
const CMD_IDENTIFY: u8 = 0x01;
const CMD_SET_ID: u8 = 0x02;
const CMD_DISCOVER: u8 = 0x04;

// Message types
const MSG_RESPONSE: u8 = 0x10;
const MSG_DISCOVERY: u8 = 0x11;

const CMD_ARRAY: [u8; 3] = [CMD_IDENTIFY, CMD_SET_ID, CMD_DISCOVER];

#[derive(Debug)]
struct DeviceInfo {
    can_id: u8,
    module_type: u8,
    fw_version: u8,
}

#[derive(Debug)]
struct UartResponse {
    msg_type: u8,
    source_device: u8,
    data: Vec<u8>,
}

struct DeviceManagerClient {
    port: Box<dyn serialport::SerialPort>,
}

impl DeviceManagerClient {
    fn new(port_name: &str, baud_rate: u32) -> Result<Self, Box<dyn std::error::Error>> {
        info!("Opening serial port {} at {} baud", port_name, baud_rate);

        let port = serialport::new(port_name, baud_rate)
            .timeout(Duration::from_millis(50))
            .data_bits(serialport::DataBits::Eight)
            .parity(serialport::Parity::None)
            .stop_bits(serialport::StopBits::One)
            .flow_control(serialport::FlowControl::None)
            .open()?;

        info!("Serial port opened successfully");
        Ok(DeviceManagerClient { port })
    }

    fn send_command(&mut self, target: u8, command: u8, payload: &[u8]) -> io::Result<()> {
        let mut buffer = vec![target, command];
        buffer.extend_from_slice(payload);

        self.port.write_all(&buffer)?;
        self.port.flush()?;
        Ok(())
    }

    fn read_response(&mut self) -> io::Result<Option<UartResponse>> {
        let mut buffer = [0u8; 256];
        match self.port.read(&mut buffer) {
            Ok(n) if n >= 2 => {
                let msg_type = buffer[0];
                if msg_type == MSG_RESPONSE || msg_type == MSG_DISCOVERY {
                    Ok(Some(UartResponse {
                        msg_type: buffer[0],
                        source_device: buffer[1],
                        data: buffer[2..n].to_vec(),
                    }))
                } else {
                    Ok(None)
                }
            }
            Ok(_) => Ok(None),
            Err(ref e) if e.kind() == io::ErrorKind::TimedOut => Ok(None),
            Err(e) => Err(e),
        }
    }

    fn identify(&mut self, target: u8) -> Result<(), Box<dyn std::error::Error>> {
        info!("Sending identify command to device 0x{:02X}", target);
        self.send_command(target, CMD_IDENTIFY, &[])?;
        Ok(())
    }

    fn set_id(&mut self, target: u8, new_id: u8) -> Result<(), Box<dyn std::error::Error>> {
        info!("Setting device 0x{:02X} to new ID 0x{:02X}", target, new_id);
        self.send_command(target, CMD_SET_ID, &[new_id])?;
        std::thread::sleep(Duration::from_millis(200));
        Ok(())
    }

    fn discover(&mut self) -> Result<Vec<DeviceInfo>, Box<dyn std::error::Error>> {
        info!("Discovering devices on CAN");
        // Send discover command to gateway (0x00), which will broadcast on CAN
        self.send_command(0x00, CMD_DISCOVER, &[])?;

        // Give the device a moment to process and respond
        std::thread::sleep(Duration::from_millis(100));

        let mut devices = Vec::new();
        let start = std::time::Instant::now();

        // Collect responses for up to 1000ms to allow time for devices to respond
        while start.elapsed() < Duration::from_millis(1000) {
            match self.read_response() {
                Ok(Some(resp)) => {
                    // println!("  Parsed: type=0x{:02X}, source=0x{:02X}, data_len={}",
                    //          resp.msg_type, resp.source_device, resp.data.len());
                    if resp.msg_type == MSG_DISCOVERY && resp.data.len() >= 2 {
                        devices.push(DeviceInfo {
                            can_id: resp.source_device,
                            module_type: resp.data[0],
                            fw_version: resp.data[1],
                        });
                        info!(
                            "  Found: CAN ID=0x{:02X}, Type=0x{:02X}, FW=0x{:02X}",
                            resp.source_device, resp.data[0], resp.data[1]
                        );
                    }
                }
                Ok(None) => {
                    // No data available, keep waiting
                    std::thread::sleep(Duration::from_millis(10));
                }
                Err(e) => {
                    error!("Read error: {}", e);
                    break;
                }
            }
        }

        Ok(devices)
    }
}

fn parse_hex(s: &str) -> Result<u8, std::num::ParseIntError> {
    let s = s.trim_start_matches("0x").trim_start_matches("0X");
    u8::from_str_radix(s, 16)
}

fn read_input() -> String {
    let mut input = String::new();
    io::stdin().read_line(&mut input).unwrap();
    input.trim().to_string()
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt::init();

    let args: Vec<String> = std::env::args().collect();

    if args.len() < 2 {
        error!("Usage: client <port>");
        return Ok(());
    }

    let port_name = &args[1];
    let mut client = DeviceManagerClient::new(port_name, 115200)?;

    loop {
        info!("\n=== Discovered Devices ===");
        match client.discover() {
            Ok(devices) => {
                if devices.is_empty() {
                    warn!("No devices found");
                } else {
                    for dev in &devices {
                        info!(
                            "ID: 0x{:02X}  Type: 0x{:02X}  FW: 0x{:02X}",
                            dev.can_id, dev.module_type, dev.fw_version
                        );
                    }
                }
            }
            Err(e) => error!("Discovery error: {}", e),
        }

        info!("\nCommands: d (discover) | i <id> (identify) | s <id> <new_id> (set) | q (quit)");
        info!("> ");
        io::stdout().flush()?;

        let input = read_input();
        let parts: Vec<&str> = input.split_whitespace().collect();

        if parts.is_empty() {
            continue;
        }

        match parts[0] {
            "q" | "quit" => break,

            "d" | "discover" => {
                // Just loop to rediscover devices
            }

            "i" | "identify" => {
                if parts.len() < 2 {
                    error!("Usage: i <id>");
                    continue;
                }
                if let Ok(id) = parse_hex(parts[1]) {
                    let _ = client.identify(id);
                }
            }

            "s" | "set" => {
                if parts.len() < 3 {
                    error!("Usage: s <id> <new_id>");
                    continue;
                }
                if let (Ok(id), Ok(new_id)) = (parse_hex(parts[1]), parse_hex(parts[2])) {
                    let _ = client.set_id(id, new_id);
                }
            }

            _ => error!("Unknown command"),
        }

        std::thread::sleep(Duration::from_millis(500));
    }

    Ok(())
}
