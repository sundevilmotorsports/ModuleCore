use std::collections::HashMap;
use std::fs;
use std::io::{self, Write};
use std::sync::mpsc::{self, Receiver};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use log::{error, info, warn};
use serde::Serialize;
use serialport::SerialPortType;
use tauri::{AppHandle, Emitter, Manager, State};

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
const DEVICE_TIMEOUT_MS: u64 = 2500;

const DEVICE_VID: u16 = 0x10C4;
const DEVICE_PID: u16 = 0xEA60;

#[derive(Debug, Clone, Serialize)]
struct DeviceEntry {
    can_id: u8,
    can_id_str: String,
    module_type_str: String,
    fw_version_str: String,
}

#[derive(Debug, Clone, Serialize)]
struct AppState {
    port_name: String,
    devices: Vec<DeviceEntry>,
}

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

type DeviceMap = Arc<Mutex<HashMap<u8, (DeviceInfo, Instant)>>>;
type SharedClient = Arc<Mutex<Option<Client>>>;

struct AppStateInner {
    client: SharedClient,
    device_map: DeviceMap,
    port_name: Mutex<String>,
}

fn find_device_port() -> Option<String> {
    serialport::available_ports().ok()?
        .into_iter()
        .find(|p| matches!(&p.port_type,
            SerialPortType::UsbPort(info)
            if info.vid == DEVICE_VID && info.pid == DEVICE_PID
        ))
        .map(|p| p.port_name)
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
        let deadline = Instant::now() + Duration::from_millis(timeout_ms);
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
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
        let deadline = Instant::now() + Duration::from_millis(1000);

        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() { break; }

            match self.rx.recv_timeout(remaining.min(Duration::from_millis(20))) {
                Ok(RxEvent::Response { msg_type: t, source_device, data })
                if t == MSG_DISCOVERY && data.len() >= 2 => {
                    devices.push(DeviceInfo {
                        can_id: source_device,
                        module_type: data[0],
                        fw_version: data[1],
                    });
                }
                _ => {}
            }
        }

        Ok(devices)
    }
}

fn emit_state(app: &AppHandle, state: &AppStateInner) {
    let port_name = state.port_name.lock().unwrap().clone();
    let map = state.device_map.lock().unwrap();
    let devices: Vec<DeviceEntry> = map.values()
        .map(|(d, _)| DeviceEntry {
            can_id: d.can_id,
            can_id_str: format!("0x{:02X}", d.can_id),
            module_type_str: format!("0x{:02X}", d.module_type),
            fw_version_str: format!("0x{:02X}", d.fw_version),
        })
        .collect();
    let _ = app.emit("state-update", AppState { port_name, devices });
}

fn start_background_workers(app: AppHandle, state: Arc<AppStateInner>) {
    // ── Device connection + discovery loop ───────────────────────────────────
    // This single thread handles both hotplug and continuous discovery so the
    // window opens immediately regardless of whether a device is plugged in.
    std::thread::spawn(move || {
        loop {
            // Wait until a device appears (non-blocking poll so window stays open)
            if state.client.lock().unwrap().is_none() {
                match find_device_port() {
                    Some(port) => {
                        match Client::new(&port, 115200) {
                            Ok(c) => {
                                info!("Connected on {}", port);
                                *state.client.lock().unwrap() = Some(c);
                                *state.port_name.lock().unwrap() = port.clone();
                                emit_state(&app, &state);
                            }
                            Err(e) => {
                                error!("Failed to open {}: {}", port, e);
                                std::thread::sleep(Duration::from_millis(500));
                                continue;
                            }
                        }
                    }
                    None => {
                        std::thread::sleep(Duration::from_millis(500));
                        continue;
                    }
                }
            }

            // Run a discover cycle
            let devices = {
                let mut slot = state.client.lock().unwrap();
                match slot.as_mut() {
                    Some(c) => c.discover().unwrap_or_default(),
                    None => vec![],
                }
            };

            // Check if port disappeared during discover
            if find_device_port().is_none() {
                warn!("Device disconnected");
                *state.client.lock().unwrap() = None;
                state.device_map.lock().unwrap().clear();
                *state.port_name.lock().unwrap() = String::new();
                emit_state(&app, &state);
                continue;
            }

            // Stamp seen devices
            {
                let mut map = state.device_map.lock().unwrap();
                for d in devices {
                    map.insert(d.can_id, (d, Instant::now()));
                }
            }

            // Cull timed-out devices
            {
                let mut map = state.device_map.lock().unwrap();
                map.retain(|_, (_, ts)| ts.elapsed() < Duration::from_millis(DEVICE_TIMEOUT_MS));
            }

            emit_state(&app, &state);
        }
    });
}

#[tauri::command]
fn identify(can_id: u8, state: State<Arc<AppStateInner>>) -> Result<(), String> {
    let mut slot = state.client.lock().unwrap();
    match slot.as_mut() {
        Some(c) => c.send(can_id, CMD_IDENTIFY, &[]).map_err(|e| e.to_string()),
        None => Err("No device connected".into()),
    }
}

#[tauri::command]
fn get_state(state: State<Arc<AppStateInner>>) -> AppState {
    let port_name = state.port_name.lock().unwrap().clone();
    let map = state.device_map.lock().unwrap();
    let devices = map.values()
        .map(|(d, _)| DeviceEntry {
            can_id: d.can_id,
            can_id_str: format!("0x{:02X}", d.can_id),
            module_type_str: format!("0x{:02X}", d.module_type),
            fw_version_str: format!("0x{:02X}", d.fw_version),
        })
        .collect();
    AppState { port_name, devices }
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    let state = Arc::new(AppStateInner {
        client: Arc::new(Mutex::new(None)),
        device_map: Arc::new(Mutex::new(HashMap::new())),
        port_name: Mutex::new(String::new()),
    });

    let state_clone = Arc::clone(&state);

    tauri::Builder::default()
        .plugin(tauri_plugin_opener::init())
        .manage(state)
        .invoke_handler(tauri::generate_handler![identify, get_state])
        .setup(move |app| {
            let app_handle = app.handle().clone();
            start_background_workers(app_handle, state_clone);
            Ok(())
        })
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}