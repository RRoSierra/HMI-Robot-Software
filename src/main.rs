use eframe::egui;
use egui::{Color32, Pos2, Stroke, Vec2};
use egui_plot::{Line, Plot, PlotPoints};
use std::f64::consts::PI;
use std::fs::File;
use std::io::{Read, Write};
use std::sync::mpsc;
use std::thread;
use std::time::{Duration, Instant};

// ==========================================
// CONFIGURACIÓN CONSTANTE
// ==========================================
const MAX_DATA_POINTS: usize = 150;
const ENCODER_RESOLUTION: f64 = 8192.0;
const STM32_SAMPLE_TIME: f64 = 0.02; // 20ms exactos (50Hz)

// Filtro Pasa-Bajos (EMA)
const FILTER_ALPHA: f64 = 0.35; 

// Parámetros físicos del robot
const ROBOT_RADIUS: f64 = 0.08215; // metros
const WHEEL_RADIUS: f64 = 0.02704; // metros

// Paleta Sysmic
const COLOR_BG: Color32 = Color32::from_rgb(8, 10, 15);
const COLOR_PANEL: Color32 = Color32::from_rgb(30, 34, 46);
const COLOR_SYSMIC_BLUE: Color32 = Color32::from_rgb(0, 168, 232);
const COLOR_TEXT: Color32 = Color32::from_rgb(226, 232, 240);
const CHART_COLORS: [Color32; 4] = [
    Color32::from_rgb(0, 168, 232),   // M1
    Color32::from_rgb(0, 255, 204),   // M2
    Color32::from_rgb(255, 176, 0),   // M3
    Color32::from_rgb(255, 51, 102),  // M4
];

// ==========================================
// ESTRUCTURAS DE DATOS Y MENSAJES
// ==========================================
#[derive(PartialEq, Clone, Copy)]
enum ControlMode {
    Manual,
    Kinematic,
}

enum SerialMessage {
    Telemetry([u16; 4]),
}

struct SysmicHmi {
    start_time: Instant,
    
    // --- HISTORIALES Y LECTURAS ---
    vel_history: [Vec<[f64; 2]>; 4],  
    act_history: [Vec<[f64; 2]>; 4],  
    current_vels: [f64; 4],
    current_actuations: [f64; 4],
    last_angles: [u16; 4],

    // --- FILTROS Y TIEMPO VIRTUAL ---
    raw_vel_buffer: [Vec<f64>; 4],
    telemetry_time_counter: f64,
    
    // --- ESTADO DE ACTUACIÓN Y CONFIGURACIÓN ---
    control_mode: ControlMode,
    instant_update: bool, // true = Continuo, false = Disparo/Escalón
    
    // Sliders UI (Memoria visual)
    slider_manual_sp: [i16; 4], 
    slider_kin_sp: [f64; 3],    
    
    // Setpoints Activos (Los que realmente se envían)
    active_manual_sp: [i16; 4],
    active_kin_sp: [f64; 3],
    
    // Cinemática Dinámica
    wheel_angles: [f64; 4], 
    wheel_map: [usize; 4],  

    last_tx_time: f64,   
    
    // --- CONEXIÓN SERIE ---
    available_ports: Vec<String>,
    selected_port: String,
    connected_port: Option<String>,
    rx_channel: Option<mpsc::Receiver<SerialMessage>>,
    tx_channel: Option<mpsc::Sender<Vec<u8>>>, 

    // Logs
    is_recording: bool,
    recorded_data: Vec<[f64; 9]>, 
}

impl Default for SysmicHmi {
    fn default() -> Self {
        let mut app = Self {
            start_time: Instant::now(),
            vel_history: [vec![], vec![], vec![], vec![]],
            act_history: [vec![], vec![], vec![], vec![]],
            current_vels: [0.0; 4],
            current_actuations: [0.0; 4],
            last_angles: [0; 4],
            
            raw_vel_buffer: [vec![0.0; 5], vec![0.0; 5], vec![0.0; 5], vec![0.0; 5]], 
            telemetry_time_counter: 0.0,

            control_mode: ControlMode::Manual,
            instant_update: true,
            
            slider_manual_sp: [0; 4],
            slider_kin_sp: [0.0; 3],
            active_manual_sp: [0; 4],
            active_kin_sp: [0.0; 3],
            
            wheel_angles: [40.0, 140.0, 210.0, 330.0],
            wheel_map: [0, 1, 2, 3], 

            last_tx_time: 0.0,
            
            available_ports: vec![],
            selected_port: String::new(),
            connected_port: None,
            rx_channel: None,
            tx_channel: None,
            is_recording: false,
            recorded_data: Vec::new(),
        };
        app.refresh_ports();
        app
    }
}

impl SysmicHmi {
    fn refresh_ports(&mut self) {
        self.available_ports.clear();
        if let Ok(ports) = serialport::available_ports() {
            for port in ports {
                self.available_ports.push(port.port_name);
            }
            if !self.available_ports.is_empty() && self.selected_port.is_empty() {
                self.selected_port = self.available_ports[0].clone();
            }
        }
    }

    fn connect_serial(&mut self) {
        if self.selected_port.is_empty() { return; }
        self.last_angles = [0; 4];
        self.current_vels = [0.0; 4];
        self.current_actuations = [0.0; 4];
        self.raw_vel_buffer = [vec![0.0; 5], vec![0.0; 5], vec![0.0; 5], vec![0.0; 5]];
        self.telemetry_time_counter = self.start_time.elapsed().as_secs_f64();

        for h in &mut self.vel_history { h.clear(); }
        for h in &mut self.act_history { h.clear(); }

        let port_name = self.selected_port.clone();
        let (data_tx, data_rx) = mpsc::channel::<SerialMessage>();
        let (cmd_tx, cmd_rx) = mpsc::channel::<Vec<u8>>(); 
        self.rx_channel = Some(data_rx);
        self.tx_channel = Some(cmd_tx);
        self.connected_port = Some(port_name.clone());

        thread::spawn(move || {
            let mut port = match serialport::new(&port_name, 115200)
                .timeout(Duration::from_millis(10))
                .open() {
                Ok(p) => p,
                Err(_) => return,
            };

            let mut buffer: Vec<u8> = Vec::new();
            let mut read_buf = [0u8; 1024];

            loop {
                while let Ok(packet) = cmd_rx.try_recv() {
                    if port.write_all(&packet).is_err() { return; }
                }

                match port.read(&mut read_buf) {
                    Ok(t) if t > 0 => buffer.extend_from_slice(&read_buf[..t]),
                    Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => (),
                    Err(_) => break,
                    _ => (),
                }

                'parse: loop {
                    if buffer.len() < 2 { break 'parse; }

                    if buffer[0] == 0xAA && buffer[1] == 0xBB {
                        if buffer.len() < 12 { break 'parse; }
                        if buffer[11] == 0x0A {
                            let payload = &buffer[2..10];
                            let chk: u8 = payload.iter().copied().fold(0u8, |acc, x| acc.wrapping_add(x));
                            if chk == buffer[10] {
                                let ang1 = u16::from_le_bytes([buffer[2], buffer[3]]);
                                let ang2 = u16::from_le_bytes([buffer[4], buffer[5]]);
                                let ang3 = u16::from_le_bytes([buffer[6], buffer[7]]);
                                let ang4 = u16::from_le_bytes([buffer[8], buffer[9]]);
                                let _ = data_tx.send(SerialMessage::Telemetry([ang1, ang2, ang3, ang4]));
                            }
                            buffer.drain(0..12);
                        } else {
                            buffer.remove(0);
                        }
                    } else {
                        buffer.remove(0);
                    }
                }
            }
        });
    }

    fn disconnect_serial(&mut self) {
        self.rx_channel = None;
        self.tx_channel = None;
        self.connected_port = None;
        self.slider_manual_sp = [0; 4];
        self.slider_kin_sp = [0.0; 3];
        self.active_manual_sp = [0; 4];
        self.active_kin_sp = [0.0; 3];
    }

    fn save_recording_csv(&mut self) {
        let timestamp = chrono::Local::now().format("%Y%m%d_%H%M%S").to_string();
        let filename = format!("recording_{}.csv", timestamp);
        if let Ok(mut file) = File::create(&filename) {
            let _ = writeln!(file, "tiempo_s,vel_m1,vel_m2,vel_m3,vel_m4,act_m1,act_m2,act_m3,act_m4");
            for row in &self.recorded_data {
                let _ = writeln!(file, "{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}", 
                    row[0], row[1], row[2], row[3], row[4], row[5], row[6], row[7], row[8]);
            }
        }
        self.recorded_data.clear();
    }
}

impl eframe::App for SysmicHmi {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let t = self.start_time.elapsed().as_secs_f64();

        // 1. RECEPCIÓN DE DATOS (RX) - PROCESAMIENTO SECUENCIAL ESTRICTO
        if let Some(rx) = &self.rx_channel {
            
            if (self.telemetry_time_counter - t).abs() > 1.0 {
                self.telemetry_time_counter = t;
            }

            while let Ok(msg) = rx.try_recv() {
                if let SerialMessage::Telemetry(angles) = msg {
                    
                    self.telemetry_time_counter += STM32_SAMPLE_TIME; 
                    
                    for i in 0..4 {
                        let mut delta_ticks = angles[i] as f64 - self.last_angles[i] as f64;
                        
                        if delta_ticks >  32768.0 { delta_ticks -= 65536.0; }
                        if delta_ticks < -32768.0 { delta_ticks += 65536.0; }

                        let raw_vel = delta_ticks * (2.0 * PI / ENCODER_RESOLUTION) / STM32_SAMPLE_TIME;

                        self.raw_vel_buffer[i].remove(0);
                        self.raw_vel_buffer[i].push(raw_vel);
                        
                        let mut sorted_buf = self.raw_vel_buffer[i].clone();
                        sorted_buf.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
                        let median_vel = sorted_buf[2]; 

                        self.current_vels[i] = (FILTER_ALPHA * median_vel) + ((1.0 - FILTER_ALPHA) * self.current_vels[i]);

                        self.vel_history[i].push([self.telemetry_time_counter, self.current_vels[i]]);
                        if self.vel_history[i].len() > MAX_DATA_POINTS { self.vel_history[i].remove(0); }
                    }
                    self.last_angles = angles;

                    if self.is_recording {
                        self.recorded_data.push([
                            self.telemetry_time_counter, 
                            self.current_vels[0], self.current_vels[1], self.current_vels[2], self.current_vels[3],
                            self.current_actuations[0], self.current_actuations[1], self.current_actuations[2], self.current_actuations[3]
                        ]);
                    }
                }
            }
        }

        // --- CÁLCULO DE ACTUACIONES Y CINEMÁTICA ---
        let mut target_phys_actuations = [0.0; 4];

        // Lógica de Transferencia Continuo/Disparo
        if self.instant_update {
            self.active_manual_sp = self.slider_manual_sp;
            self.active_kin_sp = self.slider_kin_sp;
        }

        if self.control_mode == ControlMode::Manual {
            for logical_i in 0..4 {
                let phys_i = self.wheel_map[logical_i];
                target_phys_actuations[phys_i] = self.active_manual_sp[logical_i] as f64;
            }
        } else {
            let vx = self.active_kin_sp[0];
            let vy = self.active_kin_sp[1];
            let vw = self.active_kin_sp[2];
            
            for logical_i in 0..4 {
                let a = self.wheel_angles[logical_i] * PI / 180.0;
                let v_tang = -vx * a.sin() + vy * a.cos() + vw * ROBOT_RADIUS;
                let omega_rad_s = v_tang / WHEEL_RADIUS;
                
                let phys_i = self.wheel_map[logical_i];
                target_phys_actuations[phys_i] = omega_rad_s;
            }
        }
        self.current_actuations = target_phys_actuations;


        // --- TRANSMISIÓN PERIÓDICA AL STM32 (TX a ~50 Hz) ---
        if self.connected_port.is_some() {
            if t - self.last_tx_time > 0.02 { 
                self.last_tx_time = t;
                
                for i in 0..4 {
                    self.act_history[i].push([t, self.current_actuations[i]]);
                    if self.act_history[i].len() > MAX_DATA_POINTS { self.act_history[i].remove(0); }
                }

                let mut packet = vec![0xAA]; 
                packet.push(self.control_mode as u8); 
                
                for i in 0..4 {
                    let val = if self.control_mode == ControlMode::Manual {
                        self.current_actuations[i] as i16
                    } else {
                        (self.current_actuations[i] * 100.0) as i16
                    };
                    let bytes = val.to_le_bytes();
                    packet.push(bytes[0]);
                    packet.push(bytes[1]);
                }
                
                let mut chk = 0u8;
                for b in &packet[1..] { chk = chk.wrapping_add(*b); }
                packet.push(chk);
                packet.push(0x0A); 
                
                if let Some(tx) = &self.tx_channel {
                    let _ = tx.send(packet);
                }
            }
        }

        ctx.request_repaint();

        // 2. ESTILOS UI
        let mut style = (*ctx.style()).clone();
        style.visuals.window_fill = COLOR_BG;
        style.visuals.panel_fill = COLOR_BG;
        ctx.set_style(style);

        // --- CABECERA ---
        egui::TopBottomPanel::top("header").show(ctx, |ui| {
            ui.add_space(10.0);
            ui.horizontal(|ui| {
                ui.heading(
                    egui::RichText::new("SYSMIC ROBOTICS - SSL RUST HMI")
                        .color(COLOR_SYSMIC_BLUE).size(24.0).strong(),
                );
                
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if self.connected_port.is_none() {
                        if ui.button("Conectar USB").clicked() { self.connect_serial(); }
                        if ui.button("↻").clicked() { self.refresh_ports(); }
                        egui::ComboBox::from_id_source("com_ports")
                            .selected_text(&self.selected_port)
                            .show_ui(ui, |ui| {
                                for p in &self.available_ports {
                                    ui.selectable_value(&mut self.selected_port, p.clone(), p);
                                }
                            });
                        ui.label(egui::RichText::new("⬛ Desconectado").color(Color32::from_gray(160)).strong());
                    } else {
                        if ui.button("Desconectar").clicked() { self.disconnect_serial(); }
                        let rec_text = if self.is_recording { "⏹ Detener Grabación" } else { "⏺ Grabar CSV" };
                        let rec_color = if self.is_recording { Color32::RED } else { Color32::from_rgb(0, 200, 0) };
                        if ui.add(egui::Button::new(egui::RichText::new(rec_text).color(rec_color))).clicked() {
                            if self.is_recording {
                                self.is_recording = false;
                                self.save_recording_csv();
                            } else {
                                self.recorded_data.clear();
                                self.is_recording = true;
                            }
                        }
                        ui.label(egui::RichText::new(format!("🟢 Leyendo STM32: {}", self.selected_port)).color(Color32::GREEN).strong());
                    }
                });
            });
            ui.add_space(10.0);
        });

        // --- PANEL INFERIOR (ACTUACIÓN Y CONFIGURACIÓN) ---
        egui::TopBottomPanel::bottom("control_panel")
            .exact_height(260.0) 
            .show(ctx, |ui| {
                ui.add_space(10.0);
                ui.horizontal(|ui| {
                    
                    // Columna 1: Modos y Emergencia
                    ui.vertical(|ui| {
                        ui.set_width(220.0);
                        ui.label(egui::RichText::new("MODO DE OPERACIÓN").color(COLOR_SYSMIC_BLUE).strong());
                        ui.radio_value(&mut self.control_mode, ControlMode::Manual, "Modo Manual (DAC Directo)");
                        ui.radio_value(&mut self.control_mode, ControlMode::Kinematic, "Modo Cinemático (IK en PC)");
                        
                        ui.add_space(10.0);
                        ui.label(egui::RichText::new("TIPO DE ACTUALIZACIÓN").color(COLOR_SYSMIC_BLUE).strong());
                        ui.horizontal(|ui| {
                            ui.radio_value(&mut self.instant_update, true, "Continuo");
                            ui.radio_value(&mut self.instant_update, false, "Disparo");
                        });

                        ui.add_space(10.0);
                        let stop_btn = egui::Button::new(
                            egui::RichText::new("🛑 STOP / EMERGENCIA").color(Color32::WHITE).size(14.0).strong()
                        ).fill(Color32::from_rgb(200, 0, 0));
                        
                        if ui.add_sized([200.0, 30.0], stop_btn).clicked() {
                            self.slider_manual_sp = [0; 4];
                            self.slider_kin_sp = [0.0; 3];
                            self.active_manual_sp = [0; 4];
                            self.active_kin_sp = [0.0; 3];
                        }
                    });

                    ui.separator();

                    // Columna 2: Sliders de Actuación
                    ui.vertical(|ui| {
                        ui.set_width(260.0);
                        match self.control_mode {
                            ControlMode::Manual => {
                                ui.label(egui::RichText::new("SETPOINTS MANUALES (-4095 a +4095)").strong());
                                ui.horizontal(|ui| {
                                    for i in 0..4 {
                                        ui.vertical(|ui| {
                                            ui.label(format!("R. {}", i + 1));
                                            ui.add(egui::Slider::new(&mut self.slider_manual_sp[i], -4095..=4095).orientation(egui::SliderOrientation::Vertical));
                                            
                                            ui.add_space(5.0);
                                            
                                            // Botón para volver a 0 (No inyecta si está en modo Disparo)
                                            if ui.button("0").clicked() { 
                                                self.slider_manual_sp[i] = 0; 
                                                if self.instant_update { self.active_manual_sp[i] = 0; }
                                            }

                                            // Botón "Rayo" Individual (Solo aparece en modo Disparo)
                                            if !self.instant_update {
                                                ui.add_space(2.0);
                                                let inject_mini_btn = egui::Button::new(
                                                    egui::RichText::new("⚡").color(Color32::BLACK)
                                                ).fill(Color32::from_rgb(255, 215, 0));
                                                
                                                if ui.add(inject_mini_btn).on_hover_text("Inyectar escalón solo a este motor").clicked() {
                                                    self.active_manual_sp[i] = self.slider_manual_sp[i];
                                                }
                                            }
                                        });
                                        ui.add_space(8.0);
                                    }
                                });
                            },
                            ControlMode::Kinematic => {
                                ui.label(egui::RichText::new("SETPOINTS CINEMÁTICOS").strong());
                                ui.add_space(10.0);
                                ui.horizontal(|ui| {
                                    ui.label("Vx (Frontal):");
                                    ui.add(egui::Slider::new(&mut self.slider_kin_sp[0], -10.0..=10.0).text("m/s"));
                                    if ui.button("0").clicked() { 
                                        self.slider_kin_sp[0] = 0.0; 
                                        if self.instant_update { self.active_kin_sp[0] = 0.0; }
                                    }
                                });
                                ui.horizontal(|ui| {
                                    ui.label("Vy (Lateral): ");
                                    ui.add(egui::Slider::new(&mut self.slider_kin_sp[1], -10.0..=10.0).text("m/s"));
                                    if ui.button("0").clicked() { 
                                        self.slider_kin_sp[1] = 0.0; 
                                        if self.instant_update { self.active_kin_sp[1] = 0.0; }
                                    }
                                });
                                ui.horizontal(|ui| {
                                    ui.label("Vω (Giro):    ");
                                    ui.add(egui::Slider::new(&mut self.slider_kin_sp[2], -15.0..=15.0).text("rad/s"));
                                    if ui.button("0").clicked() { 
                                        self.slider_kin_sp[2] = 0.0; 
                                        if self.instant_update { self.active_kin_sp[2] = 0.0; }
                                    }
                                });
                            }
                        }

                        // Botón de Disparo Global (Inyecta todo a la vez)
                        if !self.instant_update {
                            ui.add_space(10.0);
                            let inject_btn = egui::Button::new(
                                egui::RichText::new("⚡ INYECTAR ESCALÓN").color(Color32::BLACK).size(14.0).strong()
                            ).fill(Color32::from_rgb(255, 215, 0));
                            
                            if ui.add_sized([250.0, 30.0], inject_btn).clicked() {
                                self.active_manual_sp = self.slider_manual_sp;
                                self.active_kin_sp = self.slider_kin_sp;
                            }
                        }
                    });

                    ui.separator();

                    // Columna 3: Configuración de Ángulos
                    ui.vertical(|ui| {
                        ui.set_width(180.0);
                        ui.label(egui::RichText::new("ÁNGULOS DE RUEDA (°)").color(COLOR_SYSMIC_BLUE).strong());
                        ui.add_space(5.0);
                        for i in 0..4 {
                            ui.horizontal(|ui| {
                                ui.label(format!("Rueda Lógica {}:", i + 1));
                                ui.add(egui::DragValue::new(&mut self.wheel_angles[i]).speed(1.0).clamp_range(0.0..=360.0));
                            });
                        }
                    });

                    ui.separator();

                    // Columna 4: Mapeo de Ruedas Físicas
                    ui.vertical(|ui| {
                        ui.set_width(220.0);
                        ui.label(egui::RichText::new("MAPEO A HARDWARE").color(COLOR_SYSMIC_BLUE).strong());
                        ui.add_space(5.0);
                        for logical_i in 0..4 {
                            ui.horizontal(|ui| {
                                ui.label(format!("Señal Lógica {} →", logical_i + 1));
                                egui::ComboBox::from_id_source(format!("map_{}", logical_i))
                                    .selected_text(format!("Motor M{}", self.wheel_map[logical_i] + 1))
                                    .width(90.0)
                                    .show_ui(ui, |ui| {
                                        ui.selectable_value(&mut self.wheel_map[logical_i], 0, "Motor M1 (Azul)");
                                        ui.selectable_value(&mut self.wheel_map[logical_i], 1, "Motor M2 (Cyan)");
                                        ui.selectable_value(&mut self.wheel_map[logical_i], 2, "Motor M3 (Amarillo)");
                                        ui.selectable_value(&mut self.wheel_map[logical_i], 3, "Motor M4 (Magenta)");
                                    });
                            });
                        }
                    });

                });
            });

        // --- PANEL IZQUIERDO 1 (ACTUACIONES) ---
        egui::SidePanel::left("actuations_panel")
            .resizable(true)
            .default_width(320.0)
            .show(ctx, |ui| {
                ui.add_space(10.0);
                ui.label(egui::RichText::new("ACTUACIONES (SETPOINTS)").color(COLOR_SYSMIC_BLUE).strong().size(16.0));
                ui.separator();
                
                let unit = if self.control_mode == ControlMode::Manual { "DAC" } else { "rad/s" };
                
                for phys_i in 0..4 {
                    ui.group(|ui| {
                        ui.label(egui::RichText::new(format!("REFERENCIA M{} ({})", phys_i + 1, unit)).color(COLOR_TEXT).strong());
                        let line = Line::new(PlotPoints::new(self.act_history[phys_i].clone())).color(CHART_COLORS[phys_i]).width(2.0);

                        Plot::new(format!("act_plot_{}", phys_i))
                            .height(100.0) 
                            .show_axes([false, true])
                            .allow_drag(false)
                            .allow_zoom(false)
                            .show(ui, |plot_ui| plot_ui.line(line));
                    });
                    ui.add_space(2.0);
                }
            });

        // --- PANEL IZQUIERDO 2 (MEDICIONES) ---
        egui::SidePanel::left("plots_panel")
            .resizable(true)
            .default_width(320.0)
            .show(ctx, |ui| {
                ui.add_space(10.0);
                ui.label(egui::RichText::new("MEDICIONES (ENCODERS)").color(Color32::GREEN).strong().size(16.0));
                ui.separator();
                
                for phys_i in 0..4 {
                    ui.group(|ui| {
                        ui.label(egui::RichText::new(format!("VELOCIDAD M{} (rad/s)", phys_i + 1)).color(COLOR_TEXT).strong());
                        let line = Line::new(PlotPoints::new(self.vel_history[phys_i].clone())).color(CHART_COLORS[phys_i]).width(2.0);

                        Plot::new(format!("vel_plot_{}", phys_i))
                            .height(100.0) 
                            .show_axes([false, true])
                            .include_y(30.0)
                            .include_y(-30.0)
                            .allow_drag(false)
                            .allow_zoom(false)
                            .show(ui, |plot_ui| plot_ui.line(line));
                    });
                    ui.add_space(2.0);
                }
            });

        // --- PANEL CENTRAL (Vista 2D de la Base) ---
        egui::CentralPanel::default().show(ctx, |ui| {
            let rect = ui.available_rect_before_wrap();
            let center = rect.center();
            let scale = (rect.width().min(rect.height()) / 2.0) * 0.45; 

            let painter = ui.painter();
            let to_screen = |x: f64, y: f64| -> Pos2 {
                center + Vec2::new((x * scale as f64) as f32, (-y * scale as f64) as f32)
            };

            // 1. Dibujar Chasis
            let cut_y: f64 = -0.75;
            let start_angle = cut_y.asin();
            let end_angle = PI - start_angle;
            let mut robot_shape = vec![];
            
            let steps = 60;
            for i in 0..=steps {
                let a = start_angle + (end_angle - start_angle) * (i as f64 / steps as f64);
                robot_shape.push(to_screen(a.cos(), a.sin()));
            }
            
            painter.add(egui::Shape::convex_polygon(robot_shape, COLOR_PANEL, Stroke::new(3.0, Color32::from_gray(80))));

            // 2. Centro
            painter.circle(to_screen(0.0, 0.0), (0.25 * scale as f64) as f32, Color32::from_rgb(255, 215, 0), Stroke::new(2.0, Color32::BLACK));

            // 3. Posiciones físicas fijas de los motores en el chasis
            let mount_points = [
                (-0.45, 0.45),  // M1 (Azul)
                (0.45, 0.45),   // M2 (Cyan)
                (-0.45, -0.45), // M3 (Amarillo)
                (0.45, -0.45),  // M4 (Magenta)
            ];

            for phys_i in 0..4 {
                let (cx, cy) = mount_points[phys_i];
                painter.circle(to_screen(cx, cy), (0.15 * scale as f64) as f32, CHART_COLORS[phys_i], Stroke::new(2.0, Color32::BLACK));
            }

            // 4. Dibujar Llantas rotatorias y Vectores en el PERÍMETRO del chasis
            for logical_i in 0..4 {
                let angle_rad = self.wheel_angles[logical_i] * PI / 180.0;
                let phys_i = self.wheel_map[logical_i]; 
                let wheel_color = CHART_COLORS[phys_i]; 
                
                let x = angle_rad.cos();
                let y = angle_rad.sin();
                let wheel_center = to_screen(x, y);
                
                let tangent_dir = Vec2::new(-(angle_rad.sin() as f32), -(angle_rad.cos() as f32));
                let wheel_length = scale as f32 * 0.4;
                let p1 = wheel_center + tangent_dir * (wheel_length / 2.0);
                let p2 = wheel_center - tangent_dir * (wheel_length / 2.0);
                
                painter.line_segment([p1, p2], Stroke::new(14.0, Color32::from_rgb(17, 17, 17)));
                painter.line_segment([p1, p2], Stroke::new(4.0, wheel_color));

                // --- NORMALIZACIÓN DE FLECHAS ---
                let vel = self.current_vels[phys_i];
                let vec_scale = vel * 0.006; 
                
                let force_x = (angle_rad + PI/2.0).cos() * vec_scale;
                let force_y = (angle_rad + PI/2.0).sin() * vec_scale;
                let vector_end = to_screen(x + force_x, y + force_y);
                
                let vector_color = if vel >= 0.0 { Color32::from_rgb(0, 255, 0) } else { Color32::from_rgb(255, 0, 255) };
                painter.line_segment([wheel_center, vector_end], Stroke::new(4.0, vector_color));
                painter.circle_filled(vector_end, 5.0, vector_color); 
            }
        });
    }
}

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1400.0, 900.0]) 
            .with_title("Sysmic Robotics - SSL HMI"),
        ..Default::default()
    };
    
    eframe::run_native(
        "Sysmic HMI",
        options,
        Box::new(|_cc| Box::new(SysmicHmi::default())),
    )
}