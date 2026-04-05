import serial
import serial.tools.list_ports
import time
import csv
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from collections import deque

# --- CONFIG ---
BAUD = 921600
MAX_PLOT_POINTS = 150
PROGRAMS = {
    "Quick Ramp": [(0, 10), (30, 10), (40, 10), (50, 10), (60, 10), (70, 10),  (80, 10), (90, 10), (100, 10), (0, 10)],
    "Step Test":  [(0, 2), (20, 5), (40, 5), (60, 5), (80, 5), (100, 5), (0, 2)],
    "Burst":      [(0, 5), (100, 2), (0, 2)],
    "Manual Mode": []
}

class MotorLabGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("RPI DBF - Propulsion Test Hub (v2.5)")
        self.root.geometry("1400x950")
        self.root.configure(bg="#1e272e")

        self.ser = None
        self.running = True
        self.logging = False
        self.test_active = False
        self.counting_down = False

        # --- Latest data (always the most recent frame, not a queue) ---
        self.latest = None
        self.latest_lock = threading.Lock()

        # Plot buffers
        self.plot_thrust      = deque(maxlen=MAX_PLOT_POINTS)
        self.plot_torque_push = deque(maxlen=MAX_PLOT_POINTS)
        self.plot_torque_pull = deque(maxlen=MAX_PLOT_POINTS)
        self.plot_throttle    = deque(maxlen=MAX_PLOT_POINTS)

        self.setup_ui()
        self.find_serial()

    def setup_ui(self):
        # --- SIDEBAR ---
        self.sidebar = tk.Frame(self.root, bg="#2f3640", width=350)
        self.sidebar.pack(side="left", fill="y", padx=5, pady=5)

        tk.Label(self.sidebar, text="TELEMETRY", font=("Arial", 12, "bold"),
                 bg="#2f3640", fg="white").pack(pady=5)

        self.thrust_lbl = tk.Label(self.sidebar, text="0.0 g",
                                   font=("Arial", 36, "bold"), bg="#2f3640", fg="#4cd137")
        self.thrust_lbl.pack()

        self.torque_lbl = tk.Label(self.sidebar, text="0.000 Nm",
                                   font=("Arial", 22, "bold"), bg="#2f3640", fg="#00a8ff")
        self.torque_lbl.pack()

        self.rpm_lbl = tk.Label(self.sidebar, text="0 RPM",
                                font=("Arial", 22, "bold"), bg="#2f3640", fg="#f1c40f")
        self.rpm_lbl.pack()

        self.status_bar = tk.Label(self.sidebar, text="DISCONNECTED",
                                   font=("Arial", 12, "bold"), bg="#2f3640", fg="#e74c3c")
        self.status_bar.pack(pady=5)

        # THROTTLE SLIDER
        man_frame = tk.LabelFrame(self.sidebar, text=" Throttle Control ",
                                  bg="#2f3640", fg="white", padx=10, pady=10)
        man_frame.pack(fill="x", padx=10, pady=5)
        self.throt_val = tk.IntVar(value=0)
        self.slider = tk.Scale(man_frame, from_=100, to=0, orient="vertical", length=180,
                               variable=self.throt_val, command=self.update_manual_throttle,
                               bg="#34495e", fg="white", highlightthickness=0, troughcolor="#2c3e50")
        self.slider.pack(side="left", padx=20)

        # BUS DIAGNOSTICS
        diag_f = tk.LabelFrame(self.sidebar, text=" Power & Load Cell Raw ",
                               bg="#2f3640", fg="white")
        diag_f.pack(fill="x", padx=10, pady=5)
        self.pwr_raw  = tk.Label(diag_f, text="0.0V | 0.0A | 0.0W",
                                 bg="#2f3640", fg="#bdc3c7", font=("Courier", 10))
        self.pwr_raw.pack(anchor="w", padx=10)
        self.push_raw = tk.Label(diag_f, text="Push Raw: 0.00g",
                                 bg="#2f3640", fg="#bdc3c7", font=("Courier", 10))
        self.push_raw.pack(anchor="w", padx=10)
        self.pull_raw = tk.Label(diag_f, text="Pull Raw: 0.00g",
                                 bg="#2f3640", fg="#bdc3c7", font=("Courier", 10))
        self.pull_raw.pack(anchor="w", padx=10)

        # Latency indicator
        self.latency_lbl = tk.Label(diag_f, text="Buffer: 0 frames",
                                    bg="#2f3640", fg="#7f8c8d", font=("Courier", 9))
        self.latency_lbl.pack(anchor="w", padx=10)

        # AUTOMATED TEST
        prog_f = tk.LabelFrame(self.sidebar, text=" Automated Program ",
                               bg="#2f3640", fg="white")
        prog_f.pack(fill="x", padx=10, pady=5)
        self.file_ent = tk.Entry(prog_f)
        self.file_ent.insert(0, "DBF_Test_Run")
        self.file_ent.pack(fill="x", padx=10, pady=5)
        self.prog_sel = ttk.Combobox(prog_f, values=list(PROGRAMS.keys()), state="readonly")
        self.prog_sel.current(0)
        self.prog_sel.pack(fill="x", padx=10, pady=2)
        self.start_btn = tk.Button(prog_f, text="ARM & START TEST", command=self.arm_test,
                                   bg="#e67e22", fg="white", font=("Arial", 10, "bold"))
        self.start_btn.pack(fill="x", pady=10, padx=10)

        # SYSTEM
        sys_f = tk.LabelFrame(self.sidebar, text=" Commands ", bg="#2f3640", fg="white")
        sys_f.pack(fill="x", padx=10, pady=5)
        tk.Button(sys_f, text="TARE (ZERO)", command=self.send_zero,
                  bg="#7f8c8d", fg="white").pack(fill="x", pady=2, padx=5)

        tk.Button(self.sidebar, text="KILL MOTOR (K)", command=self.emergency_stop,
                  bg="#c0392b", fg="white", font=("Arial", 14, "bold"), height=2).pack(
                  side="bottom", fill="x", padx=10, pady=10)

        # PLOT AREA
        self.plot_panel = tk.Frame(self.root, bg="#1e272e")
        self.plot_panel.pack(side="right", fill="both", expand=True)
        self.setup_graphs()

    def setup_graphs(self):
        self.fig, (self.ax_p, self.ax_f, self.ax_qp, self.ax_ql) = plt.subplots(
            4, 1, figsize=(6, 10), facecolor='#1e272e')
        self.fig.tight_layout(pad=4.0)

        self.ln_p,  = self.ax_p.plot([], [], color="#f1c40f", label="Throttle %")
        self.ln_f,  = self.ax_f.plot([], [], color="#4cd137", label="Thrust (g)")
        self.ln_qp, = self.ax_qp.plot([], [], color="#e74c3c", label="Torque Push (Nm)")
        self.ln_ql, = self.ax_ql.plot([], [], color="#9b59b6", label="Torque Pull (Nm)")

        for ax in [self.ax_p, self.ax_f, self.ax_qp, self.ax_ql]:
            ax.set_facecolor('#2f3640')
            ax.tick_params(colors='white', labelsize=8)
            ax.grid(color='#7f8c8d', linestyle='--', alpha=0.1)
            ax.legend(loc="upper right", fontsize=7)

        self.ax_qp.axhline(0, color='white', linewidth=0.5, alpha=0.5)
        self.ax_ql.axhline(0, color='white', linewidth=0.5, alpha=0.5)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_panel)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def read_thread(self):
        """
        Reads serial as fast as possible and always keeps only the
        LATEST frame. Includes a safety voltage cutoff.
        """
        VOLTAGE_CUTOFF = 26.4
        
        while self.running:
            if not self.ser:
                time.sleep(0.05)
                continue
            try:
                line = None
                frames_this_cycle = 0

                while self.ser.in_waiting > 0:
                    candidate = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if candidate and ',' in candidate:
                        line = candidate
                        frames_this_cycle += 1

                if line is None:
                    candidate = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if candidate and ',' in candidate:
                        line = candidate
                        frames_this_cycle = 1

                if line:
                    data = line.split(",")
                    if len(data) >= 10:
                        try:
                            t  = float(data[1])  # Thrust_g
                            qp = float(data[2])  # TorquePush_Nm
                            ql = float(data[3])  # TorquePull_Nm
                            throttle_pwm = int(data[4])
                            p  = float(data[5])  # Push_g
                            l  = float(data[6])  # Pull_g
                            v  = float(data[7])  # Volts
                            a  = float(data[8])  # Amps
                            r  = float(data[9])  # RPM

                            # --- SAFETY VOLTAGE CUTOFF ---
                            # If test is active and voltage drops below threshold
                            if self.test_active and v < VOLTAGE_CUTOFF and v > 1.0: 
                                # v > 1.0 ensures we don't trigger on a disconnected sensor
                                print(f"CRITICAL: Voltage Drop Detected ({v}V). EMERGENCY STOP.")
                                self.root.after(0, self.emergency_stop)
                                self.root.after(0, messagebox.showwarning, 
                                               "Safety Cutoff", f"Test stopped! Voltage dropped to {v:.2f}V")

                            # Store latest — thread safe
                            with self.latest_lock:
                                self.latest = (t, qp, ql, r, v, a, p, l,
                                               (throttle_pwm - 1000) / 10.0,
                                               frames_this_cycle)

                            # Append to plot buffers
                            self.plot_thrust.append(t)
                            self.plot_torque_push.append(qp)
                            self.plot_torque_pull.append(ql)
                            self.plot_throttle.append((throttle_pwm - 1000) / 10.0)

                            if self.logging and self.logging_writer:
                                self.logging_writer.writerow(
                                    [datetime.now().strftime("%H:%M:%S.%f")] + data[:10])

                        except (ValueError, IndexError):
                            pass

            except serial.SerialException:
                self.ser = None
                self.root.after(0, self.status_bar.config,
                                {"text": "DISCONNECTED", "fg": "#e74c3c"})

    def update_gui(self):
        """
        Called every 50ms by tkinter's after() loop.
        Reads the latest data snapshot and updates labels + plots.
        Decoupled from read_thread so slow rendering never causes lag.
        """
        with self.latest_lock:
            snap = self.latest

        if snap:
            t, qp, ql, r, v, a, p, l, throttle_pct, frames = snap

            # Update labels
            self.thrust_lbl.config(text=f"{t:.1f} g")
            self.torque_lbl.config(text=f"Push: {qp:.4f} Nm  |  Pull: {ql:.4f} Nm")
            self.rpm_lbl.config(text=f"{int(r)} RPM")
            self.pwr_raw.config(text=f"{v:.2f}V | {a:.2f}A | {v*a:.1f}W")
            self.push_raw.config(text=f"Push: {p:.2f}g")
            self.pull_raw.config(text=f"Pull: {l:.2f}g")

            # Show how many frames were buffered (>1 means lag is building)
            color = "#2ecc71" if frames <= 2 else "#e74c3c"
            self.latency_lbl.config(text=f"Buffer: {frames} frames", fg=color)

            # Update plots
            if len(self.plot_thrust) > 1:
                x = range(len(self.plot_thrust))
                self.ln_p.set_data(x, list(self.plot_throttle))
                self.ln_f.set_data(x, list(self.plot_thrust))
                self.ln_qp.set_data(x, list(self.plot_torque_push))
                self.ln_ql.set_data(x, list(self.plot_torque_pull))

                for ax in [self.ax_p, self.ax_f, self.ax_qp, self.ax_ql]:
                    ax.relim()
                    ax.autoscale_view()

                for ax, buf in [(self.ax_qp, self.plot_torque_push),
                                (self.ax_ql, self.plot_torque_pull)]:
                    q_max = max(abs(min(buf, default=0)),
                                abs(max(buf, default=0)), 0.001)
                    ax.set_ylim(-q_max * 1.2, q_max * 1.2)

                self.canvas.draw_idle()

        self.root.after(50, self.update_gui)  # 20Hz GUI refresh

    def update_manual_throttle(self, val):
        if self.ser and not self.test_active and not self.counting_down:
            pwm = 1000 + (int(val) * 10)
            self.ser.write(f"{pwm}\n".encode())

    def arm_test(self):
        self.start_btn.config(state="disabled")
        threading.Thread(target=self.countdown_proc, daemon=True).start()

    def countdown_proc(self):
        self.counting_down = True
        for i in range(5, 0, -1):
            self.root.after(0, self.status_bar.config,
                            {"text": f"ARMING: {i}s", "fg": "#e74c3c"})
            time.sleep(1)
        self.counting_down = False
        self.root.after(0, self.status_bar.config,
                        {"text": "TEST ACTIVE", "fg": "#2ecc71"})
        self.run_auto_test()

    def run_auto_test(self):
        self.test_active = True
        fname = f"{self.file_ent.get()}_{datetime.now().strftime('%H%M%S')}.csv"
        steps = PROGRAMS[self.prog_sel.get()]
        try:
            with open(fname, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["PC_Time", "ESP_ms", "Thrust_g", "TorquePush_Nm", "TorquePull_Nm",
                                 "Throttle_PWM", "Push_g", "Pull_g", "Volts", "Amps", "RPM"])
                self.logging = True
                self.logging_writer = writer
                for thr_pct, dur in steps:
                    if not self.test_active:
                        break
                    pwm = 1000 + (thr_pct * 10)
                    if self.ser:
                        self.ser.write(f"{pwm}\n".encode())
                    time.sleep(dur)
        except Exception as e:
            print(f"Logging Error: {e}")

        self.emergency_stop()
        self.root.after(0, self.start_btn.config, {"state": "normal"})
        self.root.after(0, self.status_bar.config,
                        {"text": "SYSTEM READY", "fg": "#f1c40f"})

    def find_serial(self):
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            desc = p.description + p.hwid
            if any(x in desc for x in ["USB", "ESP32", "UART", "CP210", "CH340", "FTDI", "2303"]):
                try:
                    self.ser = serial.Serial(p.device, BAUD, timeout=0.1)
                    self.ser.reset_input_buffer()  # Flush any stale data on connect
                    self.status_bar.config(text=f"CONNECTED: {p.device}", fg="#2ecc71")
                    threading.Thread(target=self.read_thread, daemon=True).start()
                    self.root.after(100, self.update_gui)  # Start GUI update loop
                    return
                except Exception as e:
                    print(f"Failed to open {p.device}: {e}")
                    continue
        self.status_bar.config(text="NO DEVICE FOUND", fg="#e74c3c")
        self.root.after(3000, self.find_serial)  # Retry every 3s

    def send_zero(self):
        if self.ser:
            self.ser.write(b"Z\n")

    def emergency_stop(self):
        self.test_active = False
        self.logging = False
        self.logging_writer = None
        self.throt_val.set(0)
        if self.ser:
            self.ser.write(b"1000\n")

if __name__ == "__main__":
    root = tk.Tk()
    app = MotorLabGUI(root)
    root.bind('<k>', lambda e: app.emergency_stop())

    def on_closing():
        app.emergency_stop()       # Kill motor before closing
        app.running = False        # Stop read thread
        if app.ser and app.ser.is_open:
            app.ser.close()        # Release the COM port
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()