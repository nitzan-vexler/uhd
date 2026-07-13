#!/usr/bin/env python3

import os
import signal
import subprocess
import threading
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import re
import time
from collections import deque

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import math
import sys

APP_PULSE = "./rfnoc_radio_loopback"
APP_CW = os.path.expanduser("~/workarea/uhd/work/cw_tx.py")
APP_RX = "./rfnoc_rx_to_file_host"
APP_CONVERT = "./convert_samples_to_csv.py"
WORKDIR = os.path.expanduser("~/workarea/uhd/work/build")

proc = None

tx_time = deque(maxlen=200)
rx_dbfs_values = deque(maxlen=200)
tx_dbfs_values = deque(maxlen=200)
start_time = None
threshold_dbfs = None



def update_plot_from_line(line):
    global start_time
    global threshold_dbfs

    m_rx = re.search(r"RX_dBFS\s*=\s*(-?\d+(?:\.\d+)?)", line)
    m_tx = re.search(r"TX_dBFS\s*=\s*(-?\d+(?:\.\d+)?)", line)

    if not (m_rx and m_tx):
        return

    if start_time is None:
        start_time = time.time()

    t = time.time() - start_time

    rx_dbfs = float(m_rx.group(1))
    tx_dbfs = float(m_tx.group(1))

    tx_time.append(t)
    rx_dbfs_values.append(rx_dbfs)
    tx_dbfs_values.append(tx_dbfs)

    # Plot 1: tracking over time
    ax.clear()

    ax.plot(list(tx_time), list(rx_dbfs_values), label="RX dBFS")
    ax.plot(list(tx_time), list(tx_dbfs_values), label="TX dBFS")

 #   if threshold_dbfs is not None:
 #       ax.axhline(
 #           y=threshold_dbfs,
 #           linestyle="--",
 #           linewidth=2,
 #           label=f"Threshold ({threshold_dbfs:.1f} dBFS)"
 #       )

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("dBFS")
    ax.set_title("RX vs TX amplitude tracking")
    ax.grid(True)
    ax.legend()



    canvas1.draw_idle()


def add_field(parent, row, label, default):
    ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", padx=4, pady=3)
    var = tk.StringVar(value=default)
    ttk.Entry(parent, textvariable=var, width=24).grid(row=row, column=1, sticky="ew", padx=4, pady=3)
    return var
def set_field_state(var, state):
    def search(parent):
        for widget in parent.winfo_children():
            if (
                isinstance(widget, ttk.Entry)
                and widget.cget("textvariable") == str(var)
            ):
                widget.configure(state=state)
                return True

            if search(widget):
                return True

        return False

    search(frame)

def browse_output_file():
    selected_mode = mode.get()

    if selected_mode == "Convert to CSV":
        filename = filedialog.askopenfilename(
            title="Select binary file to convert",
            initialdir=os.path.dirname(rx_output.get()),
            filetypes=[
                ("Binary files", "*.dat"),
                ("All files", "*.*"),
            ],
        )
    else:
        filename = filedialog.asksaveasfilename(
            title="Select output file",
            initialdir=os.path.dirname(rx_output.get()),
            initialfile=os.path.basename(rx_output.get()),
            defaultextension=".dat",
            filetypes=[
                ("Binary files", "*.dat"),
                ("All files", "*.*"),
            ],
        )

    if filename:
        rx_output.set(filename)

def update_fields_for_mode(*args):
    selected = mode.get()
    selected_target = execution_target.get()

    # Hide all conditional sections first
    e312_frame.grid_remove()
    rx_frame.grid_remove()
    tx_frame.grid_remove()
    sampling_frame.grid_remove()
    pulse_frame.grid_remove()
    file_frame.grid_remove()
    fpga_frame.grid_remove()

    # Show the E312 connection panel only for the E312 target
    if selected_target == "USRP E312":
        e312_frame.grid()

    # Restore SPP state by default
    set_field_state(spp, "normal")

    # Hide all mode-dependent sections first
    rx_frame.grid_remove()
    tx_frame.grid_remove()
    sampling_frame.grid_remove()
    pulse_frame.grid_remove()
    file_frame.grid_remove()
    fpga_frame.grid_remove()

    # Restore SPP state by default
    set_field_state(spp, "normal")

    if selected == "Pulse":
        rx_frame.grid()
        tx_frame.grid()
        sampling_frame.grid()
        pulse_frame.grid()
        fpga_frame.grid()

        rate_note.config(
            text="Recommended rate for Pulse mode: 60 MSPS"
        )

    elif selected == "CW":
        tx_frame.grid()
        sampling_frame.grid()

        # CW uses rate but does not use SPP
        set_field_state(spp, "disabled")

        rate_note.config(
            text="CW uses TX frequency, TX gain, TX BW and sample rate"
        )

    elif selected == "RX to File":
        rx_frame.grid()
        sampling_frame.grid()
        file_frame.grid()

        file_frame.config(text="RX Output File")
        browse_output_button.config(text="Choose output file")

        rate.set("1")

        rate_note.config(
            text="Recommended rate for RX to File: 1 MSPS"
        )

    elif selected == "Convert to CSV":
        file_frame.grid()

        file_frame.config(text="Binary File to Convert")
        browse_output_button.config(text="Choose input .dat file")

def append_output(text):
    output.insert(tk.END, text)
    output.see(tk.END)


def read_process_output(p, finished_msg):
    for line in p.stdout:
        append_output(line)
        update_plot_from_line(line)

    rc = p.wait()
    append_output(f"\n{finished_msg} Return code: {rc}\n\n")

def start_app():
    global proc
    global threshold_dbfs
    global start_time

    if proc is not None:
        messagebox.showinfo("Already running", "App is already running.")
        return
    selected_mode = mode.get()
    selected_target = execution_target.get()

    if selected_target == "USRP E312":
        messagebox.showinfo(
            "USRP E312 target",
            "Remote execution on the E312 will be added in the next step."
        )
        return

    try:
        rx_freq_hz = float(rx_freq.get()) * 1e6
        tx_freq_hz = float(tx_freq.get()) * 1e6
        rate_sps   = float(rate.get()) * 1e6

        tx_bw_hz = 0.0
        rx_bw_hz = 0.0

        if tx_bw.get().strip():
            tx_bw_hz = float(tx_bw.get()) * 1e6

        if rx_bw.get().strip():
            rx_bw_hz = float(rx_bw.get()) * 1e6

    except ValueError:
        messagebox.showerror("Invalid input", "Check frequency, rate, and BW values.")
        return

    tx_time.clear()
    rx_dbfs_values.clear()
    tx_dbfs_values.clear()
    start_time = None


    if selected_mode == "CW":
        cmd = [
            APP_CW,
            "--tx-freq", str(tx_freq_hz),
            "--tx-gain", tx_gain.get(),
            "--rate", str(rate_sps),
        ]

        if tx_bw_hz > 0:
            cmd += ["--tx-bw", str(tx_bw_hz)]

        append_output("Starting CW app:\n" + " ".join(cmd) + "\n\n")
    elif selected_mode == "RX to File":

        cmd = [
            APP_RX,
            "--freq", str(rx_freq_hz),
            "--gain", rx_gain.get(),
            "--rate", str(rate_sps),
            "--threshold", threshold.get(),
            "--file", rx_output.get(),
        ]

        if rx_bw_hz > 0:
            cmd += ["--bw", str(rx_bw_hz)]

        if spp.get().strip():
            cmd += ["--spp", spp.get()]

        append_output(
            "Starting RX-to-File app:\n"
            + " ".join(cmd)
            + "\n\n"
        )
    elif selected_mode == "Convert to CSV":
        cmd = [
            "python3",
            APP_CONVERT,
            "--input",
            rx_output.get(),
        ]

        append_output(
            "Starting binary-to-CSV converter:\n"
            + " ".join(cmd)
            + "\n\n"
        )

    else:
        try:
            threshold_dbfs = float(threshold.get())
            thr = int(round(32767.0 * pow(10.0, threshold_dbfs / 20.0)))

            delay_us = float(delay.get())
            delay_counts = max(0, int(round((delay_us - 14.0) / 0.01)))

            pw_us = float(pulsewidth.get())
            pw_counts = max(1, int(round(pw_us * float(rate.get()))))

        except ValueError:
            messagebox.showerror("Invalid input", "Check threshold, delay, and pulse width values.")
            return

        cmd = [
            APP_PULSE,
            "--rx-freq", str(rx_freq_hz),
            "--tx-freq", str(tx_freq_hz),
            "--rx-gain", rx_gain.get(),
            "--tx-gain", tx_gain.get(),
            "--threshold", str(thr),
            "--avg-delay", avg_delay.get(),
            "--delay", str(delay_counts),
            "--pw", str(pw_counts),
            "--spp", spp.get(),
            "--rate", str(rate_sps),
        ]

        if rx_bw_hz > 0:
            cmd += ["--rx-bw", str(rx_bw_hz)]

        if tx_bw_hz > 0:
            cmd += ["--tx-bw", str(tx_bw_hz)]

        append_output("Starting RFNoC app:\n" + " ".join(cmd) + "\n\n")

    proc = subprocess.Popen(
        cmd,
        cwd=WORKDIR,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        preexec_fn=os.setsid,
    )

    def monitor():
        global proc

        if selected_mode == "Pulse":
            read_process_output(proc, "RFNoC app finished.")

        elif selected_mode == "CW":
            read_process_output(proc, "CW app finished.")

        elif selected_mode == "RX to File":
            read_process_output(proc, "RX-to-File app finished.")

        elif selected_mode == "Convert to CSV":
            read_process_output(proc, "CSV conversion finished.")

            output_dir = os.path.join(WORKDIR, "csv_output")

            try:
                if sys.platform.startswith("linux"):
                    subprocess.Popen(["xdg-open", output_dir])
            except Exception:
                pass

        proc = None

    threading.Thread(target=monitor, daemon=True).start()
def clear_output():
    output.delete("1.0", tk.END)
def test_connection():
    target = f"{e312_user.get().strip()}@{e312_ip.get().strip()}"

    append_output(f"Testing connection to {target}...\n")

    # Temporary behavior; real SSH test will be added next
    e312_status.set("Connected")
    status_label.configure(foreground="green")

    append_output(f"Connected to {target}\n\n")
def stop_app():
    global proc

    if proc is not None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            append_output("\nStopping RFNoC app...\n")
        except ProcessLookupError:
            pass
        proc = None


def browse_bitfile():
    path = filedialog.askopenfilename(
        title="Select FPGA bitstream",
        initialdir=os.path.expanduser("/home/nitzanv/bitstreams"),
        filetypes=[("Bitstream files", "*.bit"), ("All files", "*.*")]
    )

    if path:
        bitfile.set(path)


def update_fpga():
    path = bitfile.get().strip()

    if not path:
        messagebox.showerror("Missing bitstream", "Please select a .bit file first.")
        return

    if not os.path.exists(path):
        messagebox.showerror("File not found", f"Bitstream file not found:\n{path}")
        return

    if proc is not None:
        messagebox.showwarning(
            "RFNoC app running",
            "Stop the RFNoC app before updating the FPGA."
        )
        return

    cmd = [
        "uhd_image_loader",
        "--args", "type=e3xx",
        "--fpga-path", path,
    ]

    append_output("Updating FPGA:\n" + " ".join(cmd) + "\n\n")

    p = subprocess.Popen(
        cmd,
        cwd=os.path.dirname(path),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    threading.Thread(
        target=read_process_output,
        args=(p, "FPGA update finished."),
        daemon=True,
    ).start()

root = tk.Tk()
root.title("SDR GUI")

# Left side: controls
# Right side: plot and terminal
root.columnconfigure(0, weight=3, minsize=800)
root.columnconfigure(1, weight=2, minsize=600)
root.rowconfigure(0, weight=1)

frame = ttk.Frame(root, padding=10)
frame.grid(
    row=0,
    column=0,
    sticky="nsew",
)

frame.columnconfigure(0, weight=1)

right_frame = ttk.Frame(root, padding=(0, 10, 10, 10))
right_frame.grid(
    row=0,
    column=1,
    sticky="nsew",
)

right_frame.columnconfigure(0, weight=1)

# Plot keeps a compact height.
# Terminal receives the extra vertical space.
right_frame.rowconfigure(0, weight=0)
right_frame.rowconfigure(1, weight=1)


# =========================================================
# Operation mode
# =========================================================
mode_frame = ttk.LabelFrame(
    frame,
    text="Operation Mode",
    padding=12
)
mode_frame.grid(
    row=0,
    column=0,
    sticky="ew",
    pady=(0, 10)
)

mode_frame.columnconfigure(0, weight=1)

mode = tk.StringVar(value="Pulse")
execution_target = tk.StringVar(value="Host PC")

mode_combo = ttk.Combobox(
    mode_frame,
    textvariable=mode,
    values=[
        "Pulse",
        "CW",
        "RX to File",
        "Convert to CSV",
    ],
    state="readonly",
    font=("TkDefaultFont", 14),
    justify="center",
)
mode_combo.grid(
    row=0,
    column=0,
    sticky="ew",
    padx=5,
    pady=5,
)
ttk.Label(
    mode_frame,
    text="Execution Target",
).grid(
    row=1,
    column=0,
    sticky="w",
    padx=5,
    pady=(8, 2),
)

target_combo = ttk.Combobox(
    mode_frame,
    textvariable=execution_target,
    values=[
        "Host PC",
        "USRP E312",
    ],
    state="readonly",
    font=("TkDefaultFont", 12),
    justify="center",
)

target_combo.grid(
    row=2,
    column=0,
    sticky="ew",
    padx=5,
    pady=(2, 5),
)
# =========================================================
# E312 Connection
# =========================================================
e312_frame = ttk.LabelFrame(
    frame,
    text="USRP E312",
    padding=8,
)

e312_frame.columnconfigure(1, weight=1)
e312_frame.grid(
    row=1,
    column=0,
    sticky="ew",
    pady=4,
)

e312_ip = tk.StringVar(value="192.168.10.2")
e312_user = tk.StringVar(value="root")
e312_status = tk.StringVar(value="Disconnected")
ttk.Label(
    e312_frame,
    text="IP Address",
).grid(
    row=0,
    column=0,
    sticky="w",
)

ttk.Entry(
    e312_frame,
    textvariable=e312_ip,
).grid(
    row=0,
    column=1,
    sticky="ew",
    padx=5,
)
ttk.Label(
    e312_frame,
    text="Username",
).grid(
    row=1,
    column=0,
    sticky="w",
)

ttk.Entry(
    e312_frame,
    textvariable=e312_user,
).grid(
    row=1,
    column=1,
    sticky="ew",
    padx=5,
)
ttk.Label(
    e312_frame,
    text="Status",
).grid(
    row=2,
    column=0,
    sticky="w",
)

status_label = ttk.Label(
    e312_frame,
    textvariable=e312_status,
    foreground="red",
)

status_label.grid(
    row=2,
    column=1,
    sticky="w",
)
ttk.Button(
    e312_frame,
    text="Test Connection",
    command=test_connection,
).grid(
    row=3,
    column=0,
    columnspan=2,
    pady=5,
)
# =========================================================
# RX parameters
# =========================================================
rx_frame = ttk.LabelFrame(
    frame,
    text="RX Parameters",
    padding=8
)
rx_frame.grid(
    row=2,
    column=0,
    sticky="ew",
    pady=4
)

rx_frame.columnconfigure(1, weight=1)

rx_freq = add_field(rx_frame, 0, "RX freq [MHz]", "200")
rx_gain = add_field(rx_frame, 1, "RX gain [dB]", "10")
rx_bw = add_field(rx_frame, 2, "RX BW [MHz]", "1")


# =========================================================
# TX parameters
# =========================================================
tx_frame = ttk.LabelFrame(
    frame,
    text="TX Parameters",
    padding=8
)
tx_frame.grid(
    row=3,
    column=0,
    sticky="ew",
    pady=4
)

tx_frame.columnconfigure(1, weight=1)

tx_freq = add_field(tx_frame, 0, "TX freq [MHz]", "200")
tx_gain = add_field(tx_frame, 1, "TX gain [dB]", "70")
tx_bw = add_field(tx_frame, 2, "TX BW [MHz]", "1")


# =========================================================
# Sampling parameters
# =========================================================
sampling_frame = ttk.LabelFrame(
    frame,
    text="Sampling Parameters",
    padding=8
)
sampling_frame.grid(
    row=4,
    column=0,
    sticky="ew",
    pady=4
)

sampling_frame.columnconfigure(1, weight=1)

rate = add_field(sampling_frame, 0, "Rate [MSPS]", "60")
spp = add_field(sampling_frame, 1, "SPP", "64")

rate_note = ttk.Label(
    sampling_frame,
    text="Recommended rate for Pulse mode: 60 MSPS",
    foreground="#0066CC",
)
rate_note.grid(
    row=2,
    column=0,
    columnspan=2,
    sticky="w",
    padx=4,
    pady=(2, 4),
)


# =========================================================
# Pulse parameters
# =========================================================
pulse_frame = ttk.LabelFrame(
    frame,
    text="Pulse Parameters",
    padding=8
)
pulse_frame.grid(
    row=5,
    column=0,
    sticky="ew",
    pady=4
)

pulse_frame.columnconfigure(1, weight=1)

threshold = add_field(
    pulse_frame,
    0,
    "Threshold [dBFS]",
    "-30"
)

avg_delay = add_field(
    pulse_frame,
    1,
    "Avg start delay [samples]",
    "32"
)

delay = add_field(
    pulse_frame,
    2,
    "Delay [us]",
    "100"
)

pulsewidth = add_field(
    pulse_frame,
    3,
    "Pulse width [us]",
    "10"
)


# =========================================================
# File parameters
# =========================================================
file_frame = ttk.LabelFrame(
    frame,
    text="RX Output File",
    padding=8
)
file_frame.grid(
    row=6,
    column=0,
    sticky="ew",
    pady=4
)

file_frame.columnconfigure(1, weight=1)

rx_output = add_field(
    file_frame,
    0,
    "File path",
    os.path.expanduser(
        "~/workarea/uhd/work/build/usrp_samples.dat"
    ),
)

browse_output_button = ttk.Button(
    file_frame,
    text="Choose output file",
    command=browse_output_file,
)
browse_output_button.grid(
    row=1,
    column=0,
    columnspan=2,
    pady=5,
)


# =========================================================
# FPGA configuration
# =========================================================
fpga_frame = ttk.LabelFrame(
    frame,
    text="FPGA Configuration",
    padding=8
)
fpga_frame.grid(
    row=7,
    column=0,
    sticky="ew",
    pady=4
)

fpga_frame.columnconfigure(1, weight=1)

bitfile = add_field(
    fpga_frame,
    0,
    "FPGA bitfile",
    "./e31x.bit"
)

ttk.Label(
    fpga_frame,
    text="Select the bitfile according to Relative or Fixed Power mode",
    foreground="#0066CC",
).grid(
    row=1,
    column=0,
    columnspan=2,
    sticky="w",
    padx=4,
    pady=3,
)

fpga_button_frame = ttk.Frame(fpga_frame)
fpga_button_frame.grid(
    row=2,
    column=0,
    columnspan=2,
    pady=5,
)

ttk.Button(
    fpga_button_frame,
    text="Browse bitfile",
    command=browse_bitfile,
).pack(
    side=tk.LEFT,
    padx=5,
)

ttk.Button(
    fpga_button_frame,
    text="Update FPGA",
    command=update_fpga,
).pack(
    side=tk.LEFT,
    padx=5,
)


# =========================================================
# Main controls
# =========================================================
button_frame = ttk.Frame(frame)
button_frame.grid(
    row=8,
    column=0,
    sticky="ew",
    pady=10,
)

button_frame.columnconfigure((0, 1, 2), weight=1)

ttk.Button(
    button_frame,
    text="Start",
    command=start_app,
).grid(
    row=0,
    column=0,
    padx=5,
    sticky="ew",
)

ttk.Button(
    button_frame,
    text="Stop",
    command=stop_app,
).grid(
    row=0,
    column=1,
    padx=5,
    sticky="ew",
)

ttk.Button(
    button_frame,
    text="Clear Log",
    command=clear_output,
).grid(
    row=0,
    column=2,
    padx=5,
    sticky="ew",
)


# Update visible sections whenever the mode changes
mode.trace_add("write", update_fields_for_mode)
execution_target.trace_add("write", update_fields_for_mode)

update_fields_for_mode()


# =========================================================
# Plot - upper right
# =========================================================
plot_frame = ttk.LabelFrame(
    right_frame,
    text="RX/TX Amplitude Tracking",
    padding=5,
)
plot_frame.grid(
    row=0,
    column=0,
    sticky="ew",
    pady=(0, 8),
)

plot_frame.columnconfigure(0, weight=1)

fig1 = Figure(figsize=(7, 4), dpi=100)
ax = fig1.add_subplot(111)

ax.set_xlabel("Time [s]")
ax.set_ylabel("dBFS")
ax.grid(True)

canvas1 = FigureCanvasTkAgg(
    fig1,
    master=plot_frame,
)

canvas1.get_tk_widget().grid(
    row=0,
    column=0,
    sticky="ew",
)


# =========================================================
# Terminal - lower right
# =========================================================
log_frame = ttk.LabelFrame(
    right_frame,
    text="Application Log",
    padding=5,
)
log_frame.grid(
    row=1,
    column=0,
    sticky="nsew",
)

log_frame.columnconfigure(0, weight=1)
log_frame.rowconfigure(0, weight=1)

output = tk.Text(
    log_frame,
    width=75,
    height=20,
    wrap="none",
)

output.grid(
    row=0,
    column=0,
    sticky="nsew",
)


root.protocol("WM_DELETE_WINDOW", lambda: (stop_app(), root.destroy()))

root.mainloop()
