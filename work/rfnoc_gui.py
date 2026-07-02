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
    for widget in frame.winfo_children():
        if isinstance(widget, ttk.Entry) and widget.cget("textvariable") == str(var):
            widget.configure(state=state)

def browse_output_file():
    filename = filedialog.asksaveasfilename(
        title="Select output file",
        defaultextension=".dat",
        filetypes=[("Binary files", "*.dat"), ("All files", "*.*")]
    )

    if filename:
        rx_output.set(filename)

def update_fields_for_mode(*args):
    selected = mode.get()

    # enable all first
    all_fields = [
        rx_freq, tx_freq, rx_gain, tx_gain,
        threshold, avg_delay, delay, pulsewidth,
        spp, rate, rx_bw, tx_bw, bitfile,
        rx_output,
    ]

    for field in all_fields:
        set_field_state(field, "normal")

    if selected == "CW":
        disabled = [
            rx_freq, rx_gain, threshold,
            avg_delay, delay, pulsewidth,
            spp, rx_bw, bitfile,
            rx_output,
        ]

    elif selected == "RX to File":
        disabled = [
            tx_freq, tx_gain,
            avg_delay, delay, pulsewidth,
            tx_bw, bitfile,
        ]

    elif selected == "Convert to CSV":
        disabled = [
            rx_freq, tx_freq,
            rx_gain, tx_gain,
            threshold,
            avg_delay,
            delay,
            pulsewidth,
            spp,
            rate,
            rx_bw,
            tx_bw,
            bitfile,
            # Don't disable rx_output
        ]

    else:  # Pulse
        disabled = [
            rx_output,
        ]

    for field in disabled:
        set_field_state(field, "disabled")

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

    selected_mode = mode.get()

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

frame = ttk.Frame(root, padding=10)
frame.grid(row=0, column=0, sticky="nsew")

# =========================
# Operation mode - TOP
# =========================
mode_frame = ttk.LabelFrame(frame, text="Operation Mode", padding=10)
mode_frame.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 10))

mode = tk.StringVar(value="Pulse")

mode_combo = ttk.Combobox(
    mode_frame,
    textvariable=mode,
    values=["Pulse", "CW", "RX to File", "Convert to CSV"],
    state="readonly",
    font=("TkDefaultFont", 13),
    width=28
)
mode_combo.pack(fill="x")

# =========================
# RF parameters
# =========================
rx_freq = add_field(frame, 1, "RX freq [MHz]", "200")
tx_freq = add_field(frame, 2, "TX freq [MHz]", "200")
rx_gain = add_field(frame, 3, "RX gain [dB]", "10")
tx_gain = add_field(frame, 4, "TX gain [dB]", "70")
threshold = add_field(frame, 5, "Threshold [dBFS]", "-30")
avg_delay = add_field(frame, 6, "Avg start delay [samples]", "32")
delay = add_field(frame, 7, "Delay [us]", "100")
pulsewidth = add_field(frame, 8, "Pulse width [us]", "10")
spp = add_field(frame, 9, "SPP", "64")
rate = add_field(frame, 10, "Rate [Msps]", "60")

ttk.Label(
    frame,
    text="Recommended: Pulse = 60 MSPS, RX to File = 1 MSPS",
    foreground="#0066CC"
).grid(row=11, column=1, sticky="w", padx=4)

rx_bw = add_field(frame, 12, "RX BW [MHz]", "1")
tx_bw = add_field(frame, 13, "TX BW [MHz]", "1")
bitfile = add_field(frame, 14, "FPGA bitfile", "./e31x.bit")
ttk.Label(
    frame,
    text="Update according to the desired pulse: Relative/Fixed Power",
    foreground="#0066CC"
).grid(row=15, column=1, sticky="w", padx=4)

rx_output = add_field(
    frame,
    17,
    "Data File",
    os.path.expanduser("~/workarea/uhd/work/build/usrp_samples.dat")
)

ttk.Button(
    frame,
    text="Browse output file",
    command=browse_output_file
).grid(row=18, column=0, columnspan=2, pady=4)

# =========================
# FPGA buttons
# =========================
ttk.Button(frame, text="Browse bitfile", command=browse_bitfile).grid(row=16, column=0, pady=6)
ttk.Button(frame, text="Update FPGA", command=update_fpga).grid(row=16, column=1, pady=6)

# =========================
# Main buttons
# =========================
button_frame = ttk.Frame(frame)
button_frame.grid(row=19, column=0, columnspan=2, pady=10)

ttk.Button(button_frame, text="Start", command=start_app).pack(side=tk.LEFT, padx=5)
ttk.Button(button_frame, text="Stop", command=stop_app).pack(side=tk.LEFT, padx=5)
ttk.Button(button_frame, text="Clear Log", command=clear_output).pack(side=tk.LEFT, padx=5)

mode.trace_add("write", update_fields_for_mode)
update_fields_for_mode()

output = tk.Text(root, width=110, height=32)
output.grid(row=1, column=0, padx=10, pady=10)
# Plot 1: long-term tracking
fig1 = Figure(figsize=(8, 3), dpi=100)
ax = fig1.add_subplot(111)

ax.set_xlabel("Time [s]")
ax.set_ylabel("dBFS")
ax.set_title("RX/TX amplitude tracking")
ax.grid(True)

canvas1 = FigureCanvasTkAgg(fig1, master=root)
canvas1.get_tk_widget().grid(
    row=0,
    column=3,
    padx=10,
    pady=10,
    sticky="nsew"
)


root.protocol("WM_DELETE_WINDOW", lambda: (stop_app(), root.destroy()))

root.mainloop()
