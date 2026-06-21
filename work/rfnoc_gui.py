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


APP = "./rfnoc_radio_loopback"
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

    if proc is not None:
        messagebox.showinfo("Already running", "RFNoC app is already running.")
        return

    try:
        threshold_dbfs = float(threshold.get())
        thr = int(round(32767.0 * pow(10.0, threshold_dbfs / 20.0)))
        rx_freq_hz = float(rx_freq.get()) * 1e6
        tx_freq_hz = float(tx_freq.get()) * 1e6
        rate_sps   = float(rate.get()) * 1e6
        delay_us = float(delay.get())
        delay_counts = max(0, int(round((delay_us - 14.0) / 0.01)))
        pw_us = float(pulsewidth.get())
        pw_counts = max(1, int(round(pw_us * float(rate.get()))))
    except ValueError:
        messagebox.showerror("Invalid input", "Check RX/TX frequency, rate, and threshold values.")
        return


    cmd = [
        APP,
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

    if rx_bw.get().strip():
        try:
            cmd += ["--rx-bw", str(float(rx_bw.get()) * 1e6)]
        except ValueError:
            messagebox.showerror("Invalid input", "RX BW must be a number in MHz.")
            return

    if tx_bw.get().strip():
        try:
            cmd += ["--tx-bw", str(float(tx_bw.get()) * 1e6)]
        except ValueError:
            messagebox.showerror("Invalid input", "TX BW must be a number in MHz.")
            return

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
        read_process_output(proc, "RFNoC app finished.")
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
        initialdir=os.path.expanduser("~/workarea/uhd/work"),
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
root.title("RFNoC Pulse Loopback GUI")

frame = ttk.Frame(root, padding=10)
frame.grid(row=0, column=0, sticky="nsew")

rx_freq = add_field(frame, 0, "RX freq [MHz]", "200")
tx_freq = add_field(frame, 1, "TX freq [MHz]", "200")
rx_gain    = add_field(frame, 2,  "RX gain [dB]", "10")
tx_gain    = add_field(frame, 3,  "TX gain [dB]", "60")
threshold  = add_field(frame, 4, "Threshold [dBFS]", "-20")
avg_delay =  add_field(frame, 5, "Avg start delay [samples]", "32")
delay = add_field(frame, 6, "Delay [us]", "100")
pulsewidth = add_field(frame, 7, "Pulse width [us]", "10")
spp        = add_field(frame, 8,  "SPP", "64")
rate = add_field(frame, 9, "Rate [Msps]", "60")
rx_bw = add_field(frame, 10, "RX BW [MHz]", "1")
tx_bw = add_field(frame, 11, "TX BW [MHz]", "1")
bitfile    = add_field(frame, 12,  "FPGA bitfile", "./e31x.bit")

ttk.Button(frame, text="Browse bitfile", command=browse_bitfile).grid(row=13, column=0, pady=6)
ttk.Button(frame, text="Update FPGA", command=update_fpga).grid(row=13, column=1, pady=6)

button_frame = ttk.Frame(frame)
button_frame.grid(row=14, column=0, columnspan=2, pady=10)

ttk.Button(
    button_frame,
    text="Start",
    command=start_app
).pack(side=tk.LEFT, padx=5)

ttk.Button(
    button_frame,
    text="Stop",
    command=stop_app
).pack(side=tk.LEFT, padx=5)

ttk.Button(
    button_frame,
    text="Clear Log",
    command=clear_output
).pack(side=tk.LEFT, padx=5)

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
