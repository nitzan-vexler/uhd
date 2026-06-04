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

    ax.clear()

    ax.plot(
        list(tx_time),
        list(rx_dbfs_values),
        label="RX dBFS"
    )

    ax.plot(
        list(tx_time),
        list(tx_dbfs_values),
        label="TX dBFS"
    )

    if threshold_dbfs is not None:
        ax.axhline(
            y=threshold_dbfs,
            linestyle="--",
            linewidth=2,
            label=f"Threshold ({threshold_dbfs:.1f} dBFS)"
        )

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("dBFS")
    ax.set_title("RX vs TX amplitude")
    ax.grid(True)
    ax.legend()

    canvas.draw_idle()

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

    thr = float(threshold.get())
    threshold_dbfs = (
        20.0 * math.log10(thr / 32767.0)
        if thr > 0
        else -200.0
    )

    cmd = [
        APP,
        "--rx-freq", rx_freq.get(),
        "--tx-freq", tx_freq.get(),
        "--rx-gain", rx_gain.get(),
        "--tx-gain", tx_gain.get(),
        "--threshold", threshold.get(),
        "--delay", delay.get(),
        "--pw", pulsewidth.get(),
        "--spp", spp.get(),
        "--rate", rate.get(),
    ]
    

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

rx_freq    = add_field(frame, 0,  "RX freq [Hz]", "200000000")
tx_freq    = add_field(frame, 1,  "TX freq [Hz]", "200000000")
rx_gain    = add_field(frame, 2,  "RX gain [dB]", "50")
tx_gain    = add_field(frame, 3,  "TX gain [dB]", "70")
threshold  = add_field(frame, 4,  "Threshold [counts]", "1000")
delay      = add_field(frame, 5,  "Delay [CE clocks]", "8000")
pulsewidth = add_field(frame, 6,  "Pulse width [samples]", "800")
spp        = add_field(frame, 7,  "SPP", "64")
rate       = add_field(frame, 8,  "Rate [sps]", "60000000")
bitfile    = add_field(frame, 9,  "FPGA bitfile", "./e31x.bit")

ttk.Button(frame, text="Browse bitfile", command=browse_bitfile).grid(row=10, column=0, pady=6)
ttk.Button(frame, text="Update FPGA", command=update_fpga).grid(row=10, column=1, pady=6)

button_frame = ttk.Frame(frame)
button_frame.grid(row=11, column=0, columnspan=2, pady=10)

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
fig = Figure(figsize=(8, 3), dpi=100)
ax = fig.add_subplot(111)

ax.set_xlabel("Time [s]")
ax.set_ylabel("TX dBFS")
ax.set_title("TX amplitude vs time")
ax.grid(True)

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().grid(
    row=0,
    column=3,
    padx=10,
    pady=10,
    sticky="nsew"
)
root.protocol("WM_DELETE_WINDOW", lambda: (stop_app(), root.destroy()))

root.mainloop()
