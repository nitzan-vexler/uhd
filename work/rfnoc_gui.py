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
import shlex
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import math
import sys

APP_PULSE = "./rfnoc_radio_loopback"
APP_CW = os.path.expanduser("~/workarea/uhd/work/cw.py")
APP_RX = "./rfnoc_rx_to_file_host"
APP_CONVERT = "./convert_samples_to_csv.py"
WORKDIR = os.path.expanduser("~/workarea/uhd/work/build")
REMOTE_PULSE_DIR = "/home/work/fixed_power"
REMOTE_PULSE_APP = "./rfnoc_radio_loopback"
REMOTE_UHD_LIB_DIR = "/home/root/app/uhd_lib"
REMOTE_PULSE_PID_FILE = "/tmp/rfnoc_gui_pulse.pid"
REMOTE_CW_DIR = "/home/work/CW"
REMOTE_CW_APP = "cw.py"
REMOTE_CW_PID_FILE = "/tmp/rfnoc_gui_cw.pid"
REMOTE_RX_DIR = "/home/work/pulse_to_file"
REMOTE_RX_APP = "./rfnoc_rx_to_file_custom"
REMOTE_RX_FILE = f"{REMOTE_RX_DIR}/usrp_samples.dat"
REMOTE_RX_PID_FILE = "/tmp/rfnoc_gui_rx.pid"
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
    download_rx_button.grid_remove()
    antenna_frame.grid_remove()


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
        antenna_frame.grid()
        rx_frame.grid()
        tx_frame.grid()
        sampling_frame.grid()
        pulse_frame.grid()
        fpga_frame.grid()

        rx_port_combo.configure(state="readonly")
        tx_port_combo.configure(state="readonly")

        rate_note.config(
            text="Recommended rate for Pulse mode: 60 MSPS"
        )


    elif selected == "CW":

        antenna_frame.grid()

        tx_frame.grid()

        sampling_frame.grid()

        # CW only uses the TX antenna.

        rx_port_combo.configure(state="disabled")

        tx_port_combo.configure(state="readonly")

        rate.set("1")

        set_field_state(spp, "disabled")

        rate_note.config(

            text="Recommended rate is 1 MSPS"

        )



    elif selected == "RX to File":

        antenna_frame.grid()

        rx_frame.grid()

        sampling_frame.grid()

        file_frame.grid()

        # RX-to-file only uses the RX antenna.

        rx_port_combo.configure(state="readonly")

        tx_port_combo.configure(state="disabled")

        file_frame.config(text="RX Output File")

        browse_output_button.config(text="Choose local output file")

        rate.set("1")

        if execution_target.get() == "USRP E312":
            download_rx_button.grid()

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

def get_antenna_settings():
    rx_port = rx_port_combo.get()
    tx_port = tx_port_combo.get()

    rx_map = {
        "TRX-A": ("0", "TX/RX"),
        "RX2-A": ("0", "RX2"),
        "TRX-B": ("1", "TX/RX"),
        "RX2-B": ("1", "RX2"),
    }

    tx_map = {
        "TRX-A": ("0", "TX/RX"),
        "TRX-B": ("1", "TX/RX"),
    }

    rx_chan, rx_ant_name = rx_map[rx_port]
    tx_chan, tx_ant_name = tx_map[tx_port]

    return rx_chan, rx_ant_name, tx_chan, tx_ant_name

def start_app():
    global proc
    global threshold_dbfs
    global start_time
    process_cwd = WORKDIR

    if proc is not None:
        messagebox.showinfo("Already running", "App is already running.")
        return
    selected_mode = mode.get()
    selected_target = execution_target.get()
    rx_chan, rx_ant_name, tx_chan, tx_ant_name = get_antenna_settings()



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
        process_cwd = WORKDIR
        cw_args = [
            "--tx-freq", str(tx_freq_hz),
            "--tx-gain", tx_gain.get(),
            "--tx-ant", tx_ant_name,
            "--tx-chan", tx_chan,
            "--rate", str(rate_sps),
        ]

        if tx_bw_hz > 0:
            cw_args += [
                "--tx-bw",
                str(tx_bw_hz),
            ]

        process_cwd = WORKDIR

        if selected_target == "Host PC":
            cmd = [
                sys.executable,
                APP_CW,
                *cw_args,
            ]

            append_output(
                "Starting CW app on Host PC:\n"
                + " ".join(cmd)
                + "\n\n"
            )


        else:

            ip = e312_ip.get().strip()

            user = e312_user.get().strip()

            if not ip:
                messagebox.showerror(

                    "USRP not found",

                    "Press Find USRP first."

                )

                return

            if not user:
                messagebox.showerror(

                    "Missing username",

                    "Enter the E312 SSH username."

                )

                return

            target = f"{user}@{ip}"

            remote_command = (
                    f"export LD_LIBRARY_PATH="
                    f"{shlex.quote(REMOTE_UHD_LIB_DIR)}:"
                    "$LD_LIBRARY_PATH"
                    " && "
                    f"cd {shlex.quote(REMOTE_CW_DIR)}"
                    " && "
                    f"echo $$ > {shlex.quote(REMOTE_CW_PID_FILE)}"
                    " && "
                    f"exec python3 {shlex.quote(REMOTE_CW_APP)} "
                    + " ".join(
                shlex.quote(str(arg))
                for arg in cw_args
            )
            )

            cmd = [

                "ssh",

                "-o", "ConnectTimeout=5",

                target,

                remote_command,

            ]

            process_cwd = WORKDIR

            append_output(

                f"Starting cw application on {target}:\n"

                f"{remote_command}\n\n"

            )
    elif selected_mode == "RX to File":
        try:
            threshold_dbfs = float(threshold.get())

            thr = int(
                  round(
                    32767.0
                      * pow(10.0, threshold_dbfs / 20.0)
                   )
            )

            thr = max(0, min(32767, thr))

        except ValueError:
            messagebox.showerror(
                "Invalid input",
                  "Threshold must be a valid dBFS value."
            )
            return

        rx_args = [
                "--freq", str(rx_freq_hz),
                "--gain", rx_gain.get(),
                "--ant", rx_ant_name,
                "--radio-chan", rx_chan,
                "--rate", str(rate_sps),
                "--threshold", str(threshold.get()),
            ]

        if rx_bw_hz > 0:
            rx_args += [
                "--bw",
                str(rx_bw_hz),
            ]

        if spp.get().strip():
            rx_args += [
                "--spp",
                spp.get(),
            ]

        process_cwd = WORKDIR

        if selected_target == "Host PC":
            cmd = [
                APP_RX,
                *rx_args,
                "--file", rx_output.get(),
            ]

            append_output(
                "Starting RX-to-File on Host PC:\n"
                + " ".join(cmd)
                + "\n\n"
            )

        else:
            ip = e312_ip.get().strip()
            user = e312_user.get().strip()

            if not ip:
                messagebox.showerror(
                    "USRP not found",
                    "Press Find USRP first."
                )
                return

            if not user:
                messagebox.showerror(
                    "Missing username",
                    "Enter the E312 SSH username."
                )
                return

            target = f"{user}@{ip}"

            remote_args = [
                *rx_args,
                "--file", REMOTE_RX_FILE,
            ]

            remote_command = (
                    f"cd {shlex.quote(REMOTE_RX_DIR)}"
                    " && "
                    f"echo $$ > {shlex.quote(REMOTE_RX_PID_FILE)}"
                    " && "
                    f"exec {REMOTE_RX_APP} "
                    + " ".join(
                shlex.quote(str(arg))
                for arg in remote_args
            )
            )

            cmd = [
                "ssh",
                "-o", "ConnectTimeout=5",
                target,
                remote_command,
            ]

            append_output(
                f"Starting RX-to-File on {target}:\n"
                f"{remote_command}\n\n"
                f"Remote output file:\n{REMOTE_RX_FILE}\n\n"
            )


        append_output(
            "Starting RX-to-File app:\n"
            + " ".join(cmd)
            + "\n\n"
        )
    elif selected_mode == "Convert to CSV":

        process_cwd = WORKDIR

        cmd = [
            sys.executable,
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

            thr = int(

                round(

                    32767.0

                    * pow(10.0, threshold_dbfs / 20.0)

                )

            )

            delay_us = float(delay.get())

            delay_counts = max(

                0,

                int(round((delay_us - 14.0) / 0.01))

            )

            pw_us = float(pulsewidth.get())

            pw_counts = max(

                1,

                int(round(pw_us * float(rate.get())))

            )


        except ValueError:

            messagebox.showerror(

                "Invalid input",

                "Check threshold, delay, and pulse width values."

            )

            return

        pulse_args = [
            "--rx-freq", str(rx_freq_hz),
            "--tx-freq", str(tx_freq_hz),

            "--rx-gain", rx_gain.get(),
            "--tx-gain", tx_gain.get(),

            "--rx-ant", rx_ant_name,
            "--tx-ant", tx_ant_name,
            "--rx-chan", rx_chan,
            "--tx-chan", tx_chan,

            "--threshold", str(thr),
            "--avg-delay", avg_delay.get(),
            "--delay", str(delay_counts),
            "--pw", str(pw_counts),
            "--spp", spp.get(),
            "--rate", str(rate_sps),
        ]

        if rx_bw_hz > 0:
            pulse_args += [

                "--rx-bw",

                str(rx_bw_hz),

            ]

        if tx_bw_hz > 0:
            pulse_args += [

                "--tx-bw",

                str(tx_bw_hz),

            ]

        if selected_target == "Host PC":

            cmd = [

                APP_PULSE,

                *pulse_args,

            ]

            process_cwd = WORKDIR

            append_output(

                "Starting Pulse app on Host PC:\n"

                + " ".join(cmd)

                + "\n\n"

            )


        else:

            ip = e312_ip.get().strip()

            user = e312_user.get().strip()

            if not ip:
                messagebox.showerror(

                    "USRP not found",

                    "Press Find USRP first."

                )

                return

            target = f"{user}@{ip}"

            remote_command = (
                    f"export LD_LIBRARY_PATH="
                    f"{shlex.quote(REMOTE_UHD_LIB_DIR)}:"
                    "$LD_LIBRARY_PATH"
                    " && "
                    f"cd {shlex.quote(REMOTE_PULSE_DIR)}"
                    " && "
                    f"echo $$ > {shlex.quote(REMOTE_PULSE_PID_FILE)}"
                    " && "
                    f"exec {REMOTE_PULSE_APP} "
                    + " ".join(
                shlex.quote(str(arg))
                for arg in pulse_args
            )
            )

            cmd = [

                "ssh",

                "-o", "ConnectTimeout=5",

                target,

                remote_command,

            ]

            process_cwd = WORKDIR

            append_output(

                f"Starting Pulse app on {target}:\n"

                f"{remote_command}\n\n"

            )

    proc = subprocess.Popen(
        cmd,
        cwd=process_cwd,
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

def find_usrp():
    append_output("Searching for USRP devices...\n")

    try:
        result = subprocess.run(
            ["uhd_find_devices"],
            capture_output=True,
            text=True,
            timeout=15,
        )

    except FileNotFoundError:
        e312_status.set("● UHD not found")
        status_label.configure(foreground="red")

        append_output(
            "Error: uhd_find_devices was not found.\n\n"
        )
        return

    except subprocess.TimeoutExpired:
        e312_status.set("● Search timeout")
        status_label.configure(foreground="red")

        append_output(
            "USRP discovery timed out.\n\n"
        )
        return

    output_text = result.stdout + result.stderr
    append_output(output_text + "\n")

    ip_matches = re.findall(
        r"\baddr:\s*(\d{1,3}(?:\.\d{1,3}){3})",
        output_text,
    )

    if not ip_matches:
        e312_ip.set("")
        e312_status.set("● No USRP found")
        status_label.configure(foreground="red")

        append_output(
            "No USRP IP address was found.\n\n"
        )
        return

    detected_ip = ip_matches[0]

    e312_ip.set(detected_ip)
    e312_status.set("● USRP found")
    status_label.configure(foreground="orange")

    append_output(
        f"USRP found at {detected_ip}\n"
        "Press Test Connection to verify SSH access.\n\n"
    )

def test_connection():
    ip = e312_ip.get().strip()
    user = e312_user.get().strip()

    if not ip:
        e312_status.set("● Find USRP first")
        status_label.configure(foreground="red")
        append_output("No E312 IP address is selected.\n\n")
        return

    target = f"{user}@{ip}"

    append_output(f"Testing SSH connection to {target}...\n")

    try:
        result = subprocess.run(
            [
                "ssh",
                "-o", "ConnectTimeout=5",
                "-o", "BatchMode=yes",
                target,
                "echo E312_CONNECTED",
            ],
            capture_output=True,
            text=True,
            timeout=10,
        )

    except FileNotFoundError:
        e312_status.set("● SSH not found")
        status_label.configure(foreground="red")
        append_output("The ssh command was not found.\n\n")
        return

    except subprocess.TimeoutExpired:
        e312_status.set("● Connection timeout")
        status_label.configure(foreground="red")
        append_output("SSH connection timed out.\n\n")
        return

    if (
        result.returncode == 0
        and "E312_CONNECTED" in result.stdout
    ):
        e312_status.set("● Connected")
        status_label.configure(foreground="green")
        append_output(f"Connected successfully to {target}.\n\n")

    else:
        e312_status.set("● Connection failed")
        status_label.configure(foreground="red")

        error_text = result.stderr.strip()

        if not error_text:
            error_text = "Unknown SSH error."

        append_output(
            f"Could not connect to {target}:\n"
            f"{error_text}\n\n"
        )
def stop_app():
    global proc

    selected_target = execution_target.get()
    selected_mode = mode.get()

    # Stop the application on the E312 first
    if selected_target == "USRP E312":
        ip = e312_ip.get().strip()
        user = e312_user.get().strip()

        if not ip or not user:
            append_output(
                "\nCannot stop remote application: "
                "missing E312 connection information.\n"
            )
            return

        target = f"{user}@{ip}"

        if selected_mode == "CW":
            remote_pid_file = REMOTE_CW_PID_FILE
            app_name = "CW"

        elif selected_mode == "RX to File":
            remote_pid_file = REMOTE_RX_PID_FILE
            app_name = "RX-to-File"

        elif selected_mode == "Pulse":
            remote_pid_file = REMOTE_PULSE_PID_FILE
            app_name = "Pulse"

        else:
            remote_pid_file = None
            app_name = selected_mode

        if remote_pid_file:
            remote_stop_command = (
                f"pid=$(cat {shlex.quote(remote_pid_file)} "
                "2>/dev/null); "
                'if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then '
                'kill -INT "$pid"; '
                "fi; "
                f"rm -f {shlex.quote(remote_pid_file)}"
            )

            append_output(
                f"\nStopping {app_name} on {target}...\n"
            )

            try:
                result = subprocess.run(
                    [
                        "ssh",
                        "-o", "ConnectTimeout=5",
                        "-o", "BatchMode=yes",
                        target,
                        remote_stop_command,
                    ],
                    capture_output=True,
                    text=True,
                    timeout=10,
                )

                if result.stdout:
                    append_output(result.stdout)

                if result.stderr:
                    append_output(result.stderr)

            except subprocess.TimeoutExpired:
                append_output(
                    "Remote stop command timed out.\n"
                )

    # Wait for the main SSH/local process to finish
    if proc is not None:
        try:
            proc.wait(timeout=3)

        except subprocess.TimeoutExpired:
            # Fallback: stop the local process group
            try:
                os.killpg(
                    os.getpgid(proc.pid),
                    signal.SIGINT,
                )
            except ProcessLookupError:
                pass

        proc = None

def download_rx_recording():
    ip = e312_ip.get().strip()
    user = e312_user.get().strip()
    local_file = rx_output.get().strip()

    if not ip:
        messagebox.showerror(
            "USRP not found",
            "Press Find USRP first."
        )
        return

    if not local_file:
        messagebox.showerror(
            "Missing destination",
            "Choose a local output file."
        )
        return

    target = f"{user}@{ip}"

    local_directory = os.path.dirname(local_file)

    if local_directory:
        os.makedirs(local_directory, exist_ok=True)

    cmd = [
        "scp",
        "-o", "ConnectTimeout=5",
        f"{target}:{REMOTE_RX_FILE}",
        local_file,
    ]

    append_output(
        "Downloading recording from E312:\n"
        + " ".join(cmd)
        + "\n\n"
    )

    def download_worker():
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=60,
            )

            if result.stdout:
                append_output(result.stdout)

            if result.stderr:
                append_output(result.stderr)

            if result.returncode == 0:
                append_output(
                    f"Recording downloaded successfully:\n"
                    f"{local_file}\n\n"
                )
            else:
                append_output(
                    "Recording download failed.\n\n"
                )

        except subprocess.TimeoutExpired:
            append_output(
                "Recording download timed out.\n\n"
            )

    threading.Thread(
        target=download_worker,
        daemon=True,
    ).start()

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
style = ttk.Style()
style.configure(".", font=("TkDefaultFont", 10))
# Left side: controls
# Right side: plot and terminal
root.columnconfigure(0, weight=1, minsize=450)
root.columnconfigure(1, weight=2, minsize=750)
root.rowconfigure(0, weight=1)

# Scrollable left panel
left_container = ttk.Frame(root)
left_container.grid(
    row=0,
    column=0,
    sticky="nsew",
)

left_container.rowconfigure(0, weight=1)
left_container.columnconfigure(0, weight=1)

left_canvas = tk.Canvas(
    left_container,
    highlightthickness=0,
)

left_scrollbar = ttk.Scrollbar(
    left_container,
    orient="vertical",
    command=left_canvas.yview,
)

left_canvas.configure(
    yscrollcommand=left_scrollbar.set
)

left_canvas.grid(
    row=0,
    column=0,
    sticky="nsew",
)

left_scrollbar.grid(
    row=0,
    column=1,
    sticky="ns",
)

frame = ttk.Frame(
    left_canvas,
    padding=10,
)

left_window = left_canvas.create_window(
    (0, 0),
    window=frame,
    anchor="nw",
)

frame.columnconfigure(0, weight=1)

def update_left_scroll_region(event=None):
    left_canvas.configure(
        scrollregion=left_canvas.bbox("all")
    )


def resize_left_frame(event):
    left_canvas.itemconfigure(
        left_window,
        width=event.width
    )
frame.bind(
    "<Configure>",
    update_left_scroll_region,
)

left_canvas.bind(
    "<Configure>",
    resize_left_frame,
)

def scroll_left_panel(event):
    left_canvas.yview_scroll(
        int(-1 * (event.delta / 120)),
        "units",
    )


left_canvas.bind_all(
    "<MouseWheel>",
    scroll_left_panel,
)

left_canvas.bind_all(
    "<Button-4>",
    lambda event: left_canvas.yview_scroll(-1, "units"),
)

left_canvas.bind_all(
    "<Button-5>",
    lambda event: left_canvas.yview_scroll(1, "units"),
)

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

e312_ip = tk.StringVar(value="")
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
e312_button_frame = ttk.Frame(e312_frame)

e312_button_frame.grid(
    row=3,
    column=0,
    columnspan=2,
    pady=5,
)

ttk.Button(
    e312_button_frame,
    text="Find USRP",
    command=find_usrp,
).pack(
    side=tk.LEFT,
    padx=5,
)

ttk.Button(
    e312_button_frame,
    text="Test Connection",
    command=test_connection,
).pack(
    side=tk.LEFT,
    padx=5,
)

# =========================================================
# Antenna selection
# =========================================================
antenna_frame = ttk.LabelFrame(
    frame,
    text="Antenna Selection",
    padding=8,
)

antenna_frame.grid(
    row=2,
    column=0,
    sticky="ew",
    pady=4,
)

antenna_frame.columnconfigure(1, weight=1)

rx_channel = tk.StringVar(value="0")
tx_channel = tk.StringVar(value="0")

rx_ant = tk.StringVar(value="RX2")
tx_ant = tk.StringVar(value="TX/RX")

ttk.Label(
    antenna_frame,
    text="RX port",
).grid(
    row=0,
    column=0,
    sticky="w",
    padx=4,
    pady=3,
)

rx_port_combo = ttk.Combobox(
    antenna_frame,
    values=[
        "TRX-A",
        "RX2-A",
        "TRX-B",
        "RX2-B",
    ],
    state="readonly",
)

rx_port_combo.set("RX2-A")

rx_port_combo.grid(
    row=0,
    column=1,
    sticky="ew",
    padx=4,
    pady=3,
)

ttk.Label(
    antenna_frame,
    text="TX port",
).grid(
    row=1,
    column=0,
    sticky="w",
    padx=4,
    pady=3,
)

tx_port_combo = ttk.Combobox(
    antenna_frame,
    values=[
        "TRX-A",
        "TRX-B",
    ],
    state="readonly",
)

tx_port_combo.set("TRX-A")

tx_port_combo.grid(
    row=1,
    column=1,
    sticky="ew",
    padx=4,
    pady=3,
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
    row=3,
    column=0,
    sticky="ew",
    pady=4
)

rx_frame.columnconfigure(1, weight=1)

rx_freq = add_field(rx_frame, 0, "RX freq [MHz]", "200")
rx_gain = add_field(rx_frame, 1, "RX gain [dB]", "10")
rx_bw = add_field(rx_frame, 2, "RX BW [MHz]", "1")
threshold = add_field(rx_frame, 3, "Threshold [dBFS]", "-30")

# =========================================================
# TX parameters
# =========================================================
tx_frame = ttk.LabelFrame(
    frame,
    text="TX Parameters",
    padding=8
)
tx_frame.grid(
    row=4,
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
    row=5,
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
    row=6,
    column=0,
    sticky="ew",
    pady=4
)

pulse_frame.columnconfigure(1, weight=1)


avg_delay = add_field(
    pulse_frame,
    0,
    "Avg start delay [samples]",
    "32"
)

delay = add_field(
    pulse_frame,
    1,
    "Delay [us]",
    "100"
)

pulsewidth = add_field(
    pulse_frame,
    2,
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
    row=7,
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
download_rx_button = ttk.Button(
    file_frame,
    text="Download from E312",
    command=download_rx_recording,
)

download_rx_button.grid(
    row=2,
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
    row=8,
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
    text="Select Fixed/Relative Power bitfile",
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
    row=9,
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
