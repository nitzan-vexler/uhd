#!/usr/bin/env python3
"""Combined Mode + binary measurement parser GUI for UHD RFNoC SigGen."""
import os
import json
import queue
import re
import shlex
import signal
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from collections import deque
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from combined_parser import convert

WORKDIR = os.path.expanduser('~/workarea/uhd/work/build')
HOST_APP = './rfnoc_radio_loopback'
REMOTE_DIR = '/home/work/fixed_power'
REMOTE_APP = './rfnoc_radio_loopback'
REMOTE_LIB = '/home/root/app/uhd_lib'
REMOTE_PID = '/tmp/rfnoc_gui_pulse.pid'
REMOTE_FILE = REMOTE_DIR + '/measurements.dat'

class CombinedGUI:
    def __init__(self, root):
        self.root = root
        root.title('RFNoC — Combined Mode & Measurement Parser')
        root.geometry('1360x900')
        root.minsize(1050, 680)
        self.bg = '#0b1220'
        self.surface = '#131f31'
        self.field = '#1b2a40'
        self.fg = '#eaf1fa'
        self.muted = '#9cb0c8'
        self.accent = '#4bb8ff'
        root.configure(bg=self.bg)
        style = ttk.Style(root)
        style.theme_use('clam')
        style.configure('.', font=('DejaVu Sans', 10), background=self.bg, foreground=self.fg)
        style.configure('TFrame', background=self.bg)
        style.configure('Metric.TFrame',background=self.surface)
        style.configure('MetricName.TLabel',background=self.surface,foreground=self.muted,font=('DejaVu Sans',9,'bold'))
        style.configure('MetricValue.TLabel',background=self.surface,foreground=self.accent,font=('DejaVu Sans',17,'bold'))
        style.configure('Card.TLabelframe', background=self.surface, bordercolor='#293c55',
                        relief='solid', borderwidth=1)
        style.configure('Card.TLabelframe.Label', background=self.surface,
                        foreground=self.accent, font=('DejaVu Sans', 10, 'bold'))
        style.configure('Card.TLabel', background=self.surface, foreground=self.fg)
        style.configure('Muted.TLabel', background=self.surface, foreground=self.muted)
        style.configure('Hero.TLabel', background=self.bg, foreground=self.fg,
                        font=('DejaVu Sans', 19, 'bold'))
        style.configure('Subtitle.TLabel', background=self.bg, foreground=self.muted)
        style.configure('TEntry', fieldbackground=self.field, foreground=self.fg,
                        insertcolor=self.fg, bordercolor='#344962', padding=6)
        style.configure('TCombobox', fieldbackground=self.field, foreground=self.fg,
                        background=self.field, arrowcolor=self.accent, padding=5)
        style.map('TCombobox', fieldbackground=[('readonly', self.field)],
                  foreground=[('readonly', self.fg)])
        style.configure('TCheckbutton', background=self.surface, foreground=self.fg)
        style.map('TCheckbutton', background=[('active', self.surface)])
        style.configure('TButton', background='#263a55', foreground=self.fg,
                        padding=(12, 8), borderwidth=0, font=('DejaVu Sans', 10, 'bold'))
        style.map('TButton', background=[('active', '#365477')])
        style.configure('Accent.TButton', background='#1684d8', foreground='#ffffff')
        style.map('Accent.TButton', background=[('active', '#319eec')])
        style.configure('Danger.TButton', background='#82364c', foreground='#ffffff')
        style.map('Danger.TButton', background=[('active', '#a64b63')])
        style.configure('TNotebook', background=self.bg, borderwidth=0)
        style.configure('TNotebook.Tab', background=self.surface, foreground=self.muted,
                        padding=(20, 11), font=('DejaVu Sans', 10, 'bold'))
        style.map('TNotebook.Tab', background=[('selected', '#234c70')],
                  foreground=[('selected', '#ffffff')])
        root.option_add('*TCombobox*Listbox.background', self.field)
        root.option_add('*TCombobox*Listbox.foreground', self.fg)
        self.proc = None
        self.run_target = None
        self.messages = queue.Queue()
        self.x = deque(maxlen=250)
        self.y = deque(maxlen=250)
        self.pulse_count = 0
        self.metric_vars = {}
        self.last_seq = None
        self.gps_state = 'WAITING'
        self.gps_test_state = 'CHECKING'
        self._gps_check_running = False
        self._gps_check_generation = 0
        self.sync_test_state = 'Disconnected'
        self.session_state = 'IDLE'
        self.v = {}
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)
        header = ttk.Frame(root, padding=(22, 17, 22, 8))
        header.grid(row=0, column=0, sticky='ew')
        ttk.Label(header, text='RFNoC  /  CONTROL CENTER', style='Hero.TLabel').pack(anchor='w')
        ttk.Label(header, text='Combined Mode · Pulse detection, retransmission & GPS measurements',
                  style='Subtitle.TLabel').pack(anchor='w', pady=(4, 0))
        root.rowconfigure(0, weight=0)
        root.rowconfigure(1, weight=1)
        self.tabs = ttk.Notebook(root)
        self.tabs.grid(row=1, column=0, sticky='nsew', padx=14, pady=(4, 14))
        combined = ttk.Frame(self.tabs, padding=8)
        parser = ttk.Frame(self.tabs, padding=14)
        self.tabs.add(combined, text='Combined Mode')
        self.tabs.add(parser, text='Measurement Parser')
        combined.columnconfigure(1, weight=1)
        combined.rowconfigure(0, weight=1)
        left_outer = ttk.Frame(combined)
        left_outer.grid(row=0, column=0, sticky='ns', padx=(0, 10))
        left_canvas = tk.Canvas(left_outer, width=435, highlightthickness=0, bg=self.bg)
        scrollbar = ttk.Scrollbar(left_outer, orient='vertical', command=left_canvas.yview)
        left_canvas.configure(yscrollcommand=scrollbar.set)
        left_canvas.pack(side='left', fill='y', expand=True)
        scrollbar.pack(side='right', fill='y')
        left = ttk.Frame(left_canvas, padding=5)
        window = left_canvas.create_window((0,0), window=left, anchor='nw')
        left.bind('<Configure>', lambda e: left_canvas.configure(scrollregion=left_canvas.bbox('all')))
        left_canvas.bind('<Configure>', lambda e: left_canvas.itemconfigure(window, width=e.width))
        def group(title):
            f = ttk.LabelFrame(left, text=title, padding=12, style='Card.TLabelframe')
            f.pack(fill='x', pady=4)
            f.columnconfigure(1, weight=1)
            return f
        # A predictable row index per group (avoids fragile widget-count arithmetic).
        def fields(f, specs):
            for r,(label,key,default) in enumerate(specs):
                ttk.Label(f,text=label,style='Card.TLabel').grid(row=r,column=0,sticky='w',padx=4,pady=3)
                self.v[key]=tk.StringVar(value=str(default))
                ttk.Entry(f,textvariable=self.v[key],width=22).grid(row=r,column=1,sticky='ew',padx=4,pady=3)
        f=group('Execution')
        ttk.Label(f,text='Target',style='Card.TLabel').grid(row=0,column=0,sticky='w')
        self.target=tk.StringVar(value='Host PC')
        ttk.Combobox(f,textvariable=self.target,values=['Host PC','USRP E312'],state='readonly').grid(row=0,column=1,sticky='ew')
        self.target.trace_add('write',lambda *_: self.toggle_remote())
        f=group('RX Parameters')
        fields(f,[('RX frequency [MHz]','rx_freq','200'),('RX gain [dB]','rx_gain','10'),('RX BW [MHz]','rx_bw','1'),('Threshold [dBFS]','threshold','-30')])
        f=group('TX Parameters')
        fields(f,[('TX frequency [MHz]','tx_freq','200'),('TX gain [dB]','tx_gain','70'),('TX BW [MHz]','tx_bw','1')])
        f=group('Sampling')
        fields(f,[('Rate [MSPS]','rate','60'),('Samples per packet','spp','64')])
        f=group('Pulse Parameters')
        fields(f,[('Avg start delay [samples]','avg_delay','32'),('Delay [us]','delay','100'),('Pulse width [us]','pw','10'),('Pulse gap [us]','gap','100')])
        f=group('Measurement Recording')
        self.save=tk.BooleanVar(value=True); self.gps=tk.BooleanVar(value=True)
        ttk.Checkbutton(f,text='Save measurements',variable=self.save).grid(row=0,column=0,columnspan=2,sticky='w')
        ttk.Checkbutton(f,text='GPSD + GPS/PPS UTC sync (--gps)',variable=self.gps).grid(row=1,column=0,columnspan=2,sticky='w')
        self.v['output']=tk.StringVar(value=os.path.join(WORKDIR,'combined_measurements.dat'))
        ttk.Entry(f,textvariable=self.v['output']).grid(row=2,column=0,columnspan=2,sticky='ew')
        ttk.Button(f,text='Choose file',command=lambda:self.pick_save(self.v['output'])).grid(row=3,column=0)
        ttk.Button(f,text='Download E312 file',command=self.download).grid(row=3,column=1)
        f=group('Controls')
        for c,(label,cmd) in enumerate([('Start',self.start),('Stop',self.stop),('Clear Log',self.clear)]):
            ttk.Button(f,text=label,command=cmd,style=('Accent.TButton' if label=='Start' else 'Danger.TButton' if label=='Stop' else 'TButton')).grid(row=0,column=c,padx=3,sticky='ew')
            f.columnconfigure(c,weight=1)
        right=ttk.Frame(combined)
        right.grid(row=0,column=1,sticky='nsew')
        right.columnconfigure(0,weight=1)
        right.columnconfigure(1,weight=0)
        right.rowconfigure(2,weight=1)
        dashboard=ttk.Frame(right, padding=(0,0,0,10))
        dashboard.grid(row=0,column=0,sticky='ew')
        for c in range(3): dashboard.columnconfigure(c,weight=1)
        sidebar=ttk.Frame(right, width=390)
        sidebar.grid(row=0, column=1, rowspan=3, sticky='ns', padx=(10, 0))
        sidebar.columnconfigure(0,weight=1)
        for i,(key,title,initial) in enumerate([
            ('state','ACQUISITION','IDLE'),
            ('freq','RX FREQUENCY','200 MHz'),
            ('power','RX POWER','— dBFS'),
            ('pri','PULSE PRI','— µs'),
            ('count','PULSES','0'),
            ('gps','GPS FIX','WAITING'),
        ]):
            tile=ttk.Frame(dashboard,style='Metric.TFrame',padding=(14,12))
            tile.grid(row=i//3,column=i%3,sticky='nsew',padx=4,pady=4)
            ttk.Label(tile,text=title,style='MetricName.TLabel').pack(anchor='w')
            var=tk.StringVar(value=initial)
            ttk.Label(tile,textvariable=var,style='MetricValue.TLabel').pack(anchor='w',pady=(5,0))
            self.metric_vars[key]=var


        # Compact live GPS telemetry row, populated only by actual GPSD/C++ data.
        gps_details = ttk.Frame(dashboard)
        gps_details.grid(row=2, column=0, columnspan=3, sticky='ew', pady=(2, 0))
        for c in range(4):
            gps_details.columnconfigure(c, weight=1)
        for c, (key, title) in enumerate((
            ('latitude', 'LATITUDE'), ('longitude', 'LONGITUDE'),
            ('altitude', 'ALTITUDE'), ('gps_time', 'GPS UTC TIME')
        )):
            tile = ttk.Frame(gps_details, style='Metric.TFrame', padding=(10, 6))
            tile.grid(row=0, column=c, sticky='ew', padx=4)
            ttk.Label(tile, text=title, style='MetricName.TLabel').pack(anchor='w')
            value = tk.StringVar(value='—')
            ttk.Label(tile, textvariable=value, style='MetricName.TLabel',
                      font=('DejaVu Sans', 11, 'bold')).pack(anchor='w', pady=(3, 0))
            self.metric_vars[key] = value

        f=ttk.LabelFrame(sidebar, text='ANTENNA SELECTION / PORT STATUS', padding=10, style='Card.TLabelframe')
        f.grid(row=0, column=0, sticky='ew', pady=(0, 8))
        f.columnconfigure(1, weight=1)
        self.rxport=tk.StringVar(value='RX2-A'); self.txport=tk.StringVar(value='TRX-A')
        for r,(label,var,choices) in enumerate([('RX port',self.rxport,['TRX-A','RX2-A','TRX-B','RX2-B']),('TX port',self.txport,['TRX-A','TRX-B'])]):
            ttk.Label(f,text=label,style='Card.TLabel').grid(row=r,column=0,sticky='w')
            ttk.Combobox(f,textvariable=var,values=choices,state='readonly').grid(row=r,column=1,sticky='ew')
        # Visual antenna port selection (configuration, not hardware link status).
        ttk.Label(f, text='Selected antenna ports', style='Card.TLabel').grid(
            row=2, column=0, columnspan=2, sticky='w', pady=(12, 4))
        self.port_canvas = tk.Canvas(
            f, height=164, bg=self.surface, highlightthickness=0, bd=0)
        self.port_canvas.grid(row=3, column=0, columnspan=2, sticky='ew')
        self.port_canvas.bind('<Configure>', lambda _event: self.update_port_leds())
        self.rxport.trace_add('write', lambda *_: self.update_port_leds())
        self.txport.trace_add('write', lambda *_: self.update_port_leds())
        self.update_port_leds()
        self.gps_test_label = tk.StringVar(value='GPS: CHECKING AUTOMATICALLY...')
        ttk.Label(f, textvariable=self.gps_test_label, style='Card.TLabel').grid(
            row=4, column=0, columnspan=2, sticky='w', pady=(5, 2))
        ttk.Button(f, text='Test GPS Antenna (Mode 3)',
                   command=self.test_gps_antenna).grid(
            row=5, column=0, columnspan=2, sticky='ew', pady=(4, 2))

        # Connection tools directly below antenna indicators, in both execution modes.
        self.remote=ttk.LabelFrame(sidebar, text='USRP CONNECTION', padding=10, style='Card.TLabelframe')
        self.remote.grid(row=1, column=0, sticky='ew', pady=(0, 8))
        self.remote.columnconfigure(1, weight=1)
        fields(self.remote,[('IP address','ip',''),('Username','user','root')])
        ttk.Button(self.remote,text='Find USRP',command=self.find_usrp).grid(row=2,column=0,pady=5)
        ttk.Button(self.remote,text='Test SSH',command=self.test_ssh).grid(row=2,column=1,pady=5)
        self.usrp_connection_label = tk.StringVar(value='USRP: NOT CHECKED')
        connection_status = ttk.Frame(self.remote)
        connection_status.grid(row=3, column=0, columnspan=2, sticky='w', pady=(7, 2))
        self.usrp_connection_led = tk.Canvas(connection_status, width=22, height=22,
                                             bg=self.surface, highlightthickness=0)
        self.usrp_connection_led.pack(side='left', padx=(0, 7))
        self.usrp_connection_led_dot = self.usrp_connection_led.create_oval(
            3, 3, 19, 19, fill='#657286', outline='')
        ttk.Label(connection_status, textvariable=self.usrp_connection_label,
                  style='Card.TLabel').pack(side='left')
        self._usrp_check_running = False
        self._usrp_check_generation = 0
        self.v['ip'].trace_add('write', self._usrp_ip_changed)
        self.v['user'].trace_add('write', self._gps_identity_changed)

        # FPGA controls at the bottom of the right-hand stack.
        fpga=ttk.LabelFrame(sidebar, text='FPGA', padding=10, style='Card.TLabelframe')
        fpga.grid(row=2, column=0, sticky='ew')
        fpga.columnconfigure(1, weight=1)
        self.v['bitfile']=tk.StringVar(value='./e31x.bit')
        ttk.Entry(fpga,textvariable=self.v['bitfile']).grid(row=0,column=0,columnspan=2,sticky='ew')
        ttk.Button(fpga,text='Browse .bit',command=self.pick_bit).grid(row=1,column=0)
        ttk.Button(fpga,text='Update FPGA',command=self.update_fpga).grid(row=1,column=1)

        plot_frame=ttk.LabelFrame(right,text='LIVE  /  RX AMPLITUDE',padding=12,style='Card.TLabelframe')
        plot_frame.grid(row=1,column=0,sticky='ew')
        self.fig=Figure(figsize=(6,2.1),dpi=100)
        self.ax=self.fig.add_subplot(111)
        self.fig.patch.set_facecolor(self.surface)
        self.ax.set_facecolor(self.surface)
        self.ax.set_xlabel('Pulse number', color=self.muted)
        self.ax.set_ylabel('RX dBFS', color=self.muted)
        self.ax.tick_params(colors=self.muted)
        for spine in self.ax.spines.values(): spine.set_color('#3b526c')
        self.ax.grid(True, color='#344962', alpha=0.55)
        self.fig.tight_layout(pad=2)
        self.canvas=FigureCanvasTkAgg(self.fig,master=plot_frame)
        self.canvas.get_tk_widget().pack(fill='x')
        log_frame=ttk.LabelFrame(right,text='APPLICATION LOG',padding=10,style='Card.TLabelframe')
        log_frame.grid(row=2,column=0,sticky='nsew',pady=8)
        log_frame.rowconfigure(0,weight=1);log_frame.columnconfigure(0,weight=1)
        self.log=tk.Text(log_frame,wrap='word',height=10, bg='#0a1422', fg='#d9e9f8', insertbackground='#ffffff', selectbackground='#245b87', relief='flat', padx=14, pady=12, font=('DejaVu Sans Mono', 10))
        self.log.grid(sticky='nsew')
        log_scroll=ttk.Scrollbar(log_frame,command=self.log.yview)
        log_scroll.grid(row=0,column=1,sticky='ns');self.log.configure(yscrollcommand=log_scroll.set)
        self.make_parser(parser)
        self.toggle_remote()
        root.after(80,self.drain_messages)
        root.after(200,self.auto_check_usrp_connection)
        root.after(400,self.auto_check_gps_antenna)
        root.protocol('WM_DELETE_WINDOW',self.close)

    def make_parser(self, frame):
        frame.columnconfigure(0, weight=0)
        frame.columnconfigure(1, weight=1)
        frame.columnconfigure(2, weight=0)
        self.parser_in = tk.StringVar()
        self.parser_out = tk.StringVar()

        panel = ttk.LabelFrame(frame, text='Convert Measurements to CSV',
                               padding=20, style='Card.TLabelframe')
        panel.grid(row=0, column=0, columnspan=3, sticky='ew', padx=20, pady=20)
        panel.columnconfigure(1, weight=1)

        ttk.Label(panel, text='Input binary file (.dat)', style='Card.TLabel').grid(
            row=0, column=0, sticky='w', padx=(0, 12), pady=10)
        ttk.Entry(panel, textvariable=self.parser_in, width=55).grid(
            row=0, column=1, sticky='ew', pady=10)
        ttk.Button(panel, text='Browse…', command=lambda: self.pick_open(self.parser_in)).grid(
            row=0, column=2, padx=(10, 0), pady=10)

        ttk.Label(panel, text='Output CSV file', style='Card.TLabel').grid(
            row=1, column=0, sticky='w', padx=(0, 12), pady=10)
        ttk.Entry(panel, textvariable=self.parser_out, width=55).grid(
            row=1, column=1, sticky='ew', pady=10)
        ttk.Button(panel, text='Browse…', command=lambda: self.pick_save(self.parser_out, '.csv')).grid(
            row=1, column=2, padx=(10, 0), pady=10)

        ttk.Button(panel, text='Convert to CSV', command=self.parse_file,
                   style='Accent.TButton').grid(row=2, column=1, sticky='e', pady=(15, 6))
        self.parse_status = tk.StringVar(value='Ready')
        ttk.Label(panel, textvariable=self.parse_status, style='Card.TLabel').grid(
            row=3, column=0, columnspan=3, sticky='w', pady=(8, 0))

    def pick_open(self,var):
        path=filedialog.askopenfilename(filetypes=[('Binary measurements','*.dat'),('All files','*')]);
        if path:
            var.set(path)
            if var is self.parser_in and not self.parser_out.get():self.parser_out.set(os.path.splitext(path)[0]+'.csv')
    def pick_save(self,var,ext='.dat'):
        path=filedialog.asksaveasfilename(defaultextension=ext,filetypes=[('Files','*'+ext),('All files','*')]);
        if path:var.set(path)
    def pick_bit(self):
        path=filedialog.askopenfilename(filetypes=[('Bitstream','*.bit'),('All files','*')]);
        if path:self.v['bitfile'].set(path)
    def update_port_leds(self):
        """Show selected RX (green), TX (red), or unused (grey) ports.

        TRX-A/B can be selected for both directions; in that case the
        indicator is split into green and red halves.
        """
        canvas = self.port_canvas
        canvas.delete('all')
        ports = ('TRX-A', 'RX2-A', 'RX2-B', 'TRX-B')
        width = max(canvas.winfo_width(), 350)
        spacing = width / len(ports)
        rx, tx = self.rxport.get(), self.txport.get()
        for idx, port in enumerate(ports):
            x = (idx + 0.5) * spacing
            y, r = 22, 12
            is_rx, is_tx = (port == rx), (port == tx)
            if is_rx and is_tx:
                canvas.create_arc(x-r, y-r, x+r, y+r, start=90, extent=180,
                                  fill='#35d07f', outline='')
                canvas.create_arc(x-r, y-r, x+r, y+r, start=270, extent=180,
                                  fill='#f05a65', outline='')
                state = 'RX + TX'
            else:
                color = '#35d07f' if is_rx else '#f05a65' if is_tx else '#657286'
                canvas.create_oval(x-r, y-r, x+r, y+r, fill=color,
                                   outline='#9da9ba' if not (is_rx or is_tx) else color,
                                   width=1)
                state = 'RX' if is_rx else 'TX' if is_tx else 'Disabled'
            canvas.create_text(x, 50, text=port, fill=self.fg,
                               font=('DejaVu Sans', 10, 'bold'))
            canvas.create_text(x, 69, text=state,
                               fill=self.muted, font=('DejaVu Sans', 9))
        # Lower connectors are GPS on the left and SYNC on the right.
        for x, label, status in ((width * .25, 'GPS', self.gps_test_state),
                                 (width * .75, 'SYNC', self.sync_test_state)):
            active = (label == 'GPS' and status == 'MODE 3')
            color = '#35d07f' if active else '#657286'
            canvas.create_oval(x-11, 96, x+11, 118, fill=color, outline=color)
            canvas.create_text(x, 132, text=label, fill=self.fg,
                               font=('DejaVu Sans', 10, 'bold'))
            canvas.create_text(x, 151, text=status, fill=self.muted,
                               font=('DejaVu Sans', 9))

    def set_gps_details(self, fix):
        """Show actual 3D-fix coordinates and GPS UTC time, never placeholders as fixes."""
        if not fix or fix.get('mode') != 3:
            for key in ('latitude', 'longitude', 'altitude', 'gps_time'):
                self.metric_vars[key].set('—')
            return
        for key, source, suffix, digits in (
            ('latitude', 'lat', '°', 6),
            ('longitude', 'lon', '°', 6),
            ('altitude', 'alt', ' m', 1)
        ):
            value = fix.get(source)
            self.metric_vars[key].set(
                f'{value:.{digits}f}{suffix}' if isinstance(value, (int, float)) else '—')
        utc = fix.get('time')
        self.metric_vars['gps_time'].set(str(utc).replace('T', ' ').replace('Z', ' UTC')
                                         if utc else '—')

    def _run_gps_check(self, manual=False):
        """Check GPSD independently of Find USRP, in a background thread."""
        if self._gps_check_running:
            if manual:
                self.emit('[GPS-TEST] A GPS check is already running.\n')
            return
        self._gps_check_running = True
        generation = self._gps_check_generation
        ip = self.v['ip'].get().strip()
        user = self.v['user'].get().strip() or 'root'
        if manual:
            self.gps_test_label.set('GPS: TESTING MODE 3...')
            self.emit('[GPS-TEST] Checking GPSD for a 3D fix...\n')

        def work():
            state, label = 'UNAVAILABLE', 'GPS: GPSD / SSH UNAVAILABLE'
            best_fix = None
            details = ''
            try:
                # Discover the address automatically; Find USRP is not required.
                target_ip = ip
                if not target_ip:
                    discovery = subprocess.run(['uhd_find_devices'],
                        capture_output=True, text=True, timeout=8)
                    match = re.search(r'\baddr:\s*(\d{1,3}(?:\.\d{1,3}){3})',
                                      discovery.stdout + discovery.stderr)
                    if match:
                        target_ip = match.group(1)
                if not target_ip:
                    label = 'GPS: USRP NOT DISCOVERED'
                else:
                    cmd = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
                           '-o', 'StrictHostKeyChecking=accept-new',
                           f'{user}@{target_ip}', 'gpspipe -w -n 12']
                    result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
                    fixes = []
                    for line in result.stdout.splitlines():
                        try:
                            obj = json.loads(line)
                        except json.JSONDecodeError:
                            continue
                        if obj.get('class') == 'TPV':
                            fixes.append(obj)
                    valid = any(fix.get('mode') == 3 and
                                isinstance(fix.get('lat'), (int, float)) and
                                isinstance(fix.get('lon'), (int, float))
                                for fix in fixes)
                    if valid:
                        best_fix = next(fix for fix in reversed(fixes)
                                        if fix.get('mode') == 3
                                        and isinstance(fix.get('lat'), (int, float))
                                        and isinstance(fix.get('lon'), (int, float)))
                        state, label = 'MODE 3', 'GPS: MODE 3 (3D FIX)'
                    elif fixes:
                        state, label = 'NO FIX', 'GPS: NO MODE 3 FIX'
                    if manual:
                        details = (f'[GPS-TEST] {label} (USRP {target_ip})\n'
                                   if result.returncode == 0 or fixes else
                                   f'[GPS-TEST] {label}: {result.stderr.strip()}\n')
            except (OSError, subprocess.TimeoutExpired) as exc:
                if manual:
                    details = f'[GPS-TEST] GPS check failed: {exc}\n'
            finally:
                self.messages.put(('gps_test', (generation, state, label, best_fix)))
                if details:
                    self.messages.put(('log', details))
                self.messages.put(('gps_check_finished', None))

        threading.Thread(target=work, daemon=True).start()

    def test_gps_antenna(self):
        """Manual on-demand test, in addition to automatic GPS monitoring."""
        self._run_gps_check(manual=True)

    def auto_check_gps_antenna(self):
        """Start on GUI launch and repeat every 20 seconds."""
        self._run_gps_check()
        self.root.after(20000, self.auto_check_gps_antenna)

    def toggle_remote(self):
        # Connection tools are permanently visible in the right sidebar.
        pass
    def emit(self,s):self.messages.put(('log',s))
    def drain_messages(self):
        try:
            while True:
                kind,payload=self.messages.get_nowait()
                if kind=='log':
                    self.log.insert('end',payload);self.log.see('end')
                    self.update_instrument_metrics(payload)
                    match=re.search(r'RX_dBFS\s*=\s*(-?\d+(?:\.\d+)?)',payload)
                    if match:
                        self.x.append(self.pulse_count);self.y.append(float(match.group(1)))
                        if self.pulse_count%5==0:self.refresh_plot()
                elif kind=='usrp_connection':
                    generation, connected, label = payload
                    if generation == self._usrp_check_generation:
                        self.set_usrp_connection(connected, label)
                elif kind=='usrp_check_finished':
                    self._usrp_check_running = False
                elif kind=='gps_test':
                    generation, state, label, fix = payload
                    if generation == self._gps_check_generation:
                        self.gps_test_state = state
                        self.gps_test_label.set(label)
                        self.set_gps_details(fix)
                        self.update_port_leds()
                elif kind=='gps_check_finished':
                    self._gps_check_running = False
                elif kind=='done':
                    self.proc=None
                    self.metric_vars['state'].set('IDLE' if payload==0 else 'STOPPED')
                    self.emit(f'\nProcess finished (exit code {payload}).\n')
                elif kind=='parse':self.parse_status.set(payload)
        except queue.Empty:pass
        self.root.after(80,self.drain_messages)
    def update_instrument_metrics(self, line):
        # The physical SYNC connector is unused. GPS/PPS software sync
        # must not light its LED; UTC status is shown in GPS metrics.
        # Values come from the running C++ process; no fabricated telemetry.
        m=re.search(r'RX_dBFS\s*=\s*(-?\d+(?:\.\d+)?)',line)
        if m:self.metric_vars['power'].set(f'{float(m.group(1)):.2f} dBFS')
        m=re.search(r'\bSEQ=(\d+)',line)
        if m:
            seq=int(m.group(1))
            if self.last_seq is None:
                self.pulse_count=1
            else:
                self.pulse_count+=max(1,(seq-self.last_seq)&0xffff)
            self.last_seq=seq
            self.metric_vars['count'].set(f'{self.pulse_count:,}')
        m=re.search(r'\bPRI_US=(-?\d+(?:\.\d+)?)',line)
        if m:self.metric_vars['pri'].set(f'{float(m.group(1)):.2f} µs')
        if 'GPS_FIX_UTC=' in line:
            data = re.search(
                r'GPS_FIX_UTC=(.*?)\\s+LAT=([^\\s]+)\\s+LON=([^\\s]+)\\s+ALT=([^\\s]+)',
                line)
            if data and self.gps_test_state == 'MODE 3':
                try:
                    utc, lat, lon, alt = data.groups()
                    self.set_gps_details({'mode': 3, 'time': utc,
                                          'lat': float(lat), 'lon': float(lon),
                                          'alt': float(alt)})
                except ValueError:
                    pass
            m=re.search(r'GPS_FIX_UTC=(.*?)(?:\s+LAT=|$)',line)
            if m:
                fix=m.group(1).strip()
                self.metric_vars['gps'].set('NO FIX' if fix in ('No GPS Time','') else 'FIX RECEIVED')
        elif '[GPS-SYNC] VERIFIED:' in line:
            self.metric_vars['gps'].set('UTC SYNCED')
        elif 'GPS-SYNC' in line and ('timeout' in line.lower() or 'mismatch' in line.lower()):
            self.metric_vars['gps'].set('SYNC ERROR')
        elif '[GPS] Failed' in line:
            self.metric_vars['gps'].set('GPSD ERROR')

    def refresh_plot(self):
        self.ax.clear()
        self.ax.set_facecolor(self.surface)
        self.ax.plot(list(self.x),list(self.y),color=self.accent,linewidth=1.8,label='RX dBFS')
        self.ax.set_xlabel('Pulse number',color=self.muted)
        self.ax.set_ylabel('RX dBFS',color=self.muted)
        self.ax.tick_params(colors=self.muted)
        for spine in self.ax.spines.values(): spine.set_color('#3b526c')
        self.ax.grid(True,color='#344962',alpha=0.55)
        if self.x:
            self.ax.legend(loc='upper right',facecolor=self.surface,edgecolor='#344962',labelcolor=self.fg)
        self.canvas.draw_idle()
    def clear(self):self.log.delete('1.0','end')
    def run_command(self,cmd,cwd=WORKDIR):
        if self.proc and self.proc.poll() is None:
            messagebox.showwarning('Running','Stop the current process first.');return
        self.emit('$ '+' '.join(shlex.quote(str(x)) for x in cmd)+'\n')
        try:
            self.proc=subprocess.Popen(cmd,cwd=cwd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1,start_new_session=True)
        except Exception as e:
            self.proc=None;messagebox.showerror('Launch failed',str(e));return
        self.metric_vars['state'].set('RUNNING')
        process=self.proc
        def worker():
            for line in process.stdout:self.emit(line)
            self.messages.put(('done',process.wait()))
        threading.Thread(target=worker,daemon=True).start()
    def build_args(self):
        v=lambda k:self.v[k].get().strip()
        rate=float(v('rate'));threshold=float(v('threshold'))
        if rate<=0:raise ValueError('Rate must be positive')
        thr=max(0,min(32767,round(32767*10**(threshold/20))))
        # Retain original GUI conversion: 100 MHz CE clock, 14 us fixed delay compensation.
        delay_counts=max(0,round((float(v('delay'))-14.0)*100))
        pw_counts=max(1,round(float(v('pw'))*rate))
        gap_counts=max(0,round(float(v('gap'))*100))
        rxmap={'TRX-A':('0','TX/RX'),'RX2-A':('0','RX2'),'TRX-B':('1','TX/RX'),'RX2-B':('1','RX2')}
        txmap={'TRX-A':('0','TX/RX'),'TRX-B':('1','TX/RX')}
        rxchan,rxant=rxmap[self.rxport.get()];txchan,txant=txmap[self.txport.get()]
        args=['--rx-freq',str(float(v('rx_freq'))*1e6),'--tx-freq',str(float(v('tx_freq'))*1e6),
              '--rx-gain',v('rx_gain'),'--tx-gain',v('tx_gain'),'--rx-ant',rxant,'--tx-ant',txant,
              '--rx-chan',rxchan,'--tx-chan',txchan,'--threshold',str(thr),'--avg-delay',v('avg_delay'),
              '--delay',str(delay_counts),'--pw',str(pw_counts),'--pulse-gap',str(gap_counts),
              '--spp',v('spp'),'--rate',str(rate*1e6)]
        for key,option in [('rx_bw','--rx-bw'),('tx_bw','--tx-bw')]:
            if v(key):args.extend([option,str(float(v(key))*1e6)])
        if self.gps.get():args.extend(['--gps', '--pps', 'gpsdo', '--ref', 'internal'])
        if self.save.get():args.extend(['--save-data','--output-file',v('output') if self.target.get()=='Host PC' else REMOTE_FILE])
        return args
    def start(self):
        try:args=self.build_args()
        except Exception as e:messagebox.showerror('Invalid parameters',str(e));return
        self.x.clear();self.y.clear();self.pulse_count=0;self.refresh_plot()
        self.last_seq=None
        self.metric_vars['count'].set('0')
        self.metric_vars['power'].set('— dBFS')
        self.metric_vars['pri'].set('— µs')
        self.metric_vars['gps'].set('WAITING')
        self.sync_test_state = 'Disconnected'
        self.update_port_leds()
        self.metric_vars['freq'].set(f"{self.v['rx_freq'].get().strip()} MHz")
        self.run_target=self.target.get()
        if self.run_target=='Host PC':self.run_command([HOST_APP,*args])
        else:
            ip=self.v['ip'].get().strip();user=self.v['user'].get().strip()
            if not ip or not user:messagebox.showerror('SSH','Enter USRP IP and username');return
            command=(f'export LD_LIBRARY_PATH={shlex.quote(REMOTE_LIB)}:$LD_LIBRARY_PATH && '
                     f'cd {shlex.quote(REMOTE_DIR)} && echo $$ > {shlex.quote(REMOTE_PID)} && '
                     f'exec {REMOTE_APP} '+' '.join(shlex.quote(x) for x in args))
            self.run_command(['ssh','-o','ConnectTimeout=5',f'{user}@{ip}',command])
    def stop(self):
        process=self.proc
        if process is None or process.poll() is not None:return
        self.metric_vars['state'].set('STOPPING')
        if self.run_target=='USRP E312':
            ip=self.v['ip'].get().strip();user=self.v['user'].get().strip()
            command=(f'pid=$(cat {shlex.quote(REMOTE_PID)} 2>/dev/null); '
                     'if [ -n "$pid" ]; then kill -INT "$pid" 2>/dev/null; fi; '
                     f'rm -f {shlex.quote(REMOTE_PID)}')
            threading.Thread(target=lambda:self.remote_stop(user,ip,command),daemon=True).start()
        else:
            try:os.killpg(os.getpgid(process.pid),signal.SIGINT)
            except ProcessLookupError:pass
    def remote_stop(self,user,ip,command):
        try:
            r=subprocess.run(['ssh','-o','ConnectTimeout=5',f'{user}@{ip}',command],capture_output=True,text=True,timeout=10)
            self.emit(r.stdout+r.stderr)
        except Exception as e:self.emit(f'Remote stop failed: {e}\n')
    def set_usrp_connection(self, connected, label):
        # This LED represents UHD device discovery, not SSH or GPS/PPS lock.
        self.usrp_connection_led.itemconfigure(
            self.usrp_connection_led_dot, fill='#35d07f' if connected else '#657286')
        self.usrp_connection_label.set(label)

    def _gps_identity_changed(self, *_):
        self._gps_check_generation += 1
        self.gps_test_state = 'CHECKING'
        self.gps_test_label.set('GPS: CHECKING...')
        self.update_port_leds()

    def _usrp_ip_changed(self, *_):
        # Discard a result from a check started for the previous IP.
        self._usrp_check_generation += 1
        self.set_usrp_connection(False, 'USRP: CHECKING...')
        self._gps_check_generation += 1
        self.gps_test_state = 'CHECKING'
        self.gps_test_label.set('GPS: CHECKING...')
        self.update_port_leds()

    def auto_check_usrp_connection(self):
        """Check UHD discovery on startup and every 5 seconds, without blocking Tk."""
        if not self._usrp_check_running:
            self._usrp_check_running = True
            generation = self._usrp_check_generation
            ip = self.v['ip'].get().strip()
            # Preserve the last LED state while the next periodic check runs.

            def work():
                try:
                    cmd = ['uhd_find_devices']
                    if ip:
                        cmd += ['--args', f'addr={ip}']
                    r = subprocess.run(cmd, capture_output=True, text=True, timeout=4)
                    output = r.stdout + r.stderr
                    found = r.returncode == 0 and bool(re.search(
                        r'(?m)^\s*(?:addr|serial|type):\s*\S+', output))
                    label = 'USRP: CONNECTED' if found else 'USRP: DISCONNECTED'
                except (OSError, subprocess.TimeoutExpired):
                    found, label = False, 'USRP: DISCONNECTED'
                self.messages.put(('usrp_connection', (generation, found, label)))
                self.messages.put(('usrp_check_finished', None))

            threading.Thread(target=work, daemon=True).start()
        self.root.after(5000, self.auto_check_usrp_connection)

    def find_usrp(self):
        self.set_usrp_connection(False, 'USRP: CHECKING...')
        generation = self._usrp_check_generation
        def work():
            try:
                r = subprocess.run(['uhd_find_devices'], capture_output=True,
                                   text=True, timeout=15)
                output = r.stdout + r.stderr
                self.emit(output)
                match = re.search(r'\baddr:\s*(\d{1,3}(?:\.\d{1,3}){3})', output)
                found = r.returncode == 0 and bool(re.search(
                    r'(?m)^\s*(?:addr|serial|type):\s*\S+', output))
                if match:
                    self.root.after(0, lambda addr=match.group(1): self.v['ip'].set(addr))
                self.messages.put(('usrp_connection',
                    (generation, found, 'USRP: CONNECTED' if found else 'USRP: DISCONNECTED')))
            except Exception as e:
                self.messages.put(('usrp_connection', (generation, False, 'USRP: DISCONNECTED')))
                self.emit(f'Discovery failed: {e}\n')
        threading.Thread(target=work, daemon=True).start()
    def test_ssh(self):
        user=self.v['user'].get().strip();ip=self.v['ip'].get().strip()
        if not user or not ip:messagebox.showerror('SSH','Enter IP and username');return
        def work():
            try:
                r=subprocess.run(['ssh','-o','ConnectTimeout=5','-o','BatchMode=yes',f'{user}@{ip}','echo CONNECTED'],capture_output=True,text=True,timeout=10)
                self.emit(f'SSH test: {r.stdout}{r.stderr}Exit {r.returncode}\n')
            except Exception as e:self.emit(f'SSH test failed: {e}\n')
        threading.Thread(target=work,daemon=True).start()
    def download(self):
        user=self.v['user'].get().strip();ip=self.v['ip'].get().strip();dest=self.v['output'].get().strip()
        if not user or not ip or not dest:messagebox.showerror('Download','Enter IP, username and local file');return
        def work():
            try:
                os.makedirs(os.path.dirname(os.path.abspath(dest)),exist_ok=True)
                r=subprocess.run(['scp','-o','ConnectTimeout=5',f'{user}@{ip}:{REMOTE_FILE}',dest],capture_output=True,text=True,timeout=120)
                self.emit(f'Download exit {r.returncode}: {r.stdout}{r.stderr}\n')
            except Exception as e:self.emit(f'Download failed: {e}\n')
        threading.Thread(target=work,daemon=True).start()
    def update_fpga(self):
        path=self.v['bitfile'].get().strip()
        if self.proc and self.proc.poll() is None:messagebox.showerror('FPGA','Stop acquisition first');return
        if not os.path.isfile(path):messagebox.showerror('FPGA','Bitstream not found');return
        if not messagebox.askyesno('FPGA update','Load selected bitstream using uhd_image_loader?'):return
        self.run_command(['uhd_image_loader','--args','type=e3xx','--fpga-path',os.path.abspath(path)],cwd=WORKDIR)
    def parse_file(self):
        src=self.parser_in.get().strip();dest=self.parser_out.get().strip()
        if not src or not dest:messagebox.showerror('Parser','Select input and output files');return
        if os.path.abspath(src)==os.path.abspath(dest):messagebox.showerror('Parser','Input and output must differ');return
        self.parse_status.set('Converting...')
        def work():
            try:
                n=convert(src,dest)
                self.messages.put(('parse',f'Success: {n} pulse records written to {dest}'))
            except Exception as e:self.messages.put(('parse',f'Error: {e}'))
        threading.Thread(target=work,daemon=True).start()
    def close(self):
        if self.proc and self.proc.poll() is None:
            if not messagebox.askyesno('Exit','Acquisition is running. Stop and exit?'):return
            self.stop()
        self.root.destroy()

if __name__=='__main__':
    root=tk.Tk();CombinedGUI(root);root.mainloop()
