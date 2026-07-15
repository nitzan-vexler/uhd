
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: cw
# GNU Radio version: 3.8.1.0

from gnuradio import analog
from gnuradio import gr
from gnuradio import uhd

import argparse
import signal
import sys
import time


class cw(gr.top_block):

    def __init__(self, tx_freq, tx_gain, samp_rate, tx_bw):
        gr.top_block.__init__(self, "cw")

        ##################################################
        # Variables
        ##################################################
        self.tx_freq = tx_freq
        self.tx_gain = tx_gain
        self.samp_rate = samp_rate
        self.tx_bw = tx_bw

        ##################################################
        # Blocks
        ##################################################
        self.uhd_usrp_sink_0 = uhd.usrp_sink(
            ",".join(("", "")),
            uhd.stream_args(
                cpu_format="fc32",
                args="",
                channels=[0],
            ),
            "",
        )

        self.uhd_usrp_sink_0.set_center_freq(self.tx_freq, 0)
        self.uhd_usrp_sink_0.set_gain(self.tx_gain, 0)
        self.uhd_usrp_sink_0.set_antenna("TX/RX", 0)

        if self.tx_bw > 0:
            self.uhd_usrp_sink_0.set_bandwidth(self.tx_bw, 0)

        self.uhd_usrp_sink_0.set_samp_rate(self.samp_rate)

        # Optional for continuous CW. Can be removed if PPS messages are unwanted.
        self.uhd_usrp_sink_0.set_time_unknown_pps(uhd.time_spec())

        # Keep this exactly as in the working cw.py.
        # Frequency 0 produces a constant baseband value and therefore RF CW
        # at the configured center frequency.
        self.gr_sig_source_x_0_1_2_1_0 = analog.sig_source_c(
            self.samp_rate,
            analog.GR_CONST_WAVE,
            0,
            1.0,
            0.0,
        )
        actual_gain = self.uhd_usrp_sink_0.get_gain(0)
        print(f"Actual TX gain:    {actual_gain} dB", flush=True) 
        print(f"TX frequency: {tx_freq / 1e6:.6f} MHz", flush=True)
        print(f"TX gain:      {tx_gain} dB", flush=True)
        print(f"Sample rate:  {samp_rate / 1e6:.6f} MSPS", flush=True)
        print(f"TX bandwidth: {tx_bw / 1e6:.6f} MHz", flush=True)
        
        

        ##################################################
        # Connections
        ##################################################
        self.connect(
            (self.gr_sig_source_x_0_1_2_1_0, 0),
            (self.uhd_usrp_sink_0, 0),
        )

    def get_tx_gain(self):
        return self.tx_gain

    def set_tx_gain(self, tx_gain):
        self.tx_gain = tx_gain
        self.uhd_usrp_sink_0.set_gain(self.tx_gain, 0)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.gr_sig_source_x_0_1_2_1_0.set_sampling_freq(
            self.samp_rate
        )
        self.uhd_usrp_sink_0.set_samp_rate(self.samp_rate)


def main():
    parser = argparse.ArgumentParser(
        description="Transmit continuous-wave RF using a USRP."
    )

    parser.add_argument(
        "--tx-freq",
        type=float,
        required=True,
        help="TX center frequency in Hz",
    )

    parser.add_argument(
        "--tx-gain",
        type=float,
        default=70,
        help="TX gain in dB",
    )

    parser.add_argument(
        "--rate",
        type=float,
        default=100e3,
        help="Sample rate in samples per second",
    )

    parser.add_argument(
        "--tx-bw",
        type=float,
        default=10e3,
        help="TX analog bandwidth in Hz. Use 0 to leave unchanged.",
    )

    args = parser.parse_args()

    tb = cw(
        tx_freq=args.tx_freq,
        tx_gain=args.tx_gain,
        samp_rate=args.rate,
        tx_bw=args.tx_bw,
    )

    def sig_handler(sig=None, frame=None):
        print("\nStopping CW transmission...")
        tb.stop()
        tb.wait()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)
    print("Python:", sys.version, flush=True)
    print("GNU Radio:", gr.version(), flush=True)
    print("UHD:", uhd.get_version_string(), flush=True)

    tb.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        sig_handler()


if __name__ == "__main__":
    main()


