#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from gnuradio import analog, gr, uhd
import argparse
import signal
import sys
import time


class cw(gr.top_block):
    def __init__(self, tx_freq, tx_gain, samp_rate, tx_bw, amplitude):
        gr.top_block.__init__(self, "cw")

        self.src = analog.sig_source_c(
            samp_rate,
            analog.GR_CONST_WAVE,
            0,
            amplitude,
            0
        )

        self.usrp = uhd.usrp_sink(
            ",".join(("", "")),
            uhd.stream_args(
                cpu_format="fc32",
                channels=[0],
            ),
            "",
        )

        self.usrp.set_samp_rate(samp_rate)
        self.usrp.set_center_freq(tx_freq, 0)
        self.usrp.set_gain(tx_gain, 0)
        self.usrp.set_antenna("TX/RX", 0)

        if tx_bw > 0:
            self.usrp.set_bandwidth(tx_bw, 0)

        self.usrp.set_time_unknown_pps(uhd.time_spec())

        self.connect(self.src, self.usrp)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--tx-freq", type=float, required=True)
    parser.add_argument("--tx-gain", type=float, default=70)
    parser.add_argument("--rate", type=float, default=100e3)
    parser.add_argument("--tx-bw", type=float, default=10e3)
    parser.add_argument("--amplitude", type=float, default=0.7)

    args = parser.parse_args()

    tb = cw(
        tx_freq=args.tx_freq,
        tx_gain=args.tx_gain,
        samp_rate=args.rate,
        tx_bw=args.tx_bw,
        amplitude=args.amplitude,
    )

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    tb.start()

    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
