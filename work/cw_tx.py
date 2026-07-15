```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from gnuradio import analog, gr, uhd
import argparse
import signal
import sys
import time


class cw(gr.top_block):
    def __init__(self, tx_freq, tx_gain, samp_rate, tx_bw):
        gr.top_block.__init__(self, "Pure CW")

        # Constant complex baseband value:
        # RF output is exactly at the configured center frequency.
        self.src = analog.sig_source_c(
            samp_rate,
            analog.GR_CONST_WAVE,
            0.0,
            1.0,
            0.0
        )

        self.usrp = uhd.usrp_sink(
            ",".join(("", "")),
            uhd.stream_args(
                cpu_format="sc16",
                args="",
                channels=[0],
            ),
            "",
        )

        self.usrp.set_center_freq(tx_freq, 0)
        self.usrp.set_gain(tx_gain, 0)
        self.usrp.set_antenna("TX/RX", 0)

        if tx_bw > 0:
            self.usrp.set_bandwidth(tx_bw, 0)

        self.usrp.set_samp_rate(samp_rate)
        self.usrp.set_time_unknown_pps(uhd.time_spec())

        self.connect(self.src, self.usrp)


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--tx-freq", type=float, required=True)
    parser.add_argument("--tx-gain", type=float, default=70)
    parser.add_argument("--rate", type=float, default=100e3)
    parser.add_argument("--tx-bw", type=float, default=10e3)

    args = parser.parse_args()

    tb = cw(
        tx_freq=args.tx_freq,
        tx_gain=args.tx_gain,
        samp_rate=args.rate,
        tx_bw=args.tx_bw,
    )

    def stop_flowgraph(sig=None, frame=None):
        tb.stop()
        tb.wait()
        sys.exit(0)

    signal.signal(signal.SIGINT, stop_flowgraph)
    signal.signal(signal.SIGTERM, stop_flowgraph)

    tb.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop_flowgraph()


if __name__ == "__main__":
    main()
```

