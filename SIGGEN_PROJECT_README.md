# RFNoC SigGen — E310 SG3 pulse retransmission and measurement

This project extends **UHD 4.8** with a custom RFNoC SigGen FPGA block and UHD block controller. The application detects incoming RF pulses, generates delayed output pulses, and can stream pulse measurements to a host PC or to an application running natively on the E310 ARM processor.

> **Project branch:** `siggen-next-dev` in [nitzan-vexler/uhd](https://github.com/nitzan-vexler/uhd/tree/siggen-next-dev). This is a modified UHD source tree, not a standalone GUI package.

## Main source files

| Component | Location (relative to repository root) |
|---|---|
| FPGA core | `fpga/usrp3/lib/rfnoc/blocks/rfnoc_block_siggen/rfnoc_siggen_core.v` |
| RFNoC FPGA wrapper and shell | `fpga/usrp3/lib/rfnoc/blocks/rfnoc_block_siggen/` |
| E310 SG3 RFNoC image definition | `fpga/usrp3/top/e31x/e310_siggen_image.yml` |
| Block definition | `host/include/uhd/rfnoc/blocks/siggen.yml` |
| UHD block controller | `host/lib/rfnoc/siggen_block_control.cpp` and `host/include/uhd/rfnoc/siggen_block_control.hpp` |
| Host C++ app | `work/rfnoc_radio_loopback.cpp` |
| E310-native ARM C++ app | `work/rfnoc_radio_loopback_e310.cpp` |
| E310 app CMake template | `work/CMakeLists_e310.txt` |
| GUI | `work/rfnoc_gui.py` |
| Binary measurement parser | `work/combined_parser.py` |
| FPGA testbenches | `fpga/usrp3/lib/rfnoc/blocks/rfnoc_block_siggen/*tb.sv` |

## Requirements

- E310 SG3 with a **compatible custom FPGA image** containing the SigGen block.
- Matching **custom UHD 4.8** library and headers, built from this repository. Stock UHD may not recognize the custom controller.
- For native E310 execution: an ARM Cortex-A9 Yocto/OE cross-compilation SDK compatible with the E310's Linux image.
- CMake, C++14 compiler, Boost (`program_options`, `system`, `thread`), and pthreads.
- For GUI: Python 3, Tkinter, Matplotlib, and the local `combined_parser.py` module.
- For GPS features: a configured GPS/PPS source and `gpsd`/`gpspipe` on the relevant device.

**Version matching matters:** keep the FPGA register map, UHD controller, and applications from compatible revisions. Building the ARM application against host x86 UHD will not work.

## 1. Clone the project

```bash
git clone -b siggen-next-dev git@github.com:nitzan-vexler/uhd.git
cd uhd
```

If the repository is private, request GitHub collaborator access first.

## 2. FPGA image

The custom SigGen RTL and the E310 SG3 image YAML are in the paths above. Build the image using the project's UHD/RFNoC FPGA flow and the supported Vivado version for the selected UHD branch and device. Verify that the generated image actually contains the SigGen block before running the application.

A typical image-builder invocation from an appropriately configured UHD environment is:

```bash
rfnoc_image_builder \
  -y fpga/usrp3/top/e31x/e310_siggen_image.yml \
  -t E310_SG3 \
  --fpga-dir fpga
```

**Check the installed `rfnoc_image_builder --help` and the YAML target settings before using this command.** Vivado installation, FPGA build environment, and final bitstream-loading steps depend on the machine and UHD setup. Generated Vivado build directories are intentionally not version-controlled.

## 3. Build the custom UHD library

Build UHD using the repository's `host/` CMake project, selecting the required device support and installation prefix. Keep native x86 and ARM builds in **separate build directories**.

Example for a **native Linux host** (adapt the prefix and options to the machine):

```bash
cmake -S host -B host/build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$HOME/uhd_siggen_host"
cmake --build host/build -j4
cmake --install host/build
```

For the **E310 ARM** library, first source the compatible OE SDK environment and configure a *separate* cross-compiled UHD build:

```bash
source "$HOME/e310_sdk_4p8/environment-setup-cortexa9t2hf-neon-oe-linux-gnueabi"
cmake -S host -B host/build_e310 \
  -DCMAKE_TOOLCHAIN_FILE="$OE_CMAKE_TOOLCHAIN_FILE" \
  -DCMAKE_INSTALL_PREFIX="$HOME/uhd_e310_siggen_new" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build host/build_e310 -j4
cmake --install host/build_e310
```

The SDK path and install prefix above are **example paths from the original development machine**. On another machine, change them. Review the CMake configuration and ensure UHD device support is enabled before building.

## 4. Build the E310-native recorder

The CMake template `work/CMakeLists_e310.txt` expects the source file to be named `rfnoc_radio_loopback_e310.cpp` in the same directory and expects a custom ARM UHD installation under `$HOME/uhd_e310_siggen_new`.

```bash
mkdir -p "$HOME/siggen_e310_build"
cp work/rfnoc_radio_loopback_e310.cpp "$HOME/siggen_e310_build/"
cp work/CMakeLists_e310.txt "$HOME/siggen_e310_build/CMakeLists.txt"
source "$HOME/e310_sdk_4p8/environment-setup-cortexa9t2hf-neon-oe-linux-gnueabi"
cmake -S "$HOME/siggen_e310_build" -B "$HOME/siggen_e310_build/build" \
  -DCMAKE_TOOLCHAIN_FILE="$OE_CMAKE_TOOLCHAIN_FILE" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "$HOME/siggen_e310_build/build" -j4
file "$HOME/siggen_e310_build/build/siggen_e310_recorder"
```

Expected architecture: **32-bit ARM**, not x86-64. Use the matching cross-compiled Boost libraries supplied by the SDK.

## 5. Deploy to E310

The application uses `/data/siggen` on the E310 because `/home/root` may not have enough free space for the custom UHD shared library.

On the development PC (replace the address as needed):

```bash
ssh root@192.168.0.100 'mkdir -p /data/siggen/lib'
scp "$HOME/siggen_e310_build/build/siggen_e310_recorder" \
  root@192.168.0.100:/data/siggen/
scp "$HOME/uhd_e310_siggen_new/lib/libuhd.so.4.8.0" \
  root@192.168.0.100:/data/siggen/lib/
```

On the E310:

```bash
cd /data/siggen
chmod +x siggen_e310_recorder
ln -sfn libuhd.so.4.8.0 lib/libuhd.so
export LD_LIBRARY_PATH=/data/siggen/lib:$LD_LIBRARY_PATH
ldd ./siggen_e310_recorder
./siggen_e310_recorder --help
```

Verify `ldd` resolves `libuhd.so` to `/data/siggen/lib/` and that the loaded FPGA image matches the software. Avoid overwriting the E310 system UHD installation while testing.

## 6. GUI and measurement files

The GUI entry point is `work/rfnoc_gui.py`; it imports `combined_parser.py`, so run it from a location where that module is available:

```bash
cd work
python3 rfnoc_gui.py
```

The GUI supports **Host PC** and **USRP E312** targets (the latter label is used for the E31x native-device workflow). Configure the USRP IP and SSH access before remote execution. The remote application path is `/data/siggen/siggen_e310_recorder`.

Measurement locations in the tested setup:

- E310 on-device binary recording: `/data/siggen/measurements.dat`
- Local download destination: `~/SDR_Records/combined_measurements.dat`
- Local CSV conversion output: `~/SDR_Records/combined_measurements.csv`

Create the local directory if necessary:

```bash
mkdir -p "$HOME/SDR_Records"
```

The binary record format is **72 bytes per pulse** in the tested recorder. Use `work/combined_parser.py` or the GUI's **Measurement Parser** tab to convert recordings; do not interpret the binary file as raw IQ. The `dBm` output requires an appropriate calibration offset for absolute RF power.

## 7. Verification and troubleshooting

1. Confirm the E310 boots with the intended custom SigGen FPGA image.
2. Confirm the host or E310 process loads the **matching custom UHD library**.
3. Check the application `--help` output for actual command-line options.
4. Test with a safe RF setup (appropriate attenuation; do not directly connect TX to RX without checking power limits).
5. Check pulse PRI, width, and delay with external RF instrumentation.
6. Check measurement count, sequence continuity, queue drops, and timestamps in application logs.
7. Compare binary file size with the expected 72-byte record size (`size % 72 == 0`).

**Known observation:** Some configurations print repeated `U` characters from UHD while FPGA pulse transmission remains correct on external instrumentation. The exact source of these messages has not yet been confirmed; do not assume they prove measurement loss or change the working FPGA TX behavior solely to silence them.

## Sharing and version control

Share the `siggen-next-dev` branch with collaborators. Commit source code and documentation, **not** Vivado `.Xil/` directories, generated FPGA build folders, local UHD build directories, `.so` binaries, large `.dat` recordings, or machine-specific credentials.

For a reproducible handoff, record the exact commit hash, FPGA image build, UHD version, and E310 firmware/SDK versions used during testing.
