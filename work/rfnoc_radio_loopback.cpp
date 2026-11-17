//
// Copyright 2016 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

// Example UHD/RFNoC application: Connect an rx radio to a tx radio and
// run a loopback.

#include <uhd/rfnoc/block_id.hpp>
#include <uhd/rfnoc/mb_controller.hpp>
#include <uhd/rfnoc/radio_control.hpp>
#include <uhd/rfnoc_graph.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/utils/graph_utils.hpp>
#include <uhd/utils/math.hpp>
#include <uhd/utils/safe_main.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
#include <uhd/rfnoc/siggen_block_control.hpp>
#include <cmath>
#include <iomanip>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <mutex>
#include <regex>
#include <string>
#include <tuple>
#include <vector>
#include <deque>
#include <condition_variable>
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#include <uhd/types/time_spec.hpp>
#include <uhd/stream.hpp>
#include <complex>
#include <array>

// Measurement protocol: five 32-bit payload words per record.
// IMPORTANT: Confirm word ordering and status bit definitions against the
// FPGA measurement serializer before relying on decoded timestamps.
// Expected layout here:
//   word 0: avg_power[31:0]
//   word 1: sequence[31:16], status[15:0]
//   word 2: packet_timestamp[63:32]
//   word 3: packet_timestamp[31:0]
//   word 4: sample_offset[31:0]

static uhd::rfnoc::pulse_info decode_measurement(
    const std::array<uint32_t, 5>& words)
{
    uhd::rfnoc::pulse_info pulse{};

    // FPGA: [159:128]
    pulse.avg_power = words[0];

    // FPGA: [127:112]
    pulse.sequence =
        static_cast<uint16_t>(words[1] >> 16);

    // FPGA: [111:48] - 64-bit packet timestamp
    pulse.packet_timestamp =
        (uint64_t(words[1] & 0xFFFFu) << 48) |
        (uint64_t(words[2]) << 16) |
        (uint64_t(words[3]) >> 16);

    // FPGA: [47:16] - 32-bit sample offset
    pulse.sample_offset =
        ((words[3] & 0xFFFFu) << 16) |
        (words[4] >> 16);

    // FPGA: [15:0] - status
    const uint16_t status =
        static_cast<uint16_t>(words[4] & 0xFFFFu);

    pulse.has_time = (status & 0x1u) != 0;
    pulse.sample_index = 0;

    return pulse;
}


// Treat SC16 as an opaque 32-bit transport word. Verify I/Q halfword
// order using a known FPGA test pattern before trusting measurements.
static uint32_t unpack_measurement_word(const std::complex<int16_t>& s)
{
    return (uint32_t(uint16_t(s.real())) << 16) | uint16_t(s.imag());
}

namespace po = boost::program_options;
using uhd::rfnoc::radio_control;
using namespace std::chrono_literals;

using gps_data_t =
    std::tuple<std::string, std::string, std::string, std::string>;

std::mutex gps_mutex;
std::chrono::steady_clock::time_point latest_gps_received{};

gps_data_t latest_gps_data{
    "No GPS Time",
    "No Latitude",
    "No Longitude",
    "No Altitude"
};

std::atomic<bool> keep_gps_running{false};


gps_data_t extract_gps_data(const std::string& gps_data)
{
    std::string gps_time  = "No GPS Time";
    std::string latitude  = "No Latitude";
    std::string longitude = "No Longitude";
    std::string altitude  = "No Altitude";

    if (gps_data.find("\"class\":\"TPV\"") == std::string::npos) {
        return {gps_time, latitude, longitude, altitude};
    }

    std::smatch match;

    // Save complete GPS UTC timestamp
const std::regex time_regex(
    "\"time\"\\s*:\\s*\"([^\"]+)\""
);

const std::regex lat_regex(
    "\"lat\"\\s*:\\s*([-+]?\\d+(?:\\.\\d+)?)"
);

const std::regex lon_regex(
    "\"lon\"\\s*:\\s*([-+]?\\d+(?:\\.\\d+)?)"
);

const std::regex alt_regex(
    "\"alt(?:HAE|MSL)?\"\\s*:\\s*([-+]?\\d+(?:\\.\\d+)?)"
);

    if (std::regex_search(gps_data, match, time_regex)) {
        gps_time = match[1].str();
    }

    if (std::regex_search(gps_data, match, lat_regex)) {
        latitude = match[1].str();
    }

    if (std::regex_search(gps_data, match, lon_regex)) {
        longitude = match[1].str();
    }

    if (std::regex_search(gps_data, match, alt_regex)) {
        altitude = match[1].str();
    }

    return {gps_time, latitude, longitude, altitude};
}


void gps_stream_thread()
{
    std::cout << "[GPS] Starting gpspipe..." << std::endl;

    // GPSD runs on the E310. Connect over SSH, as the GUI does.
    // GNU timeout bounds the subprocess so the thread can exit on Stop.
    // Change this address if your USRP address changes.
    const char* gps_command =
        "timeout 12s ssh -o BatchMode=yes -o ConnectTimeout=5 "
        "-o ServerAliveInterval=3 -o ServerAliveCountMax=2 "
        "root@192.168.0.100 'timeout 10s gpspipe -w'";
    char buffer[4096];
    while (keep_gps_running.load()) {
        FILE* pipe = popen(gps_command, "r");
        if (pipe == nullptr) {
            std::cerr << "[GPS] Failed to start remote gpspipe" << std::endl;
            break;
        }
        while (keep_gps_running.load() &&
               fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        const std::string message(buffer);

        if (message.find("\"class\":\"TPV\"") == std::string::npos) {
            continue;
        }

        const gps_data_t parsed = extract_gps_data(message);

        if (std::get<0>(parsed) != "No GPS Time") {
            std::lock_guard<std::mutex> lock(gps_mutex);
            latest_gps_data = parsed;
            latest_gps_received = std::chrono::steady_clock::now();
        }
    }

        pclose(pipe);
        if (keep_gps_running.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
    }

    std::cout << "[GPS] GPS thread stopped" << std::endl;
}


gps_data_t get_latest_gps_data()
{
    std::lock_guard<std::mutex> lock(gps_mutex);
    // Reject stale fixes, including fixes from before the receiver started.
    if (latest_gps_received.time_since_epoch().count() == 0 ||
        std::chrono::steady_clock::now() - latest_gps_received >
            std::chrono::seconds(3)) {
        return {"No GPS Time", "No Latitude", "No Longitude", "No Altitude"};
    }
    return latest_gps_data;
}




// Align the RFNoC timekeeper to GPS UTC at a PPS edge.
// Fail closed: never label device timestamps UTC without verification.
void synchronize_timekeeper_to_gps(const uhd::rfnoc::mb_controller::sptr& mb)
{
    auto tk = mb->get_timekeeper(0);
    std::cout << "[GPS-SYNC] Waiting for GPS timing lock..." << std::endl;
    bool locked = false;
    for (int i = 0; i < 90; ++i) {
        if (mb->get_sensor("gps_locked").to_bool()) {
            locked = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    if (!locked) throw std::runtime_error("GPS timing lock timeout (gps_locked=false)");

    // Observe an actual PPS boundary, rather than guessing its phase.
    auto old_pps = tk->get_ticks_last_pps();
    bool edge = false;
    for (int i = 0; i < 250; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        const auto new_pps = tk->get_ticks_last_pps();
        if (new_pps != old_pps) { edge = true; break; }
    }
    if (!edge) throw std::runtime_error("No PPS edges observed by RFNoC timekeeper");

    // Read receiver UTC second immediately after the detected edge.
    // Device firmware may report the prior second; verification below detects this.
    const int64_t gps_at_edge = static_cast<int64_t>(
        mb->get_sensor("gps_time").to_int());
    if (gps_at_edge < 1600000000LL)
        throw std::runtime_error("GPS UTC seconds invalid");

    const int64_t programmed_next_second = gps_at_edge + 1;
    tk->set_time_next_pps(uhd::time_spec_t(programmed_next_second));
    std::cout << "[GPS-SYNC] Programming next PPS UTC="
              << programmed_next_second << std::endl;

    // Wait for the programmed edge and verify integer seconds at that edge.
    old_pps = tk->get_ticks_last_pps();
    edge = false;
    for (int i = 0; i < 250; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        const auto new_pps = tk->get_ticks_last_pps();
        if (new_pps != old_pps) { edge = true; break; }
    }
    if (!edge) throw std::runtime_error("No PPS after GPS time programming");

    const double pps_utc = tk->get_time_last_pps().get_real_secs();
    const int64_t gps_after = static_cast<int64_t>(
        mb->get_sensor("gps_time").to_int());
    std::cout << "[GPS-SYNC] PPS device UTC=" << std::fixed
              << std::setprecision(0) << pps_utc
              << " GPS sensor UTC=" << gps_after << std::endl;
    if (std::llabs(static_cast<int64_t>(std::llround(pps_utc))
                   - programmed_next_second) != 0)
        throw std::runtime_error("PPS programming did not take effect");
    if (std::llabs(static_cast<int64_t>(std::llround(pps_utc))
                   - gps_after) != 0)
        throw std::runtime_error("GPS/PPS seconds mismatch; UTC alignment NOT verified");
    std::cout << "[GPS-SYNC] VERIFIED: device timestamps are GPS-aligned UTC"
              << std::endl;
}

/****************************************************************************
 * SIGINT handling
 ***************************************************************************/
static bool stop_signal_called = false;
void sig_int_handler(int)
{
    stop_signal_called = true;
}

/****************************************************************************
 * main
 ***************************************************************************/
int UHD_SAFE_MAIN(int argc, char* argv[])
{
    // variables to be set by po
    std::string args, rx_ant, tx_ant, rx_blockid, tx_blockid, ref, pps, output_file;
    size_t total_num_samps, spp, rx_chan, tx_chan, threshold, pulsewidth, delay, avg_delay, pulse_gap;
    double rate, rx_freq, tx_freq, rx_gain, tx_gain, rx_bw, tx_bw, total_time, setup_time, dbm_offset;
    bool rx_timestamps, save_data, enable_gps ;

    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "UHD device address args")
        ("spp", po::value<size_t>(&spp)->default_value(64), "Samples per packet (reduce for lower latency)")
        ("threshold", po::value<size_t>(&threshold)->default_value(1000), "Input pulse detection threshold (ADC counts)")
        ("pw", po::value<size_t>(&pulsewidth)->default_value(800), "Transmit pulse width in samples")
        ("delay", po::value<size_t>(&delay)->default_value(8000), "Delay from trigger to transmission in CE clock cycles")
        ("pulse-gap", po::value<size_t>(&pulse_gap)->default_value(10000), "Gap between fixed and relative pulse in CE clock cycles")
        ("save-data",
 po::bool_switch(&save_data)->default_value(false),
 "Save FPGA pulse measurements to a binary file")
      ("output-file",
 po::value<std::string>(&output_file)
     ->default_value("combined_measurements.dat"),
 "Binary FPGA measurement output file")
        ("dbm-offset", po::value<double>(&dbm_offset)->default_value(0.0), "Calibrated offset: dBm = dBFS + offset (dB)")
        ("gps", po::bool_switch(&enable_gps)->default_value(false), "Enable GPS data collection")
        ("avg-delay",po::value<size_t>(&avg_delay)->default_value(32),"Samples to wait after trigger before averaging")
        ("rx-freq", po::value<double>(&rx_freq)->default_value(200000000.0), "Rx RF center frequency in Hz")
        ("tx-freq", po::value<double>(&tx_freq)->default_value(200000000.0), "Tx RF center frequency in Hz")
        ("rx-gain", po::value<double>(&rx_gain)->default_value(50.0), "Rx RF center gain in Hz")
        ("tx-gain", po::value<double>(&tx_gain)->default_value(70.0), "Tx RF center gain in Hz")
        ("rx-ant", po::value<std::string>(&rx_ant), "Receive antenna selection")
        ("tx-ant", po::value<std::string>(&tx_ant), "Transmit antenna selection")
        ("rx-blockid", po::value<std::string>(&rx_blockid)->default_value("0/Radio#0"), "Receive radio block ID")
        ("tx-blockid", po::value<std::string>(&tx_blockid)->default_value("0/Radio#0"), "Transmit radio block ID")
        ("rx-chan", po::value<size_t>(&rx_chan)->default_value(0), "Channel index on receive radio")
        ("tx-chan", po::value<size_t>(&tx_chan)->default_value(1), "Channel index on transmit radio")
        ("rx-bw", po::value<double>(&rx_bw), "RX analog frontend filter bandwidth in Hz")
        ("tx-bw", po::value<double>(&tx_bw), "TX analog frontend filter bandwidth in Hz")
        ("rx-timestamps", po::value<bool>(&rx_timestamps)->default_value(true), "Set timestamps on RX")
        ("setup", po::value<double>(&setup_time)->default_value(0.1), "seconds of setup time")
        ("nsamps", po::value<size_t>(&total_num_samps)->default_value(0), "total number of samples to receive")
        ("rate", po::value<double>(&rate)->default_value(60000000.0), "Sampling rate")
        ("duration", po::value<double>(&total_time)->default_value(0), "total number of seconds to receive")
        ("int-n", "Tune USRP with integer-N tuning")
        ("ref", po::value<std::string>(&ref), "clock reference (internal, external, gpsdo, mimo)")
        ("pps", po::value<std::string>(&pps), "PPS source (internal, external, mimo, gpsdo)")
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // print the help message
    if (vm.count("help")) {
        std::cout << boost::format("RFNoC: Radio loopback test %s") % desc << std::endl;
        std::cout
            << std::endl
            << "This application streams data from one radio to another using RFNoC.\n"
            << std::endl;
        return ~0;
    }

    /************************************************************************
     * Create device and block controls
     ***********************************************************************/
    std::cout << std::endl;
    std::cout << boost::format("Creating the RFNoC graph with args: %s...") % args
              << std::endl;
    uhd::rfnoc::rfnoc_graph::sptr graph = uhd::rfnoc::rfnoc_graph::make(args);

    // Create handles for radio objects
    uhd::rfnoc::block_id_t rx_radio_ctrl_id(rx_blockid);
    uhd::rfnoc::block_id_t tx_radio_ctrl_id(tx_blockid);
    // This next line will fail if the radio is not actually available
    uhd::rfnoc::radio_control::sptr rx_radio_ctrl =
        graph->get_block<uhd::rfnoc::radio_control>(rx_radio_ctrl_id);
    uhd::rfnoc::radio_control::sptr tx_radio_ctrl =
        graph->get_block<uhd::rfnoc::radio_control>(tx_radio_ctrl_id);
    std::cout << "Using RX radio " << rx_radio_ctrl_id << ", channel " << rx_chan
              << std::endl;
    std::cout << "Using TX radio " << tx_radio_ctrl_id << ", channel " << tx_chan
              << std::endl;
              
using uhd::rfnoc::siggen_block_control;

// Get the block
uhd::rfnoc::block_id_t siggen_id("0/SigGen#0");   // adjust if different
auto siggen = graph->get_block<siggen_block_control>(siggen_id);
std::cout << "\n=== SigGen ports ===\n";
std::cout << "Input ports: "
          << siggen->get_num_input_ports() << "\n";
std::cout << "Output ports: "
          << siggen->get_num_output_ports() << "\n";

std::cout << "\n=== Static FPGA connections ===\n";

for (const auto& edge : graph->enumerate_static_connections()) {
    std::cout << edge.to_string() << std::endl;
}


// Wire the graph: RX -> SigGen -> TX
uhd::rfnoc::connect_through_blocks(graph, rx_radio_ctrl_id, rx_chan, siggen_id, 0, /*skip_pp=*/false);
uhd::rfnoc::connect_through_blocks(graph, siggen_id, 0, tx_radio_ctrl_id, tx_chan, /*skip_pp=*/true);
// SigGen output 1 carries opaque 32-bit measurement words.
// SC16 is used solely as a 4-byte transport format.
if (siggen->get_num_output_ports() < 2) {
    throw std::runtime_error("SigGen output port 1 is not available");
}
uhd::stream_args_t meas_args("sc16", "sc16");
auto meas_streamer = graph->create_rx_streamer(1, meas_args);
graph->connect(siggen_id, 1, meas_streamer, 0);
graph->commit();
std::cout << "[MEAS] SigGen output 1 connected to host RX streamer" << std::endl;

const size_t port = 0;
          
          
// --- Configure radios as you already do (rates, freqs, gains, etc.) ---

// Configure SigGen (example numbers)

const double tone_hz = 0;                 
siggen->set_samples_per_packet(spp, port);
siggen->set_waveform(uhd::rfnoc::siggen_waveform::SINE_WAVE, port);
siggen->set_amplitude(1, port);             // 0.0 .. 1.0
siggen->set_sine_phase_increment(0, port);
siggen->set_avg_start_delay(avg_delay, port);

// Trigger gating (your new regs)
siggen->set_threshold(threshold /*LSBs*/, port);   // pick based on RX magnitude
//siggen->set_holdcount(holdcount /*LSBs*/, port);   
siggen->set_delay(delay /*ce_clk cycles*/, port);
siggen->set_pulse_gap(pulse_gap, port);
siggen->set_pulsewidth(pulsewidth /*samples*/, port);
    std::cout << "delay= " << siggen->get_delay(port) << " ce clk cycles " << std::endl;
    std::cout << "pulse_gap= "
          << siggen->get_pulse_gap(port)
          << " ce clk cycles"
          << std::endl;
    std::cout << "pulsewidth= " << siggen->get_pulsewidth(port) << " samples " << std::endl;
const double thr_dbfs =
    (threshold > 0)
    ? 20.0 * std::log10((double)threshold / 32767.0)
    : -200.0;

std::cout << "threshold = "
          << threshold
          << " counts ("
          << std::fixed << std::setprecision(2)
          << thr_dbfs
          << " dBFS)"
          << std::endl;

    
std::cout << "avg_start_delay = "
          << siggen->get_avg_start_delay(port)
          << " samples"
          << std::endl;

std::cout << "Actual RX BW: "
          << rx_radio_ctrl->get_rx_bandwidth(rx_chan)
          << " Hz" << std::endl;

std::cout << "Actual TX BW: "
          << tx_radio_ctrl->get_tx_bandwidth(tx_chan)
          << " Hz" << std::endl;
          
std::cout << "avg_start_delay = "
          << siggen->get_avg_start_delay(port)
          << " samples"
          << std::endl;
          
// Enable the generator only after GPS/PPS synchronization, before streaming.

    size_t rx_mb_idx = rx_radio_ctrl_id.get_device_no();


	

    /************************************************************************
     * Set up radio
     ***********************************************************************/
    // Only forward properties once per block in the chain. In the case of
    // looping back to a single radio block, skip property propagation after
    // traversing back to the starting point of the chain.
    const bool skip_pp = rx_radio_ctrl_id == tx_radio_ctrl_id;
    // Connect the RX radio to the TX radio
   // uhd::rfnoc::connect_through_blocks(
     //   graph, rx_radio_ctrl_id, rx_chan, tx_radio_ctrl_id, tx_chan, skip_pp);
   // graph->commit();
    
    

    rx_radio_ctrl->enable_rx_timestamps(rx_timestamps, rx_chan);
    
    //auto tk = graph->get_mb_controller(rx_mb_idx)->get_timekeeper(0);
//std::cout << "Time now (radio) = " 
     //     << rx_radio_ctrl->get_time_now().get_real_secs() << " s\n";
//std::cout << "Timekeeper tick rate = " << tk->get_tick_rate() << " Hz\n";


    // Set time and clock reference
    if (vm.count("ref") && vm.count("pps")) {
        for (size_t i = 0; i < graph->get_num_mboards(); ++i) {
            graph->get_mb_controller(i)->set_sync_source(ref, pps);
        }
    } else if (vm.count("ref")) {
        // Lock mboard clocks
        for (size_t i = 0; i < graph->get_num_mboards(); ++i) {
            graph->get_mb_controller(i)->set_clock_source(ref);
        }
    } else if (vm.count("pps")) {
        // Lock mboard clocks
        for (size_t i = 0; i < graph->get_num_mboards(); ++i) {
            graph->get_mb_controller(i)->set_time_source(pps);
        }
    }

    // GPS position (--gps) also requires GPS/PPS UTC synchronization.
    // The E310 SG3 probe reports clock sources: internal; time sources: gpsdo.
    // Keep the internal clock source, select GPSDO as the time/PPS source.
    if (enable_gps) {
        if (vm.count("pps") && pps != "gpsdo") {
            throw std::runtime_error("--gps requires --pps gpsdo (or omit --pps)");
        }
        if (vm.count("ref") && ref != "internal") {
            throw std::runtime_error("E310 SG3 GPS mode requires --ref internal (or omit --ref)");
        }
        auto mb = graph->get_mb_controller(rx_mb_idx);
        mb->set_time_source("gpsdo");
        // GPS/PPS synchronization is performed after configuring the sample rate.
    }

    // set the sample rate
    if (rate <= 0.0) {
        rate = rx_radio_ctrl->get_rate();
    } else {
        std::cout << boost::format("Setting RX Rate: %f Msps...") % (rate / 1e6)
                  << std::endl;
        rate = rx_radio_ctrl->set_rate(rate);
        std::cout << boost::format("Actual RX Rate: %f Msps...") % (rate / 1e6)
                  << std::endl
                  << std::endl;
    }

    // Synchronize only after the final RX rate has been applied.
    // Changing the rate after programming GPS time can reinterpret existing ticks.
    if (enable_gps) {
        auto mb = graph->get_mb_controller(rx_mb_idx);
        synchronize_timekeeper_to_gps(mb);

        const double device_utc = mb->get_timekeeper(0)->get_time_now().get_real_secs();
        const double gps_utc = static_cast<double>(mb->get_sensor("gps_time").to_int());
        std::cout << "[GPS-SYNC] Post-rate device UTC=" << std::fixed
                  << std::setprecision(3) << device_utc
                  << " GPS UTC=" << std::setprecision(0) << gps_utc << std::endl;
        if (std::fabs(device_utc - gps_utc) > 2.0) {
            throw std::runtime_error("GPS UTC mismatch after RX rate configuration");
        }
    }

    // set the center frequency
    if (vm.count("rx-freq")) {
        std::cout << boost::format("Setting RX Freq: %f MHz...") % (rx_freq / 1e6)
                  << std::endl;
        uhd::tune_request_t tune_request(rx_freq);
        if (vm.count("int-n")) {
            tune_request.args = uhd::device_addr_t("mode_n=integer");
        }
        rx_radio_ctrl->set_rx_frequency(rx_freq, rx_chan);
        std::cout << boost::format("Actual RX Freq: %f MHz...")
                         % (rx_radio_ctrl->get_rx_frequency(rx_chan) / 1e6)
                  << std::endl
                  << std::endl;
    }
    if (vm.count("tx-freq")) {
        std::cout << boost::format("Setting TX Freq: %f MHz...") % (tx_freq / 1e6)
                  << std::endl;
        uhd::tune_request_t tune_request(tx_freq);
        if (vm.count("int-n")) {
            tune_request.args = uhd::device_addr_t("mode_n=integer");
        }
        tx_radio_ctrl->set_tx_frequency(tx_freq, tx_chan);
        std::cout << boost::format("Actual TX Freq: %f MHz...")
                         % (tx_radio_ctrl->get_tx_frequency(tx_chan) / 1e6)
                  << std::endl
                  << std::endl;
    }

    // set the rf gain
    if (vm.count("rx-gain")) {
        std::cout << boost::format("Setting RX Gain: %f dB...") % rx_gain << std::endl;
        rx_radio_ctrl->set_rx_gain(rx_gain, rx_chan);
        std::cout << boost::format("Actual RX Gain: %f dB...")
                         % rx_radio_ctrl->get_rx_gain(rx_chan)
                  << std::endl
                  << std::endl;
    }
    if (vm.count("tx-gain")) {
        std::cout << boost::format("Setting TX Gain: %f dB...") % tx_gain << std::endl;
        tx_radio_ctrl->set_tx_gain(tx_gain, tx_chan);
        std::cout << boost::format("Actual TX Gain: %f dB...")
                         % tx_radio_ctrl->get_tx_gain(tx_chan)
                  << std::endl
                  << std::endl;
    }

    // set the IF filter bandwidth
    if (vm.count("rx-bw")) {
        std::cout << boost::format("Setting RX Bandwidth: %f MHz...") % (rx_bw / 1e6)
                  << std::endl;
        rx_radio_ctrl->set_rx_bandwidth(rx_bw, rx_chan);
        std::cout << boost::format("Actual RX Bandwidth: %f MHz...")
                         % (rx_radio_ctrl->get_rx_bandwidth(rx_chan) / 1e6)
                  << std::endl
                  << std::endl;
    }
    if (vm.count("tx-bw")) {
        std::cout << boost::format("Setting TX Bandwidth: %f MHz...") % (tx_bw / 1e6)
                  << std::endl;
        tx_radio_ctrl->set_tx_bandwidth(tx_bw, tx_chan);
        std::cout << boost::format("Actual TX Bandwidth: %f MHz...")
                         % (tx_radio_ctrl->get_tx_bandwidth(tx_chan) / 1e6)
                  << std::endl
                  << std::endl;
    }

    // set the antennas
    if (vm.count("rx-ant")) {
        rx_radio_ctrl->set_rx_antenna(rx_ant, rx_chan);
    }
    if (vm.count("tx-ant")) {
        tx_radio_ctrl->set_tx_antenna(tx_ant, tx_chan);
    }

    // check Ref and LO Lock detect
    if (not vm.count("skip-lo")) {
        // TODO
        // check_locked_sensor(usrp->get_rx_sensor_names(0), "lo_locked",
        // boost::bind(&uhd::usrp::multi_usrp::get_rx_sensor, usrp, _1, radio_id),
        // setup_time); if (ref == "external")
        // check_locked_sensor(usrp->get_mboard_sensor_names(0), "ref_locked",
        // boost::bind(&uhd::usrp::multi_usrp::get_mboard_sensor, usrp, _1, radio_id),
        // setup_time);
    }

    if (vm.count("spp")) {
        std::cout << "Setting samples per packet to: " << spp << std::endl;
        rx_radio_ctrl->set_property<int>("spp", spp, 0);
        spp = rx_radio_ctrl->get_property<int>("spp", 0);
        std::cout << "Actual samples per packet = " << spp << std::endl;
    }

    // Allow for some setup time
    std::this_thread::sleep_for(1s * setup_time);

    // Arm SIGINT handler
    std::signal(SIGINT, &sig_int_handler);

    // Calculate timeout and set timers
    // We just need to check is nsamps was set, otherwise we'll use the duration
    if (total_num_samps > 0) {
        total_time = total_num_samps / rate;
        std::cout << boost::format("Expected streaming time: %.3f") % total_time
                  << std::endl;
    }

    siggen->set_enable(true, port);
    
    

    // Start streaming
    uhd::stream_cmd_t stream_cmd((total_num_samps == 0)
                                     ? uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS
                                     : uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps  = size_t(total_num_samps);
    stream_cmd.stream_now = false;
    stream_cmd.time_spec =
        graph->get_mb_controller(rx_mb_idx)->get_timekeeper(0)->get_time_now()
        + setup_time;
    std::cout << "Issuing start stream cmd..." << std::endl;
    
    auto tk_debug = graph->get_mb_controller(rx_mb_idx)->get_timekeeper(0);

std::cout << "\n[TIMEKEEPER DEBUG]\n"
          << "Timekeeper tick rate = "
          << tk_debug->get_tick_rate() << "\n"
          << "Timekeeper seconds   = "
          << tk_debug->get_time_now().get_real_secs() << "\n"
          << "Radio seconds        = "
          << rx_radio_ctrl->get_time_now().get_real_secs()
          << std::endl;
          
          
    rx_radio_ctrl->issue_stream_cmd(stream_cmd, rx_chan);
    std::cout << "Wait..." << std::endl;

std::thread gps_thread;

if (enable_gps) {
    keep_gps_running.store(true);
    gps_thread = std::thread(gps_stream_thread);
}
#pragma pack(push, 1)

struct MeasurementRecord
{
    int64_t timestamp_us;     // 8 bytes (UTC Unix microseconds if --gps, otherwise device time)
    char    gps_time[32];     // 32 bytes (full GPS ISO-8601 UTC fix)
    double  latitude;         // 8 bytes
    double  longitude;        // 8 bytes
    double  altitude;         // 8 bytes
    double  amplitude_dbm;    // 8 bytes
};

#pragma pack(pop)

static_assert(
    sizeof(MeasurementRecord) == 72,
    "Unexpected MeasurementRecord size"
);

std::vector<MeasurementRecord> write_buffer;
write_buffer.reserve(512);


std::ofstream binary_file;

if (save_data) {
    binary_file.open(
        output_file,
        std::ios::out
        | std::ios::binary
        | std::ios::trunc
    );

    if (!binary_file.is_open()) {
        keep_gps_running.store(false);

        if (gps_thread.joinable()) {
            gps_thread.join();
        }

        throw std::runtime_error(
            "Could not open binary output file: "
            + output_file
        );
    }

    std::cout
        << "Saving binary measurements to: "
        << output_file
        << std::endl;

    std::cout
    << "Binary record size: "
    << sizeof(MeasurementRecord)
    << " bytes"
    << std::endl;
}

// Receive measurement records over the RFNoC output-1 data stream.
// GPS formatting and disk I/O remain on the existing worker thread.
const double tick_rate = graph->get_mb_controller(rx_mb_idx)
    ->get_timekeeper(0)->get_tick_rate();
if (tick_rate <= 0.0) throw std::runtime_error("Invalid USRP timekeeper tick rate");
std::cout << "USRP tick rate: " << tick_rate << " Hz\n";
std::cout << "Recorded timestamp_us is "
          << (enable_gps ? "GPS-aligned Unix UTC" : "UNSYNCHRONIZED device time")
          << " (microseconds).\n";

constexpr size_t QUEUE_LIMIT = 16384;
std::deque<uhd::rfnoc::pulse_info> pulse_queue;
std::mutex queue_mutex;
std::condition_variable queue_cv;
bool acquisition_finished = false;
std::atomic<bool> worker_failed{false};
std::atomic<uint64_t> processed{0};
std::atomic<uint64_t> invalid_timestamps{0};
std::atomic<uint64_t> host_queue_drops{0};
std::atomic<uint64_t> acquired{0};
std::atomic<uint64_t> missing_sequences{0};
std::atomic<size_t> max_queue_depth{0};
uint16_t previous_seq = 0;
bool have_previous_seq = false;
uint64_t rx_errors = 0;
uint64_t partial_words = 0;
std::array<uint32_t, 5> pending_words{};
size_t pending_count = 0;
bool first_record = true;
auto last_report = std::chrono::steady_clock::now();

std::thread processing_thread([&]() {
    int64_t previous_timestamp_us = 0;
    bool have_previous_timestamp = false;
    try {
        std::vector<MeasurementRecord> local_buffer;
        local_buffer.reserve(512);
        for (;;) {
            uhd::rfnoc::pulse_info pulse;
            {
                std::unique_lock<std::mutex> lock(queue_mutex);
                queue_cv.wait(lock, [&] { return acquisition_finished || !pulse_queue.empty(); });
                if (pulse_queue.empty() && acquisition_finished) break;
                pulse = pulse_queue.front();
                pulse_queue.pop_front();
            }
            if (!pulse.has_time) {
                ++invalid_timestamps;
                continue;
            }
            const double pulse_seconds = pulse.trigger_time_seconds(tick_rate, rate);
            const int64_t timestamp_us =
                static_cast<int64_t>(std::llround(pulse_seconds * 1e6));
            const int64_t pri_us = have_previous_timestamp
                ? timestamp_us - previous_timestamp_us : 0;
            previous_timestamp_us = timestamp_us;
            have_previous_timestamp = true;
            const double amp = pulse.avg_power > 0
                ? std::sqrt(static_cast<double>(pulse.avg_power)) : 0.0;
            const double dbfs = amp > 0.0
                ? 20.0 * std::log10(amp / 32767.0) : -200.0;
            if (save_data) {
                MeasurementRecord record{};
                record.timestamp_us = timestamp_us;
                const gps_data_t gps = get_latest_gps_data();
                const auto& gps_time = std::get<0>(gps);
                if (gps_time != "No GPS Time")
                    std::strncpy(record.gps_time, gps_time.c_str(), sizeof(record.gps_time) - 1);
                record.latitude = 999.0;
                record.longitude = 999.0;
                record.altitude = 99999.0;
                try {
                    if (std::get<1>(gps) != "No Latitude") record.latitude = std::stod(std::get<1>(gps));
                    if (std::get<2>(gps) != "No Longitude") record.longitude = std::stod(std::get<2>(gps));
                    if (std::get<3>(gps) != "No Altitude") record.altitude = std::stod(std::get<3>(gps));
                } catch (const std::exception&) {
                    record.latitude = 999.0;
                    record.longitude = 999.0;
                    record.altitude = 99999.0;
                }
                record.amplitude_dbm = dbfs; // Binary format remains dBFS for existing parser compatibility.
                local_buffer.push_back(record);
                if (local_buffer.size() >= 512) {
                    binary_file.write(reinterpret_cast<const char*>(local_buffer.data()),
                                      local_buffer.size() * sizeof(MeasurementRecord));
                    if (!binary_file) throw std::runtime_error("Binary file write failed");
                    local_buffer.clear();
                }
            }
            const uint64_t count = ++processed;
            // One flushed telemetry line per 50 pulses (plus the first pulse).
            // Tkinter reads these lines from stdout to update live metrics.
            if (count == 1 || count % 50 == 0) {
                std::cout << "[PULSE] COUNT=" << count
                          << " SEQ=" << pulse.sequence
                          << " RX_dBm=" << std::fixed << std::setprecision(2)
                          << (dbfs + dbm_offset)
                          << " PRI_US=" << pri_us
                          << std::endl;
            }
        }
        if (save_data && !local_buffer.empty()) {
            binary_file.write(reinterpret_cast<const char*>(local_buffer.data()),
                              local_buffer.size() * sizeof(MeasurementRecord));
            if (!binary_file) throw std::runtime_error("Final binary file write failed");
        }
    } catch (const std::exception& e) {
        std::cerr << "[WORKER] " << e.what() << std::endl;
        worker_failed.store(true);
    }
});

try {
    std::vector<std::complex<int16_t>> meas_buffer(1024);
    uhd::rx_metadata_t meas_md;
    const auto acquisition_start = std::chrono::steady_clock::now();
    while (!stop_signal_called && !worker_failed.load()) {
        if (total_time > 0.0 &&
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - acquisition_start).count() >= total_time) {
            break;
        }

        const size_t received = meas_streamer->recv(
            meas_buffer.data(), meas_buffer.size(), meas_md, 0.25);

        if (meas_md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
            continue;
        }
        if (meas_md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
            ++rx_errors;
            std::cerr << "[MEAS] RX error: " << meas_md.strerror() << std::endl;
            // Lost words can destroy record alignment; stop rather than
            // silently decode corrupted measurements.
            throw std::runtime_error("Measurement stream error; record alignment lost");
        }

        for (size_t i = 0; i < received; ++i) {
            pending_words[pending_count++] = unpack_measurement_word(meas_buffer[i]);
            if (pending_count != pending_words.size()) continue;
            pending_count = 0;

            if (first_record) {
                first_record = false;
                std::cout << "[MEAS] First raw record:";
                for (const auto word : pending_words) {
                    std::cout << " 0x" << std::hex << std::setw(8)
                              << std::setfill('0') << word << std::dec;
                }
                std::cout << std::setfill(' ') << std::endl;
            }

            const auto pulse = decode_measurement(pending_words);
            static bool first_timestamp_printed = false;

if (!first_timestamp_printed) {
    first_timestamp_printed = true;

    std::cout << "\n[TIMESTAMP DEBUG]\n"
              << "packet_timestamp = " << pulse.packet_timestamp << "\n"
              << "sample_offset    = " << pulse.sample_offset << "\n"
              << "has_time         = " << pulse.has_time << "\n"
              << "tick_rate        = " << tick_rate << "\n"
              << "rx_sample_rate   = " << rate << "\n"
              << "trigger_seconds  = "
              << pulse.trigger_time_seconds(tick_rate, rate)
              << std::endl;
}
            ++acquired;
            if (have_previous_seq) {
                const uint16_t delta = static_cast<uint16_t>(pulse.sequence - previous_seq);
                if (delta > 1) missing_sequences += static_cast<uint64_t>(delta - 1);
            }
            previous_seq = pulse.sequence;
            have_previous_seq = true;
            {
                std::lock_guard<std::mutex> lock(queue_mutex);
                if (pulse_queue.size() >= QUEUE_LIMIT) {
                    ++host_queue_drops;
                } else {
                    pulse_queue.push_back(pulse);
                    max_queue_depth.store(std::max(max_queue_depth.load(), pulse_queue.size()));
                    queue_cv.notify_one();
                }
            }
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - last_report >= std::chrono::seconds(1)) {
            size_t queue_depth = 0;
            {
                std::lock_guard<std::mutex> lock(queue_mutex);
                queue_depth = pulse_queue.size();
            }
            std::cout << "[STATUS] acquired=" << acquired.load()
                      << " processed=" << processed.load()
                      << " missing_seq=" << missing_sequences.load()
                      << " queue=" << queue_depth
                      << " queue_max=" << max_queue_depth.load()
                      << " queue_drops=" << host_queue_drops.load()
                      << " invalid_time=" << invalid_timestamps.load()
                      << " rx_errors=" << rx_errors
                      << std::endl;
            last_report = now;
        }
    }
    partial_words = pending_count;
} catch (const std::exception& e) {
    std::cerr << "[ACQUISITION] " << e.what() << std::endl;
    worker_failed.store(true);
}
{
    std::lock_guard<std::mutex> lock(queue_mutex);
    acquisition_finished = true;
}
queue_cv.notify_all();
processing_thread.join();
std::cout << "[STATUS] FINAL acquired=" << acquired.load()
          << " processed=" << processed.load()
          << " missing_seq=" << missing_sequences.load()
          << " invalid_time=" << invalid_timestamps.load()
          << " queue_drops=" << host_queue_drops.load()
          << " queue_max=" << max_queue_depth.load()
          << " partial_words=" << partial_words
          << " rx_errors=" << rx_errors << std::endl;
if (worker_failed.load()) std::cerr << "[ERROR] Acquisition or worker failed" << std::endl;

    // Stop radio
    siggen->set_enable(false, port);
    stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
    std::cout << "Issuing stop stream cmd..." << std::endl;
    rx_radio_ctrl->issue_stream_cmd(
    stream_cmd,
    rx_chan
);

keep_gps_running.store(false);

if (gps_thread.joinable()) {
    gps_thread.join();
}

if (binary_file.is_open()) {
    binary_file.close();

    std::cout
        << "Binary measurement file closed: "
        << output_file
        << std::endl;
}

std::cout << "Done" << std::endl;
    // Allow for the samples and ACKs to propagate
    std::this_thread::sleep_for(1000ms);

    return EXIT_SUCCESS;
}
