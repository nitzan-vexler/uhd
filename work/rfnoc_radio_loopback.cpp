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

namespace po = boost::program_options;
using uhd::rfnoc::radio_control;
using namespace std::chrono_literals;

using gps_data_t =
    std::tuple<std::string, std::string, std::string, std::string>;

std::mutex gps_mutex;

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

    // This application runs directly on the E312
    FILE* pipe = popen("gpspipe -w", "r");

    if (pipe == nullptr) {
        std::cerr << "[GPS] Failed to start gpspipe" << std::endl;
        return;
    }

    char buffer[2048];

    while (
        keep_gps_running.load()
        && fgets(buffer, sizeof(buffer), pipe) != nullptr
    ) {
        const std::string message(buffer);

        if (message.find("\"class\":\"TPV\"") == std::string::npos) {
            continue;
        }

        const gps_data_t parsed = extract_gps_data(message);

        if (std::get<0>(parsed) != "No GPS Time") {
            std::lock_guard<std::mutex> lock(gps_mutex);
            latest_gps_data = parsed;
        }
    }

    pclose(pipe);

    std::cout << "[GPS] GPS thread stopped" << std::endl;
}


gps_data_t get_latest_gps_data()
{
    std::lock_guard<std::mutex> lock(gps_mutex);
    return latest_gps_data;
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
    double rate, rx_freq, tx_freq, rx_gain, tx_gain, rx_bw, tx_bw, total_time, setup_time;
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

// Wire the graph: RX -> SigGen -> TX
uhd::rfnoc::connect_through_blocks(graph, rx_radio_ctrl_id, rx_chan, siggen_id, 0, /*skip_pp=*/false);
uhd::rfnoc::connect_through_blocks(graph, siggen_id, 0, tx_radio_ctrl_id, tx_chan, /*skip_pp=*/true);
graph->commit();

// -------------------------------------------------
// Clear old FIFO measurements before starting
// -------------------------------------------------

const size_t port = 0;

std::cout << "FIFO count before clearing: "
          << siggen->get_measurement_fifo_count(port)
          << std::endl;

uhd::rfnoc::pulse_info old_pulse;

size_t cleared_count = 0;

while (siggen->pop_measurement_fifo(old_pulse, port)) {
    ++cleared_count;
}

std::cout << "Discarded " << cleared_count
          << " old measurements" << std::endl;

std::cout << "FIFO count after clearing: "
          << siggen->get_measurement_fifo_count(port)
          << std::endl;
          
          
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
          
// Finally enable the generator
siggen->set_enable(true, port);

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
    
    //auto tk = graph->get_mb_controller(rx_mb_idx)->get_timekeeper(rx_mb_idx);
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

    // Start streaming
    uhd::stream_cmd_t stream_cmd((total_num_samps == 0)
                                     ? uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS
                                     : uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps  = size_t(total_num_samps);
    stream_cmd.stream_now = false;
    stream_cmd.time_spec =
        graph->get_mb_controller(rx_mb_idx)->get_timekeeper(rx_mb_idx)->get_time_now()
        + setup_time;
    std::cout << "Issuing start stream cmd..." << std::endl;
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
    int64_t timestamp_us;     // 8 bytes
    char    gps_time[20];     // 20 bytes
    double  latitude;         // 8 bytes
    double  longitude;        // 8 bytes
    double  altitude;         // 8 bytes
    double  amplitude_dbm;    // 8 bytes
};

#pragma pack(pop)

static_assert(
    sizeof(MeasurementRecord) == 60,
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

uint16_t last_sequence = 0;
uint64_t last_sample_index = 0;
bool first_measurement = true;


bool timestamp_reference_valid = false;
uint64_t reference_sample_index = 0;
int64_t reference_timestamp_us = 0;



// -------------------------------------------------

try
{
    while (!stop_signal_called)
    {
        uhd::rfnoc::pulse_info pulse;

        if (!siggen->pop_measurement_fifo(
                pulse,
                port))
        {
            std::this_thread::sleep_for(
                std::chrono::microseconds(100));

            continue;
        }

        // -------------------------------------------------
        // Pulse time
        // -------------------------------------------------

        const double pulse_time_seconds =
            static_cast<double>(
                pulse.sample_index) / rate;

        // -------------------------------------------------
        // RX amplitude / dBFS
        // -------------------------------------------------

        const double rx_amp =
            (pulse.avg_power > 0)
                ? std::sqrt(
                      static_cast<double>(
                          pulse.avg_power))
                : 0.0;

        const double rx_dbfs =
            (rx_amp > 0.0)
                ? 20.0 * std::log10(
                      rx_amp / 32767.0)
                : -200.0;

        // -------------------------------------------------
        // PRI
        // -------------------------------------------------

        uint16_t sequence_delta = 0;
        uint64_t sample_delta   = 0;

        if (!first_measurement)
        {
            sequence_delta =
                static_cast<uint16_t>(
                    pulse.sequence -
                    last_sequence);

            sample_delta =
                pulse.sample_index -
                last_sample_index;
        }

        // -------------------------------------------------
        // Print every pulse
        // -------------------------------------------------

        std::cout
            << "SEQ="
            << pulse.sequence
            << " SAMPLE="
            << pulse.sample_index
            << " TIME_S="
            << std::fixed
            << std::setprecision(9)
            << pulse_time_seconds;

        if (!first_measurement)
        {
            const double delta_seconds =
                static_cast<double>(
                    sample_delta) / rate;

            const double pri_seconds =
                (sequence_delta > 0)
                    ? delta_seconds /
                      static_cast<double>(
                          sequence_delta)
                    : 0.0;

            const uint16_t missed =
                (sequence_delta > 0)
                    ? sequence_delta - 1
                    : 0;

            std::cout
                << " DELTA_S="
                << delta_seconds
                << " PRI_US="
                << pri_seconds * 1e6
                << " SEQ_DELTA="
                << sequence_delta
                << " MISSED="
                << missed;
        }

        std::cout
            << " RX_dBFS="
            << std::setprecision(2)
            << rx_dbfs
            << std::endl;


 // -------------------------------------------------
// Save measurement
// -------------------------------------------------

if (save_data)
{
    // Establish the time reference using the first FPGA measurement
    if (!timestamp_reference_valid)
    {
        reference_sample_index = pulse.sample_index;

        reference_timestamp_us =
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now()
                    .time_since_epoch())
                .count();

        timestamp_reference_valid = true;
    }

    // Convert FPGA sample index to elapsed microseconds
    const uint64_t sample_offset =
        pulse.sample_index - reference_sample_index;

    const int64_t timestamp_us =
        reference_timestamp_us
        + static_cast<int64_t>(
            (static_cast<double>(sample_offset) / rate)
            * 1e6);


    // -------------------------------------------------
    // GPS
    // -------------------------------------------------

    const gps_data_t gps = get_latest_gps_data();

    const std::string gps_time_str = std::get<0>(gps);
    const std::string lat_str      = std::get<1>(gps);
    const std::string lon_str      = std::get<2>(gps);
    const std::string alt_str      = std::get<3>(gps);

    double latitude  = 999.0;
    double longitude = 999.0;
    double altitude  = 99999.0;

    try
    {
        if (lat_str != "No Latitude")
            latitude = std::stod(lat_str);

        if (lon_str != "No Longitude")
            longitude = std::stod(lon_str);

        if (alt_str != "No Altitude")
            altitude = std::stod(alt_str);
    }
    catch (...)
    {
        latitude  = 999.0;
        longitude = 999.0;
        altitude  = 99999.0;
    }


    // -------------------------------------------------
    // Create binary record
    // -------------------------------------------------

    MeasurementRecord record{};

    record.timestamp_us = timestamp_us;

    std::memset(
        record.gps_time,
        0,
        sizeof(record.gps_time));

    if (gps_time_str != "No GPS Time")
    {
        std::strncpy(
            record.gps_time,
            gps_time_str.c_str(),
            sizeof(record.gps_time) - 1);
    }

    record.latitude  = latitude;
    record.longitude = longitude;
    record.altitude  = altitude;

    // This is currently dBFS.
    // We can calibrate to dBm later.
    record.amplitude_dbm = rx_dbfs;


    // -------------------------------------------------
    // Buffer measurement
    // -------------------------------------------------

    write_buffer.push_back(record);

    if (write_buffer.size() >= 512)
    {
        binary_file.write(
            reinterpret_cast<const char*>(
                write_buffer.data()),
            write_buffer.size()
                * sizeof(MeasurementRecord));

        if (!binary_file.good())
        {
            std::cerr
                << "Error writing measurement file"
                << std::endl;

            stop_signal_called = true;
            break;
        }

        write_buffer.clear();
    }
}

        // -------------------------------------------------
        // Update previous pulse
        // -------------------------------------------------

        last_sequence =
            pulse.sequence;

        last_sample_index =
            pulse.sample_index;

        first_measurement = false;
    }
}
catch (const std::exception& ex)
{
    std::cerr
        << "\nMeasurement FIFO exception: "
        << ex.what()
        << std::endl;

    stop_signal_called = true;
}
catch (...)
{
    std::cerr
        << "\nUnknown exception in measurement FIFO loop"
        << std::endl;

    stop_signal_called = true;
}

if (save_data && !write_buffer.empty())
{
    binary_file.write(
        reinterpret_cast<const char*>(
            write_buffer.data()),
        write_buffer.size() *
            sizeof(MeasurementRecord));

    if (!binary_file.good())
    {
        std::cerr
            << "Error writing final measurement data"
            << std::endl;
    }

    write_buffer.clear();
}




    // Stop radio
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
