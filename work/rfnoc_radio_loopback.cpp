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



namespace po = boost::program_options;
using uhd::rfnoc::radio_control;
using namespace std::chrono_literals;

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
    std::string args, rx_ant, tx_ant, rx_blockid, tx_blockid, ref, pps;
    size_t total_num_samps, spp, rx_chan, tx_chan, threshold, pulsewidth, delay, holdcount;
    double rate, rx_freq, tx_freq, rx_gain, tx_gain, rx_bw, tx_bw, total_time, setup_time;
    bool rx_timestamps;

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
        ("holdcount", po::value<size_t>(&holdcount)->default_value(2), "Samples per packet (reduce for lower latency)")
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


// --- Configure radios as you already do (rates, freqs, gains, etc.) ---

// Configure SigGen (example numbers)
const size_t port = 0;
const double tone_hz = 0;                 
siggen->set_samples_per_packet(spp, port);
siggen->set_waveform(uhd::rfnoc::siggen_waveform::SINE_WAVE, port);
siggen->set_amplitude(0.5, port);             // 0.0 .. 1.0
siggen->set_sine_phase_increment(0, port);

// Trigger gating (your new regs)
siggen->set_threshold(threshold /*LSBs*/, port);   // pick based on RX magnitude
//siggen->set_holdcount(holdcount /*LSBs*/, port);   
siggen->set_delay(delay /*ce_clk cycles*/, port);
siggen->set_pulsewidth(pulsewidth /*samples*/, port);
    std::cout << "delay= " << siggen->get_delay(port) << " ce clk cycles " << std::endl;
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
   // std::cout << "holdcount= " << siggen->get_holdcount(port) << " samples for trigger " << std::endl;
    
    


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




uint32_t last_avg_power = 0;
uint32_t last_tx_amp    = 0;

while (!stop_signal_called) {

    uint32_t avg_power = siggen->get_avg_power(port);
    uint32_t tx_amp    = siggen->get_tx_amp(port) & 0xFFFF;

    if ((avg_power != last_avg_power) || (tx_amp != last_tx_amp)) {

        double rx_amp =
            (avg_power > 0)
            ? std::sqrt((double)avg_power)
            : 0.0;

        double rx_dbfs =
            (rx_amp > 0.0)
            ? 20.0 * std::log10(rx_amp / 32767.0)
            : -200.0;

        double tx_dbfs =
            (tx_amp > 0)
            ? 20.0 * std::log10((double)tx_amp / 32767.0)
            : -200.0;

        std::cout
            << "RX_dBFS=" << std::fixed << std::setprecision(2)
            << rx_dbfs
            << " TX_dBFS=" << tx_dbfs
            << " AVG_POWER=" << avg_power
            << " TX_AMP=" << tx_amp
            << std::endl;

        last_avg_power = avg_power;
        last_tx_amp    = tx_amp;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

    // Stop radio
    stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
    std::cout << "Issuing stop stream cmd..." << std::endl;
    rx_radio_ctrl->issue_stream_cmd(stream_cmd, rx_chan);
    std::cout << "Done" << std::endl;
    // Allow for the samples and ACKs to propagate
    std::this_thread::sleep_for(100ms);

    return EXIT_SUCCESS;
}
