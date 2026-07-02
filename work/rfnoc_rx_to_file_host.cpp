//
// Copyright 2014-2016 Ettus Research LLC
// Copyright 2019 Ettus Research, a National Instruments Brand
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhd/exception.hpp>
#include <uhd/rfnoc/ddc_block_control.hpp>
#include <uhd/rfnoc/defaults.hpp>
#include <uhd/rfnoc/mb_controller.hpp>
#include <uhd/rfnoc/radio_control.hpp>
#include <uhd/rfnoc_graph.hpp>
#include <uhd/types/sensors.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/utils/graph_utils.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/thread.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <complex>
#include <csignal>
#include <fstream>
#include <functional>
#include <iostream>
#include <thread>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <mutex>
#include <queue>
#include <regex>
#include <vector>
#include <tuple>


// Define USRP IP or hostname
const std::string USRP_HOST = "root@192.168.0.100"; 

std::mutex gps_mutex;
std::queue<std::tuple<std::string, std::string, std::string, std::string>> gps_data_queue;
bool keep_streaming = true;

// Function to extract GPS time, latitude, longitude, and altitude from TPV messages
// Function to extract GPS time, latitude, longitude, and altitude

// Function to extract GPS time, latitude, longitude, and altitude
std::tuple<std::string, std::string, std::string, std::string> extract_gps_data(const std::string& gps_data) {
    std::string gps_time = "No GPS Time";
    std::string latitude = "No Latitude";
    std::string longitude = "No Longitude";
    std::string altitude = "No Altitude";

    // Ensure this is a TPV message
    if (gps_data.find("\"class\":\"TPV\"") == std::string::npos) {
        return {gps_time, latitude, longitude, altitude};
    }

    // Match Time
    std::regex time_regex(R"("time"\s*:\s*"\d{4}-\d{2}-\d{2}T(\d{2}:\d{2}:\d{2})\.\d{3}Z")");
    std::smatch time_match;
    if (std::regex_search(gps_data, time_match, time_regex)) {
        gps_time = time_match[1];
    }

    // Match Latitude
    std::regex lat_regex(R"("lat"\s*:\s*([-+]?\d+\.\d+))");
    std::smatch lat_match;
    if (std::regex_search(gps_data, lat_match, lat_regex)) {
        latitude = lat_match[1];
    }

    // Match Longitude
    std::regex lon_regex(R"("lon"\s*:\s*([-+]?\d+\.\d+))");
    std::smatch lon_match;
    if (std::regex_search(gps_data, lon_match, lon_regex)) {
        longitude = lon_match[1];
    }

    // Match Altitude
    std::regex alt_regex(R"("alt(?:HAE|MSL)?"\s*:\s*([-+]?\d+\.\d+))");
    std::smatch alt_match;
    if (std::regex_search(gps_data, alt_match, alt_regex)) {
        altitude = alt_match[1];
    }

    return {gps_time, latitude, longitude, altitude};
}
void gps_stream_thread() {
    std::cout << "[GPS Thread] Starting GPS streaming..." << std::endl;
    //std::string ssh_command = "ssh " + USRP_HOST + " gpspipe -w";  // Connect to USRP
    //FILE* pipe = popen(ssh_command.c_str(), "r");
    FILE* pipe = popen("gpspipe -w", "r");   // runs on the USRP itself
    if (!pipe) {
        std::cerr << "[GPS Thread] Failed to connect to GPS" << std::endl;
        return;
    }

    char buffer[2048];  // Larger buffer for full JSON messages
    std::string gps_data_buffer = "";  // Accumulate incoming data

    while (keep_streaming && fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        gps_data_buffer += buffer;  // Append new data to buffer

        // Process full JSON messages
        size_t start = gps_data_buffer.find("{");
        size_t end = gps_data_buffer.find("}", start);
        
        while (start != std::string::npos && end != std::string::npos) {
            std::string full_message = gps_data_buffer.substr(start, end - start + 1);
            gps_data_buffer.erase(0, end + 1);  // Remove processed message
            
            std::cout << "[RAW GPS Data]: " << full_message << std::endl;

            auto [gps_time, latitude, longitude, altitude] = extract_gps_data(full_message);

            if (gps_time != "No GPS Time") {
                std::lock_guard<std::mutex> lock(gps_mutex);
                gps_data_queue.push({gps_time, latitude, longitude, altitude});
                if (gps_data_queue.size() > 10) {
                    gps_data_queue.pop();  // Keep queue small, store only latest timestamps
                }
            }

            // Look for next JSON object
            start = gps_data_buffer.find("{");
            end = gps_data_buffer.find("}", start);
        }
    }
    pclose(pipe);
}




std::tuple<std::string, std::string, std::string, std::string> get_latest_gps_data() {
    std::lock_guard<std::mutex> lock(gps_mutex);

    if (!gps_data_queue.empty()) {
        return gps_data_queue.back(); // Return latest GPS entry
    }

    return {"No GPS Time", "No Latitude", "No Longitude", "No Altitude"};
}


namespace po = boost::program_options;

constexpr int64_t UPDATE_INTERVAL = 1; // 1 second update interval for BW summary

static bool stop_signal_called = false;
void sig_int_handler(int)
{
    stop_signal_called = true;
}

template <typename samp_type>
void recv_to_file(uhd::rx_streamer::sptr rx_stream,
    const std::string& file,
    const size_t samps_per_buff,
    const double rx_rate,
    const unsigned long long num_requested_samples,
    double time_requested       = 0.0,
    bool bw_summary             = false,
    bool stats                  = false,
    bool enable_size_map        = false,
    bool continue_on_bad_packet = false,
    double threshold= 0)
{
    unsigned long long num_total_samps = 0;

    uhd::rx_metadata_t md;
    std::vector<samp_type> buff(samps_per_buff);
    std::ofstream outfile;
    if (not file.empty()) {
        outfile.open(file.c_str(), std::ofstream::binary);
    }
    bool overflow_message = true;

    // setup streaming
    uhd::stream_cmd_t stream_cmd((num_requested_samples == 0)
                                     ? uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS
                                     : uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps  = size_t(num_requested_samples);
    stream_cmd.stream_now = true;
    stream_cmd.time_spec  = uhd::time_spec_t();
    std::cout << "Issuing stream cmd" << std::endl;
    rx_stream->issue_stream_cmd(stream_cmd);

    const auto start_time = std::chrono::steady_clock::now();
    const auto stop_time =
        start_time + std::chrono::milliseconds(int64_t(1000 * time_requested));
    // Track time and samps between updating the BW summary
    auto last_update                     = start_time;
    unsigned long long last_update_samps = 0;

    typedef std::map<size_t, size_t> SizeMap;
    SizeMap mapSizes;

    // Run this loop until either time expired (if a duration was given), until
    // the requested number of samples were collected (if such a number was
    // given), or until Ctrl-C was pressed.
    while (not stop_signal_called
           and (num_requested_samples != num_total_samps or num_requested_samples == 0)
           and (time_requested == 0.0 or std::chrono::steady_clock::now() <= stop_time)) {
        const auto now = std::chrono::steady_clock::now();

        //std::cout << "th: " << threshold << " ..." << std::endl;
        size_t num_rx_samps =
            rx_stream->recv(&buff.front(), buff.size(), md, 3.0, enable_size_map);

        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
            std::cout << "Timeout while streaming" << std::endl;
            break;
        }
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
            if (overflow_message) {
                overflow_message = false;
                std::cerr
                    << "Got an overflow indication. Please consider the following:\n"
                       "  Your write medium must sustain a rate of "
                    << (rx_rate * sizeof(samp_type) / 1e6)
                    << "MB/s.\n"
                       "  Dropped samples will not be written to the file.\n"
                       "  Please modify this example for your purposes.\n"
                       "  This message will not appear again.\n";
            }
            continue;
        }
        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
            std::string error = std::string("Receiver error: ") + md.strerror();
            if (continue_on_bad_packet) {
                std::cerr << error << std::endl;
                continue;
            } else
                throw std::runtime_error(error);
        }

        if (enable_size_map) {
            SizeMap::iterator it = mapSizes.find(num_rx_samps);
            if (it == mapSizes.end())
                mapSizes[num_rx_samps] = 0;
            mapSizes[num_rx_samps] += 1;
        }

        num_total_samps += num_rx_samps;

        // Loop through the samples and check amplitude before writing to file
        double amplitude = 0.0;
        std::vector<samp_type> buffer(10);
        for (size_t i = 0; i < num_rx_samps; ++i) {
            amplitude = 20*log10(std::abs(buff[i])) + 7 ;
            //      std::cout << "num_rx_samps:" << num_rx_samps << " ..." << std::endl;


           if (amplitude >= threshold) {
                // ✅ Get System Time in Microseconds
                auto now = std::chrono::system_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                    now.time_since_epoch()).count();
                
                // ✅ Get Latest GPS Data (time, lat, lon, alt)
                auto [gps_time, latitude, longitude, altitude] = get_latest_gps_data();
                
                // ✅ Print to Console for Debugging
                std::cout << "[SAVING] TIME: " << duration << " µs | GPS: " 
                          << gps_time << " | Lat: " << latitude << " | Lon: " 
                          << longitude << " | Alt: " << altitude 
                          << " | Amplitude: " << amplitude << " dBm" << std::endl;

 if (outfile.is_open()) {
                    outfile.write(reinterpret_cast<const char*>(&duration), sizeof(duration));
                    
                    // Write fixed-size GPS time (20 bytes, including null terminator)
                    char gps_time_str[20] = {};
                    strncpy(gps_time_str, gps_time.c_str(), sizeof(gps_time_str) - 1);
                    outfile.write(gps_time_str, sizeof(gps_time_str));

                    // Write Latitude
                    double lat = latitude == "No Latitude" ? 0.0 : std::stod(latitude);
                    outfile.write(reinterpret_cast<const char*>(&lat), sizeof(lat));

                    // Write Longitude
                    double lon = longitude == "No Longitude" ? 0.0 : std::stod(longitude);
                    outfile.write(reinterpret_cast<const char*>(&lon), sizeof(lon));

                    // Write Altitude
                    double alt = altitude == "No Altitude" ? 0.0 : std::stod(altitude);
                    outfile.write(reinterpret_cast<const char*>(&alt), sizeof(alt));

                    // Write Amplitude
                    outfile.write(reinterpret_cast<const char*>(&amplitude), sizeof(amplitude));
                }



            }
        }




            //  std::cout << "end of loop" <<std::endl;
            if (bw_summary) {
                last_update_samps += num_rx_samps;
                const auto time_since_last_update = now - last_update;
                if (time_since_last_update > std::chrono::seconds(UPDATE_INTERVAL)) {
                    const double time_since_last_update_s =
                        std::chrono::duration<double>(time_since_last_update).count();
                    const double rate = double(last_update_samps) / time_since_last_update_s;
                    std::cout << "\t" << (rate / 1e6) << " MSps" << std::endl;
                    last_update_samps = 0;
                    last_update       = now;
                }
            }

    }
    const auto actual_stop_time = std::chrono::steady_clock::now();

    stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
    std::cout << "Issuing stop stream cmd" << std::endl;
    rx_stream->issue_stream_cmd(stream_cmd);

    // Run recv until nothing is left
    int num_post_samps = 0;
    do {
        num_post_samps = rx_stream->recv(&buff.front(), buff.size(), md, 3.0);
    } while (num_post_samps and md.error_code == uhd::rx_metadata_t::ERROR_CODE_NONE);

    if (outfile.is_open())
        outfile.close();

    if (stats) {
        std::cout << std::endl;

        const double actual_duration_seconds =
            std::chrono::duration<float>(actual_stop_time - start_time).count();

        std::cout << "Received " << num_total_samps << " samples in "
                  << actual_duration_seconds << " seconds" << std::endl;
        const double rate = (double)num_total_samps / actual_duration_seconds;
        std::cout << (rate / 1e6) << " MSps" << std::endl;

        if (enable_size_map) {
            std::cout << std::endl;
            std::cout << "Packet size map (bytes: count)" << std::endl;
            for (SizeMap::iterator it = mapSizes.begin(); it != mapSizes.end(); it++)
                std::cout << it->first << ":\t" << it->second << std::endl;
        }
    }
}

typedef std::function<uhd::sensor_value_t(const std::string&)> get_sensor_fn_t;

bool check_locked_sensor(std::vector<std::string> sensor_names,
    const char* sensor_name,
    get_sensor_fn_t get_sensor_fn,
    double setup_time)
{
    if (std::find(sensor_names.begin(), sensor_names.end(), sensor_name)
        == sensor_names.end())
        return false;

    auto setup_timeout = std::chrono::steady_clock::now()
                         + std::chrono::milliseconds(int64_t(setup_time * 1000));
    bool lock_detected = false;

    std::cout << "Waiting for \"" << sensor_name << "\": ";
    std::cout.flush();

    while (true) {
        if (lock_detected and (std::chrono::steady_clock::now() > setup_timeout)) {
            std::cout << " locked." << std::endl;
            break;
        }
        if (get_sensor_fn(sensor_name).to_bool()) {
            std::cout << "+";
            std::cout.flush();
            lock_detected = true;
        } else {
            if (std::chrono::steady_clock::now() > setup_timeout) {
                std::cout << std::endl;
                throw std::runtime_error(
                    std::string("timed out waiting for consecutive locks on sensor \"")
                    + sensor_name + "\"");
            }
            std::cout << "_";
            std::cout.flush();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << std::endl;
    return true;
}

int UHD_SAFE_MAIN(int argc, char* argv[])
{
    // variables to be set by po
    std::string args, file, format, ant, subdev, ref, wirefmt, streamargs, block_id,
        block_props;
    size_t total_num_samps, spb, spp, radio_id, radio_chan, block_port;
    double rate, freq, gain, bw, total_time, setup_time, lo_offset, threshold;

    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help", "help message")
        ("file", po::value<std::string>(&file)->default_value("usrp_samples.dat"), "name of the file to write binary samples to")
        ("format", po::value<std::string>(&format)->default_value("fc64"), "File sample format: sc16, fc32, or fc64")
        ("duration", po::value<double>(&total_time)->default_value(0), "total number of seconds to receive")
        ("nsamps", po::value<size_t>(&total_num_samps)->default_value(0), "total number of samples to receive")
        ("spb", po::value<size_t>(&spb)->default_value(1000), "samples per buffer")
        ("threshold", po::value<double>(&threshold)->default_value(-20), "samples per buffer")
        ("spp", po::value<size_t>(&spp), "samples per packet (on FPGA and wire)")
        ("streamargs", po::value<std::string>(&streamargs)->default_value(""), "stream args")
        ("progress", "periodically display short-term bandwidth")
        ("stats", "show average bandwidth on exit")
        ("sizemap", "track packet size and display breakdown on exit")
        ("null", "run without writing to file")
        ("continue", "don't abort on a bad packet")

        ("args", po::value<std::string>(&args)->default_value(""), "USRP device address args")
        ("setup", po::value<double>(&setup_time)->default_value(1.0), "seconds of setup time")
        

        ("radio-id", po::value<size_t>(&radio_id)->default_value(0), "Radio ID to use (0 or 1).")
        ("radio-chan", po::value<size_t>(&radio_chan)->default_value(0), "Radio channel")
        ("rate", po::value<double>(&rate)->default_value(1e6), "RX rate of the radio block")
        ("freq", po::value<double>(&freq)->default_value(200e6), "RF center frequency in Hz")
        ("lo-offset", po::value<double>(&lo_offset), "Offset for frontend LO in Hz (optional)")
        ("gain", po::value<double>(&gain), "gain for the RF chain")
        ("ant", po::value<std::string>(&ant), "antenna selection")
        ("bw", po::value<double>(&bw), "analog frontend filter bandwidth in Hz")
        ("ref", po::value<std::string>(&ref), "reference source (internal, external, mimo)")
        ("skip-lo", "skip checking LO lock status")
        ("int-n", "tune USRP with integer-N tuning")

        ("block-id", po::value<std::string>(&block_id), "If block ID is specified, this block is inserted between radio and host.")
        ("block-port", po::value<size_t>(&block_port)->default_value(0), "If block ID is specified, this block is inserted between radio and host.")
        ("block-props", po::value<std::string>(&block_props), "These are passed straight to the block as properties (see set_properties()).")
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // print the help message
    if (vm.count("help")) {
        std::cout << "UHD/RFNoC RX samples to file " << desc << std::endl;
        std::cout << std::endl
                  << "This application streams data from a single channel of a USRP "
                     "device to a file.\n"
                  << std::endl;
        return ~0;
    }

    bool bw_summary = vm.count("progress") > 0;
    bool stats      = vm.count("stats") > 0;
    if (vm.count("null") > 0) {
        file = "";
    }
    bool enable_size_map        = vm.count("sizemap") > 0;
    bool continue_on_bad_packet = vm.count("continue") > 0;

    if (enable_size_map) {
        std::cout << "Packet size tracking enabled - will only recv one packet at a time!"
                  << std::endl;
    }

    if (format != "sc16" and format != "fc32" and format != "fc64") {
        std::cout << "Invalid sample format: " << format << std::endl;
        return EXIT_FAILURE;
    }

    /************************************************************************
     * Create device and block controls
     ***********************************************************************/
    std::cout << std::endl;
    std::cout << "Creating the RFNoC graph with args: " << args << std::endl;
    auto graph = uhd::rfnoc::rfnoc_graph::make(args);

    // Create handle for radio object
    uhd::rfnoc::block_id_t radio_ctrl_id(0, "Radio", radio_id);
    // This next line will fail if the radio is not actually available
    auto radio_ctrl = graph->get_block<uhd::rfnoc::radio_control>(radio_ctrl_id);
    std::cout << "Using radio " << radio_id << ", channel " << radio_chan << std::endl;

    uhd::rfnoc::block_id_t last_block_in_chain;
    size_t last_port_in_chain;
    uhd::rfnoc::ddc_block_control::sptr ddc_ctrl;
    size_t ddc_chan       = 0;
    bool user_block_found = false;

    { // First, connect everything dangling off of the radio
        auto edges = uhd::rfnoc::get_block_chain(graph, radio_ctrl_id, radio_chan, true);
        last_block_in_chain = edges.back().src_blockid;
        last_port_in_chain  = edges.back().src_port;
        if (edges.size() > 1) {
            uhd::rfnoc::connect_through_blocks(graph,
                radio_ctrl_id,
                radio_chan,
                last_block_in_chain,
                last_port_in_chain);
            for (auto& edge : edges) {
                if (uhd::rfnoc::block_id_t(edge.dst_blockid).get_block_name() == "DDC") {
                    ddc_ctrl =
                        graph->get_block<uhd::rfnoc::ddc_block_control>(edge.dst_blockid);
                    ddc_chan = edge.dst_port;
                }
                if (vm.count("block-id") && edge.dst_blockid == block_id) {
                    user_block_found = true;
                }
            }
        }
    }

    // If the user block is not in the chain yet, see if we can connect that
    // separately
    if (vm.count("block-id") && !user_block_found) {
        const auto user_block_id = uhd::rfnoc::block_id_t(block_id);
        if (!graph->has_block(user_block_id)) {
            std::cout << "ERROR! No such block: " << block_id << std::endl;
            return EXIT_FAILURE;
        }
        std::cout << "Attempting to connect " << block_id << ":" << last_port_in_chain
                  << " to " << last_block_in_chain << ":" << block_port << "..."
                  << std::endl;
        uhd::rfnoc::connect_through_blocks(
            graph, last_block_in_chain, last_port_in_chain, user_block_id, block_port);
        last_block_in_chain = uhd::rfnoc::block_id_t(block_id);
        last_port_in_chain  = block_port;
        // Now we have to make sure that there are no more static connections
        // after the user-defined block
        auto edges = uhd::rfnoc::get_block_chain(
            graph, last_block_in_chain, last_port_in_chain, true);
        if (edges.size() > 1) {
            uhd::rfnoc::connect_through_blocks(graph,
                last_block_in_chain,
                last_port_in_chain,
                edges.back().src_blockid,
                edges.back().src_port);
            last_block_in_chain = edges.back().src_blockid;
            last_port_in_chain  = edges.back().src_port;
        }
    }
    /************************************************************************
     * Set up radio
     ***********************************************************************/
    // Lock mboard clocks
    if (vm.count("ref")) {
        graph->get_mb_controller(0)->set_clock_source(ref);
    }

    // set the rf gain
    if (vm.count("gain")) {
        std::cout << "Requesting RX Gain: " << gain << " dB..." << std::endl;
        radio_ctrl->set_rx_gain(gain, radio_chan);
        std::cout << "Actual RX Gain: " << radio_ctrl->get_rx_gain(radio_chan) << " dB..."
                  << std::endl
                  << std::endl;
    }

    // set the IF filter bandwidth
    if (vm.count("bw")) {
        std::cout << "Requesting RX Bandwidth: " << (bw / 1e6) << " MHz..." << std::endl;
        radio_ctrl->set_rx_bandwidth(bw, radio_chan);
        std::cout << "Actual RX Bandwidth: "
                  << (radio_ctrl->get_rx_bandwidth(radio_chan) / 1e6) << " MHz..."
                  << std::endl
                  << std::endl;
    }

    // set the antenna
    if (vm.count("ant")) {
        radio_ctrl->set_rx_antenna(ant, radio_chan);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(int64_t(1000 * setup_time)));

    // check Ref and LO Lock detect
    if (not vm.count("skip-lo")) {
        check_locked_sensor(
            radio_ctrl->get_rx_sensor_names(radio_chan),
            "lo_locked",
            [&](const std::string& sensor_name) {
                return radio_ctrl->get_rx_sensor(sensor_name, radio_chan);
            },
            setup_time);
        if (ref == "external") {
            check_locked_sensor(
                graph->get_mb_controller(0)->get_sensor_names(),
                "ref_locked",
                [&](const std::string& sensor_name) {
                    return graph->get_mb_controller(0)->get_sensor(sensor_name);
                },
                setup_time);
        }
    }

    if (vm.count("spp")) {
        std::cout << "Requesting samples per packet of: " << spp << std::endl;
        radio_ctrl->set_property<int>("spp", spp, radio_chan);
        spp = radio_ctrl->get_property<int>("spp", radio_chan);
        std::cout << "Actual samples per packet = " << spp << std::endl;
    }

    /************************************************************************
     * Set up streaming
     ***********************************************************************/
    uhd::device_addr_t streamer_args(streamargs);

    // create a receive streamer
    uhd::stream_args_t stream_args(
        format, "sc16"); // We should read the wire format from the blocks
    stream_args.args = streamer_args;
    std::cout << "Using streamer args: " << stream_args.args.to_string() << std::endl;
    auto rx_stream = graph->create_rx_streamer(1, stream_args);

    // Connect streamer to last block and commit the graph
    graph->connect(last_block_in_chain, last_port_in_chain, rx_stream, 0);
    graph->commit();
    std::cout << "Active connections:" << std::endl;
    for (auto& edge : graph->enumerate_active_connections()) {
        std::cout << "* " << edge.to_string() << std::endl;
    }

    /************************************************************************
     * Set up sampling rate and (optional) user block properties. We do this
     * after commit() so we can use the property propagation.
     ***********************************************************************/

    // set the center frequency
    if (vm.count("freq")) {
        std::cout << "Requesting RX Freq: " << (freq / 1e6) << " MHz..." << std::endl;
        uhd::tune_request_t tune_request(freq);
        if (vm.count("int-n")) {
            tune_request.args = uhd::device_addr_t("mode_n=integer");
        }
        radio_ctrl->set_rx_frequency(freq, radio_chan);
        std::cout << boost::format("Actual RX Freq: %f MHz...")
                         % (radio_ctrl->get_rx_frequency(radio_chan) / 1e6)
                  << std::endl
                  << std::endl;
	}

      


    // set the sample rate
    if (rate <= 0.0) {
        std::cerr << "Please specify a valid sample rate" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Requesting RX Rate: " << (rate / 1e6) << " Msps..." << std::endl;
    if (ddc_ctrl) {
        std::cout << "Setting rate on DDC block!" << std::endl;
        rate = ddc_ctrl->set_output_rate(rate, ddc_chan);
    } else {
        std::cout << "Setting rate on radio block!" << std::endl;
        rate = radio_ctrl->set_rate(rate);
    }
    std::cout << "Actual RX Rate: " << (rate / 1e6) << " Msps..." << std::endl
              << std::endl;

    if (vm.count("block-props")) {
        std::cout << "Setting block properties to: " << block_props << std::endl;
        graph->get_block(uhd::rfnoc::block_id_t(block_id))
            ->set_properties(uhd::device_addr_t(block_props));
    }

    if (total_num_samps == 0) {
        std::signal(SIGINT, &sig_int_handler);
        std::cout << "Press Ctrl + C to stop streaming..." << std::endl;
    }
    
    std::cout << "Starting GPS streaming and sample processing..." << std::endl;
    
    std::thread gps_thread(gps_stream_thread);
    
#define recv_to_file_args() \
    (rx_stream,             \
        file,               \
        spb,                \
        rate,               \
        total_num_samps,    \
        total_time,         \
        bw_summary,         \
        stats,              \
        enable_size_map,    \
        continue_on_bad_packet,   \
        threshold )
    // recv to file
    if (format == "fc64")
        recv_to_file<std::complex<double>> recv_to_file_args();
    else if (format == "fc32")
        recv_to_file<std::complex<float>> recv_to_file_args();
    else if (format == "sc16")
        recv_to_file<std::complex<short>> recv_to_file_args();
    else
        throw std::runtime_error("Unknown data format: " + format);

    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;

    return EXIT_SUCCESS;
}
