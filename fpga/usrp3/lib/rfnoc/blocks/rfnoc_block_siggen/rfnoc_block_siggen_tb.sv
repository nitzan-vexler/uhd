//
// Copyright 2020 Ettus Research, a National Instruments Brand
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//
// Module: rfnoc_block_siggen_tb
//
// Description: Testbench for the siggen RFNoC block.
//

`default_nettype none


module rfnoc_block_siggen_tb #(
  parameter CHDR_W    = 64,
  parameter NUM_PORTS = 1
);

  `include "test_exec.svh"

  import PkgTestExec::*;
  import rfnoc_chdr_utils_pkg::*;
  import PkgChdrData::*;
  import PkgRfnocBlockCtrlBfm::*;
  import PkgRfnocItemUtils::*;

  `include "rfnoc_block_siggen_regs.vh"


  //---------------------------------------------------------------------------
  // Testbench Configuration
  //---------------------------------------------------------------------------

  localparam [31:0] NOC_ID          = 32'h51663110;
  localparam [ 9:0] THIS_PORTID     = 10'h123;
  localparam int    MTU             = 10;    // Log2 of max transmission unit in CHDR words
  localparam int    NUM_PORTS_I     = 1;
  localparam int    NUM_PORTS_O     = 0+NUM_PORTS;
  localparam int    ITEM_W          = 32;    // Sample size in bits
  localparam int    SPP             = 64;    // Samples per packet
  localparam int    PKT_SIZE_BYTES  = SPP * (ITEM_W/8);
  localparam int    STALL_PROB      = 25;    // Default BFM stall probability
  localparam real   CHDR_CLK_PER    = 5.0;   // 200 MHz
  localparam real   CTRL_CLK_PER    = 8.0;   // 125 MHz
  localparam real   CE_CLK_PER      = 4.0;   // 250 MHz

  localparam real PI = 2*$acos(0);

  // Number of fractional bits used for fixed point values of the different
  // settings (derived from the DUT).
  localparam int GAIN_FRAC  = 15;
  localparam int CONST_FRAC = 15;
  localparam int PHASE_FRAC = 13;
  localparam int CART_FRAC  = 14;

  // Maximum real (floating point) values allowed for the different fixed
  // point formats (for range checking). All of the fixed point values are
  // signed 16-bit.
  localparam real MAX_GAIN_R  =  (2.0**15-1) / (2.0**GAIN_FRAC);
  localparam real MIN_GAIN_R  = -(2.0**15)   / (2.0**GAIN_FRAC);
  localparam real MAX_CONST_R =  (2.0**15-1) / (2.0**CONST_FRAC);
  localparam real MIN_CONST_R = -(2.0**15)   / (2.0**CONST_FRAC);
  localparam real MAX_CART_R  =  (2.0**15-1) / (2.0**CART_FRAC);
  localparam real MIN_CART_R  = -(2.0**15)   / (2.0**CART_FRAC);
  // Note that the CORDIC only supports phase values from -1.0 to +1.0.
  localparam real MAX_PHASE_R = +1.0;
  localparam real MIN_PHASE_R = -1.0;


  //---------------------------------------------------------------------------
  // Clocks and Resets
  //---------------------------------------------------------------------------

  bit rfnoc_chdr_clk;
  bit rfnoc_ctrl_clk;
  bit ce_clk;

  sim_clock_gen #(.PERIOD(CHDR_CLK_PER), .AUTOSTART(0))
   rfnoc_chdr_clk_gen (.clk(rfnoc_chdr_clk), .rst());
  sim_clock_gen #(.PERIOD(CTRL_CLK_PER), .AUTOSTART(0))
   rfnoc_ctrl_clk_gen (.clk(rfnoc_ctrl_clk), .rst());
  sim_clock_gen #(.PERIOD(CE_CLK_PER), .AUTOSTART(0))
     ce_clk_gen (.clk(ce_clk), .rst());


  //---------------------------------------------------------------------------
  // Bus Functional Models
  //---------------------------------------------------------------------------

  // Backend Interface
  RfnocBackendIf backend (rfnoc_chdr_clk, rfnoc_ctrl_clk);

  // AXIS-Ctrl Interface
  AxiStreamIf #(32) m_ctrl (rfnoc_ctrl_clk, 1'b0);
  AxiStreamIf #(32) s_ctrl (rfnoc_ctrl_clk, 1'b0);

  // AXIS-CHDR Interfaces
  AxiStreamIf #(CHDR_W) m_chdr [NUM_PORTS_I] (rfnoc_chdr_clk, 1'b0);
  AxiStreamIf #(CHDR_W) s_chdr [NUM_PORTS_O] (rfnoc_chdr_clk, 1'b0);

  // Block Controller BFM
  RfnocBlockCtrlBfm #(CHDR_W, ITEM_W) blk_ctrl = new(backend, m_ctrl, s_ctrl);

  // CHDR word and item/sample data types
  typedef ChdrData #(CHDR_W, ITEM_W)::chdr_word_t chdr_word_t;
  typedef ChdrData #(CHDR_W, ITEM_W)::item_t      item_t;

  // Connect block controller to BFMs
  for (genvar i = 0; i < NUM_PORTS_I; i++) begin : gen_bfm_input_connections
    initial begin
      blk_ctrl.connect_master_data_port(i, m_chdr[i], PKT_SIZE_BYTES);
      blk_ctrl.set_master_stall_prob(i, STALL_PROB);
    end
  end
  for (genvar i = 0; i < NUM_PORTS_O; i++) begin : gen_bfm_output_connections
    initial begin
      blk_ctrl.connect_slave_data_port(i, s_chdr[i]);
      blk_ctrl.set_slave_stall_prob(i, STALL_PROB);
    end
  end


  //---------------------------------------------------------------------------
  // Device Under Test (DUT)
  //---------------------------------------------------------------------------

  // DUT Slave (Input) Port Signals
  logic [CHDR_W*NUM_PORTS_I-1:0] s_rfnoc_chdr_tdata;
  logic [       NUM_PORTS_I-1:0] s_rfnoc_chdr_tlast;
  logic [       NUM_PORTS_I-1:0] s_rfnoc_chdr_tvalid;
  logic [       NUM_PORTS_I-1:0] s_rfnoc_chdr_tready;

  // DUT Master (Output) Port Signals
  logic [CHDR_W*NUM_PORTS_O-1:0] m_rfnoc_chdr_tdata;
  logic [       NUM_PORTS_O-1:0] m_rfnoc_chdr_tlast;
  logic [       NUM_PORTS_O-1:0] m_rfnoc_chdr_tvalid;
  logic [       NUM_PORTS_O-1:0] m_rfnoc_chdr_tready;

  // Map the array of BFMs to a flat vector for the DUT connections
  for (genvar i = 0; i < NUM_PORTS_I; i++) begin : gen_dut_input_connections
    // Connect BFM master to DUT slave port
    assign s_rfnoc_chdr_tdata[CHDR_W*i+:CHDR_W] = m_chdr[i].tdata;
    assign s_rfnoc_chdr_tlast[i]                = m_chdr[i].tlast;
    assign s_rfnoc_chdr_tvalid[i]               = m_chdr[i].tvalid;
    assign m_chdr[i].tready                     = s_rfnoc_chdr_tready[i];
  end
  for (genvar i = 0; i < NUM_PORTS_O; i++) begin : gen_dut_output_connections
    // Connect BFM slave to DUT master port
    assign s_chdr[i].tdata        = m_rfnoc_chdr_tdata[CHDR_W*i+:CHDR_W];
    assign s_chdr[i].tlast        = m_rfnoc_chdr_tlast[i];
    assign s_chdr[i].tvalid       = m_rfnoc_chdr_tvalid[i];
    assign m_rfnoc_chdr_tready[i] = s_chdr[i].tready;
  end

  rfnoc_block_siggen #(
    .THIS_PORTID         (THIS_PORTID),
    .CHDR_W              (CHDR_W),
    .MTU                 (MTU),
    .NUM_PORTS           (NUM_PORTS)
  ) dut (
    .rfnoc_chdr_clk      (rfnoc_chdr_clk),
    .rfnoc_ctrl_clk      (rfnoc_ctrl_clk),
    .ce_clk              (ce_clk),
    .rfnoc_core_config   (backend.cfg),
    .rfnoc_core_status   (backend.sts),
    .s_rfnoc_chdr_tdata  (s_rfnoc_chdr_tdata),
    .s_rfnoc_chdr_tlast  (s_rfnoc_chdr_tlast),
    .s_rfnoc_chdr_tvalid (s_rfnoc_chdr_tvalid),
    .s_rfnoc_chdr_tready (s_rfnoc_chdr_tready),
    .m_rfnoc_chdr_tdata  (m_rfnoc_chdr_tdata),
    .m_rfnoc_chdr_tlast  (m_rfnoc_chdr_tlast),
    .m_rfnoc_chdr_tvalid (m_rfnoc_chdr_tvalid),
    .m_rfnoc_chdr_tready (m_rfnoc_chdr_tready),
    .s_rfnoc_ctrl_tdata  (m_ctrl.tdata),
    .s_rfnoc_ctrl_tlast  (m_ctrl.tlast),
    .s_rfnoc_ctrl_tvalid (m_ctrl.tvalid),
    .s_rfnoc_ctrl_tready (m_ctrl.tready),
    .m_rfnoc_ctrl_tdata  (s_ctrl.tdata),
    .m_rfnoc_ctrl_tlast  (s_ctrl.tlast),
    .m_rfnoc_ctrl_tvalid (s_ctrl.tvalid),
    .m_rfnoc_ctrl_tready (s_ctrl.tready)
  );


  // --------------------------------------------------------------------------
  // RFNoC input metadata monitor (observes the actual NoC shell output)
  // --------------------------------------------------------------------------
  bit timestamp_probe_active = 0;
  int unsigned timestamp_probe_samples = 0;
  int unsigned timestamp_probe_packets = 0;
  int unsigned timestamp_probe_timed_samples = 0;
  bit timestamp_probe_first = 1;
  // Enable strict timestamp checks only for the timestamped-CHDR test.
  bit timestamp_check_active = 0;
  int unsigned timestamp_packet_sample_count = 0;


  always @(posedge ce_clk) begin
    if (dut.ce_rst) begin
      timestamp_probe_first = 1;
    end else if (timestamp_probe_active &&
                 dut.s_in_axis_tvalid[0] && dut.s_in_axis_tready[0]) begin
      timestamp_probe_samples++;
      if (timestamp_check_active) begin
        // The BFM sends exactly two 64-sample timed packets.
        `ASSERT_ERROR(timestamp_probe_packets < 2,
                      "Received an unexpected third timed packet");
        `ASSERT_ERROR(dut.s_in_axis_thas_time[0] === 1'b1,
                      "CHDR timestamp-valid bit was not propagated");
        if (timestamp_probe_packets == 0) begin
          `ASSERT_ERROR(dut.s_in_axis_ttimestamp[63:0] === 64'd100000,
                        "First packet timestamp is not 100000");
        end else begin
          `ASSERT_ERROR(dut.s_in_axis_ttimestamp[63:0] === 64'd200000,
                        "Second packet timestamp is not 200000");
        end
        timestamp_packet_sample_count++;
        `ASSERT_ERROR(timestamp_packet_sample_count <= 64,
                      "Timed packet contained more than 64 samples");
        `ASSERT_ERROR(dut.s_in_axis_tlast[0] ===
                      (timestamp_packet_sample_count == 64),
                      "Timed packet TLAST does not match 64 samples");
      end
      if (dut.s_in_axis_thas_time[0])
        timestamp_probe_timed_samples++;
      if (timestamp_probe_first || dut.s_in_axis_tlast[0]) begin
        $display("RFNOC_TS sample=%0d first=%0b last=%0b has_time=%0b timestamp=%0d data=%08h",
                 timestamp_probe_samples-1, timestamp_probe_first,
                 dut.s_in_axis_tlast[0], dut.s_in_axis_thas_time[0],
                 dut.s_in_axis_ttimestamp[63:0], dut.s_in_axis_tdata[31:0]);
      end
      if (dut.s_in_axis_tlast[0]) begin
        if (timestamp_check_active) timestamp_packet_sample_count = 0;
        timestamp_probe_packets++;
        timestamp_probe_first = 1;
      end else begin
        timestamp_probe_first = 0;
      end
    end
  end

  // Checks that input packet metadata reaches the SIGGEN core boundary.
  // NOTE: send_items() in this testbench does not explicitly set CHDR timestamps.
  // A separate timed-CHDR stimulus is required to verify actual timestamp values.
  task automatic test_rfnoc_timestamp_path(int port_in = 0, int port_out = 0);
    item_t tx[$];
    test.start_test("RFNoC timestamp path visibility", 2ms);
    write_reg(port_out, REG_ENABLE, 0);
    write_reg(port_out, REG_THRESHOLD, 16'd500);
    tx.delete();
    repeat (128) tx.push_back(pack_iq(16'sd100, 16'sd0));

    timestamp_probe_samples = 0;
    timestamp_probe_packets = 0;
    timestamp_probe_timed_samples = 0;
    timestamp_probe_first = 1;
    timestamp_probe_active = 1;
    blk_ctrl.send_items(port_in, tx);
    #(CE_CLK_PER * 2000);
    timestamp_probe_active = 0;

    $display("RFNOC_TS SUMMARY: samples=%0d packets=%0d timed_samples=%0d",
             timestamp_probe_samples, timestamp_probe_packets,
             timestamp_probe_timed_samples);
    `ASSERT_ERROR(timestamp_probe_samples == 128,
                  "RFNoC shell did not deliver all 128 samples");
    `ASSERT_ERROR(timestamp_probe_packets > 0,
                  "RFNoC shell did not deliver a complete packet");
    if (timestamp_probe_timed_samples == 0)
      $display("RFNOC_TS NOTE: No valid timestamps observed. Untimed BFM stimulus cannot validate timestamp reconstruction.");
    test.end_test();
  endtask

  //---------------------------------------------------------------------------
  // Helper Tasks
  //---------------------------------------------------------------------------

  // Write a 32-bit register
  task automatic write_reg(int port, bit [19:0] addr, bit [31:0] value);
    blk_ctrl.reg_write(port * (2**SIGGEN_ADDR_W) + addr, value);
  endtask : write_reg

  // Read a 32-bit register
  task automatic read_reg(int port, bit [19:0] addr, output logic [31:0] value);
    blk_ctrl.reg_read(port * (2**SIGGEN_ADDR_W) + addr, value);
  endtask : read_reg


  // Check if two samples are within a given distance from each other (i.e.,
  // check if the Cartesian distance is < threshold).
  function bit samples_are_close(
    logic [31:0] samp_a, samp_b,
    real         threshold = 3.0
  );
    real ax, ay, bx, by;
    real distance;

    // Treat the samples and signed 16-bit numbers (not fixed point)
    ax = signed'(samp_a[31:16]);
    ay = signed'(samp_a[15: 0]);
    bx = signed'(samp_b[31:16]);
    by = signed'(samp_b[15: 0]);

    distance = $sqrt( (ax-bx)*(ax-bx) + (ay-by)*(ay-by) );

    return distance <= threshold;
  endfunction : samples_are_close

// Pack 16-bit signed I/Q into one 32-bit item_t
function automatic item_t pack_iq(
  logic signed [15:0] i,
  logic signed [15:0] q
);
  item_t v;
  v[31:16] = i;
  v[15:0]  = q;
  return v;
endfunction

//---------------------------------------------------------------------------
  // Pipeline latency measurement helper
  //---------------------------------------------------------------------------
task automatic measure_pipeline_latency(
    int port_in  = 0,
    int port_out = 0
);
    item_t tx[$];
    item_t items[$];
    int cycles;
    localparam int WARMUP = 8; // match your core warm-up

    test.start_test("Measure pipeline latency", 2ms);

    // Configure block
    write_reg(port_out, REG_ENABLE,     1);
    write_reg(port_out, REG_SPP,        64);
    write_reg(port_out, REG_GAIN,       16'h7FFF);
    write_reg(port_out, REG_PHASE_INC,  16'd64);
    write_reg(port_out, REG_CARTESIAN,  {16'sh7FFF,16'sh0000});
    write_reg(port_out, REG_THRESHOLD,  16'd2000);
    write_reg(port_out, REG_PULSEWIDTH, 1);       // single-sample "burst"
    write_reg(port_out, REG_DELAY,      0);       // no extra delay

    // Build payload: many below-threshold, 1 above-threshold, then more below
    tx.delete();
    for (int i = 0; i < 16; i++) tx.push_back(pack_iq(16'sd200, 16'sd0)); // below thr
    tx.push_back(pack_iq(16'sd4000, 16'sd0));                             // trigger
    for (int i = 0; i < 16; i++) tx.push_back(pack_iq(16'sd200, 16'sd0)); // below thr
    blk_ctrl.send_items(port_in, tx);

    // Wait for output
    cycles = 0;
    forever begin
        #(CE_CLK_PER);
        cycles = cycles + 1;

        if (blk_ctrl.num_received(port_out) > 0) begin
            blk_ctrl.recv_items(port_out, items);
            $display("Pipeline latency observed: %0d CE_CLK cycles", cycles);
            `ASSERT_ERROR(items.size() > 0, "Empty packet received");
            break;
        end

        if (cycles > 10000) begin
            $display("Timeout: No output observed within 10000 cycles");
            break;
        end
    end

    // Clean up
    write_reg(port_out, REG_ENABLE, 0);
    flush_output(port_out);

    test.end_test();
endtask






  // Convert real to signed 16-bit fixed point with "frac" fractional bits
  function automatic logic [15:0] real_to_fixed(real value, int frac = 15);
    // Convert to fixed point value
    value = value * 2.0**frac;

    // Round
    value = $floor(value + 0.5);

    // Saturate
    if (value > 16'sh7FFF) value = 16'sh7FFF;
    if (value < 16'sh8000) value = 16'sh8000;
    return int'(value);
  endfunction : real_to_fixed


  // Convert signed 16-bit fixed point to real, where the fixed point has
  // "frac" fractional bits.
  function automatic real fixed_to_real(
    logic signed [15:0] value,
    int                 frac = 15
  );
    return real'(value) / (2.0 ** frac);
  endfunction : fixed_to_real


  // Compute the next sine value we expect based on the previous. This should
  // be a point (X,Y) rotated counter-clockwise around the origin, where X is
  // in the MSBs and Y is in the LSBs.
  function automatic logic [31:0] next_sine_value(
    logic [31:0] sample,
    logic [15:0] phase_inc
  );
    real x, y, phase, new_x, new_y;
    x        = fixed_to_real(sample[31:16], CART_FRAC);
    y        = fixed_to_real(sample[15: 0], CART_FRAC);
    phase    = fixed_to_real(phase_inc, PHASE_FRAC) * PI;

    // Compute the counter-clockwise rotated coordinates
    new_x = x*$cos(phase) - y*$sin(phase);
    new_y = x*$sin(phase) + y*$cos(phase);

    return { real_to_fixed(new_x, CART_FRAC), real_to_fixed(new_y, CART_FRAC) };
  endfunction : next_sine_value


  // Apply a gain to an input value, then round and clip the same way the DUT
  // does.
  function automatic logic [15:0] apply_gain(
    logic signed [15:0] gain,
    logic signed [15:0] value
  );
    logic signed [31:0] result;
    bit round;

    // Apply gain
    result = gain * value;

    // Now we "round and clip". The round and clip block is configured with
    // 32-bit input, 16-bit output, and one "clip_bit". This means it takes
    // the upper 17-bits of the result, rounded, then converts that to a
    // 16-bit result, saturated.

    // Round the value in the upper 17 bits to nearest (biased towards +inf,
    // but don't allow overflow).
    if (result[31:15] != 17'h0FFFF) begin
      round = result[14];
    end else begin
      round = 0;
    end
    result = result >>> 15;   // Arithmetic right shift
    result = result + round;  // Round the result

    // Saturate to 16-bit number
    if (result < 16'sh8000) begin
      result = 16'sh8000;
    end else if (result > 16'sh7FFF) begin
      result = 16'sh7FFF;
    end

    return result[15:0];
  endfunction : apply_gain


  // Flush (drop) any queued up packets on the output
  task automatic flush_output(int port, timeout = 100);
    item_t items[$];

    forever begin
      fork
        begin : wait_for_data_fork
          // Wait for tvalid to rise for up to "timeout" clock cycles
          if (m_rfnoc_chdr_tvalid[port])
            wait(!m_rfnoc_chdr_tvalid[port]);
          wait(m_rfnoc_chdr_tvalid[port]);
        end
        begin : wait_for_timeout_fork
          #(CHDR_CLK_PER*timeout);
        end
      join_any

      // Check if we timed out or if new data arrived
      if (!m_rfnoc_chdr_tvalid[port]) break;
    end

    // Dump all the packets that were received
    while (blk_ctrl.num_received(port)) begin
      blk_ctrl.recv_items(port, items);
    end
  endtask : flush_output


  // Test a read/write register for correct functionality
  //
  //   port          : Replay block port to use
  //   addr          : Register byte address
  //   mask          : Mask of the bits we expect to be writable
  //   initial_value : Value we expect to read initially
  //
  task automatic test_read_write_reg(
    int          port,
    bit   [19:0] addr,
    bit   [31:0] mask = 32'hFFFFFFFF,
    logic [31:0] initial_value = '0
  );
    string       err_msg;
    logic [31:0] value;
    logic [31:0] expected;

    err_msg = $sformatf("Register 0x%X failed read/write test: ", addr);

    // Check initial value
    expected = initial_value;
    read_reg(port, addr, value);
    `ASSERT_ERROR(value === expected, {err_msg, "initial value"});

    // Write maximum value
    expected = (initial_value & ~mask) | mask;
    write_reg(port, addr, '1);
    read_reg(port, addr, value);
    `ASSERT_ERROR(value === expected, {err_msg, "write max value"});

    // Test writing 0
    expected = (initial_value & ~mask);
    write_reg(port, addr, '0);
    read_reg(port, addr, value);
    `ASSERT_ERROR(value === expected, {err_msg, "write zero"});

    // Restore original value
    write_reg(port, addr, initial_value);
  endtask : test_read_write_reg


  // Run the block using the indicated settings and verify the output.
  task automatic run_waveform(
    int                 port,
    logic signed [15:0] gain        = 16'h7FFF,  // 0.99997
    logic         [2:0] mode        = WAVE_SINE, // default = SINE now
    int                 num_packets = 1,
    int                 spp         = SPP,
    logic signed [15:0] const_re    = 16'h7FFF,  // 0.99997
    logic signed [15:0] const_im    = 16'h7FFF,  // 0.99997
    logic signed [15:0] phase_inc   = real_to_fixed(2.0/16, 13), // 2*pi/16 radians
    logic signed [15:0] cart_x      = real_to_fixed(1.0, 14),
    logic signed [15:0] cart_y      = real_to_fixed(0.0, 14)
  );
    write_reg(port, REG_SPP, spp);
    write_reg(port, REG_WAVEFORM, mode);
    write_reg(port, REG_GAIN, gain);
    if (mode == WAVE_CONST) begin
      write_reg(port, REG_CONSTANT, {const_re, const_im});
    end else if (mode == WAVE_SINE) begin
      write_reg(port, REG_PHASE_INC, phase_inc);
      write_reg(port, REG_CARTESIAN, {cart_x, cart_y});
    end
    write_reg(port, REG_ENABLE, 1);

    for (int packet_count = 0; packet_count < num_packets; packet_count++) begin
      item_t items[$];
      item_t expected_const, expected_sine, actual;

      // Receive the next packet
      blk_ctrl.recv_items(port, items);

      // Verify the length
      `ASSERT_ERROR(
        items.size() == spp,
        "Packet length didn't match configured SPP"
      );

      // Verify the payload
      foreach (items[i]) begin
        actual = items[i];

        // Determine the expected constant output
        expected_const[31:16] = apply_gain(gain, const_re);
        expected_const[15: 0] = apply_gain(gain, const_im);

        // Determine the expected sine output
        if (i == 0) begin
          // We have no basis for comparison on the first sample, so don't
          // check it. It will be used to compute the next output.
          expected_sine = actual;
        end else begin
          expected_sine = next_sine_value(items[i-1], phase_inc);
        end

        // Check the output
        if (mode == WAVE_CONST) begin
          // For the constant, we expect the output to match exactly
          `ASSERT_ERROR(
            actual == expected_const,
            $sformatf("Incorrect constant sample on packet %0d. Expected 0x%X, received 0x%X.",
              packet_count, expected_const, actual)
          );
        end else if (mode == WAVE_SINE) begin
          // For sine, it's hard to reproduce the rounding behavior of the IP
          // exactly, so we just check if we're close to the expected answer.
          `ASSERT_ERROR(
            samples_are_close(actual, expected_sine),
            $sformatf("Incorrect sine sample on packet %0d. Expected 0x%X, received 0x%X.",
              packet_count, expected_sine, actual)
          );
        end else if (mode == WAVE_NOISE) begin
          if (i != 0) begin
            // For noise, it's hard to even estimate the output, so make sure
            // it's changing.
            `ASSERT_ERROR(items[i] !== items[i-1],
              $sformatf("Noise output didn't update on packet %0d.Received 0x%X.",
                packet_count, actual)
            );
          end
        end

      end
    end

    // Disable the output and flush any output
    write_reg(port, REG_ENABLE, 0);
    flush_output(port);
  endtask : run_waveform


  // Run the block using the "constant" waveform mode using the indicated
  // settings and verify the output.
  task automatic run_const(
    int  port,
    int  num_packets = 50,
    int  spp         = SPP,
    real gain,
    real re,
    real im
  );
    logic signed [15:0] fgain, fre, fim;   // Fixed-point versions

    // Check the ranges
    `ASSERT_FATAL(gain <= MAX_GAIN_R   || gain >= MIN_GAIN_R,  "Gain out of range");
    `ASSERT_FATAL(re   <= MAX_CONST_R  || re   >= MIN_CONST_R, "Real out of range");
    `ASSERT_FATAL(im   <= MAX_CONST_R  || im   >= MIN_CONST_R, "Imag out of range");

    // Convert arguments to fixed point
    fgain  = real_to_fixed(gain, GAIN_FRAC);
    fre    = real_to_fixed(re, CONST_FRAC);
    fim    = real_to_fixed(im, CONST_FRAC);

    // Test the waveform
    run_waveform(
      .port(port),
      .gain(fgain),
      .mode(WAVE_CONST),
      .num_packets(num_packets),
      .spp(spp),
      .const_re(fre),
      .const_im(fim)
    );
  endtask : run_const


  // Run the block using the "sine" waveform mode using the indicated settings
  // and verify the output.
  task automatic run_sine(
    int  port,
    int  num_packets = 50,
    int  spp         = SPP,
    real gain,
    real x,
    real y,
    real phase
  );
    logic signed [15:0] fgain, fx, fy, fphase;   // Fixed-point versions

    // Check the ranges
    `ASSERT_FATAL(gain  <= MAX_GAIN_R  || gain  >= MIN_GAIN_R,  "Gain out of range");
    `ASSERT_FATAL(x     <= MAX_CART_R  || x     >= MIN_CART_R,  "X out of range");
    `ASSERT_FATAL(y     <= MAX_CART_R  || y     >= MIN_CART_R,  "Y out of range");
    `ASSERT_FATAL(phase <= MAX_PHASE_R || phase >= MIN_PHASE_R, "Phase out of range");

    // Convert arguments to fixed point.
    fgain  = real_to_fixed(gain, GAIN_FRAC);
    fx     = real_to_fixed(x, CART_FRAC);
    fy     = real_to_fixed(y, CART_FRAC);
    fphase = real_to_fixed(phase, PHASE_FRAC);

    // Test the waveform
    run_waveform(
      .port(port),
      .gain(fgain),
      .mode(WAVE_SINE),
      .num_packets(num_packets),
      .spp(spp),
      .cart_x(fx),
      .cart_y(fy),
      .phase_inc(fphase)
    );
  endtask : run_sine


  // Run the block using the "noise" waveform mode using the indicated
  // settings and verify the output.
  task automatic run_noise(
    int  port,
    int  num_packets = 50,
    int  spp         = SPP,
    real gain
  );
    logic signed [15:0] fgain;   // Fixed-point versions

    // Check the ranges
    `ASSERT_FATAL(gain <= MAX_GAIN_R || gain >= MIN_GAIN_R, "Gain out of range");

    // Convert arguments to fixed point
    fgain = real_to_fixed(gain, GAIN_FRAC);

    // Test the waveform
    run_waveform(
      .port(port),
      .gain(fgain),
      .mode(WAVE_NOISE),
      .num_packets(num_packets),
      .spp(spp)
    );
  endtask : run_noise

// --- CtrlPort transaction monitor (hierarchical references to DUT) ---
always @(posedge dut.ctrlport_clk_s) if (!dut.ctrlport_rst_s) begin
  if (dut.m_ctrlport_req_wr)
    $display("%0t CTRL WR addr=0x%05h data=0x%08h",
             $time, dut.m_ctrlport_req_addr, dut.m_ctrlport_req_data);
  if (dut.m_ctrlport_req_rd)
    $display("%0t CTRL RD addr=0x%05h",
             $time, dut.m_ctrlport_req_addr);
  if (dut.m_ctrlport_resp_ack)
    $display("%0t CTRL ACK data=0x%08h",
             $time, dut.m_ctrlport_resp_data);
end

// --- Tap the decoded bus going into core 0 ---
wire core0_req_wr     = dut.ctrlport_req_wr[0];
wire core0_req_rd     = dut.ctrlport_req_rd[0];
wire [19:0] core0_addr= dut.ctrlport_req_addr[20*0 +: 20];
wire [31:0] core0_wdat= dut.ctrlport_req_data[32*0 +: 32];
wire core0_ack        = dut.ctrlport_resp_ack[0];
wire [31:0] core0_rdat= dut.ctrlport_resp_data[32*0 +: 32];

always @(posedge dut.ctrlport_clk_s) if (!dut.ctrlport_rst_s) begin
  if (core0_req_wr)
    $display("%0t CORE0 WR addr=0x%05h data=0x%08h", $time, core0_addr, core0_wdat);
  if (core0_req_rd)
    $display("%0t CORE0 RD addr=0x%05h", $time, core0_addr);
  if (core0_ack)
    $display("%0t CORE0 ACK data=0x%08h", $time, core0_rdat);
end

// --- Optional: ACK must arrive within 64 ctrlport cycles ---
property ctrlport_ack_within_64;
  @(posedge dut.ctrlport_clk_s) disable iff (dut.ctrlport_rst_s)
    (dut.m_ctrlport_req_wr || dut.m_ctrlport_req_rd) |-> ##[1:64] dut.m_ctrlport_resp_ack;
endproperty
assert property (ctrlport_ack_within_64)
  else $fatal(1, "%0t CtrlPort ACK timeout", $time);

  //---------------------------------------------------------------------------
  // Test Procedures
  //---------------------------------------------------------------------------
  
task automatic test_no_trigger_no_output(
  int port_in  = 0,
  int port_out = 0
);
  item_t tx[$]; int i;

  test.start_test("Input below threshold produces no output", 1ms);

  // Start clean
  write_reg(port_out, REG_ENABLE, 0);
  flush_output(port_out);

  // Put the block in the path you actually gate
  write_reg(port_out, REG_WAVEFORM, WAVE_SINE);      // <-- important
  write_reg(port_out, REG_SPP,       32);
  write_reg(port_out, REG_GAIN,      16'h7FFF);
  write_reg(port_out, REG_PHASE_INC, 16'd64);
  write_reg(port_out, REG_CARTESIAN, {16'sh7FFF,16'sh0000});

  // Arm trigger gating if your RTL added a bit for it
  // (Uncomment and set the right address/bit if you have it)
  // write_reg(port_out, REG_CFG, CFG_USE_TRIGGER_BIT);

  // Set trigger thresholds so our input won't fire it
  write_reg(port_out, REG_THRESHOLD,  16'd2000);
  write_reg(port_out, REG_PULSEWIDTH, 16'd32);
  write_reg(port_out, REG_DELAY,      32'd20);

  // Now enable output (gated by trigger in RTL)
  write_reg(port_out, REG_ENABLE, 1);

  // Feed 96 sub-threshold samples (I=100,Q=0)
  tx.delete();
  for (i = 0; i < 96; i++) tx.push_back(pack_iq(16'sd100, 16'sd0));
  blk_ctrl.send_items(port_in, tx);

  // Give time for any accidental output to show up
  #(CE_CLK_PER * 2000);

  `ASSERT_ERROR(blk_ctrl.num_received(port_out) == 0,
    $sformatf("Unexpected output without trigger (got %0d pkts)",
      blk_ctrl.num_received(port_out)))

  write_reg(port_out, REG_ENABLE, 0);
  flush_output(port_out);
  test.end_test();
endtask

task automatic test_pulse_timestamp(
    int port_in  = 0,
    int port_out = 0
);
    item_t tx[$];

    logic [31:0] meta_before;
    logic [31:0] lo_before;
    logic [15:0] seq_before;
    logic [47:0] sample_before;

    logic [31:0] meta_after;
    logic [31:0] lo_after;
    logic [15:0] seq_after;
    logic [47:0] sample_after;

    logic [47:0] sample_delta;

    test.start_test("RX trigger timestamp", 2ms);

    // Configure block
    write_reg(port_out, REG_ENABLE,          0);
    flush_output(port_out);

    write_reg(port_out, REG_SPP,             64);
    write_reg(port_out, REG_GAIN,            16'h7FFF);
    write_reg(port_out, REG_PHASE_INC,       16'd64);
    write_reg(
        port_out,
        REG_CARTESIAN,
        {16'sh7FFF, 16'sh0000}
    );

    write_reg(port_out, REG_THRESHOLD,       16'd500);
    write_reg(port_out, REG_AVG_START_DELAY, 0);
    write_reg(port_out, REG_PULSEWIDTH,      64);
    write_reg(port_out, REG_DELAY,           0);
    write_reg(port_out, REG_PULSE_GAP,       100);

    /*
     * Read the current snapshot before generating the new trigger.
     */
    read_reg(port_out, REG_PULSE_TIME_META, meta_before);
    read_reg(port_out, REG_PULSE_TIME_LO,   lo_before);

    seq_before    = meta_before[31:16];
    sample_before = {meta_before[15:0], lo_before};

    $display(
        "Before trigger: sequence=%0d sample_index=%0d",
        seq_before,
        sample_before
    );

    write_reg(port_out, REG_ENABLE, 1);

    //-------------------------------------------------------
    // Generate one RX input pulse
    //-------------------------------------------------------
    tx.delete();

    // 64 accepted samples before threshold crossing
    repeat (64)
        tx.push_back(pack_iq(16'sd200, 16'sd0));

    // One continuous above-threshold region = one trigger edge
    repeat (300)
        tx.push_back(pack_iq(16'sd4000, 16'sd0));

    repeat (300)
        tx.push_back(pack_iq(16'sd200, 16'sd0));

    blk_ctrl.send_items(port_in, tx);

    /*
     * Wait for averaging and both generated TX pulses.
     */
    #(CE_CLK_PER * 5000);

    //-------------------------------------------------------
    // Read the new published snapshot
    //-------------------------------------------------------
    read_reg(port_out, REG_PULSE_TIME_META, meta_after);
    read_reg(port_out, REG_PULSE_TIME_LO,   lo_after);

    seq_after    = meta_after[31:16];
    sample_after = {meta_after[15:0], lo_after};

    sample_delta = sample_after - sample_before;

    $display(
        "After trigger: sequence=%0d sample_index=%0d delta=%0d",
        seq_after,
        sample_after,
        sample_delta
    );

    //-------------------------------------------------------
    // Checks
    //-------------------------------------------------------

    `ASSERT_ERROR(
        seq_after == (seq_before + 16'd1),
        "One RX trigger should increment sequence exactly once"
    );

    /*
     * The new pulse starts after 64 below-threshold samples.
     *
     * sample_before is the timestamp of the previous pulse, not the
     * current input-stream start, so sample_delta may include samples
     * sent by other tests. Therefore only check that it increased here.
     */
    `ASSERT_ERROR(
        sample_after > sample_before,
        "Pulse timestamp should increase"
    );

    /*
     * Reading the snapshot again without another trigger must return
     * exactly the same result.
     */
    begin
        logic [31:0] meta_stable;
        logic [31:0] lo_stable;

        read_reg(port_out, REG_PULSE_TIME_META, meta_stable);
        read_reg(port_out, REG_PULSE_TIME_LO,   lo_stable);

        `ASSERT_ERROR(
            meta_stable == meta_after,
            "Pulse metadata changed without a new trigger"
        );

        `ASSERT_ERROR(
            lo_stable == lo_after,
            "Pulse timestamp changed without a new trigger"
        );
    end

    write_reg(port_out, REG_ENABLE, 0);
    flush_output(port_out);

    test.end_test();
endtask

// Single over-threshold sample -> wait for delay -> see one burst.
task automatic test_trigger_delay_burst_len(
  int port_in  = 0,
  int port_out = 0
);
  int delay_cycles;
  int pw_samples;
  item_t tx[$];
  item_t items[$];
  int i;
  localparam int WARMUP = 8; // whatever you set warm_ctr to

  test.start_test("Trigger with delay produces delayed burst", 2ms);

  // Easy numbers to reason about
  delay_cycles = 200;   // ce_clk cycles
  pw_samples   = 64;    // burst length (samples)

  write_reg(port_out, REG_ENABLE,     1);
  write_reg(port_out, REG_SPP,        64);
  write_reg(port_out, REG_GAIN,       16'h7FFF);
  write_reg(port_out, REG_PHASE_INC,  16'd64);
  write_reg(port_out, REG_CARTESIAN,  {16'sh7FFF,16'sh0000});

  write_reg(port_out, REG_THRESHOLD,  16'd500);
  write_reg(port_out, REG_PULSEWIDTH, pw_samples[15:0]);
  write_reg(port_out, REG_DELAY,      delay_cycles);

  // Build payload: below-thr, then a real above-thr pulse, then below-thr
  tx.delete();

for (i = 0; i < 64; i++)
  tx.push_back(pack_iq(16'sd200, 16'sd0));

for (i = 0; i < 300; i++)
  tx.push_back(pack_iq(16'sd4000, 16'sd0));

for (i = 0; i < 300; i++)
  tx.push_back(pack_iq(16'sd200, 16'sd0));

  blk_ctrl.send_items(port_in, tx);

  // BEFORE delay expires: no output expected
  #(CE_CLK_PER * (delay_cycles/2 + 10));
  `ASSERT_ERROR(blk_ctrl.num_received(port_out) == 0,
    "Output appeared before delay elapsed")

  // AFTER delay + margin: output must exist
  #(CE_CLK_PER * (delay_cycles + WARMUP + pw_samples + 10000));
  `ASSERT_FATAL(blk_ctrl.num_received(port_out) > 0,
    "No output packet after trigger+delay")

  // Read first packet and sanity-check it's non-empty
  blk_ctrl.recv_items(port_out, items);
  `ASSERT_ERROR(items.size() > 0, "Empty packet received")

  // Clean up
  write_reg(port_out, REG_ENABLE, 0);
  flush_output(port_out);

  test.end_test();
endtask


task automatic test_rfnoc_hw_timestamp(
    int port_in = 0,
    int port_out = 0
);
    item_t tx[$];
    packet_info_t pkt_info;
    int unsigned wait_cycles;

    test.start_test("RFNoC hardware timestamp test", 1ms);

    // Keep the core disabled: this test observes the RX NoC shell metadata.
    write_reg(port_out, REG_ENABLE, 0);

    timestamp_probe_samples = 0;
    timestamp_probe_packets = 0;
    timestamp_probe_timed_samples = 0;
    timestamp_probe_first = 1;
    timestamp_packet_sample_count = 0;
    timestamp_check_active = 1;
    timestamp_probe_active = 1;

    tx.delete();
    repeat (64) tx.push_back(pack_iq(16'sd100, 16'sd0));

    pkt_info = '0;
    pkt_info.has_time = 1'b1;
    pkt_info.timestamp = 64'd100000;
    blk_ctrl.send_items(port_in, tx, {}, pkt_info);

    pkt_info = '0;
    pkt_info.has_time = 1'b1;
    pkt_info.timestamp = 64'd200000;
    blk_ctrl.send_items(port_in, tx, {}, pkt_info);

    // Wait for both complete packets, with a bounded timeout.
    wait_cycles = 0;
    while ((timestamp_probe_packets < 2) && (wait_cycles < 10000)) begin
        @(negedge ce_clk);
        wait_cycles++;
    end
    // Allow the final posedge monitor to finish before disabling checks.
    timestamp_probe_active = 0;
    timestamp_check_active = 0;

    $display("RFNOC_HW_TS SUMMARY: samples=%0d packets=%0d timed_samples=%0d",
             timestamp_probe_samples, timestamp_probe_packets,
             timestamp_probe_timed_samples);
    `ASSERT_ERROR(timestamp_probe_packets == 2,
                  "Timed CHDR test did not receive exactly two packets");
    `ASSERT_ERROR(timestamp_probe_samples == 128,
                  "Timed CHDR test did not receive exactly 128 samples");
    `ASSERT_ERROR(timestamp_probe_timed_samples == 128,
                  "Timed CHDR test did not mark all samples as timed");
    test.end_test();
endtask




task automatic test_registers(int port);

  logic [31:0] dbg_avg_power_rb;
  logic [31:0] dbg_tx_amp_rb;

  test.start_test($sformatf("Test registers (port %0d)", port), 1ms);
  write_reg(port, REG_THRESHOLD, 32'd0);

  test_read_write_reg(port, REG_SPP,       {REG_SPP_LEN{1'b1}},       32'd16);
  test_read_write_reg(port, REG_GAIN,      {REG_GAIN_LEN{1'b1}},      32'h7FFF);
  test_read_write_reg(port, REG_CONSTANT,  {REG_CONSTANT_LEN{1'b1}},  32'h0);
  test_read_write_reg(port, REG_PHASE_INC, {REG_PHASE_INC_LEN{1'b1}}, {REG_PHASE_INC_LEN{1'bX}});
  test_read_write_reg(port, REG_CARTESIAN, {REG_CARTESIAN_LEN{1'b1}}, {REG_CARTESIAN_LEN{1'bX}});

  test_read_write_reg(port, REG_THRESHOLD,  {REG_THRESHOLD_LEN{1'b1}},  32'h0000_0000);
  test_read_write_reg(port, REG_PULSEWIDTH, {REG_PULSEWIDTH_LEN{1'b1}}, 32'h0000_0020);
  test_read_write_reg(port, REG_DELAY,      {REG_DELAY_LEN{1'b1}},      32'h0000_0000);
  test_read_write_reg(
  port,
  REG_AVG_START_DELAY,
  {REG_AVG_START_DELAY_LEN{1'b1}},
  32'h0000_0020
);

  // Debug registers are read-only
  read_reg(port, REG_DBG_AVG_POWER, dbg_avg_power_rb);
  read_reg(port, REG_DBG_TX_AMP,    dbg_tx_amp_rb);

  $display("DBG_AVG_POWER = %0d", dbg_avg_power_rb);
  $display("DBG_TX_AMP    = %0d", dbg_tx_amp_rb[15:0]);

  `ASSERT_ERROR(dbg_avg_power_rb == 32'h0000_0000,
    "REG_DBG_AVG_POWER should reset to 0");

  `ASSERT_ERROR(dbg_tx_amp_rb[31:16] == 16'h0000,
    "REG_DBG_TX_AMP upper 16 bits should be 0");

  test.end_test();

endtask


  // Run through all the waveform modes to make sure they work as expected
  task automatic test_waveforms(int port);
    test.start_test($sformatf("Test waveforms (port %0d)", port), 1ms);
    run_const(.port(port), .gain(0.5), .re(0.25), .im(0.5));
    run_sine(.port(port), .gain(0.75), .x(0.25), .y(0.5), .phase(2.0/64));
    run_noise(.port(port), .gain(0.999));
    test.end_test();
  endtask : test_waveforms

  // Use the SINE waveform to test the gain.
  // (Our DUT only implements sine; CONST mode is not used.)
  task automatic test_gain(int port);
    logic signed [15:0] min_val;
    logic signed [15:0] max_val;

    test.start_test($sformatf("Test gain (port %0d)", port), 1ms);

    max_val = 16'sh7FFF;
    min_val = 16'sh8000;

    // Test max gain with default sine params
    run_waveform(.port(port), .mode(WAVE_SINE), .gain(max_val));
    // Test min gain with default sine params
    run_waveform(.port(port), .mode(WAVE_SINE), .gain(min_val));
    // Test zero gain
    run_waveform(.port(port), .mode(WAVE_SINE), .gain(0));
    // Test mid gain (≈0.5)
    run_waveform(
      .port(port),
      .mode(WAVE_SINE),
      .gain(real_to_fixed(0.5, GAIN_FRAC))
    );

    test.end_test();
  endtask : test_gain

  // Test the phase setting for the sine waveform
  task automatic test_phase(int port);
    test.start_test($sformatf("Test phase (port %0d)", port), 1ms);
    // Test typical phase
    run_sine(.port(port), .gain(0.5), .x(1.0), .y(0.0), .phase(2.0/16), .num_packets(2));
    // Test max phase
    run_sine(.port(port), .gain(0.5), .x(1.0), .y(0.0), .phase(MAX_PHASE_R), .num_packets(2));
    // Test min phase
    run_sine(.port(port), .gain(0.5), .x(1.0), .y(0.0), .phase(MIN_PHASE_R), .num_packets(2));
    test.end_test();
  endtask : test_phase


  // Use constant waveform to test min and max packet lengths
  task automatic test_packet_length(int port);
    test.start_test($sformatf("Test packet length (port %0d)", port), 1ms);
    run_waveform(.port(port), .spp(2));
    run_waveform(.port(port), .spp(SPP));
    run_waveform(.port(port), .spp((2**MTU-1) * (CHDR_W / ITEM_W))); // Test MTU size
    test.end_test();
  endtask : test_packet_length

task automatic test_measurement_fifo(
    int port_in  = 0,
    int port_out = 0
);
    item_t tx[$];

    logic [31:0] count_rb;
    logic [31:0] meta_rb;
    logic [31:0] lo_rb;
    logic [31:0] power_rb;

    logic [15:0] seq0;
    logic [15:0] seq1;
    logic [15:0] seq2;

    logic [47:0] time0;
    logic [47:0] time1;
    logic [47:0] time2;

    logic [31:0] power0;
    logic [31:0] power1;
    logic [31:0] power2;
logic [31:0] time_rb;
logic [31:0] valid_rb;

    int drain_guard;

    test.start_test(
        "Measurement FIFO FWFT + explicit POP",
        5ms
    );

    //-------------------------------------------------------
    // Configure block
    //-------------------------------------------------------

    write_reg(port_out, REG_ENABLE, 0);
    flush_output(port_out);

    write_reg(
        port_out,
        REG_SPP,
        64
    );

    write_reg(
        port_out,
        REG_GAIN,
        16'h7FFF
    );

    write_reg(
        port_out,
        REG_PHASE_INC,
        16'd64
    );

    write_reg(
        port_out,
        REG_CARTESIAN,
        {16'sh7FFF, 16'sh0000}
    );

    write_reg(
        port_out,
        REG_THRESHOLD,
        16'd500
    );

    write_reg(
        port_out,
        REG_AVG_START_DELAY,
        0
    );

    write_reg(
        port_out,
        REG_PULSEWIDTH,
        16
    );

    write_reg(
        port_out,
        REG_DELAY,
        0
    );

    write_reg(
        port_out,
        REG_PULSE_GAP,
        20
    );


    //-------------------------------------------------------
    // Drain FIFO entries left by previous tests
    //-------------------------------------------------------

    read_reg(
        port_out,
        REG_MEAS_FIFO_COUNT,
        count_rb
    );

    $display(
        "FIFO count before drain = %0d",
        count_rb
    );

    drain_guard = 0;

    while ((count_rb != 0) &&
           (drain_guard < 2048))
    begin

        // META does not pop
        read_reg(
            port_out,
            REG_MEAS_FIFO_META,
            meta_rb
        );

        // TIME_LO does not pop
        read_reg(
            port_out,
            REG_MEAS_FIFO_TIME_LO,
            lo_rb
        );

        // AVG_POWER is read-only: does NOT pop the entry
        read_reg(
            port_out,
            REG_MEAS_FIFO_AVG_POWER,
            power_rb
        );

        // Reading all fields must not advance the FWFT FIFO.
        // Explicit POP: advance FIFO by exactly one entry.
        write_reg(port_out, REG_MEAS_FIFO_POP, 32'd1);

        /*
         * Give the CE-clock FIFO controller time
         * to observe meas_fifo_pop_req.
         */
        #(CE_CLK_PER * 8);

        read_reg(
            port_out,
            REG_MEAS_FIFO_COUNT,
            count_rb
        );

        drain_guard++;
    end


    `ASSERT_ERROR(
        drain_guard < 2048,
        "Timeout while draining measurement FIFO"
    );


    read_reg(
        port_out,
        REG_MEAS_FIFO_COUNT,
        count_rb
    );

    $display(
        "FIFO count after drain = %0d",
        count_rb
    );

    `ASSERT_ERROR(
        count_rb == 0,
        "Measurement FIFO should be empty after drain"
    );


    //-------------------------------------------------------
    // Enable trigger operation
    //-------------------------------------------------------

    write_reg(
        port_out,
        REG_ENABLE,
        1
    );


    //-------------------------------------------------------
    // Trigger #1
    //
    // Amplitude = 4000 counts
    // Expected average power ~= 4000^2 = 16,000,000
    //-------------------------------------------------------

    tx.delete();

    repeat (64)
        tx.push_back(
            pack_iq(
                16'sd200,
                16'sd0
            )
        );

    repeat (200)
        tx.push_back(
            pack_iq(
                16'sd4000,
                16'sd0
            )
        );

    repeat (300)
        tx.push_back(
            pack_iq(
                16'sd200,
                16'sd0
            )
        );

    blk_ctrl.send_items(
        port_in,
        tx
    );

    #(CE_CLK_PER * 3000);


    //-------------------------------------------------------
    // Trigger #2
    //
    // Use a different amplitude so we can verify that
    // each FIFO entry keeps its own power measurement.
    //
    // Amplitude = 3000
    // Expected power ~= 9,000,000
    //-------------------------------------------------------

    tx.delete();

    repeat (64)
        tx.push_back(
            pack_iq(
                16'sd200,
                16'sd0
            )
        );

    repeat (200)
        tx.push_back(
            pack_iq(
                16'sd3000,
                16'sd0
            )
        );

    repeat (300)
        tx.push_back(
            pack_iq(
                16'sd200,
                16'sd0
            )
        );

    blk_ctrl.send_items(
        port_in,
        tx
    );

    #(CE_CLK_PER * 3000);


    //-------------------------------------------------------
    // Trigger #3
    //
    // Amplitude = 2000
    // Expected power ~= 4,000,000
    //-------------------------------------------------------

    tx.delete();

    repeat (64)
        tx.push_back(
            pack_iq(
                16'sd200,
                16'sd0
            )
        );

    repeat (200)
        tx.push_back(
            pack_iq(
                16'sd2000,
                16'sd0
            )
        );

    repeat (300)
        tx.push_back(
            pack_iq(
                16'sd200,
                16'sd0
            )
        );

    blk_ctrl.send_items(
        port_in,
        tx
    );

    #(CE_CLK_PER * 3000);
    
    // -----------------------------------------
// NEW: CtrlPort verification
// -----------------------------------------

read_reg(port_out, REG_MEAS_FIFO_COUNT, count_rb);
`ASSERT_ERROR(count_rb == 3, "Expected 3 FIFO entries");

read_reg(port_out, REG_MEAS_FIFO_VALID, valid_rb);
`ASSERT_ERROR(valid_rb[0] == 1'b1, "FIFO should be valid");

read_reg(port_out, REG_MEAS_FIFO_META, meta_rb);
read_reg(port_out, REG_MEAS_FIFO_TIME_LO, lo_rb);
read_reg(port_out, REG_MEAS_FIFO_AVG_POWER, power_rb);

$display("CTRL META  = 0x%08h", meta_rb);
$display("CTRL TIME  = 0x%08h", lo_rb);
$display("CTRL POWER = %0d", power_rb);

// Reading must NOT remove the measurement
read_reg(port_out, REG_MEAS_FIFO_COUNT, count_rb);
`ASSERT_ERROR(count_rb == 3, "CtrlPort read popped FIFO");



    //-------------------------------------------------------
    // Verify exactly 3 measurements were pushed
    //-------------------------------------------------------

    read_reg(
        port_out,
        REG_MEAS_FIFO_COUNT,
        count_rb
    );

    $display(
        "FIFO count after 3 triggers = %0d",
        count_rb
    );

    `ASSERT_ERROR(
        count_rb == 3,
        "FIFO should contain exactly 3 measurements"
    );


    //-------------------------------------------------------
    // Read FIFO entry #0
    //-------------------------------------------------------

    read_reg(
        port_out,
        REG_MEAS_FIFO_META,
        meta_rb
    );

    //-------------------------------------------------------
    // META must NOT pop
    //-------------------------------------------------------

    read_reg(
        port_out,
        REG_MEAS_FIFO_COUNT,
        count_rb
    );

    `ASSERT_ERROR(
        count_rb == 3,
        "Reading FIFO META must not pop an entry"
    );


    //-------------------------------------------------------
    // Read timestamp low
    //-------------------------------------------------------

    read_reg(
        port_out,
        REG_MEAS_FIFO_TIME_LO,
        lo_rb
    );

    //-------------------------------------------------------
    // TIME_LO must NOT pop either
    //-------------------------------------------------------

    read_reg(
        port_out,
        REG_MEAS_FIFO_COUNT,
        count_rb
    );

    `ASSERT_ERROR(
        count_rb == 3,
        "Reading FIFO TIME_LO must not pop an entry"
    );


    seq0 =
        meta_rb[31:16];

    time0 = {
        meta_rb[15:0],
        lo_rb
    };


    //-------------------------------------------------------
    // Read power.
    // Reading AVG_POWER does NOT pop the FIFO entry.
    //-------------------------------------------------------

    read_reg(
        port_out,
        REG_MEAS_FIFO_AVG_POWER,
        power_rb
    );

        // Reading all fields must not advance the FWFT FIFO.
        read_reg(port_out, REG_MEAS_FIFO_COUNT, count_rb);
        `ASSERT_ERROR(count_rb == 3,
            "Reading AVG_POWER must not pop entry #0");
        // Explicit POP: advance FIFO by exactly one entry.
        write_reg(port_out, REG_MEAS_FIFO_POP, 32'd1);

    power0 = power_rb;

    #(CE_CLK_PER * 8);

    read_reg(
        port_out,
        REG_MEAS_FIFO_COUNT,
        count_rb
    );

    $display(
        "FIFO[0]: SEQ=%0d TIME=%0d POWER=%0d COUNT=%0d",
        seq0,
        time0,
        power0,
        count_rb
    );

    `ASSERT_ERROR(
        count_rb == 2,
        "FIFO count should be 2 after first explicit POP"
    );

    `ASSERT_ERROR(
        power0 != 0,
        "FIFO[0] average power should not be zero"
    );


    //-------------------------------------------------------
    // Read FIFO entry #1
    //-------------------------------------------------------

    read_reg(
        port_out,
        REG_MEAS_FIFO_META,
        meta_rb
    );

    read_reg(
        port_out,
        REG_MEAS_FIFO_TIME_LO,
        lo_rb
    );

    seq1 =
        meta_rb[31:16];

    time1 = {
        meta_rb[15:0],
        lo_rb
    };

    read_reg(
        port_out,
        REG_MEAS_FIFO_AVG_POWER,
        power_rb
    );

        // Reading all fields must not advance the FWFT FIFO.
        read_reg(port_out, REG_MEAS_FIFO_COUNT, count_rb);
        `ASSERT_ERROR(count_rb == 2,
            "Reading AVG_POWER must not pop entry #1");
        // Explicit POP: advance FIFO by exactly one entry.
        write_reg(port_out, REG_MEAS_FIFO_POP, 32'd1);

    power1 = power_rb;

    #(CE_CLK_PER * 8);

    read_reg(
        port_out,
        REG_MEAS_FIFO_COUNT,
        count_rb
    );

    $display(
        "FIFO[1]: SEQ=%0d TIME=%0d POWER=%0d COUNT=%0d",
        seq1,
        time1,
        power1,
        count_rb
    );

    `ASSERT_ERROR(
        count_rb == 1,
        "FIFO count should be 1 after second pop"
    );

    `ASSERT_ERROR(
        power1 != 0,
        "FIFO[1] average power should not be zero"
    );


    //-------------------------------------------------------
    // Read FIFO entry #2
    //-------------------------------------------------------

    read_reg(
        port_out,
        REG_MEAS_FIFO_META,
        meta_rb
    );

    read_reg(
        port_out,
        REG_MEAS_FIFO_TIME_LO,
        lo_rb
    );

    seq2 =
        meta_rb[31:16];

    time2 = {
        meta_rb[15:0],
        lo_rb
    };

    read_reg(
        port_out,
        REG_MEAS_FIFO_AVG_POWER,
        power_rb
    );

        // Reading all fields must not advance the FWFT FIFO.
        read_reg(port_out, REG_MEAS_FIFO_COUNT, count_rb);
        `ASSERT_ERROR(count_rb == 1,
            "Reading AVG_POWER must not pop entry #2");
        // Explicit POP: advance FIFO by exactly one entry.
        write_reg(port_out, REG_MEAS_FIFO_POP, 32'd1);

    power2 = power_rb;

    #(CE_CLK_PER * 8);

    read_reg(
        port_out,
        REG_MEAS_FIFO_COUNT,
        count_rb
    );

    $display(
        "FIFO[2]: SEQ=%0d TIME=%0d POWER=%0d COUNT=%0d",
        seq2,
        time2,
        power2,
        count_rb
    );

    `ASSERT_ERROR(
        count_rb == 0,
        "FIFO should be empty after third pop"
    );

    `ASSERT_ERROR(
        power2 != 0,
        "FIFO[2] average power should not be zero"
    );


    //-------------------------------------------------------
    // Verify sequence ordering
    //-------------------------------------------------------

    $display(
        "FIFO sequences: %0d, %0d, %0d",
        seq0,
        seq1,
        seq2
    );

    `ASSERT_ERROR(
        seq1 == (seq0 + 16'd1),
        "FIFO sequence #1 is not consecutive"
    );

    `ASSERT_ERROR(
        seq2 == (seq1 + 16'd1),
        "FIFO sequence #2 is not consecutive"
    );


    //-------------------------------------------------------
    // Verify timestamp ordering
    //-------------------------------------------------------

    $display(
        "FIFO timestamps: %0d, %0d, %0d",
        time0,
        time1,
        time2
    );

    `ASSERT_ERROR(
        time1 > time0,
        "Second FIFO timestamp must be greater than first"
    );

    `ASSERT_ERROR(
        time2 > time1,
        "Third FIFO timestamp must be greater than second"
    );


    //-------------------------------------------------------
    // Verify amplitude/power ordering
    //
    // Inputs were:
    //
    // #0 = 4000
    // #1 = 3000
    // #2 = 2000
    //
    // Therefore:
    //
    // power0 > power1 > power2
    //-------------------------------------------------------

    $display(
        "FIFO powers: %0d, %0d, %0d",
        power0,
        power1,
        power2
    );

    `ASSERT_ERROR(
        power0 > power1,
        "FIFO power #0 should be greater than power #1"
    );

    `ASSERT_ERROR(
        power1 > power2,
        "FIFO power #1 should be greater than power #2"
    );


    //-------------------------------------------------------
    // Optional stronger sanity checks
    //
    // With Q=0 and constant I during the averaging window:
    //
    //   4000^2 = 16,000,000
    //   3000^2 =  9,000,000
    //   2000^2 =  4,000,000
    //
    // Don't require exact equality here yet because the
    // exact averaging-window alignment can include edge
    // samples depending on your AVG_START_DELAY.
    //-------------------------------------------------------

    `ASSERT_ERROR(
        power0 > 32'd12000000,
        "FIFO[0] power unexpectedly low"
    );

    `ASSERT_ERROR(
        power1 > 32'd6000000,
        "FIFO[1] power unexpectedly low"
    );

    `ASSERT_ERROR(
        power2 > 32'd2000000,
        "FIFO[2] power unexpectedly low"
    );


    //-------------------------------------------------------
    // Verify empty FIFO reads
    //-------------------------------------------------------

    read_reg(
        port_out,
        REG_MEAS_FIFO_META,
        meta_rb
    );

    read_reg(
        port_out,
        REG_MEAS_FIFO_TIME_LO,
        lo_rb
    );

    read_reg(
        port_out,
        REG_MEAS_FIFO_AVG_POWER,
        power_rb
    );

    #(CE_CLK_PER * 8);

    read_reg(
        port_out,
        REG_MEAS_FIFO_COUNT,
        count_rb
    );

    `ASSERT_ERROR(
        count_rb == 0,
        "Empty FIFO read must not decrement count"
    );

    `ASSERT_ERROR(
        meta_rb == 0,
        "Empty FIFO META should return zero"
    );

    `ASSERT_ERROR(
        lo_rb == 0,
        "Empty FIFO TIME_LO should return zero"
    );

    `ASSERT_ERROR(
        power_rb == 0,
        "Empty FIFO AVG_POWER should return zero"
    );


    // Explicit POP while empty must not underflow.
    write_reg(port_out, REG_MEAS_FIFO_POP, 32'd1);
    #(CE_CLK_PER * 8);
    read_reg(port_out, REG_MEAS_FIFO_COUNT, count_rb);
    `ASSERT_ERROR(count_rb == 0,
        "POP on empty FIFO must not decrement count");

    //-------------------------------------------------------
    // Check overflow stayed clear
    //-------------------------------------------------------

    read_reg(
        port_out,
        REG_MEAS_FIFO_OVERFLOW,
        count_rb
    );

    `ASSERT_ERROR(
        count_rb[0] == 1'b0,
        "FIFO overflow unexpectedly asserted"
    );


    //-------------------------------------------------------
    // Cleanup
    //-------------------------------------------------------

    write_reg(
        port_out,
        REG_ENABLE,
        0
    );

    flush_output(port_out);

    test.end_test();

endtask


  // -------------------------------------------------------------------------
  // Verify that the FIFO retains the timestamp of the triggering CHDR packet
  // and the zero-based sample offset of the threshold crossing.
  // -------------------------------------------------------------------------
  task automatic test_chdr_trigger_fifo(int port_in = 0, int port_out = 0);
    item_t tx[$];
    packet_info_t pkt_info;
    logic [31:0] count_rb, valid_rb, meta_rb, lo_rb, hi_rb;
    logic [31:0] offset_rb, status_rb, power_rb, meta_again;
    logic [63:0] timestamp_rb;
    logic [15:0] seq_rb;
    int unsigned guard;
    localparam logic [63:0] TEST_TS = 64'h1234_5678_9ABC_DEF0;
    localparam int TRIGGER_OFFSET = 16;

    test.start_test("CHDR timestamp + trigger offset FIFO", 5ms);
    write_reg(port_out, REG_ENABLE, 0);
    write_reg(port_out, REG_THRESHOLD, 0);
    flush_output(port_out);

    // Remove measurements from earlier tests (FWFT FIFO).
    read_reg(port_out, REG_MEAS_FIFO_COUNT, count_rb);
    guard = 0;
    while ((count_rb != 0) && (guard < 2048)) begin
      write_reg(port_out, REG_MEAS_FIFO_POP, 32'd1);
      #(CE_CLK_PER * 8);
      read_reg(port_out, REG_MEAS_FIFO_COUNT, count_rb);
      guard++;
    end
    `ASSERT_FATAL(count_rb == 0, "Could not drain measurement FIFO");

    write_reg(port_out, REG_SPP, 64);
    write_reg(port_out, REG_GAIN, 16'h7FFF);
    write_reg(port_out, REG_AVG_START_DELAY, 0);
    write_reg(port_out, REG_PULSEWIDTH, 16);
    write_reg(port_out, REG_DELAY, 0);
    write_reg(port_out, REG_PULSE_GAP, 20);
    write_reg(port_out, REG_THRESHOLD, 500);
    write_reg(port_out, REG_ENABLE, 1);

    // Exactly 64 samples in the first timed packet.
    // Samples 0..15 are below threshold; sample 16 crosses it.
    tx.delete();
    repeat (TRIGGER_OFFSET) tx.push_back(pack_iq(16'sd100, 16'sd0));
    repeat (64-TRIGGER_OFFSET) tx.push_back(pack_iq(16'sd4000, 16'sd0));
    pkt_info = '0;
    pkt_info.has_time = 1'b1;
    pkt_info.timestamp = TEST_TS;
    blk_ctrl.send_items(port_in, tx, {}, pkt_info);

    // Keep the input above threshold long enough for the 128-sample
    // averaging window. This is a second timed packet, but it must not
    // replace the triggering packet's timestamp.
    tx.delete();
    repeat (64) tx.push_back(pack_iq(16'sd4000, 16'sd0));
    pkt_info = '0;
    pkt_info.has_time = 1'b1;
    pkt_info.timestamp = TEST_TS + 64'd64;
    blk_ctrl.send_items(port_in, tx, {}, pkt_info);

    tx.delete();
    repeat (64) tx.push_back(pack_iq(16'sd4000, 16'sd0));
    pkt_info.timestamp = TEST_TS + 64'd128;
    blk_ctrl.send_items(port_in, tx, {}, pkt_info);

    // Wait for measurement push (not merely for the trigger edge).
    guard = 0;
    count_rb = 0;
    while ((count_rb == 0) && (guard < 200)) begin
      #(CE_CLK_PER * 100);
      read_reg(port_out, REG_MEAS_FIFO_COUNT, count_rb);
      guard++;
    end
    `ASSERT_FATAL(count_rb == 1, "Expected one CHDR-triggered measurement");

    read_reg(port_out, REG_MEAS_FIFO_VALID, valid_rb);
    read_reg(port_out, REG_MEAS_FIFO_META, meta_rb);
    read_reg(port_out, REG_MEAS_FIFO_TIME_LO, lo_rb);
    read_reg(port_out, REG_MEAS_FIFO_TIME_HI, hi_rb);
    read_reg(port_out, REG_MEAS_FIFO_OFFSET, offset_rb);
    read_reg(port_out, REG_MEAS_FIFO_STATUS, status_rb);
    read_reg(port_out, REG_MEAS_FIFO_AVG_POWER, power_rb);

    seq_rb = meta_rb[31:16];
    timestamp_rb = {hi_rb[15:0], meta_rb[15:0], lo_rb};

    $display("CHDR_FIFO: seq=%0d packet_ts=0x%016h offset=%0d valid=%0b avg_power=%0d",
             seq_rb, timestamp_rb, offset_rb, status_rb[0], power_rb);

    `ASSERT_ERROR(timestamp_rb === TEST_TS,
                  "FIFO did not preserve triggering CHDR packet timestamp");
    `ASSERT_ERROR(offset_rb === TRIGGER_OFFSET,
                  "Trigger offset is not 16 (possible off-by-one)");
    `ASSERT_ERROR(status_rb[0] === 1'b1,
                  "Trigger timestamp validity was lost");
    `ASSERT_ERROR(power_rb == 32'd16000000,
                  "Unexpected 128-sample average power");

    // Readback must not pop the FWFT FIFO.
    read_reg(port_out, REG_MEAS_FIFO_META, meta_again);
    `ASSERT_ERROR(meta_again === meta_rb, "FIFO data changed without POP");
    read_reg(port_out, REG_MEAS_FIFO_COUNT, count_rb);
    `ASSERT_ERROR(count_rb == 1, "FIFO read unexpectedly popped measurement");

    write_reg(port_out, REG_MEAS_FIFO_POP, 32'd1);
    #(CE_CLK_PER * 8);
    read_reg(port_out, REG_MEAS_FIFO_COUNT, count_rb);
    `ASSERT_ERROR(count_rb == 0, "FIFO POP did not remove measurement");
    read_reg(port_out, REG_MEAS_FIFO_VALID, valid_rb);
    `ASSERT_ERROR(valid_rb[0] == 1'b0, "FIFO VALID remains set after POP");

    write_reg(port_out, REG_ENABLE, 0);
    flush_output(port_out);
    test.end_test();
  endtask


  //---------------------------------------------------------------------------
  // Main Test Process
  //---------------------------------------------------------------------------

  initial begin : tb_main
    int port;

    // Initialize the test exec object for this testbench
    test.start_tb(
      $sformatf("rfnoc_block_siggen_tb (CHDR_W = %0d, NUM_PORTS = %0d)",
      CHDR_W, NUM_PORTS));

    // Don't start the clocks until after start_tb() returns. This ensures that
    // the clocks aren't toggling while other instances of this testbench are
    // running, which speeds up simulation time.
    rfnoc_chdr_clk_gen.start();
    rfnoc_ctrl_clk_gen.start();
    ce_clk_gen.start();

    // Start the BFMs running
    blk_ctrl.run();

    //--------------------------------
    // Reset
    //--------------------------------

    test.start_test("Flush block then reset it", 10us);
    blk_ctrl.flush_and_reset();
    test.end_test();

    //--------------------------------
    // Verify Block Info
    //--------------------------------

    test.start_test("Verify Block Info", 2us);
    `ASSERT_ERROR(blk_ctrl.get_noc_id() == NOC_ID, "Incorrect NOC_ID Value");
    `ASSERT_ERROR(blk_ctrl.get_num_data_i() == NUM_PORTS_I, "Incorrect NUM_DATA_I Value");
    `ASSERT_ERROR(blk_ctrl.get_num_data_o() == NUM_PORTS_O, "Incorrect NUM_DATA_O Value");
    `ASSERT_ERROR(blk_ctrl.get_mtu() == MTU, "Incorrect MTU Value");
    test.end_test();

    //--------------------------------
    // Test Sequences
    //--------------------------------

    test_rfnoc_timestamp_path(0, 0);

    // Run basic test all ports
    for(port = 0; port < NUM_PORTS; port++) begin
      test_registers(port);
  //    test_waveforms(port);
    end

    // Run remaining tests on single port
    port = 0;
    test_gain(port);
    test_packet_length(port);
    test_phase(port);
    
    // NEW input-trigger tests
    test_no_trigger_no_output(0, 0);
    test_trigger_delay_burst_len(0, 0);
    test_measurement_fifo(0, 0);

    test_pulse_timestamp(0, 0);
    test_rfnoc_hw_timestamp(0, 0);
    test_chdr_trigger_fifo(0, 0);
    // NEW
    // Measure the pipeline latency for port 0
    measure_pipeline_latency(0);


    //--------------------------------
    // Finish Up
    //--------------------------------

    // Display final statistics and results, but don't call $finish, since we
    // don't want to kill other instances of this testbench that may be
    // running.
    test.end_tb(0);

    // Kill the clocks to end this instance of the testbench
    rfnoc_chdr_clk_gen.kill();
    rfnoc_ctrl_clk_gen.kill();
    ce_clk_gen.kill();
  end : tb_main

endmodule : rfnoc_block_siggen_tb


`default_nettype wire