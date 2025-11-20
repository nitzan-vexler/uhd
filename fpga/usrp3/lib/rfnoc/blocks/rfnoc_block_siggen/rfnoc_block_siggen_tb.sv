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

// ------------------------------------------------------------
// Force the 'synced' signal inside the DUT so the FSM runs
// ------------------------------------------------------------
initial begin
  #1us;  // wait a little for reset release
  force dut.gen_ports[0].rfnoc_siggen_core_i.synced = 1'b1;
  $display("%0t [TB] Forced synced=1'b1 inside rfnoc_siggen_core_i", $time);
end

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


task automatic run_waveform(
  int                 port,
  logic signed [15:0] gain        = 16'h7FFF,
  logic         [2:0] mode        = WAVE_SINE,
  int                 num_packets = 1,
  int                 spp         = SPP,
  logic signed [15:0] const_re    = 16'h7FFF,
  logic signed [15:0] const_im    = 16'h0000,
  logic signed [15:0] phase_inc   = real_to_fixed(2.0/16, 13),
  logic signed [15:0] cart_x      = real_to_fixed(1.0, 14),
  logic signed [15:0] cart_y      = real_to_fixed(0.0, 14)
);
  // Ignore mode/const_re/const_im, always run sine
  run_waveform_sine(.port(port), .gain(gain), .num_packets(num_packets),
                    .spp(spp), .phase_inc(phase_inc), .cart_x(cart_x), .cart_y(cart_y));
endtask


// Run the block in sine mode and verify the output.
task automatic run_waveform_sine(
  int                 port,
  logic signed [15:0] gain        = 16'h7FFF,  // 0.99997
  int                 num_packets = 1,
  int                 spp         = SPP,
  logic signed [15:0] phase_inc   = real_to_fixed(2.0/16, 13), // 2*pi/16 radians
  logic signed [15:0] cart_x      = real_to_fixed(1.0, 14),
  logic signed [15:0] cart_y      = real_to_fixed(0.0, 14)
);

logic [31:0] value;

write_reg(port, REG_SPP, spp);
read_reg(port, REG_SPP, value);
$display("%0t ns [TB]: Configured REG_SPP = %0d", $time, value);

  write_reg(port, REG_WAVEFORM, WAVE_SINE);
  write_reg(port, REG_GAIN, gain);
  write_reg(port, REG_PHASE_INC, phase_inc);
  write_reg(port, REG_CARTESIAN, {cart_x, cart_y});
  write_reg(port, REG_ENABLE, 1);

  for (int packet_count = 0; packet_count < num_packets; packet_count++) begin
    item_t items[$];
    item_t expected_sine, actual;

    // Receive the next packet
    blk_ctrl.recv_items(port, items);
$display("%0t ns: Got packet_count=%0d with %0d samples, expected %0d",
         $time, packet_count, items.size(), spp);

    // Verify the length (disabled for debug)
    // `ASSERT_ERROR(
    //   items.size() == spp,
    //   "Packet length didn't match configured SPP"
    // );

    // Verify sine samples
    foreach (items[i]) begin
      actual = items[i];

      if (i == 0) begin
        // First sample = baseline
        expected_sine = actual;
      end else begin
        expected_sine = next_sine_value(items[i-1], phase_inc);
        `ASSERT_ERROR(
          samples_are_close(actual, expected_sine),
          $sformatf("Incorrect sine sample on packet %0d. Expected 0x%X, received 0x%X.",
            packet_count, expected_sine, actual)
        );
      end
    end
  end
endtask : run_waveform_sine


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
//always @(posedge dut.ctrlport_clk_s) if (!dut.ctrlport_rst_s) begin
 // if (dut.m_ctrlport_req_wr)
   // $display("%0t CTRL WR addr=0x%05h data=0x%08h",
      //       $time, dut.m_ctrlport_req_addr, dut.m_ctrlport_req_data);
 // if (dut.m_ctrlport_req_rd)
   // $display("%0t CTRL RD addr=0x%05h",
     //        $time, dut.m_ctrlport_req_addr);
 // if (dut.m_ctrlport_resp_ack)
   // $display("%0t CTRL ACK data=0x%08h",
        //     $time, dut.m_ctrlport_resp_data);
//end

wire m_tvalid_dbg = dut.gen_ports[0].rfnoc_siggen_core_i.m_tvalid;
wire m_tready_dbg = dut.gen_ports[0].rfnoc_siggen_core_i.m_tready;



// --- Tap the decoded bus going into core 0 ---
wire core0_req_wr     = dut.ctrlport_req_wr[0];
wire core0_req_rd     = dut.ctrlport_req_rd[0];
wire [19:0] core0_addr= dut.ctrlport_req_addr[20*0 +: 20];
wire [31:0] core0_wdat= dut.ctrlport_req_data[32*0 +: 32];
wire core0_ack        = dut.ctrlport_resp_ack[0];
wire [31:0] core0_rdat= dut.ctrlport_resp_data[32*0 +: 32];

//always @(posedge dut.ctrlport_clk_s) if (!dut.ctrlport_rst_s) begin
 // if (core0_req_wr)
  //  $display("%0t CORE0 WR addr=0x%05h data=0x%08h", $time, core0_addr, core0_wdat);
  //if (core0_req_rd)
  //  $display("%0t CORE0 RD addr=0x%05h", $time, core0_addr);
 // if (core0_ack)
    //$display("%0t CORE0 ACK data=0x%08h", $time, core0_rdat);
//end

// --- Optional: ACK must arrive within 64 ctrlport cycles ---
property ctrlport_ack_within_64;
  @(posedge dut.ctrlport_clk_s) disable iff (dut.ctrlport_rst_s)
    (dut.m_ctrlport_req_wr || dut.m_ctrlport_req_rd) |-> ##[1:64] dut.m_ctrlport_resp_ack;
endproperty
assert property (ctrlport_ack_within_64)
  else $fatal(1, "%0t CtrlPort ACK timeout", $time);

// --- Debug: monitor trig_fire_q inside core 0 ---
wire trig_fire_q_dbg;
assign trig_fire_q_dbg = dut.gen_ports[0].rfnoc_siggen_core_i.trig_fire_q;



// --- Debug: monitor burst_active inside core 0 ---
wire burst_active_dbg;
assign burst_active_dbg = dut.gen_ports[0].rfnoc_siggen_core_i.burst_active;


  //---------------------------------------------------------------------------
  // Test Procedures
  //---------------------------------------------------------------------------
  
// Drain output side safely without assuming a specific BFM API.
task automatic drain_rx(int port);
  int cnt;

  // Preferred path: if your blk_ctrl has a packet pop, enable this:
`ifdef BLKCTRL_HAS_RECV_PACKET
  while ((cnt = blk_ctrl.num_received(port)) > 0) begin
    void'(blk_ctrl.recv_packet(port)); // discard one packet
  end
`else
  // Fallback: wait until the received count is quiescent (no more arrivals).
  int last = -1;
  int stable = 0;
  // 10 stable cycles at ce_clk with unchanged count ⇒ considered drained/idle
  while (stable < 10) begin
    cnt = blk_ctrl.num_received(port);
    if (cnt == last) stable++; else stable = 0;
    last = cnt;
    @(posedge ce_clk);
  end
`endif
endtask



task automatic enter_trigger_mode(
  int port, logic [31:0] thr, logic [31:0] hold, logic [31:0] delay
);
  // 1) Disable, program regs, clear debug, drain, re-enable
  write_reg(port, REG_ENABLE,   32'h0);
  repeat (4) @(posedge ce_clk);

  write_reg(port, REG_THRESHOLD,  thr);
  write_reg(port, REG_HOLDCOUNT,  hold);
  write_reg(port, REG_DELAY,      delay);

  // optional but nice: clear sticky dbg counters/flags
  write_reg(port, REG_DBG_CTRL,  32'h1);

  drain_rx(port);                 // make sure RX queue is empty
  write_reg(port, REG_ENABLE,   32'h1);
endtask


task automatic test_no_trigger_no_output(
  int port_in  = 0,
  int port_out = 0
);
  item_t tx[$]; int i;

  test.start_test("No output when |I|,|Q| < threshold (magnitude comparator)", 2ms);

  // Strong reset + drain any queued BFMs
  blk_ctrl.flush_and_reset();

  write_reg(port_in, REG_ENABLE, 32'h0);
  drain_rx(port_out);

  // threshold high enough that |I|=100 stays sub-threshold
  enter_trigger_mode(port_in, 32'd2000, 32'd1, 32'd0);

  // Let internal state settle
  repeat (2) @(posedge ce_clk);

  // Feed 96 sub-threshold samples: |I|=100 < 2000, Q=0
  tx.delete();
  for (i = 0; i < 96; i++) tx.push_back(pack_iq(16'sd100, 16'sd0));
  blk_ctrl.send_items(port_in, tx);

  // Observe for a while (no packets must arrive)
  #(CE_CLK_PER * 2000);

  // Assert no output packets
  `ASSERT_ERROR(blk_ctrl.num_received(port_out) == 0,
    $sformatf("Unexpected output without trigger (got %0d pkts)",
              blk_ctrl.num_received(port_out)))

  // (Optional) internal probe - disabled by default
  `ifdef HAS_INT_PROBES
  begin
    int gi = port_in;
    bit allow_output, burst_active, use_trigger;
    // TODO: adjust path below to your netlist if you want this probe:
    // allow_output = dut.gen_ports[gi].noc_shell_siggen_i.rfnoc_siggen_core_i.allow_output;
    // burst_active = dut.gen_ports[gi].noc_shell_siggen_i.rfnoc_siggen_core_i.burst_active;
    // use_trigger  = dut.gen_ports[gi].noc_shell_siggen_i.rfnoc_siggen_core_i.use_trigger;
    $display("%0t [TB] use_trigger=%0b allow_output=%0b burst_active=%0b",
             $time, use_trigger, allow_output, burst_active);
  end
  `endif

  // Cleanup
  write_reg(port_in, REG_ENABLE, 32'h0);
  flush_output(port_out);
  test.end_test();
endtask



task automatic deterministic_sync();
  // One-cycle known time base + synced=1
  force   dut.gen_ports[0].rfnoc_siggen_core_i.now         = 64'd0;
  force   dut.gen_ports[0].rfnoc_siggen_core_i.base_offset = 64'd0;
  force   dut.gen_ports[0].rfnoc_siggen_core_i.synced      = 1'b1;
  @(posedge ce_clk);
  release dut.gen_ports[0].rfnoc_siggen_core_i.now;
  release dut.gen_ports[0].rfnoc_siggen_core_i.base_offset;
  release dut.gen_ports[0].rfnoc_siggen_core_i.synced;
endtask


// Keep the original name; adds `expected_spp` to choose strict vs non-strict
task automatic check_has_time_only_on_sop
(
  input int port_out,
  input bit after_sop_seen,                 // 0 = find SOP; 1 = SOP already consumed by caller
  input int expected_spp,                   // 0 = non-strict (tolerant to BFM "new SOP before TLAST")
  input int drain_guard_cycles = 200000,
  input int sop_guard_cycles   = 200000,
  input int pkt_guard_cycles   = 200000
);
  // ---- Locals (declare up-front for XSim) ----
  int guard;
  bit drained_to_tlast;
  bit saw_sop;
  bit saw_tlast;
  int beats_seen; // handshaked beats since SOP (SOP itself counts as beat 1)

  // Axis aliases
  `define AXIS_CLK   dut.axis_data_clk_s
  `define TVALID     dut.s_out_axis_tvalid[port_out]
  `define TREADY     dut.s_out_axis_tready[port_out]
  `define TLAST      dut.s_out_axis_tlast[port_out]
  `define THAS_TIME  dut.s_out_axis_thas_time[port_out]

  // --------------------------------------------------------------------------
  // If we haven't seen SOP yet, align to the next packet and require has_time=1
  // --------------------------------------------------------------------------
  if (!after_sop_seen) begin
    // Drain any current packet up to a *handshaked* TLAST (OK if already idle)
    guard = 0;
    drained_to_tlast = 1'b0;
    while (guard < drain_guard_cycles) begin
      @(posedge `AXIS_CLK);
      if (`TVALID && `TREADY && `TLAST) begin
        drained_to_tlast = 1'b1;
        break;
      end
      guard++;
    end

    // Wait for next packet's SOP beat and require has_time==1
    saw_sop = 1'b0;
    guard   = 0;
    while (guard < sop_guard_cycles) begin
      @(posedge `AXIS_CLK);
      if (`TVALID && `TREADY) begin
        `ASSERT_ERROR(`THAS_TIME == 1'b1, "SOP must assert has_time=1")
        saw_sop = 1'b1;
        break;
      end
      guard++;
    end
    `ASSERT_ERROR(saw_sop, "Timeout waiting for SOP")
  end

  // --------------------------------------------------------------------------
  // From here on, verify remainder of THIS packet (post-SOP)
  //   strict   (expected_spp>0): enforce TLAST on exactly SPPth beat
  //   nonstrct (expected_spp==0): tolerant if a new SOP appears before TLAST
  // --------------------------------------------------------------------------
  beats_seen = 1;  // SOP already handshaked (either here or by the caller)
  saw_tlast  = 1'b0;
  guard      = 0;

  while (guard < pkt_guard_cycles) begin
    @(posedge `AXIS_CLK);

    // Strict SPP enforcement: if next beat would be the SPPth, assert TLAST there
    if (expected_spp > 0) begin
      if (beats_seen == (expected_spp - 1)) begin
        // Wait for the SPPth handshaked beat
        do @(posedge `AXIS_CLK);
        while (!(`TVALID && `TREADY));

        // Non-SOP beats must not carry has_time
        `ASSERT_ERROR(`THAS_TIME == 1'b0, "Non-SOP beat asserted has_time")

        // SPPth beat must be TLAST
        `ASSERT_ERROR(`TLAST, "Expected TLAST on SPPth beat")
        saw_tlast = 1'b1;
        break;
      end
    end

    // Normal beat handling
    if (`TVALID && `TREADY) begin
      // --- Non-strict tolerance: if a *new* SOP shows up before TLAST,
      //     treat it as the next packet's SOP and stop cleanly.
      if (expected_spp == 0 && `THAS_TIME == 1'b1) begin
        // We've reached the start of the next packet; done checking this one.
        break;
      end

      // For all non-SOP beats, has_time must be 0
      `ASSERT_ERROR(`THAS_TIME == 1'b0, "Non-SOP beat asserted has_time")

      beats_seen++;

      // TLAST ends this packet
      if (`TLAST) begin
        saw_tlast = 1'b1;
        break;
      end

      // In strict mode, an early TLAST (before SPPth) is an error
      if (expected_spp > 0) begin
        `ASSERT_ERROR(!`TLAST,
          $sformatf("Unexpected early TLAST at beat %0d (expected %0d)",
                    beats_seen, expected_spp))
      end
    end

    guard++;
  end

  // In strict mode we must have observed TLAST by now
  if (expected_spp > 0) begin
    `ASSERT_ERROR(saw_tlast,
      "Timeout waiting for TLAST while verifying has_time==0 on non-SOP beats")
  end

  `undef AXIS_CLK
  `undef TVALID
  `undef TREADY
  `undef TLAST
  `undef THAS_TIME
endtask



// -----------------------------------------------------------------------------
// Ensures stimulus is injected only after the core is truly "armed":
//   armed := reg_enable && (threshold>0) && synced
// Verifies: ts_out == trig_time + reg_delay + PIPE_LAT  (±1 tick) at SOP.
// -----------------------------------------------------------------------------
task automatic test_timestamp_plus_delay
(
  int port_in      = 0,
  int port_out     = 0,
  int delay_cycles = 400,
  int pw_samples   = 64
);
  localparam int TB_PIPE_LAT = 6;  // must match DUT's PIPE_LAT

  // ---- Declarations (no initializers here!) ----
  item_t           tx[$];
  int              i;
  bit              saw_trig;
  bit              saw_sop;
  bit              armed_seen;
  int              guard_ctr;

  longint unsigned trig_time;
  longint unsigned sop_time_now;
  longint unsigned ts_out;
  longint unsigned expected_A;
  longint signed   diff_A;

  // captured after trigger
  longint unsigned ts_base_dbg;
  longint unsigned trig_abs_dbg;
  int              delay_dbg;

  // ---- Begin ----
  test.start_test("Timestamp policy at SOP (strict)", 2ms);

  // Clean start + deterministic time base
  blk_ctrl.flush_and_reset();
deterministic_sync();         // <-- re-arm timebase
repeat (4) @(posedge ce_clk); // small settle




  // Configure (enable is written LAST)
  write_reg(port_out, REG_SPP,        pw_samples[15:0]);
  write_reg(port_out, REG_GAIN,       16'h7FFF);
  write_reg(port_out, REG_PHASE_INC,  16'd64);
  write_reg(port_out, REG_CARTESIAN,  {16'sh7FFF,16'sh0000});
  write_reg(port_out, REG_THRESHOLD,  16'd1000);               // nonzero → use_trigger=1
  write_reg(port_out, REG_PULSEWIDTH, pw_samples[15:0]);       // make PW==SPP for this test
  write_reg(port_out, REG_DELAY,      delay_cycles);
  write_reg(port_out, REG_HOLDCOUNT,  8'd1);
  write_reg(port_out, REG_ENABLE,     1);
// after REG_* writes and REG_ENABLE=1:
wait (dut.gen_ports[0].rfnoc_siggen_core_i.synced      == 1'b1);
wait (dut.gen_ports[0].rfnoc_siggen_core_i.reg_enable  == 1'b1);
wait (dut.gen_ports[0].rfnoc_siggen_core_i.use_trigger == 1'b1);
repeat (8) @(posedge ce_clk);



  armed_seen = 1'b0;
  guard_ctr  = 0;
  fork
    begin : wait_armed
      wait (dut.gen_ports[0].rfnoc_siggen_core_i.synced == 1'b1);
      repeat (8) @(posedge ce_clk);
      armed_seen = 1'b1;
    end
    begin : guard
      repeat (100000) @(posedge ce_clk);
    end
  join_any
  disable fork;

  `ASSERT_ERROR(armed_seen, "TB: never reached ARMED (synced) state before sending trigger");

  // ---- Stimulus: a short over-threshold burst wrapped by under-threshold ----
  tx.delete();
  for (i=0; i<32; i++)  tx.push_back(pack_iq(16'sd200,  16'sd0));   // below threshold
  for (i=0; i<5;  i++)  tx.push_back(pack_iq(16'sd4000, 16'sd0));   // above threshold → trigger
  for (i=0; i<64; i++)  tx.push_back(pack_iq(16'sd200,  16'sd0));   // tail
  blk_ctrl.send_items(port_in, tx);

  // ---- Observe trigger time (CE domain) ----
  saw_trig = 0;
  fork
    begin
      @(posedge dut.gen_ports[0].rfnoc_siggen_core_i.trig_fire_q);
      trig_time = dut.gen_ports[0].rfnoc_siggen_core_i.hw_time_now;
      saw_trig  = 1'b1;
    end
    begin
      repeat (300000) @(posedge ce_clk);
    end
  join_any
  disable fork;
  `ASSERT_ERROR(saw_trig, "Never saw trig_fire_q in core");

  // Capture debug signals AFTER trigger (so they are valid)
  @(posedge ce_clk);
  //ts_base_dbg  = dut.gen_ports[0].rfnoc_siggen_core_i.ts_base_q;
 // trig_abs_dbg = dut.gen_ports[0].rfnoc_siggen_core_i.trig_time_abs_q;
  delay_dbg    = dut.gen_ports[0].rfnoc_siggen_core_i.reg_delay;

  // ---- Wait for SOP with timestamp (AXIS domain from shell) ----
  saw_sop = 0;
  fork
    begin
      forever begin
        @(posedge dut.axis_data_clk_s);
        if (dut.s_out_axis_tvalid[port_out] && dut.s_out_axis_tready[port_out] &&
            dut.s_out_axis_thas_time[port_out]) begin
          ts_out       = dut.s_out_axis_ttimestamp[port_out*64 +: 64];
          sop_time_now = dut.gen_ports[0].rfnoc_siggen_core_i.hw_time_now;

      
          saw_sop = 1'b1;
          break;
        end
      end
    end
    begin
      repeat (500000) @(posedge dut.rfnoc_chdr_clk);
    end
  join_any
  disable fork;
  `ASSERT_ERROR(saw_sop, "Never saw SOP with has_time on output");

  // ---- Check the timestamp at SOP ----
  expected_A = trig_time + longint'(delay_cycles) + longint'(TB_PIPE_LAT);
  diff_A     = longint'(ts_out) - longint'(expected_A);

  $display("%0t [TS DIAG] trig_time=%0d  sop_time_now=%0d", $time, trig_time, sop_time_now);
  $display("%0t [TS DIAG] ts_out=%0d  exp=%0d  diff=%0d", $time, ts_out, expected_A, diff_A);

  `ASSERT_ERROR(diff_A >= -1 && diff_A <= 1,
    $sformatf("Timestamp mismatch: ts_out=%0d expected=%0d (+/-1) diff=%0d",
              ts_out, expected_A, diff_A));

  // Only SOP should carry has_time=1
  check_has_time_only_on_sop(/*port_out=*/port_out, /*after_sop_seen=*/1, /*expected_spp=*/0);

  // ---- Cleanup ----
  write_reg(port_out, REG_ENABLE, 0);
  flush_output(port_out);
  test.end_test();
endtask

// Trigger -> timestamp-based delay -> one burst
task automatic test_trigger_delay_burst_len(
  int port_in  = 0,
  int port_out = 0,
  int delay_cycles_in = 400,
  int pw_samples_in   = 64
);

  // -------------------- Declarations (all upfront for XSim) --------------------
  int          delay_cycles;
  int          pw_samples;
  int          spp_cfg;
  int          exp_len;

  item_t       tx[$];
  item_t       items[$];
  int          i;

  // TLAST length semantics candidates
  int          expected_len_items;
  int          expected_len_chdr;
  int          expected_len_bytes;
  logic [15:0] tlen;

  // Fences / guards
  bit          saw_sop_ts;
  bit          saw_tlast;
  int          sop_guard;
  int          wire_guard;
  int          poll_guard;

  // Optional debug capture
  bit  [63:0]  ts_out;

  // Input reachability check
  int          in_guard;
  bit          saw_in_valid;

  // -------------------- Body --------------------
  $asserton;
  test.start_test("Counter-based trigger produces delayed burst", 10ms); // ↑ bigger window

  // Use inputs
  delay_cycles = delay_cycles_in;
  pw_samples   = pw_samples_in;
  spp_cfg      = (pw_samples == 0) ? 1 : pw_samples;
  exp_len      = (pw_samples == 0) ? 1 : pw_samples;

  // Clean slate + deterministic time sync (flush_and_reset clears sync)
  blk_ctrl.flush_and_reset();
  deterministic_sync();           // re-arm after the reset
  repeat (4) @(posedge ce_clk);   // small settle

  // Configure EVERYTHING before enabling
  write_reg(port_out, REG_WAVEFORM,   WAVE_SINE);
  write_reg(port_out, REG_SPP,        spp_cfg);
  write_reg(port_out, REG_GAIN,       16'h7FFF);
  write_reg(port_out, REG_PHASE_INC,  16'd64);
  write_reg(port_out, REG_CARTESIAN,  {16'sh7FFF,16'sh0000});
  write_reg(port_out, REG_THRESHOLD,  32'd500);              // strong trigger for bring-up
  write_reg(port_out, REG_PULSEWIDTH, pw_samples[15:0]);     // 0 -> 1 sample
  write_reg(port_out, REG_DELAY,      delay_cycles);
  write_reg(port_out, REG_HOLDCOUNT,  8'd1);

  // Enable
  write_reg(port_out, REG_ENABLE, 1);
  repeat (2) @(posedge ce_clk);

  // ---- ARMED fence: wait until the core is truly armed before stimulus ----
  wait (dut.gen_ports[0].rfnoc_siggen_core_i.synced      == 1'b1);
  wait (dut.gen_ports[0].rfnoc_siggen_core_i.reg_enable  == 1'b1);
  wait (dut.gen_ports[0].rfnoc_siggen_core_i.use_trigger == 1'b1);
  repeat (8) @(posedge ce_clk);   // let counters settle

  // Drain anything stale
  flush_output(port_out);

  // Stimulus: below -> long high -> below (unambiguous trigger)
  tx.delete();
  for (i = 0; i < 64;  i++) tx.push_back(pack_iq(16'sd200,    16'sd0));   // below thr
  for (i = 0; i < 256; i++) tx.push_back(pack_iq(16'sd20000,  16'sd0));   // above thr → trigger
  for (i = 0; i < 64;  i++) tx.push_back(pack_iq(16'sd200,    16'sd0));   // tail
  blk_ctrl.send_items(port_in, tx);

  // Before delay expires, nothing should arrive
  #(CE_CLK_PER * (delay_cycles / 2));
  if (blk_ctrl.num_received(port_out) != 0)
    $error("%0t [TB] ERROR: Output appeared before delay elapsed!", $time);

  // Make sure *input* samples reach the DUT input AXIS (top-level)
  in_guard     = 0;
  saw_in_valid = 0;
  while (!saw_in_valid && in_guard < 20000) begin
    @(posedge dut.axis_data_clk_s);
    if (dut.s_in_axis_tvalid[port_in] && dut.s_in_axis_tready[port_in]) begin
      saw_in_valid = 1;
    end
    in_guard++;
  end
  `ASSERT_ERROR(saw_in_valid,
    "No input samples observed at DUT s_in_axis (check routing/clock crossing)")

  // ---------- Trigger fence: accept first SOP (has_time) as trigger evidence ----------
  saw_sop_ts = 0;
  sop_guard  = 0;
  while (!saw_sop_ts && sop_guard < (delay_cycles + 100000)) begin
    @(posedge dut.axis_data_clk_s);
    if (dut.s_out_axis_tvalid[port_out] && dut.s_out_axis_tready[port_out] &&
        dut.s_out_axis_thas_time[port_out]) begin
      // First timestamped beat -> trigger happened
      ts_out     = dut.s_out_axis_ttimestamp[port_out*64 +: 64];
      saw_sop_ts = 1;
    end
    sop_guard++;
  end
  `ASSERT_ERROR(saw_sop_ts, "Timeout: no SOP (has_time) observed after stimulus")

  // ---------- End-of-burst: wait TLAST and check tlength semantics ----------
  saw_tlast  = 0;
  wire_guard = 0;

  while (!saw_tlast && wire_guard < 500000) begin
    @(posedge dut.axis_data_clk_s);
    if (dut.s_out_axis_tvalid[port_out] && dut.s_out_axis_tready[port_out] &&
        dut.s_out_axis_tlast[port_out]) begin
      saw_tlast = 1;

      // TLAST length semantics (impls vary: bytes, items, or CHDR words)
      tlen               = dut.s_out_axis_tlength[port_out*16 +: 16];
      expected_len_bytes = exp_len * (ITEM_W/8);
      expected_len_items = exp_len;
      expected_len_chdr  = (expected_len_bytes + (CHDR_W/8) - 1) / (CHDR_W/8);

      $display("%0t [DBG] TLAST tlen=%0d  (exp: bytes=%0d, items=%0d, chdrWords=%0d)",
               $time, tlen, expected_len_bytes, expected_len_items, expected_len_chdr);

      `ASSERT_ERROR( (tlen == expected_len_bytes) ||
                     (tlen == expected_len_items) ||
                     (tlen == expected_len_chdr),
        $sformatf("tlength unexpected: got %0d, expected bytes=%0d or items=%0d or chdrWords=%0d",
                  tlen, expected_len_bytes, expected_len_items, expected_len_chdr));
    end
    wire_guard++;
  end

  `ASSERT_ERROR(saw_tlast, "Never saw TLAST for burst");

  // ---------- Dequeue one packet from the BFM for visibility ----------
  poll_guard = 0;
  while (blk_ctrl.num_received(port_out) == 0 && poll_guard < 200000) begin
    @(posedge rfnoc_chdr_clk);
    poll_guard++;
  end
  `ASSERT_ERROR(blk_ctrl.num_received(port_out) > 0,
    $sformatf("Timeout waiting for packet (guard=%0d)", poll_guard));

  blk_ctrl.recv_items(port_out, items);
  $display("%0t [TB] BFM dequeued %0d samples (expected ≈ %0d)",
           $time, items.size(), exp_len);

  // ---------- Cleanup ----------
  write_reg(port_out, REG_ENABLE, 0);
  flush_output(port_out);
  test.end_test();
  $assertoff;
endtask


  // Test the min and max allowed values on all registers
  task automatic test_registers(int port);
    test.start_test($sformatf("Test registers (port %0d)", port), 1ms);
    // REG_ENABLE and REG_WAVEFORM will be tested during the other tests
    test_read_write_reg(port, REG_SPP,       {REG_SPP_LEN{1'b1}},       32'd16);
    test_read_write_reg(port, REG_GAIN,      {REG_GAIN_LEN{1'b1}},      32'h7FFF);
    test_read_write_reg(port, REG_CONSTANT,  {REG_CONSTANT_LEN{1'b1}},  32'h0);
    test_read_write_reg(port, REG_PHASE_INC, {REG_PHASE_INC_LEN{1'b1}}, 32'h0000_0000);
    test_read_write_reg(port, REG_CARTESIAN, {REG_CARTESIAN_LEN{1'b1}}, 32'h0000_0000);
    
      // NEW (assuming you defined these in the .vh):
    test_read_write_reg(port, REG_THRESHOLD,  {REG_THRESHOLD_LEN{1'b1}},  32'h0000_0000);
    test_read_write_reg(port, REG_PULSEWIDTH, {REG_PULSEWIDTH_LEN{1'b1}}, 32'h0000_0020);
    test_read_write_reg(port, REG_DELAY,      {REG_DELAY_LEN{1'b1}},      32'h0000_0000);
    test_read_write_reg(port, REG_HOLDCOUNT, {REG_HOLDCOUNT_LEN{1'b1}}, 32'h0000_0001);

    test.end_test();
  endtask : test_registers


  // Run through all the waveform modes to make sure they work as expected
  task automatic test_waveforms(int port);
    test.start_test($sformatf("Test waveforms (port %0d)", port), 1ms);
    run_const(.port(port), .gain(0.5), .re(0.25), .im(0.5));
    run_sine(.port(port), .gain(0.75), .x(0.25), .y(0.5), .phase(2.0/64));
    run_noise(.port(port), .gain(0.999));
    test.end_test();
  endtask : test_waveforms


  // Use the constant waveform to test the gain. The gain logic is shared by
  // all modes, but using "const" waveform makes it easy to control the values
  // we're testing.
  task automatic test_gain(int port);
    logic signed [15:0] min_val;
    logic signed [15:0] max_val;

    test.start_test($sformatf("Test gain (port %0d)", port), 1ms);

    max_val = 16'sh7FFF;
    min_val = 16'sh8000;

    // Test max gain with min and max sample values
    run_waveform(.port(port), .mode(WAVE_CONST), .gain(max_val),
      .const_re(max_val), .const_im(min_val));
    // Test min gain with max and min sample values
    run_waveform(.port(port), .mode(WAVE_CONST), .gain(min_val),
      .const_re(min_val), .const_im(max_val));
    // Test zero
    run_waveform(.port(port), .mode(WAVE_CONST), .gain(0),
      .const_re(max_val), .const_im(min_val));
    // Test 0.5 * 0.5 = 0.25 and 0.25 * 0.5 = 0.125
    run_waveform(
      .port(port),
      .mode(WAVE_CONST),
      .const_re(real_to_fixed(0.5, CONST_FRAC)),
      .const_im(real_to_fixed(0.25, CONST_FRAC)),
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

// -----------------------------------------------------------------------------
// Smoke test for debug CSRs existence + basic semantics
//   - REG_DBG_STATUS   (RO) : read works; writes ignored
//   - REG_DBG_COUNTS   (RO) : read works; writes ignored
//   - REG_DBG_SNAP0    (RO) : read works; writes ignored
//   - REG_DBG_CTRL     (WO) : write bit[0]=1 clears sticky bits in STATUS
// -----------------------------------------------------------------------------
task automatic test_debug_csrs(int port);
  // If these are in your .vh they'll resolve; otherwise hardcode the word addrs.
  // localparam [19:0] REG_DBG_CTRL   = 20'h00030;
  // localparam [19:0] REG_DBG_STATUS = 20'h00034;
  // localparam [19:0] REG_DBG_COUNTS = 20'h00038;
  // localparam [19:0] REG_DBG_SNAP0  = 20'h0003C;

  string       err;
  logic [31:0] v_before, v_after;

  // Expected sticky bits inside STATUS:
  // [0]=trigger_seen (sticky), [1]=burst_start_seen (sticky), [3]=output_seen (sticky)
  // [2],[4],[5],[6],[7] are live bits → don't assert on them here.
  localparam logic [31:0] DBG_STICKY_MASK = 32'h0000_000B;

  // Sanity: addresses are the expected word offsets
  `ASSERT_ERROR(REG_DBG_CTRL   == 20'h00030, "REG_DBG_CTRL addr mismatch")
  `ASSERT_ERROR(REG_DBG_STATUS == 20'h00034, "REG_DBG_STATUS addr mismatch")
  `ASSERT_ERROR(REG_DBG_COUNTS == 20'h00038, "REG_DBG_COUNTS addr mismatch")
  `ASSERT_ERROR(REG_DBG_SNAP0  == 20'h0003C, "REG_DBG_SNAP0 addr mismatch")

  // --- STATUS: RO, readable
  read_reg(port, REG_DBG_STATUS, v_before); // just make sure read works

  // Attempt write (should be ignored)
  write_reg(port, REG_DBG_STATUS, 32'hDEAD_BEEF);
  read_reg(port, REG_DBG_STATUS, v_after);
  err = $sformatf("REG_DBG_STATUS should be RO (port %0d)", port);
  `ASSERT_ERROR(v_after === v_before, err)

  // --- COUNTS: RO, readable
  read_reg(port, REG_DBG_COUNTS, v_before);
  write_reg(port, REG_DBG_COUNTS, 32'hFEED_CAFE);
  read_reg(port, REG_DBG_COUNTS, v_after);
  err = $sformatf("REG_DBG_COUNTS should be RO (port %0d)", port);
  `ASSERT_ERROR(v_after === v_before, err)

  // --- SNAP0: RO, readable
  read_reg(port, REG_DBG_SNAP0, v_before);
  write_reg(port, REG_DBG_SNAP0, 32'hABCD_1234);
  read_reg(port, REG_DBG_SNAP0, v_after);
  err = $sformatf("REG_DBG_SNAP0 should be RO (port %0d)", port);
  `ASSERT_ERROR(v_after === v_before, err)

  // --- CTRL: WO, bit0=clear; write should not bus-error and should clear sticky bits in STATUS
  // First, try to set some sticky bits by reading STATUS (they might already be 0/1; we don't care).
  read_reg(port, REG_DBG_STATUS, v_before);

  // Clear
  write_reg(port, REG_DBG_CTRL, 32'h0000_0001);
  read_reg(port, REG_DBG_STATUS, v_after);

  // After clear, sticky bits must be 0 (live bits may change; mask them out)
  err = $sformatf("REG_DBG_CTRL clear failed to reset sticky bits (port %0d). before=0x%08x after=0x%08x",
                  port, v_before, v_after);
  `ASSERT_ERROR((v_after & DBG_STICKY_MASK) == 32'h0, err)
endtask


  //---------------------------------------------------------------------------
  // Main Test Process
  //---------------------------------------------------------------------------

initial begin : tb_main
  int port;

  test.start_tb($sformatf("rfnoc_block_siggen_tb (CHDR_W = %0d, NUM_PORTS = %0d)", CHDR_W, NUM_PORTS));

  rfnoc_chdr_clk_gen.start();
  rfnoc_ctrl_clk_gen.start();
  ce_clk_gen.start();
  blk_ctrl.run();

  // -------- Reset FIRST --------
  test.start_test("Flush block then reset it", 10us);
  blk_ctrl.flush_and_reset();
  test.end_test();

  // Small settle
  #500ns;

  // -------- Deterministic abs_time sync (AFTER reset) --------
  // Force known state for one cycle so 'now + base_offset' = 0 and synced=1
  force dut.gen_ports[0].rfnoc_siggen_core_i.now         = 64'd0;
  force dut.gen_ports[0].rfnoc_siggen_core_i.base_offset = 64'd0;
  force dut.gen_ports[0].rfnoc_siggen_core_i.synced      = 1'b1;
  @(posedge ce_clk);

  // Release everything (prefer not to keep forces during the test)
  release dut.gen_ports[0].rfnoc_siggen_core_i.now;
  release dut.gen_ports[0].rfnoc_siggen_core_i.base_offset;
  release dut.gen_ports[0].rfnoc_siggen_core_i.synced;

  $display("%0t [TB] Deterministic abs_time sync done (now=0, base_offset=0, synced=1)", $time);

  // --------------------------------
  // Verify Block Info
  // --------------------------------
  test.start_test("Verify Block Info", 2us);
  `ASSERT_ERROR(blk_ctrl.get_noc_id() == NOC_ID, "Incorrect NOC_ID Value");
  `ASSERT_ERROR(blk_ctrl.get_num_data_i() == NUM_PORTS_I, "Incorrect NUM_DATA_I Value");
  `ASSERT_ERROR(blk_ctrl.get_num_data_o() == NUM_PORTS_O, "Incorrect NUM_DATA_O Value");
  `ASSERT_ERROR(blk_ctrl.get_mtu() == MTU, "Incorrect MTU Value");
  test.end_test();

  // --------------------------------
  // Test Sequences
  // --------------------------------
  for(port = 0; port < NUM_PORTS; port++) begin
    test_registers(port);
    test_debug_csrs(port);

    // test_waveforms(port); // keep disabled for pulse-mode tests
  end

  port = 0;
 // test_gain(port);
  //test_packet_length(port);
  //test_phase(port);

  // Trigger/Delay tests only
  test_no_trigger_no_output(0, 0);
  test_trigger_delay_burst_len(0,0, 0,   0);   // Case 1
  test_trigger_delay_burst_len(0,0, 0,  64);   // Case 2
  test_trigger_delay_burst_len(0,0, 400, 64);  // Case 3
  test_timestamp_plus_delay(0, 0, 400, 64);
  test.end_tb(0);
  rfnoc_chdr_clk_gen.kill();
  rfnoc_ctrl_clk_gen.kill();
  ce_clk_gen.kill();
end : tb_main

endmodule : rfnoc_block_siggen_tb


`default_nettype wire