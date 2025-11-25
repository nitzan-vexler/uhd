//
// Copyright 2020 Ettus Research, a National Instruments Brand
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//
// Module: rfnoc_siggen_core
//
// Description:
//
//   This module contains the registers and core logic for a single RFNoC
//   Signal Generator module instance.
//


module rfnoc_siggen_core (
  input wire clk,
  input wire rst,

  // CtrlPort Slave
  input  wire        s_ctrlport_req_wr,
  input  wire        s_ctrlport_req_rd,
  input  wire [19:0] s_ctrlport_req_addr,
  input  wire [31:0] s_ctrlport_req_data,
  output reg         s_ctrlport_resp_ack,
  output reg  [31:0] s_ctrlport_resp_data,
  
  input  wire [31:0] s_tdata,
  input  wire        s_tvalid,
  input  wire        s_tlast,
  output wire        s_tready,
  input  wire [15:0] s_tlength,

  // Output data stream
  output wire [31:0] m_tdata,
  output wire        m_tlast,
  output wire        m_tvalid,
  input  wire        m_tready,
  output wire [15:0] m_tlength
);

  `include "rfnoc_block_siggen_regs.vh"

assign s_tready = 1'b1;   // always ready to sample input for trigger detection

  //---------------------------------------------------------------------------
  // Registers
  //---------------------------------------------------------------------------

  // Define maximum fixed point value for the gain, equal to about 0.9999
  localparam MAX_GAIN = {REG_GAIN_LEN-1{1'b1}};

  reg [   REG_ENABLE_LEN-1:0] reg_enable    = 0;
  reg [      REG_SPP_LEN-1:0] reg_spp       = 16;
  reg [ REG_WAVEFORM_LEN-1:0] reg_waveform  = WAVE_CONST;
  reg [     REG_GAIN_LEN-1:0] reg_gain      = MAX_GAIN;
  reg [ REG_CONSTANT_LEN-1:0] reg_constant  = 0;
  reg [REG_PHASE_INC_LEN-1:0] reg_phase_inc;
  reg [REG_CARTESIAN_LEN-1:0] reg_cartesian;
  reg [REG_THRESHOLD_LEN-1:0]  reg_threshold  = 0;
  reg [REG_PULSEWIDTH_LEN-1:0] reg_pulsewidth = 16'd32;
  reg [REG_DELAY_LEN-1:0]      reg_delay      = 32'd0;
  reg [REG_WARMUP_LEN-1:0]    reg_warmup;

  reg reg_phase_inc_stb;
  reg reg_cartesian_stb;

  always @(posedge clk) begin
    if (rst) begin
      reg_enable           <= 0;
      reg_spp              <= 16;
      reg_waveform         <= WAVE_CONST;
      reg_gain             <= MAX_GAIN;
      reg_constant         <= 0;
      reg_phase_inc        <= 'bX;
      reg_cartesian        <= 'bX;
      reg_threshold        <= 0;        // threshold disabled by default
      reg_pulsewidth       <= 16'd32;
      reg_delay            <= 0;
      s_ctrlport_resp_ack  <= 1'b0;
      s_ctrlport_resp_data <= 'bX;
      reg_phase_inc_stb    <= 1'b0;
      reg_cartesian_stb    <= 1'b0;
      reg_warmup <= {REG_WARMUP_LEN{1'b0}};   // default 0 (no warm-up)
    end else begin

      // Default assignments
      s_ctrlport_resp_ack  <= 1'b0;
      s_ctrlport_resp_data <= 0;
      reg_phase_inc_stb    <= 1'b0;
      reg_cartesian_stb    <= 1'b0;

      // Handle register writes
      if (s_ctrlport_req_wr) begin
        s_ctrlport_resp_ack <= 1;
        case (s_ctrlport_req_addr)
          REG_ENABLE    : reg_enable    <= s_ctrlport_req_data[REG_ENABLE_LEN-1:0];
          REG_SPP       : reg_spp       <= s_ctrlport_req_data[REG_SPP_LEN-1:0];
          REG_WAVEFORM  : reg_waveform  <= s_ctrlport_req_data[REG_WAVEFORM_LEN-1:0];
          REG_GAIN      : reg_gain      <= s_ctrlport_req_data[REG_GAIN_LEN-1:0];
          REG_CONSTANT  : reg_constant  <= s_ctrlport_req_data[REG_CONSTANT_LEN-1:0];
          REG_THRESHOLD  : reg_threshold  <= s_ctrlport_req_data[REG_THRESHOLD_LEN-1:0];
          REG_PULSEWIDTH : reg_pulsewidth <= s_ctrlport_req_data[REG_PULSEWIDTH_LEN-1:0];
          REG_DELAY      : reg_delay      <= s_ctrlport_req_data[REG_DELAY_LEN-1:0];
          REG_WARMUP     : reg_warmup     <= s_ctrlport_req_data[REG_WARMUP_LEN-1:0];
          REG_PHASE_INC : begin
            reg_phase_inc     <= s_ctrlport_req_data[REG_PHASE_INC_LEN-1:0];
            reg_phase_inc_stb <= 1'b1;
          end
          REG_CARTESIAN : begin
            reg_cartesian     <= s_ctrlport_req_data[REG_CARTESIAN_LEN-1:0];
            reg_cartesian_stb <= 1'b1;
          end
        endcase
      end

      // Handle register reads
      if (s_ctrlport_req_rd) begin
        s_ctrlport_resp_ack <= 1;
        case (s_ctrlport_req_addr)
          REG_ENABLE    : s_ctrlport_resp_data[REG_ENABLE_LEN-1:0]     <= reg_enable;
          REG_SPP       : s_ctrlport_resp_data[REG_SPP_LEN-1:0]        <= reg_spp;
          REG_WAVEFORM  : s_ctrlport_resp_data[REG_WAVEFORM_LEN-1:0]   <= reg_waveform;
          REG_GAIN      : s_ctrlport_resp_data[REG_GAIN_LEN-1:0]       <= reg_gain;
          REG_CONSTANT  : s_ctrlport_resp_data[REG_CONSTANT_LEN-1:0]   <= reg_constant;
          REG_PHASE_INC : s_ctrlport_resp_data[REG_PHASE_INC_LEN-1:0]  <= reg_phase_inc;
          REG_CARTESIAN : s_ctrlport_resp_data[REG_CARTESIAN_LEN-1:0]  <= reg_cartesian;
          REG_THRESHOLD  : s_ctrlport_resp_data[REG_THRESHOLD_LEN-1:0]  <= reg_threshold;
          REG_PULSEWIDTH : s_ctrlport_resp_data[REG_PULSEWIDTH_LEN-1:0] <= reg_pulsewidth;
          REG_DELAY      : s_ctrlport_resp_data[REG_DELAY_LEN-1:0]      <= reg_delay;
          REG_WARMUP      : s_ctrlport_resp_data[REG_WARMUP_LEN-1:0]      <= reg_warmup;
        endcase
      end
    end
  end

// -----------------------------
// Trigger / Delay / Burst control (Verilog-2001)
// Free-run if REG_THRESHOLD==0; otherwise use trigger FSM
// -----------------------------

// Extract signed I/Q from 32-bit input sample (I:[31:16], Q:[15:0])
wire signed [15:0] in_i = s_tdata[31:16];
wire signed [15:0] in_q = s_tdata[15:0];

// 16-bit absolute value (two's complement)
function [15:0] abs16;
  input signed [15:0] x;
  begin
    abs16 = x[15] ? (~x + 16'd1) : x;
  end
endfunction

wire [15:0] thr         = reg_threshold[15:0];
wire        use_trigger = (thr != 16'd0);
wire        over_thr    = s_tvalid & ((abs16(in_i) >= thr) | (abs16(in_q) >= thr));
// after 'over_thr' is defined
reg over_thr_q;
always @(posedge clk) begin
  if (rst) over_thr_q <= 1'b0;
  else     over_thr_q <= over_thr;
end

wire trig_pulse = over_thr & ~over_thr_q;  // fire once on rising edge

// FSM states
localparam [1:0] ST_IDLE  = 2'd0,
                 ST_DELAY = 2'd1,
                 ST_BURST = 2'd2;

reg [1:0] state;

// Counters (match your register widths)
reg [REG_DELAY_LEN-1:0]      delay_ctr;
reg [REG_PULSEWIDTH_LEN-1:0] pw_ctr;

// FSM-controlled burst flag
reg fsm_burst_active;

wire burst_active = use_trigger ? fsm_burst_active : 1'b1;

// IMPORTANT: Count pulsewidth only for *visible* samples
wire sample_fire = m_tvalid & m_tready;

// FSM (armed only when use_trigger==1)
always @(posedge clk) begin
  if (rst) begin
    state            <= ST_IDLE;
    delay_ctr        <= {REG_DELAY_LEN{1'b0}};
    pw_ctr           <= {REG_PULSEWIDTH_LEN{1'b0}};
    fsm_burst_active <= 1'b0;
  end else if (use_trigger) begin
    case (state)
      ST_IDLE: begin
        fsm_burst_active <= 1'b0;
        if (trig_pulse) begin
          delay_ctr <= reg_delay;
          pw_ctr    <= reg_pulsewidth;
          state     <= (reg_delay == {REG_DELAY_LEN{1'b0}}) ? ST_BURST : ST_DELAY;
        end
      end
      ST_DELAY: begin
        if (delay_ctr != {REG_DELAY_LEN{1'b0}})
          delay_ctr <= delay_ctr - {{(REG_DELAY_LEN-1){1'b0}},1'b1};
        else
          state <= ST_BURST;
      end
      ST_BURST: begin
        fsm_burst_active <= 1'b1;
        if (sample_fire) begin
          if (pw_ctr != {REG_PULSEWIDTH_LEN{1'b0}})
            pw_ctr <= pw_ctr - {{(REG_PULSEWIDTH_LEN-1){1'b0}},1'b1};
          else begin
            fsm_burst_active <= 1'b0;
            state            <= ST_IDLE;
          end
        end
      end
      default: begin
        state            <= ST_IDLE;
        fsm_burst_active <= 1'b0;
      end
    endcase
  end else begin
    // Free-run mode (no trigger)
    state            <= ST_IDLE;
    fsm_burst_active <= 1'b0;
  end
end
// Track previous state to detect entering ST_BURST
reg [1:0] prev_state;
always @(posedge clk) begin
  if (rst) prev_state <= ST_IDLE;
  else     prev_state <= state;
end
wire burst_start = use_trigger && (prev_state != ST_BURST) && (state == ST_BURST);



assign m_tlength = { m_tlength_samples, 2'b0 }; // 4 bytes/sample

// If you generate tlast elsewhere, gate it too:
// assign m_tlast = your_spp_tlast & gated_tvalid;


  //---------------------------------------------------------------------------
  // Waveform Generation
  //---------------------------------------------------------------------------

  wire [31:0]  axis_sine_tdata;
  wire         axis_sine_tvalid;
  wire         axis_sine_tready;


  //------------------------------------
  // Sine waveform generation
  //------------------------------------

  // Convert the registers writes to settings bus transactions. Only one
  // register strobe will assert at a time.
  wire        sine_set_stb  = reg_cartesian_stb | reg_phase_inc_stb;
  wire [31:0] sine_set_data = reg_cartesian_stb ? reg_cartesian : reg_phase_inc;
  wire [ 7:0] sine_set_addr = reg_cartesian_stb;

  sine_tone #(
    .WIDTH             (32),
    .SR_PHASE_INC_ADDR (0),
    .SR_CARTESIAN_ADDR (1)
  ) sine_tone_i (
    .clk      (clk),
    .reset    (rst),
    .clear    (1'b0),
    .enable   (1'b1),
    .set_stb  (sine_set_stb),
    .set_data (sine_set_data),
    .set_addr (sine_set_addr),
    .o_tdata  (axis_sine_tdata),
    .o_tlast  (),
    .o_tvalid (axis_sine_tvalid),
    .o_tready (axis_sine_tready)
  );




  //---------------------------------------------------------------------------
  // Gain
  //---------------------------------------------------------------------------
wire [63:0] axis_gain_tdata;
wire        axis_gain_tvalid, axis_gain_tready;

// Source is always the sine
wire [31:0] axis_src_tdata  = axis_sine_tdata;
wire        axis_src_tvalid = axis_sine_tvalid;
wire        axis_src_tready;
assign      axis_sine_tready = axis_src_tready;

// Feed gain directly from sine
mult_rc #(.WIDTH_REAL(16), .WIDTH_CPLX(16), .WIDTH_P(32), .DROP_TOP_P(5), .LATENCY(4)) mult_rc_i (
  .clk(clk), .reset(rst),
  .real_tdata(reg_gain),
  .real_tlast(1'b0), .real_tvalid(1'b1), .real_tready(),
  .cplx_tdata(axis_src_tdata),
  .cplx_tlast(1'b0), .cplx_tvalid(axis_src_tvalid), .cplx_tready(axis_src_tready),
  .p_tdata(axis_gain_tdata), .p_tlast(), .p_tvalid(axis_gain_tvalid), .p_tready(axis_gain_tready)
);


  axi_round_and_clip_complex #(
    .WIDTH_IN  (32),
    .WIDTH_OUT (16),
    .CLIP_BITS (1)
  ) axi_round_and_clip_complex_i (
    .clk      (clk),
    .reset    (rst),
    .i_tdata  (axis_gain_tdata),
    .i_tlast  (1'b0),
    .i_tvalid (axis_gain_tvalid),
    .i_tready (axis_gain_tready),
    .o_tdata  (axis_round_tdata),
    .o_tlast  (),
    .o_tvalid (axis_round_tvalid),
    .o_tready (axis_round_tready)
  );


  //---------------------------------------------------------------------------
  // Packet Length Control
  //---------------------------------------------------------------------------

  wire [REG_SPP_LEN-1:0] m_tlength_samples;

  assign m_tlength = { m_tlength_samples, 2'b0 };   // 4 bytes per sample

// Trigger mode only when a threshold is set (free-run otherwise)

wire allow_output = reg_enable & (use_trigger ? burst_active : 1'b1);



axis_packetize #(
  .DATA_W (32), .SIZE_W (REG_SPP_LEN), .FLUSH(1)
) axis_packetize_i (
  .clk      (clk),
  .rst      (rst),
  .gate     (~allow_output),           // <-- only gate here
  .size     (reg_spp),
  .i_tdata  (axis_round_tdata),        // <-- do NOT AND with allow_output
  .i_tvalid (axis_round_tvalid),       // <-- leave this untouched
  .i_tready (axis_round_tready),
  .o_tdata  (m_tdata),
  .o_tlast  (m_tlast),
  .o_tvalid (m_tvalid),
  .o_tready (m_tready),
  .o_tuser  (m_tlength_samples)
);




endmodule
