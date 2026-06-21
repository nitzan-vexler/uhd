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


module rfnoc_siggen_core #(
  parameter integer PIPE_LATENCY = 16 // Measured pipeline latency in CE_CLK cycles
)(

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
  reg [REG_AVG_START_DELAY_LEN-1:0] reg_avg_start_delay;
  reg [31:0] dbg_avg_power;
  reg [15:0] dbg_tx_amp;

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
      reg_avg_start_delay <= 8'd32;
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
          REG_AVG_START_DELAY : reg_avg_start_delay <= s_ctrlport_req_data[REG_AVG_START_DELAY_LEN-1:0];
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
          REG_DBG_AVG_POWER : s_ctrlport_resp_data <= dbg_avg_power;
          REG_DBG_TX_AMP    : s_ctrlport_resp_data <= {16'd0, dbg_tx_amp};
          REG_AVG_START_DELAY :
    s_ctrlport_resp_data[REG_AVG_START_DELAY_LEN-1:0] <= reg_avg_start_delay;
        endcase
      end
    end
  end

// -----------------------------
// Trigger / Delay / Burst control (Verilog-2001)
// Free-run if REG_THRESHOLD==0; otherwise use trigger FSM
// -----------------------------


// Extract I/Q
wire signed [15:0] in_i = s_tdata[31:16];
wire signed [15:0] in_q = s_tdata[15:0];

// Threshold
wire [15:0] thr = reg_threshold[15:0];
wire        use_trigger = (thr != 16'd0);

// I^2 and Q^2
wire [31:0] i_sq = in_i * in_i;
wire [31:0] q_sq = in_q * in_q;
wire [32:0] mag_sq = {1'b0,i_sq} + {1'b0,q_sq};

// thr^2 (registered)
reg [31:0] thr_sq;
always @(posedge clk) begin
  if (rst)
    thr_sq <= 0;
  else
    thr_sq <= thr * thr;
end

// Comparison (registered)
reg over_thr_d;
always @(posedge clk) begin
  if (rst)
    over_thr_d <= 1'b0;
  else
    over_thr_d <= s_tvalid && (mag_sq >= {1'b0,thr_sq});
end

// Edge detector
reg over_thr_q;
always @(posedge clk) begin
  if (rst)
    over_thr_q <= 1'b0;
  else
    over_thr_q <= over_thr_d;
end

wire trig_pulse = over_thr_d & ~over_thr_q;


localparam [2:0]
    ST_IDLE       = 3'd0,
    ST_WAIT_AVG   = 3'd1,
    ST_PEAK       = 3'd2,
    ST_DELAY      = 3'd3,
    ST_BURST      = 3'd4;
    
reg [7:0] avg_delay_ctr;
reg [2:0] state;

// Counters (match your register widths)
reg [REG_DELAY_LEN-1:0]      delay_ctr;
reg [REG_PULSEWIDTH_LEN-1:0] pw_ctr;

// FSM-controlled burst flag
reg fsm_burst_active;

localparam PEAK_SEARCH_SAMPLES = 128;
reg [7:0] peak_cnt;
reg        peak_search_active;

reg [31:0] trigger_sample;


reg [6:0] tail_ctr;
reg tail_active;
wire burst_active = use_trigger ? (fsm_burst_active || tail_active) : 1'b1;
// IMPORTANT: Count pulsewidth only for *visible* samples
wire sample_fire = m_tvalid & m_tready;

// synthesis translate_off
always @(posedge clk) begin
  if (!rst) begin
    if (trig_pulse)
      $display("%0t SIGGEN trig_pulse s_tdata=0x%08x mag_sq=%0d thr_sq=%0d",
        $time, s_tdata, mag_sq, thr_sq);

    if (burst_start)
      $display("%0t SIGGEN burst_start trigger_sample=0x%08x",
        $time, trigger_sample);

    if (m_tvalid && m_tready)
      $display("%0t SIGGEN OUT m_tdata=0x%08x m_tlast=%0d",
        $time, m_tdata, m_tlast);
  end
end
// synthesis translate_on

function [15:0] isqrt_sat16;
  input [39:0] x;
  integer i;
  reg [31:0] test;
  begin
    isqrt_sat16 = 16'd0;

    for (i = 15; i >= 0; i = i - 1) begin
      test = ({16'd0, isqrt_sat16} | (32'd1 << i));
      if ((test * test) <= x)
        isqrt_sat16[i] = 1'b1;
    end
  end
endfunction
    
reg [47:0] power_accum;

wire [47:0] power_accum_next = power_accum + {15'd0, mag_sq};
wire [39:0] avg_power_128 = power_accum_next[47:7]; // divide by 128

wire [15:0] tx_amp_from_power = isqrt_sat16(avg_power_128);

    
always @(posedge clk) begin
  if (rst) begin
    state            <= ST_IDLE;
    delay_ctr        <= {REG_DELAY_LEN{1'b0}};
    pw_ctr           <= {REG_PULSEWIDTH_LEN{1'b0}};
    fsm_burst_active <= 1'b0;
    tail_active      <= 1'b0;
    tail_ctr         <= 0;

    peak_cnt           <= 8'd0;
    peak_search_active <= 1'b0;
    trigger_sample     <= 32'd0;
    power_accum <= 48'd0;
    dbg_avg_power      <= 32'd0;   // MOVE HERE
    dbg_tx_amp      <= 16'd0;   // MOVE HERE
    avg_delay_ctr <= 8'd0;



  end else if (use_trigger) begin
    case (state)

      ST_IDLE: begin
        fsm_burst_active <= 1'b0;
        tail_active      <= 1'b0;
        peak_search_active <= 1'b0;

if (trig_pulse) begin
  peak_search_active <= 1'b0;
avg_delay_ctr <= reg_avg_start_delay;
  power_accum        <= 48'd0;
  state              <= ST_WAIT_AVG;
end
      end

ST_WAIT_AVG: begin
  if (s_tvalid) begin
    if (avg_delay_ctr != 0) begin
      avg_delay_ctr <= avg_delay_ctr - 1;
    end else begin
      peak_search_active <= 1'b1;
      peak_cnt           <= PEAK_SEARCH_SAMPLES - 1;
      power_accum        <= {15'd0, mag_sq};
      state              <= ST_PEAK;
    end
  end
end

ST_PEAK: begin
  if (s_tvalid) begin
power_accum <= power_accum_next;
    if (peak_cnt != 0) begin
      peak_cnt <= peak_cnt - 1;
    end else begin
      peak_search_active <= 1'b0;
dbg_tx_amp <= tx_amp_from_power;
dbg_avg_power <= avg_power_128[31:0];
trigger_sample <= {tx_amp_from_power, 16'sd0};

      delay_ctr <= reg_delay;
      pw_ctr    <= reg_pulsewidth;
      state     <= (reg_delay == 0) ? ST_BURST : ST_DELAY;
    end
  end
end

      ST_DELAY: begin
        if (delay_ctr != 0)
          delay_ctr <= delay_ctr - 1;
        else
          state <= ST_BURST;
      end

      ST_BURST: begin
        fsm_burst_active <= 1'b1;

        if (sample_fire && !tail_active) begin
          if (pw_ctr > 1) begin
            pw_ctr <= pw_ctr - 1;
          end else begin
            pw_ctr      <= 0;
            tail_active <= 1'b1;
            tail_ctr    <= PIPE_LATENCY[6:0];
          end
        end

        if (tail_active) begin
          if (tail_ctr != 0)
            tail_ctr <= tail_ctr - 1;
          else begin
            fsm_burst_active <= 1'b0;
            tail_active      <= 1'b0;
            state            <= ST_IDLE;
          end
        end
      end

      default: begin
        state            <= ST_IDLE;
        fsm_burst_active <= 1'b0;
        tail_active      <= 1'b0;
        peak_search_active <= 1'b0;
      end

    endcase
  end else begin
    state            <= ST_IDLE;
    fsm_burst_active <= 1'b0;
    tail_active      <= 1'b0;
    peak_search_active <= 1'b0;
  end
end


// Track previous state to detect entering ST_BURST
reg [2:0] prev_state;
always @(posedge clk) begin
  if (rst) prev_state <= ST_IDLE;
  else     prev_state <= state;
end
wire burst_start = use_trigger && (prev_state != ST_BURST) && (state == ST_BURST);




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

//---------------------------------------------------------------------------
 // Round + Clip (you need these wires here!)
//---------------------------------------------------------------------------
wire [31:0] axis_round_tdata;
wire        axis_round_tvalid;
wire        axis_round_tready;


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


//-----------------------------------------------------------------------
// Packet Length Control + SAFE gating logic
//-----------------------------------------------------------------------

wire [REG_SPP_LEN-1:0] m_tlength_samples;
assign m_tlength = {m_tlength_samples, 2'b00};   // 4 bytes per sample

//----------------------------------------------
// Track packet activity
//----------------------------------------------
reg pkt_active;

always @(posedge clk) begin
  if (rst)
    pkt_active <= 1'b0;
  else if (m_tvalid && m_tready) begin
    if (!pkt_active)
      pkt_active <= 1'b1;
    if (m_tlast)
      pkt_active <= 1'b0;
  end
end



//----------------------------------------------
// Final allow_output signal
//----------------------------------------------
wire allow_output = reg_enable & (use_trigger ? burst_active : 1'b1);
wire axis_rx_tready;


wire [31:0] repeat_tdata;
wire        repeat_tvalid;
assign repeat_tdata  = use_trigger ? trigger_sample : axis_round_tdata;
assign repeat_tvalid = use_trigger ? allow_output    : axis_round_tvalid;

//----------------------------------------------
// Packetizer
//----------------------------------------------
axis_packetize #(
  .DATA_W (32),
  .SIZE_W (REG_SPP_LEN),
  .FLUSH  (1)
) axis_packetize_i (
  .clk      (clk),
  .rst      (rst),
  .gate     (~allow_output),  // SAFE gating
  .size     (reg_spp),

  .i_tdata  (repeat_tdata),
  .i_tvalid (repeat_tvalid),
  .i_tready (axis_rx_tready),

  .o_tdata  (m_tdata),
  .o_tlast  (m_tlast),
  .o_tvalid (m_tvalid),
  .o_tready (m_tready),

  .o_tuser  (m_tlength_samples)
);




endmodule
