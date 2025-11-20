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
  parameter integer CHDR_W = 64   // <- new: width of the CHDR/AXIS data path in bits
)(
  input  wire        clk,
  input  wire        rst,

  // CtrlPort Slave
  input  wire        s_ctrlport_req_wr,
  input  wire        s_ctrlport_req_rd,
  input  wire [19:0] s_ctrlport_req_addr,
  input  wire [31:0] s_ctrlport_req_data,
  output reg         s_ctrlport_resp_ack,
  output reg  [31:0] s_ctrlport_resp_data,

  // Input stream (RX-side data for trigger detection)
  input  wire [31:0] s_tdata,
  input  wire        s_tvalid,
  input  wire        s_tlast,
  output wire        s_tready,
  input  wire [15:0] s_tlength,
  input  wire [63:0] s_ttimestamp,
  input  wire        s_thas_time,

  // Output stream (generated waveform)
  output wire [31:0] m_tdata,
  output wire        m_tlast,
  output wire        m_tvalid,
  input  wire        m_tready,
  output wire [15:0] m_tlength,   // stays a wire

  // Timestamp sideband for scheduled TX
  output reg  [63:0] m_ttimestamp,
  output reg         m_thas_time
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
  reg [REG_PHASE_INC_LEN-1:0] reg_phase_inc = 0;
  reg [REG_CARTESIAN_LEN-1:0] reg_cartesian = 0;
  reg [REG_THRESHOLD_LEN-1:0]  reg_threshold  = 0;
  reg [REG_PULSEWIDTH_LEN-1:0] reg_pulsewidth = 16'd32;
  reg [REG_DELAY_LEN-1:0]      reg_delay      = 32'd0;
 // New register: hold count (number of consecutive samples above threshold required)
  reg [REG_HOLDCOUNT_LEN-1:0] reg_holdcount = 8'd1;  // default = 1 → trigger immediately


  reg reg_phase_inc_stb;
  reg reg_cartesian_stb;
wire [15:0] thr = reg_threshold[15:0];
wire        use_trigger = |reg_threshold;

// --------------------------- DEBUG: state -----------------------------------
reg        dbg_trigger_seen, dbg_burst_start_seen, dbg_output_seen;
reg [31:0] dbg_trigger_cnt,  dbg_burst_cnt,       dbg_output_cnt;
reg [31:0] dbg_last_amp,     dbg_last_thr;
reg [15:0] dbg_last_warm_ctr;
reg        dbg_last_gate;  // 1 = gate blocked when comparator fired

// Timestamp debug
reg [63:0] dbg_ts_pkt;       // timestamp from incoming packet (if available)
reg [63:0] dbg_ts_now_trig;  // hw_time_now at trigger pulse
reg [63:0] dbg_ts_target;    // scheduled target timestamp (ts_target)
reg        dbg_ts_seen;      // we latched a packet timestamp at least once


wire dbg_clear = s_ctrlport_req_wr
              && (s_ctrlport_req_addr == REG_DBG_CTRL)
              && s_ctrlport_req_data[0];

// ===========================================================
// Absolute-time scheduling registers
// ===========================================================
localparam integer PIPE_LAT   = 6;          // adjust to your real pipeline (mult_rc + round/clip + regs)

reg         ts_arm_q;        // we owe a timestamp on SOP of next burst
reg [7:0] warm_cnt_q;

reg [63:0]  target_time_abs;   // <-- add this


// ===========================================================
// Trigger edge detection
// ===========================================================
reg trigger_prev;
reg trig_pulse;

// ===========================================================
// Time base: use incoming CHDR timestamps from RX stream
// ===========================================================
reg  [63:0] last_ts;   // last s_ttimestamp we saw
reg         have_ts;   // have we ever seen a valid timestamp?

always @(posedge clk) begin
  if (rst) begin
    last_ts <= 64'd0;
    have_ts <= 1'b0;
  end else if (s_tvalid && s_thas_time) begin
    // Every packet with a timestamp refreshes our notion of "now"
    last_ts <= s_ttimestamp;
    have_ts <= 1'b1;
  end
end

// This is our "current time" in the same domain as the radio timekeeper
wire [63:0] hw_time_now = last_ts;
// --------------------------------------------------------------------
// Legacy TB-compat signals: now / base_offset / synced
// The original rfnoc_block_siggen_tb.sv still probes these.
// They are *not* used by the functional logic anymore.
// --------------------------------------------------------------------
reg [63:0] now;
reg [63:0] base_offset;
reg        synced;

always @(posedge clk) begin
  if (rst) begin
    now         <= 64'd0;
    base_offset <= 64'd0;
    synced      <= 1'b0;
  end else begin
    // Mirror our "current" timebase for debug
    now         <= hw_time_now;
    base_offset <= 64'd0;    // we don't use an offset anymore
    synced      <= have_ts;  // treat "synced" as "we have seen a timestamp"
  end
end

// --------------------------------------------------------------------
// SIM/TB visibility helpers (do not affect functionality)
// --------------------------------------------------------------------
`ifdef SIM_DEBUG
  // Ensure the TB's hierarchical paths resolve and are not trimmed
  (* keep = "true", mark_debug = "true" *)
  wire trig_fire_q = trig_pulse;

  // hw_time_now already exists; if you want to belt-and-suspenders it:
  (* keep = "true", mark_debug = "true" *)
  wire [63:0] hw_time_now_keep = hw_time_now;
`else
  // In non-sim builds, still provide the alias so elaboration succeeds
  (* keep = "true" *)
  wire trig_fire_q = trig_pulse;
`endif



// -----------------------------
// Trigger / Delay / Burst control (Verilog-2001)
// Free-run if REG_THRESHOLD==0; otherwise use trigger FSM
// -----------------------------
// ===========================================================
// Internal signal declarations (trigger, counters, FSM)
// ===========================================================

// --- Trigger detection wires ---
wire signed [15:0] in_i = s_tdata[31:16];
wire signed [15:0] in_q = s_tdata[15:0];

// --- abs() helper ---
function [15:0] abs16;
  input signed [15:0] x;
  begin
    abs16 = x[15] ? (~x + 16'd1) : x;
  end
endfunction


reg [REG_HOLDCOUNT_LEN-1:0] above_cnt;
// add near the top with the other regs/wires
wire armed = reg_enable; // only arm when we have seen a timestamp

wire over_thr = s_tvalid && armed &&
                ((abs16(in_i) >= thr) || (abs16(in_q) >= thr));

always @(posedge clk) begin
  if (rst || !armed) begin
    above_cnt <= 0;
  end else if (s_tvalid) begin
    if (over_thr)  above_cnt <= (above_cnt == {REG_HOLDCOUNT_LEN{1'b1}}) ? above_cnt : (above_cnt + 1'b1);
    else           above_cnt <= 0;
  end
end


wire trig_level = use_trigger && (above_cnt >= reg_holdcount);

always @(posedge clk) begin
  if (rst) trigger_prev <= 1'b0;
  else     trigger_prev <= trig_level && armed; // remember the armed-gated level
end

always @(posedge clk) begin
  if (rst) trig_pulse <= 1'b0;
  else     trig_pulse <= armed && trig_level && !trigger_prev;
end


reg trig_pulse_q;
always @(posedge clk) begin
  if (rst) trig_pulse_q <= 1'b0;
  else     trig_pulse_q <= trig_pulse;
end

// Fire only from IDLE on the rising edge



// From rounding module outputs
wire [31:0] axis_round_tdata;
wire        axis_round_tvalid;
wire        axis_round_tready;

// ===========================================================
// Counter-based FSM: IDLE → DELAY → BURST → IDLE
// ===========================================================
localparam [1:0] ST_IDLE  = 2'd0,
                 ST_BURST = 2'd1;


reg  [1:0]  state_q, state_d;
reg  [15:0] pw_cnt_q;               // only sequentially updated
reg         pkt_rst_pulse_q, pkt_rst_pulse_d;
reg [63:0] trig_time_abs_q;


wire burst_active = (state_q == ST_BURST);

wire allow_output    = reg_enable & (use_trigger ? burst_active : 1'b1);
wire ready_to_output = use_trigger ? burst_active : 1'b1;

// One "beat" leaves the datapath only if upstream has valid, downstream ready, and gate open
wire beat_to_pkt = axis_round_tvalid && axis_round_tready && allow_output;

// Sync trigger only in IDLE

// Add near PIPE_LAT:
localparam [31:0] TS_MARGIN = 32'd200;  // safety slack; tune per clk rate
localparam [63:0]  TS_EARLY  = 64'd0 + PIPE_LAT + TS_MARGIN;
always @* begin
  state_d         = state_q;
  pkt_rst_pulse_d = 1'b0;

  // If not enabled, always go back to IDLE
  if (!reg_enable) begin
    state_d = ST_IDLE;
  end else begin
    case (state_q)
      ST_IDLE: begin
        if (use_trigger) begin
          // Triggered mode: wait for rising trig_pulse
          if (trig_pulse) begin
            pkt_rst_pulse_d = 1'b1;          // reset packetizer for a fresh burst
            state_d         = ST_BURST;
          end
        end else begin
          // Free-run mode: just start bursting immediately
          pkt_rst_pulse_d = 1'b1;            // reset packetizer
          state_d         = ST_BURST;
        end
      end

      ST_BURST: begin
        // Finish the current burst when packet completes
        if (m_tvalid && m_tready && m_tlast) begin
          // Trigger mode: one-shot bursts → go back to IDLE
          // Free-run: keep bursting forever
          state_d = use_trigger ? ST_IDLE : ST_BURST;
        end
      end

      default: begin
        state_d = ST_IDLE;
      end
    endcase
  end
end


// One cycle when leaving IDLE due to a trigger
wire trig_go = (state_q == ST_IDLE) && trig_pulse && !trig_pulse_q;
reg  [63:0] ts_base_q;   // absolute time for *first* valid sample in datapath
reg [63:0] ts_target_q = 64'd0;
reg        ts_target_valid_q;



// ---------------- SEQUENTIAL: state, counters, warm-up, timestamps ----------
always @(posedge clk) begin
  if (rst) begin
    state_q         <= ST_IDLE;
    pw_cnt_q        <= 16'd0;
    pkt_rst_pulse_q <= 1'b0;
    ts_target_q        <= 64'd0;
    ts_target_valid_q  <= 1'b0;
    warm_cnt_q      <= 8'd0;
    ts_arm_q        <= 1'b0;
    m_thas_time     <= 1'b0;
    m_ttimestamp    <= 64'd0;
    ts_base_q       <= 64'd0;

    target_time_abs <= 64'd0;
    trig_time_abs_q <= 64'd0;
  end else begin
    state_q         <= state_d;
    pkt_rst_pulse_q <= pkt_rst_pulse_d;


if (trig_go) begin
  trig_time_abs_q <= hw_time_now;

  // exact target timestamp for SOP:
  ts_target_q <= hw_time_now + {32'd0, reg_delay} + {56'd0, PIPE_LAT[7:0]};
  ts_target_valid_q <= 1'b1;


  // start the datapath EARLY so data reaches the shell before ts_target_q
  // guard underflow:
  if ( (hw_time_now + {32'd0, reg_delay}) > TS_EARLY )
    ts_base_q <= (hw_time_now + {32'd0, reg_delay}) - TS_EARLY;
  else
    ts_base_q <= hw_time_now;  // nothing to gain; start now
end


    // (Optional) keep this if you want a debug copy of the intended target time
    if ((state_q == ST_IDLE) && (state_d != ST_IDLE) && trig_go) begin
      target_time_abs <= trig_time_abs_q + {32'd0, reg_delay};
    end

// Warm-up, pw counter, etc. (unchanged) ...
if ((state_q != ST_BURST) && (state_d == ST_BURST)) begin
  $display("%0t [BURST ENTRY] reg_threshold=0x%08h use_trigger=%0d ts_target_valid=%0d",
           $time, reg_threshold, use_trigger, ts_target_valid_q);

  // FIX B: arm if a target is already valid OR we're entering BURST due to a trigger this cycle
  // (trig_go is the one-cycle pulse: (state_q==ST_IDLE) && trig_pulse && !trig_pulse_q)
  ts_arm_q   <= (use_trigger && (ts_target_valid_q || trig_go)) ? 1'b1 : 1'b0;

  warm_cnt_q <= PIPE_LAT[7:0];                        // prime warm-up
  pw_cnt_q   <= (reg_pulsewidth == 16'd0) ? 16'd1 : reg_pulsewidth;  // load PW once
end else begin
  if (burst_active && (warm_cnt_q != 8'd0))
    warm_cnt_q <= warm_cnt_q - 8'd1;
  if (burst_active && beat_to_pkt && (pw_cnt_q != 16'd0))
    pw_cnt_q <= pw_cnt_q - 16'd1;
end

// DEBUG: disable timed scheduling, transmit immediately, no timestamp
m_thas_time  <= 1'b0;
m_ttimestamp <= 64'd0;   // not used when m_thas_time=0
// Don't touch ts_arm_q / ts_target_valid_q for now
if (ts_arm_q && ts_target_valid_q && m_tvalid && m_tready && use_trigger) begin
    // just consume the arm/target so they don't accumulate
    ts_arm_q          <= 1'b0;
    ts_target_valid_q <= 1'b0;
end



  end
end

// When TLAST fires, show the length you're actually driving
always @(posedge clk)
  if (m_tvalid && m_tready && m_tlast)
    $display("%0t [DBG LEN] reg_spp=%0d  CHDR_W=%0d  m_tlength(words)=%0d",
             $time, reg_spp, CHDR_W, m_tlength);

// m_thas_time must only coincide with the first beat of a packet
`ifndef SYNTHESIS
always @(posedge clk)
  if (m_thas_time) begin
    if (!(m_tvalid && m_tready))
      $error("%0t m_thas_time asserted without handshake", $time);
  end
`endif


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
wire [31:0] axis_gain_tdata;
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
  //---------------------------------------------------------------

// ---------------- CtrlPort R/W ----------------
always @(posedge clk) begin
  // defaults
  s_ctrlport_resp_ack  <= 1'b0;
  s_ctrlport_resp_data <= 32'd0;
  reg_phase_inc_stb    <= 1'b0;
  reg_cartesian_stb    <= 1'b0;

  if (rst) begin
    // no special action
  end else begin
    // WRITES
    if (s_ctrlport_req_wr) begin
      s_ctrlport_resp_ack <= 1'b1;
      case (s_ctrlport_req_addr)
        REG_DBG_CTRL   : /* bit0 clear handled via dbg_clear */;
        REG_ENABLE     : reg_enable    <= s_ctrlport_req_data[REG_ENABLE_LEN-1:0];
        REG_SPP        : reg_spp       <= s_ctrlport_req_data[REG_SPP_LEN-1:0];
        REG_WAVEFORM   : reg_waveform  <= s_ctrlport_req_data[REG_WAVEFORM_LEN-1:0];
        REG_GAIN       : reg_gain      <= s_ctrlport_req_data[REG_GAIN_LEN-1:0];
        REG_CONSTANT   : reg_constant  <= s_ctrlport_req_data[REG_CONSTANT_LEN-1:0];
        REG_THRESHOLD  : reg_threshold <= s_ctrlport_req_data[REG_THRESHOLD_LEN-1:0];
        REG_PULSEWIDTH : reg_pulsewidth<= s_ctrlport_req_data[REG_PULSEWIDTH_LEN-1:0];
        REG_DELAY      : reg_delay     <= s_ctrlport_req_data[REG_DELAY_LEN-1:0];
        REG_HOLDCOUNT  : reg_holdcount <= s_ctrlport_req_data[REG_HOLDCOUNT_LEN-1:0];
        REG_PHASE_INC  : begin
          reg_phase_inc     <= s_ctrlport_req_data[REG_PHASE_INC_LEN-1:0];
          reg_phase_inc_stb <= 1'b1;
        end
        REG_CARTESIAN  : begin
          reg_cartesian     <= s_ctrlport_req_data[REG_CARTESIAN_LEN-1:0];
          reg_cartesian_stb <= 1'b1;
        end
        default: ; // do nothing
      endcase
    end

// READS
if (s_ctrlport_req_rd) begin
  s_ctrlport_resp_ack <= 1'b1;
  case (s_ctrlport_req_addr)
    REG_ENABLE     : s_ctrlport_resp_data[REG_ENABLE_LEN-1:0]     <= reg_enable;
    REG_SPP        : s_ctrlport_resp_data[REG_SPP_LEN-1:0]        <= reg_spp;
    REG_WAVEFORM   : s_ctrlport_resp_data[REG_WAVEFORM_LEN-1:0]   <= reg_waveform;
    REG_GAIN       : s_ctrlport_resp_data[REG_GAIN_LEN-1:0]       <= reg_gain;
    REG_CONSTANT   : s_ctrlport_resp_data[REG_CONSTANT_LEN-1:0]   <= reg_constant;
    REG_PHASE_INC  : s_ctrlport_resp_data[REG_PHASE_INC_LEN-1:0]  <= reg_phase_inc;
    REG_CARTESIAN  : s_ctrlport_resp_data[REG_CARTESIAN_LEN-1:0]  <= reg_cartesian;
    REG_THRESHOLD  : s_ctrlport_resp_data[REG_THRESHOLD_LEN-1:0]  <= reg_threshold;
    REG_PULSEWIDTH : s_ctrlport_resp_data[REG_PULSEWIDTH_LEN-1:0] <= reg_pulsewidth;
    REG_DELAY      : s_ctrlport_resp_data[REG_DELAY_LEN-1:0]      <= reg_delay;
    REG_HOLDCOUNT  : s_ctrlport_resp_data[REG_HOLDCOUNT_LEN-1:0]  <= reg_holdcount;

   REG_DBG_FLAGS: begin
      s_ctrlport_resp_data <= {31'd0, dbg_ts_seen};
    end

    REG_DBG_CTRL: begin
      // usually write-only; for reads you can return 0 or last written
      s_ctrlport_resp_data <= 32'd0;
    end

    REG_DBG_STATUS: begin
      s_ctrlport_resp_data <= {
        24'd0,
        allow_output,         // [7]
        ~allow_output,        // [6]
        ready_to_output,      // [5]
        use_trigger,          // [4]
        dbg_output_seen,      // [3]
        burst_active,         // [2]
        dbg_burst_start_seen, // [1]
        dbg_trigger_seen      // [0]
      };
    end

    REG_DBG_TS_NOW_LO: begin
      s_ctrlport_resp_data <= dbg_ts_now_trig[31:0];
    end

    REG_DBG_TS_NOW_HI: begin
      s_ctrlport_resp_data <= dbg_ts_now_trig[63:32];
    end

    default: s_ctrlport_resp_data <= 32'd0;
  endcase
end

  end
end



// --- Debug helpers and BURST start pulse (now after FSM & allow_output) ---
wire [15:0] abs_i_dbg = abs16(in_i);
wire [15:0] abs_q_dbg = abs16(in_q);
wire [15:0] amp_dbg   = (abs_i_dbg >= abs_q_dbg) ? abs_i_dbg : abs_q_dbg;

// One-cycle pulse when entering ST_BURST
wire burst_start_pulse = (state_q != ST_BURST) && (state_d == ST_BURST);
// TEMP STUBS so debug compiles; replace with real signals later
wire        pkt_has_time  = 1'b0;
wire [63:0] pkt_timestamp = 64'd0;
wire [63:0] ts_target     = hw_time_now;  // or ts_base_q if you have it
// --------------------- DEBUG: clear & capture ---------------------------
always @(posedge clk) begin
  if (rst || dbg_clear) begin
    // Existing debug
    dbg_trigger_seen     <= 1'b0;
    dbg_burst_start_seen <= 1'b0;
    dbg_output_seen      <= 1'b0;
    dbg_trigger_cnt      <= 32'd0;
    dbg_burst_cnt        <= 32'd0;
    dbg_output_cnt       <= 32'd0;
    dbg_last_amp         <= 32'd0;
    dbg_last_thr         <= 32'd0;
    dbg_last_warm_ctr    <= 16'd0;
    dbg_last_gate        <= 1'b0;

    // Timestamp debug
    dbg_ts_pkt       <= 64'd0;   // optional: keep if you still want it
    dbg_ts_now_trig  <= 64'd0;
    dbg_ts_target    <= 64'd0;   // optional: if you track target ts
    dbg_ts_seen      <= 1'b0;

  end else begin
    // ---------------- trigger event (over threshold) ----------------
    if (over_thr) begin
      // old stuff
      dbg_trigger_seen  <= 1'b1;
      dbg_trigger_cnt   <= dbg_trigger_cnt + 1'b1;
      dbg_last_amp      <= {16'd0, amp_dbg};   // zero-extend to 32b
      dbg_last_thr      <= {16'd0, thr};
      dbg_last_warm_ctr <= {8'd0, warm_cnt_q}; // snapshot warm-up counter
      dbg_last_gate     <= ~allow_output;      // 1 = gate was closed

      // NEW: latch timestamp on trigger
      if (!dbg_ts_seen) begin        // only capture first event until cleared
        dbg_ts_now_trig <= hw_time_now;
        dbg_ts_target   <= ts_target;   // if you have a ts_target signal
        dbg_ts_seen     <= 1'b1;        // this is what get_ts_seen() reads
      end
    end

    // --------------- FSM actually entered BURST ---------------------
    if (burst_start_pulse) begin
      dbg_burst_start_seen <= 1'b1;
      dbg_burst_cnt        <= dbg_burst_cnt + 1'b1;
    end

    // ------------------- Any output beat ----------------------------
    if (m_tvalid && m_tready) begin
      dbg_output_seen <= 1'b1;
      dbg_output_cnt  <= dbg_output_cnt + 1'b1;
    end

    // (OPTIONAL) packet-level timestamp capture, but do NOT tie it to dbg_ts_seen
    if (pkt_has_time) begin
      dbg_ts_pkt <= pkt_timestamp;   // just a snapshot, doesn't affect ts_seen
    end
  end
end

// -----------------------------------------------------------------------


assign m_tlength = ((reg_spp * 4) + (CHDR_W/8 - 1)) / (CHDR_W/8);

// At TLAST, prove what you're driving:
always @(posedge clk)
  if (m_tvalid && m_tready && m_tlast)
    $display("%0t [CORE LEN] reg_spp=%0d CHDR_W=%0d m_tlength(words)=%0d",
             $time, reg_spp, CHDR_W, m_tlength);

// Count items only for stamped packets
reg        cnt_active;
reg [15:0] beat_ctr;

wire sop_stamped = m_thas_time && m_tvalid && m_tready; // SOP of a stamped pkt
wire beat        = m_tvalid && m_tready;

always @(posedge clk) begin
  if (rst || pkt_rst_pulse_q) begin
    cnt_active <= 1'b0;
    beat_ctr   <= 16'd0;
  end else begin
    if (sop_stamped) begin
      cnt_active <= 1'b1;
      beat_ctr   <= 16'd1;            // count this first beat
    end else if (cnt_active && beat) begin
      beat_ctr   <= beat_ctr + 16'd1;
    end
    // Completed stamped packet
    if (cnt_active && beat && m_tlast) begin
`ifdef SIM_STRICT
      if (beat_ctr !== reg_spp)
        $error("%0t Packet had %0d items, expected %0d", $time, beat_ctr, reg_spp);
`else
      if (beat_ctr !== reg_spp)
        $warning("%0t Packet had %0d items, expected %0d", $time, beat_ctr, reg_spp);
`endif
      cnt_active <= 1'b0;
      beat_ctr   <= 16'd0;
    end
    // If we ever exit BURST without TLAST (flush), also stop counting
    if ((state_q == ST_BURST) && (state_d != ST_BURST) && !m_tlast) begin
      cnt_active <= 1'b0;
      beat_ctr   <= 16'd0;
    end
  end
end




axis_packetize #(
  .DATA_W (32), .SIZE_W (REG_SPP_LEN), .FLUSH(1)
) axis_packetize_i (
  .clk      (clk),
  .rst      (rst | pkt_rst_pulse_q),   // <-- added per-trigger reset/flush
  .gate     (~allow_output),
  .size     (reg_spp),
  .i_tdata  (axis_round_tdata),
  .i_tvalid (axis_round_tvalid),
  .i_tready (axis_round_tready),
  .o_tdata  (m_tdata),
  .o_tlast  (m_tlast),
  .o_tvalid (m_tvalid),
  .o_tready (m_tready)
  // .o_tuser removed
);




endmodule
