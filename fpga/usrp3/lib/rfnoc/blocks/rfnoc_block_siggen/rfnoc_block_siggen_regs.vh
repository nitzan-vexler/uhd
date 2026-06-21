//
// Copyright 2020 Ettus Research, a National Instruments Brand
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//
// Module:  rfnoc_block_siggen_regs (Header)
//
// Description: RFNoC Signal Generator block register descriptions
//


// Address space size, per signal generator core. That is, each signal
// generator core's address space is separated in the CtrlPort address space
// by 2^SIGGEN_ADDR_W bytes.
localparam SIGGEN_ADDR_W = 6;



// REG_ENABLE (R/W)
//
// Starts or stops the waveform output. Write a 1 to enable waveform output, 0
// to disable waveform output. Starting and stopping occurs on packet
// boundaries.
//
// [31:1] Reserved
// [0]    Enable bit
//
localparam [19:0] REG_ENABLE     = 20'h00000;
//
localparam REG_ENABLE_LEN = 1;


// REG_SPP (R/W)
//
// The number of samples per packet to output for the selected waveform. This
// is read at the start of each new packet.
//
localparam [19:0] REG_SPP        = 20'h00004;
//
localparam REG_SPP_LEN = 14;


// REG_WAVEFORM (R/W)
//
// Selects the type of waveform to output. The possible values are:
//
//   0 : (WAVE_CONST) Constant data
//   1 : (WAVE_SINE)  Sine wave
//   2 : (WAVE_NOISE) Noise / random data
//
localparam [19:0] REG_WAVEFORM = 20'h00008;
//
localparam REG_WAVEFORM_LEN = 2;
//
localparam WAVE_CONST = 2'h0;
localparam WAVE_SINE  = 2'h1;
localparam WAVE_NOISE = 2'h2;


// REG_GAIN (R/W)
//
// Sets the gain for the output. This is a 16-bit signed fixed point value
// with 15 fractional bits. The gain is applied to both the real and imaginary
// parts of each output sample. This gain is applied to all waveform output
// types.
//
localparam [19:0] REG_GAIN       = 20'h0000C;
//
localparam REG_GAIN_LEN = 16;


// REG_CONSTANT (R/W)
//
// Sets the value for the sample to output for the constant waveform. Both the
// real and imaginary components are treated as 16-bit signed fixed point
// values with 15 fractional bits.
//
// [31:16] X/I/Real component
// [15: 0] Y/Q/Imaginary component

localparam [19:0] REG_CONSTANT   = 20'h00010;
//
localparam REG_CONSTANT_LEN = 32;


// REG_PHASE_INC (R/W)
//
// Sets the phase increment, in "scaled radians", for the sine waveform
// generator. This is the amount by which REG_CARTESIAN is rotated
// counter-clockwise each clock cycle. In other words, it controls the rate of
// rotation, or the frequency, of the sine wave. The range of the phase value
// is -1.0 to +1.0. In scaled radians, the value range -1 to +1 corresponds to
// -Pi to Pi in radians.
//
// In other words, the normalized frequency (in cycles/sample) of the
// sinusoidal output is equal to 0.5*REG_PHASE_INC.
//
// [31:16] : Reserved
// [15: 0] : Signed fixed-point phase value with 3 integer bits and 13
//           fractional bits.
//
localparam [19:0] REG_PHASE_INC  = 20'h00014;
//
localparam REG_PHASE_INC_LEN = 16;


// REG_CARTESIAN (R/W)
//
// Sets the (X,Y) Cartesian coordinate that will be rotated to generate the
// sine output. The rate of rotation is controlled by REG_PHASE_INC. Note that
// this input vector is also scaled by a "CORDIC scale factor" that equals
// about 1.16444 (the product of sqrt(1 + 2^(-2i)) for i = 1 to n, where n =
// 14, the number of fractional bits used by the CORDIC IP).
//
// Both the X and Y coordinates are signed fixed-point values with 15
// fractional bits.
//
// For example, supposed you wanted a sinusoidal output with an amplitude of
// about 0.9. In that case, you could set the Y coordinate to 0 and the X
// coordinate to 0.9/1.16444 = 0.7729. In fixed-point, that's 0.7729 * 2^15 =
// 0x62EE.
//
// NOTE: The Xilinx CORDIC IP describes the input and output as 16-bit signed
// fixed point with 2 integer and 14 fractional bits, which is accurate.
// However, since we treat the output as sc16 (15 fractional bits), we need to
// double the value of the CARTESIAN inputs to get the output we want for sc16.
// This is mathematically equivalent to simply saying the CARTESIAN inputs have
// 15 fractional bits instead of 14.
//
// [31:16] : X/I/Real component
// [15: 0] : Y/Q/Imaginary component
//
localparam [19:0] REG_CARTESIAN  = 20'h00018;
//
localparam REG_CARTESIAN_LEN = 32;


// New mode (optional)
localparam [2:0] WAVE_SINE_TRIG      = 3'd4;

// 0x1C: threshold for trigger
localparam [19:0] REG_THRESHOLD  = 20'h0001C;
localparam        REG_THRESHOLD_LEN  = 16;

// 0x20: pulse width (samples in the burst)
localparam [19:0] REG_PULSEWIDTH = 20'h00020;
localparam        REG_PULSEWIDTH_LEN = 16;

// 0x24: delay before transmitting the burst (in ce_clk cycles)
localparam [19:0] REG_DELAY      = 20'h00024;  // <-- yes, 0x24 is correct
localparam        REG_DELAY_LEN      = 32;

localparam [19:0] REG_AVG_START_DELAY = 20'h00028;
localparam        REG_AVG_START_DELAY_LEN = 8;

localparam [19:0] REG_DBG_AVG_POWER = 20'h0002C;
localparam        REG_DBG_AVG_POWER_LEN = 32;

localparam [19:0] REG_DBG_TX_AMP = 20'h00030;
localparam        REG_DBG_TX_AMP_LEN = 16;