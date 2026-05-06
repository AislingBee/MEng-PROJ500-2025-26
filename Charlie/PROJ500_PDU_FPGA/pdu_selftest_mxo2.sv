`timescale 1ns/1ps
`default_nettype none
// =============================================================================
// pdu_selftest_mxo2.sv  —  Board-level continuity / pin-ring self-test image
//                          for the PROJ500 PDU FPGA (MachXO2-256HC-5SG48C)
//
// PURPOSE
//   Provide a stand-alone FPGA image that drives every output pin in a
//   deterministic, repeating pattern.  Used during PCB bring-up to verify:
//     • FPGA configures successfully from FLASH.
//     • Every output net reaches its connector / test-point.
//     • No assembly short or open on any output.
//
// DESIGN
//   The module reuses the exact pin names and LPF from pdu_glue_mxo2.sv so
//   the same implementation constraints apply and the pinout is guaranteed
//   identical.  All input pins are consumed by a harmless XOR chain to
//   prevent the synthesiser from optimising them away (any input-rail short
//   would then cause visible glitches on SPARE_OUT_1).
//
//   Two compile modes:
//     ALL_HIGH=1 (default)  — All outputs permanently driven high.
//                             Fastest way to confirm the power rail and pull-
//                             ups see the correct levels before moving to
//                             pattern testing.
//     ALL_HIGH=0            — Autonomous step-sequencer walks through 24
//                             patterns (one-hot, all-on, checkerboard, mixed)
//                             at STEP_TIME_MS per step.  At 2 MHz the visible
//                             step rate in hardware is 2 seconds per step.
//
// RECOMMENDED USE
//   1. Add pdu_selftest_mxo2.sv to the Diamond project.
//   2. Create a new implementation named SELFTEST.
//   3. Set pdu_selftest_mxo2 as the top module for that implementation.
//   4. Re-use PROJ500_PDU_FPGA.lpf unchanged (same pinout).
//   5. Build and program; use a scope or logic analyser to observe outputs.
//
// SIMULATION
//   Compile with +define+SIM to substitute sim_clk for the OSCH primitive.
//   tb_pdu_selftest_mxo2.sv drives sim_clk at 10 MHz and runs for 5 ms.
//
// =============================================================================
// REVISION HISTORY
//   2026-05-06  Initial release.
// =============================================================================

`define ALL_HIGH 1

module pdu_selftest_mxo2 (
    // Inputs (same names as real design)
	`ifdef SIM
		input  wire sim_clk,
	`endif
    input  wire SW_COMPUTE,
    input  wire SW_ARM,
    input  wire SW_RST_FAULT,
    input  wire SW_REMOTE_ARM,
    input  wire ESTOP_OK,
    input  wire OV,
    input  wire UV,
    input  wire VBUS_OV,
    input  wire MCU_ALIVE,
    input  wire MCU_CMD_ARM,
    input  wire MCU_CMD_FAULT,
    input  wire PRECHARGE_OK,
    input  wire FB_CLOSED,
	input  wire SW_REMOTE_DISARM,

    // I2C pins left unused / tri-stated
    inout  wire i2c1_scl,
    inout  wire i2c1_sda,

    // Outputs
    output logic FAULT_LATCH,
    output logic PRECHARGE_LATCH,
    output logic MOTOR_EN,
    output logic COMPUTE_EN,
    output logic K_SEL,
    output logic K_EN,

    output logic [3:0] FAULT_CODE,
    output logic [1:0] STATE_CODE,
    output logic       OVUV_OK,
    output logic       ARM_PERMIT,
    output logic       PRECHARGE_REQ,
    output logic       DUMP_EN,
    output logic       SPARE_OUT_0,
    output logic       SPARE_OUT_1
);

    // ── Input sink ────────────────────────────────────────────────────────
    // XOR all inputs into a single wire.  This forces the synthesiser to
    // preserve every input net; any input-rail short during board test causes
    // a visible glitch on SPARE_OUT_1 which drives this wire.
    wire _unused_inputs = SW_COMPUTE ^ SW_ARM ^ SW_RST_FAULT ^ SW_REMOTE_ARM ^
                          ESTOP_OK ^ OV ^ UV ^ VBUS_OV ^ MCU_ALIVE ^ MCU_CMD_ARM ^
                          MCU_CMD_FAULT ^ PRECHARGE_OK ^ FB_CLOSED ^ SW_REMOTE_DISARM;

    // ── I2C pins ──────────────────────────────────────────────────────────
    // Left tri-stated.  The EFB I2C hard IP is not instantiated in this image
    // so the pads default to high-impedance inputs.  External pull-ups on the
    // PCB will hold the bus idle.
    assign i2c1_scl = 1'bz;
    assign i2c1_sda = 1'bz;

    // ── Internal oscillator ───────────────────────────────────────────────
    // 2.08 MHz OSCH (MachXO2 on-chip ring oscillator).  In simulation the
    // OSCH primitive is not available; sim_clk drives clk_osc directly.
    `ifdef SIM
		wire clk_osc = sim_clk;
	`else
		wire clk_osc;
		localparam string OSCH_NOM_FREQ_MHZ = "2.08";
		OSCH #(.NOM_FREQ(OSCH_NOM_FREQ_MHZ)) u_osch (
			.STDBY   (1'b0),
			.OSC     (clk_osc),
			.SEDSTDBY()
		);
	`endif

    // ── Utility: integer ceiling log2 ─────────────────────────────────────
    // Used to size the step_div counter to exactly the number of bits needed.
    // Prevents the synthesiser inserting unnecessary flip-flops.
    function automatic integer clog2(input integer value);
        integer i;
        begin
            value = value - 1;
            for (i = 0; value > 0; i = i + 1)
                value = value >> 1;
            clog2 = i;
        end
    endfunction

    // ── Step sequencer timing ─────────────────────────────────────────────
    //
    // In hardware: STEP_TIME_MS=2000ms → each output pattern holds for 2 s,
    //              allowing easy observation with a voltmeter or scope.
    // In simulation: STEP_TIME_MS=1ms so the full 24-step cycle completes
    //              in 24 ms.
    localparam integer OSC_HZ          = 2080000;
    `ifdef SIM
		localparam integer STEP_TIME_MS = 1;
	`else
		localparam integer STEP_TIME_MS = 2000;
	`endif
    localparam integer STEP_COUNT      = 24;    // number of output patterns
    localparam integer TICKS_PER_STEP  = (OSC_HZ / 1000) * STEP_TIME_MS;

    logic [clog2(TICKS_PER_STEP)-1:0] step_div = '0;
    logic [clog2(STEP_COUNT)-1:0]     step_idx = '0;

    always_ff @(posedge clk_osc) begin
        if (step_div == TICKS_PER_STEP-1) begin
            step_div <= '0;

            if (step_idx == STEP_COUNT-1)
                step_idx <= '0;
            else
                step_idx <= step_idx + 1'b1;
        end else begin
            step_div <= step_div + 1'b1;
        end
    end

    // ── Output pattern generator ──────────────────────────────────────────
    //
    // With ALL_HIGH=0 (pattern mode), each step_idx value drives a different
    // combination of outputs.  The sequence is:
    //   Step  0:    All outputs 0 (continuity check: measure quiescent levels)
    //   Steps 1-12: One-hot walk (exactly one output high at a time)
    //   Steps 13-15: STATE_CODE walk (00 → 01 → 10 → 11)
    //   Steps 16-19: FAULT_CODE walk (one hot 4-bit)
    //   Step  20:   All outputs 1 (shorts check: all high simultaneously)
    //   Step  21:   Checkerboard A (alternating 1/0)
    //   Step  22:   Checkerboard B (inverse of A)
    //   Step  23:   Mixed alive pattern (SPARE_OUT_1 = ~inputs XOR chain)
`ifndef ALL_HIGH
    always_comb begin
        // Safe defaults
        FAULT_LATCH     = 1'b0;
        PRECHARGE_LATCH = 1'b0;
        MOTOR_EN        = 1'b0;
        COMPUTE_EN      = 1'b0;
        K_SEL           = 1'b0;
        K_EN            = 1'b0;

        FAULT_CODE      = 4'b0000;
        STATE_CODE      = 2'b00;
        OVUV_OK         = 1'b0;
        ARM_PERMIT      = 1'b0;
        PRECHARGE_REQ   = 1'b0;
        DUMP_EN         = 1'b0;
        SPARE_OUT_0     = 1'b0;
        SPARE_OUT_1     = 1'b0;

        unique case (step_idx)
            // 0: all off
            5'd0: begin
            end

            // One-hot walk through single-bit outputs
            5'd1:  FAULT_LATCH     = 1'b1;
            5'd2:  PRECHARGE_LATCH = 1'b1;
            5'd3:  MOTOR_EN        = 1'b1;
            5'd4:  COMPUTE_EN      = 1'b1;
            5'd5:  K_SEL           = 1'b1;
            5'd6:  K_EN            = 1'b1;
            5'd7:  DUMP_EN         = 1'b1;
            5'd8:  PRECHARGE_REQ   = 1'b1;
            5'd9:  ARM_PERMIT      = 1'b1;
            5'd10: OVUV_OK         = 1'b1;
            5'd11: SPARE_OUT_0     = 1'b1;
            5'd12: SPARE_OUT_1     = 1'b1;

            // STATE_CODE walk
            5'd13: STATE_CODE      = 2'b01;
            5'd14: STATE_CODE      = 2'b10;
            5'd15: STATE_CODE      = 2'b11;

            // FAULT_CODE walk
            5'd16: FAULT_CODE      = 4'b0001;
            5'd17: FAULT_CODE      = 4'b0010;
            5'd18: FAULT_CODE      = 4'b0100;
            5'd19: FAULT_CODE      = 4'b1000;

            // Everything on
            5'd20: begin
                FAULT_LATCH     = 1'b1;
                PRECHARGE_LATCH = 1'b1;
                MOTOR_EN        = 1'b1;
                COMPUTE_EN      = 1'b1;
                K_SEL           = 1'b1;
                K_EN            = 1'b1;
                FAULT_CODE      = 4'b1111;
                STATE_CODE      = 2'b11;
                OVUV_OK         = 1'b1;
                ARM_PERMIT      = 1'b1;
                PRECHARGE_REQ   = 1'b1;
                DUMP_EN         = 1'b1;
                SPARE_OUT_0     = 1'b1;
                SPARE_OUT_1     = 1'b1;
            end

            // Checkerboard pattern A
            5'd21: begin
                FAULT_LATCH     = 1'b1;
                PRECHARGE_LATCH = 1'b0;
                MOTOR_EN        = 1'b1;
                COMPUTE_EN      = 1'b0;
                K_SEL           = 1'b1;
                K_EN            = 1'b0;
                FAULT_CODE      = 4'b1010;
                STATE_CODE      = 2'b01;
                OVUV_OK         = 1'b1;
                ARM_PERMIT      = 1'b0;
                PRECHARGE_REQ   = 1'b1;
                DUMP_EN         = 1'b0;
                SPARE_OUT_0     = 1'b1;
                SPARE_OUT_1     = 1'b0;
            end

            // Checkerboard pattern B
            5'd22: begin
                FAULT_LATCH     = 1'b0;
                PRECHARGE_LATCH = 1'b1;
                MOTOR_EN        = 1'b0;
                COMPUTE_EN      = 1'b1;
                K_SEL           = 1'b0;
                K_EN            = 1'b1;
                FAULT_CODE      = 4'b0101;
                STATE_CODE      = 2'b10;
                OVUV_OK         = 1'b0;
                ARM_PERMIT      = 1'b1;
                PRECHARGE_REQ   = 1'b0;
                DUMP_EN         = 1'b1;
                SPARE_OUT_0     = 1'b0;
                SPARE_OUT_1     = 1'b1;
            end

            // Mixed "alive" pattern
            5'd23: begin
                FAULT_LATCH     = 1'b1;
                PRECHARGE_LATCH = 1'b1;
                MOTOR_EN        = 1'b0;
                COMPUTE_EN      = 1'b0;
                K_SEL           = 1'b1;
                K_EN            = 1'b1;
                FAULT_CODE      = 4'b0011;
                STATE_CODE      = 2'b11;
                OVUV_OK         = 1'b1;
                ARM_PERMIT      = 1'b1;
                PRECHARGE_REQ   = 1'b0;
                DUMP_EN         = 1'b0;
                SPARE_OUT_0     = _unused_inputs; // just to prove this net is alive
                SPARE_OUT_1     = ~_unused_inputs;
            end

            default: begin
            end
        endcase
    end
	
`else
    // ── ALL_HIGH=1 mode ───────────────────────────────────────────────────
    // All outputs permanently driven high.  SPARE_OUT_1 is driven by the
    // input XOR chain so any input-rail fault appears on the output.
    always_comb begin
		FAULT_LATCH     = 1'b1;
		PRECHARGE_LATCH = 1'b1;
		MOTOR_EN        = 1'b1;
		COMPUTE_EN      = 1'b1;
		K_SEL           = 1'b1;
		K_EN            = 1'b1;
		FAULT_CODE      = 4'b1111;
		STATE_CODE      = 2'b11;
		OVUV_OK         = 1'b1;
		ARM_PERMIT      = 1'b1;
		PRECHARGE_REQ   = 1'b1;
		DUMP_EN         = 1'b1;
		SPARE_OUT_0     = 1'b1;
		SPARE_OUT_1     = _unused_inputs; // input-rail probe
	end
`endif

endmodule

`default_nettype wire