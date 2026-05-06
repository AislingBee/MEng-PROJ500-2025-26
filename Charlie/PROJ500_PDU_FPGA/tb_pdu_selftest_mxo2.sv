`timescale 1ns/1ps
`define SIM
// =============================================================================
// tb_pdu_selftest_mxo2.sv  —  Smoke-test testbench for pdu_selftest_mxo2
//
// Drives sim_clk at 10 MHz (100 ns period).  All DUT inputs are tied low;
// the design uses them only as an XOR probe so their values do not affect
// the output sequence.  Runs for 5 ms of simulation time — long enough to
// observe several complete output-pattern steps (STEP_TIME_MS=1ms in SIM
// mode, 24 steps → 24 ms; 5 ms covers the first 5 patterns).
//
// Compile:
//   vlog -sv -mfcu +define+SIM pdu_selftest_mxo2.sv tb_pdu_selftest_mxo2.sv
// Run:
//   vsim -voptargs=+acc tb_pdu_selftest_mxo2 -do "run 5ms; quit"
// =============================================================================

module tb_pdu_selftest_mxo2;

    logic sim_clk = 0;

    // Inputs
    logic SW_COMPUTE       = 0;
    logic SW_ARM           = 0;
    logic SW_RST_FAULT     = 0;
    logic SW_REMOTE_ARM    = 0;
    logic ESTOP_OK         = 0;
    logic OV               = 0;
    logic UV               = 0;
    logic VBUS_OV          = 0;
    logic MCU_ALIVE        = 0;
    logic MCU_CMD_ARM      = 0;
    logic MCU_CMD_FAULT    = 0;
    logic PRECHARGE_OK     = 0;
    logic FB_CLOSED        = 0;
    logic SW_REMOTE_DISARM = 0;

    tri i2c1_scl;
    tri i2c1_sda;

    wire FAULT_LATCH;
    wire PRECHARGE_LATCH;
    wire MOTOR_EN;
    wire COMPUTE_EN;
    wire K_SEL;
    wire K_EN;
    wire [3:0] FAULT_CODE;
    wire [1:0] STATE_CODE;
    wire OVUV_OK;
    wire ARM_PERMIT;
    wire PRECHARGE_REQ;
    wire DUMP_EN;
    wire SPARE_OUT_0;  // 500 Hz heartbeat in real design; any value in self-test
    wire SPARE_OUT_1;  // driven by input XOR chain

    // ── Clock: 10 MHz ─────────────────────────────────────────────────────
    // Faster than the real 2.08 MHz so the 24-step cycle completes in
    // much less simulation wall time.
    always #50 sim_clk = ~sim_clk;

    pdu_selftest_mxo2 dut (
        .sim_clk(sim_clk),

        .SW_COMPUTE(SW_COMPUTE),
        .SW_ARM(SW_ARM),
        .SW_RST_FAULT(SW_RST_FAULT),
        .SW_REMOTE_ARM(SW_REMOTE_ARM),
        .ESTOP_OK(ESTOP_OK),
        .OV(OV),
        .UV(UV),
        .VBUS_OV(VBUS_OV),
        .MCU_ALIVE(MCU_ALIVE),
        .MCU_CMD_ARM(MCU_CMD_ARM),
        .MCU_CMD_FAULT(MCU_CMD_FAULT),
        .PRECHARGE_OK(PRECHARGE_OK),
        .FB_CLOSED(FB_CLOSED),
        .SW_REMOTE_DISARM(SW_REMOTE_DISARM),

        .i2c1_scl(i2c1_scl),
        .i2c1_sda(i2c1_sda),

        .FAULT_LATCH(FAULT_LATCH),
        .PRECHARGE_LATCH(PRECHARGE_LATCH),
        .MOTOR_EN(MOTOR_EN),
        .COMPUTE_EN(COMPUTE_EN),
        .K_SEL(K_SEL),
        .K_EN(K_EN),
        .FAULT_CODE(FAULT_CODE),
        .STATE_CODE(STATE_CODE),
        .OVUV_OK(OVUV_OK),
        .ARM_PERMIT(ARM_PERMIT),
        .PRECHARGE_REQ(PRECHARGE_REQ),
        .DUMP_EN(DUMP_EN),
        .SPARE_OUT_0(SPARE_OUT_0),
        .SPARE_OUT_1(SPARE_OUT_1)
    );

    initial begin
        // Run for 5 ms of simulation time.  In SIM mode STEP_TIME_MS=1ms so
        // this covers the first 5 output patterns of the 24-step sequence.
        // Extend this time (or increase STEP_COUNT iterations in a loop) to
        // validate the full pattern cycle.
        #5000000;
        $display("tb_pdu_selftest_mxo2: 5 ms elapsed, stopping.");
        $stop;
    end

endmodule