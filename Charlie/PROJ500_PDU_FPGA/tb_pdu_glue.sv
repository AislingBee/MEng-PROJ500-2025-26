`timescale 1ns/1ps
`define SIM

// =============================================================================
// tb_pdu_glue.sv  —  Functional testbench for pdu_glue (pdu_glue_mxo2.sv)
//
// Clock: 2.08 MHz simulated with a scaled 1ms tick for speed.
//        The DUT OSC_DIV_1MS = 2080; we run the real clock (480ns period)
//        so 1 ms of sim time costs 2080 cycles = ~1us wall time in ModelSim.
//        Total sim time ~130 ms covers all test cases.
//
// Tests (sequential, each with pass/fail $display):
//   T01  Power-on defaults — all outputs de-asserted
//   T02  COMPUTE_EN follows SW_COMPUTE immediately in all states
//   T03  arm_ok conditions — missing any one inhibits precharge entry
//   T04  Normal precharge → armed → MOTOR_EN path
//   T05  Precharge timeout → ST_PRECHARGE_ABORT hold active → exit to FAULT
//   T06  ENABLE_FB_CLOSED=0: hold ARMED >500ms without FB_CLOSED — no fault
//   T07  ENABLE_FB_CLOSED=0: FB_CLOSED with MOTOR_EN=0 >500ms — no weld fault
//   T08  ESTOP fault from ARMED → FAULT, code=1, K_EN drops
//   T09  MCU_CMD_FAULT rising edge → FAULT, code=6
//   T10  VBUS_OV orthogonal dump: vbusov_filt→K_SEL=1, K_EN asserts after blank
//   T11  ESTOP fault from COMPUTE → FAULT, code=1
//   T12  Soft de-arm (MCU_CMD_ARM drops while ARMED) → back to COMPUTE, no fault
//   T13  Fault reset succeeds while SW_ARM held
//   T14  Remote arm / disarm latch
//   T15  Dump timing: VBUS_OV→vbusov_filt→K_SEL=1, K_EN after blank, clears
//   T16  OV fault (code 2) and UV fault (code 3)
//   T17  VBUS_OV coexists with ARMED — dump active, contactor stays closed, no fault
//   T18  Soft abort→abort-hold: K_EN/PRECHARGE_REQ held; exit clean; re-arm
//   T19  Fault reset blocked while ESTOP still held; succeeds after ESTOP released
//   T20  SW_ARM toggling in FAULT state never clears fault (only RST does)
//   T21  Fault-abort → pchgok in hold → exit to ST_FAULT
//   T22  Soft-abort → pchgok in hold → exit to ST_COMPUTE, no fault
//   T23  Soft-abort → hard timeout (no pchgok) → exit to ST_COMPUTE, no fault
//   T24  VBUS_OV during abort-hold: precharge_active gates dump (K_SEL stays 0)
//   T25  Simultaneous fault+pchgok in PRECHARGE — fault wins → ST_FAULT
//   T26  OV fault during PRECHARGE (A) and during ARMED (B) → FAULT code 2
//   T27  PRECHARGE_LATCH set on PRECHARGE entry, cleared on exit to ARMED
//   T28  Fault from ST_IDLE → FAULT, code=1
//   T29  Fault reset with SW_COMPUTE=0 → exits to ST_IDLE (COMPUTE_EN=0)
//   T30  OV fires fault_set in ST_COMPUTE → FAULT, code=2
//   T31  arm_hold drop (MCU_CMD_ARM) while SW_COMPUTE=0 → ST_IDLE, no fault
//   T32  STATE_CODE explicit values: IDLE=0, COMPUTE=3, PRECHARGE=1, ARMED=2, FAULT=0
//   T33  Soft-abort exit with SW_COMPUTE=0 → ST_IDLE, no fault
//   T34  Remote arm pulse during abort-hold silently dropped (state guard)
// =============================================================================

module tb_pdu_glue;

    // -------------------------------------------------------------------------
    // Clock — real 2.08 MHz (period 480.77 ns, rounded to 480 ns)
    // -------------------------------------------------------------------------
    localparam real CLK_PERIOD_NS = 480.77;
    localparam integer OSC_DIV_1MS = 2080;     // matches DUT localparam
    // ms expressed in simulation time
    localparam real NS_PER_MS = OSC_DIV_1MS * CLK_PERIOD_NS;

    logic sim_clk = 0;
    always #(CLK_PERIOD_NS/2.0) sim_clk = ~sim_clk;

    // -------------------------------------------------------------------------
    // DUT ports
    // -------------------------------------------------------------------------
    logic SW_COMPUTE       = 0;
    logic SW_ARM           = 0;
    logic SW_RST_FAULT     = 0;
    logic SW_REMOTE_ARM    = 0;
    logic SW_REMOTE_DISARM = 0;
    logic ESTOP_OK         = 1;   // start healthy
    logic OV               = 0;
    logic UV               = 0;
    logic VBUS_OV          = 0;
    logic MCU_ALIVE        = 1;
    logic MCU_CMD_ARM      = 0;
    logic MCU_CMD_FAULT    = 0;
    logic PRECHARGE_OK     = 0;
    logic FB_CLOSED        = 0;

    wire  FAULT_LATCH;
    wire  PRECHARGE_LATCH;
    wire  MOTOR_EN;
    wire  COMPUTE_EN;
    wire  K_SEL;
    wire  K_EN;
    wire  [3:0] FAULT_CODE;
    wire  [1:0] STATE_CODE;
    wire  OVUV_OK;
    wire  ARM_PERMIT;
    wire  PRECHARGE_REQ;
    wire  DUMP_EN;
    wire  SPARE_OUT_0;
    wire  SPARE_OUT_1;
    tri   i2c1_scl;
    tri   i2c1_sda;

    // Weak pull-ups for I2C bus (open-drain)
    assign (weak1, highz0) i2c1_scl = 1'b1;
    assign (weak1, highz0) i2c1_sda = 1'b1;

    pdu_glue dut (
        .sim_clk        (sim_clk),
        .SW_COMPUTE     (SW_COMPUTE),
        .SW_ARM         (SW_ARM),
        .SW_RST_FAULT   (SW_RST_FAULT),
        .SW_REMOTE_ARM  (SW_REMOTE_ARM),
        .SW_REMOTE_DISARM(SW_REMOTE_DISARM),
        .ESTOP_OK       (ESTOP_OK),
        .OV             (OV),
        .UV             (UV),
        .VBUS_OV        (VBUS_OV),
        .MCU_ALIVE      (MCU_ALIVE),
        .MCU_CMD_ARM    (MCU_CMD_ARM),
        .MCU_CMD_FAULT  (MCU_CMD_FAULT),
        .PRECHARGE_OK   (PRECHARGE_OK),
        .FB_CLOSED      (FB_CLOSED),
        .i2c1_scl       (i2c1_scl),
        .i2c1_sda       (i2c1_sda),
        .FAULT_LATCH    (FAULT_LATCH),
        .PRECHARGE_LATCH(PRECHARGE_LATCH),
        .MOTOR_EN       (MOTOR_EN),
        .COMPUTE_EN     (COMPUTE_EN),
        .K_SEL          (K_SEL),
        .K_EN           (K_EN),
        .FAULT_CODE     (FAULT_CODE),
        .STATE_CODE     (STATE_CODE),
        .OVUV_OK        (OVUV_OK),
        .ARM_PERMIT     (ARM_PERMIT),
        .PRECHARGE_REQ  (PRECHARGE_REQ),
        .DUMP_EN        (DUMP_EN),
        .SPARE_OUT_0    (SPARE_OUT_0),
        .SPARE_OUT_1    (SPARE_OUT_1)
    );

    // -------------------------------------------------------------------------
    // Test infrastructure
    // -------------------------------------------------------------------------
    integer pass_count = 0;
    integer fail_count = 0;

    task automatic check(input string name, input logic got, input logic expected);
        if (got === expected) begin
            $display("  PASS  %s", name);
            pass_count++;
        end else begin
            $display("  FAIL  %s  (got=%0b expected=%0b) at t=%0t ns", name, got, expected, $time);
            fail_count++;
        end
    endtask

    task automatic check_eq4(input string name, input logic [3:0] got, input logic [3:0] expected);
        if (got === expected) begin
            $display("  PASS  %s", name);
            pass_count++;
        end else begin
            $display("  FAIL  %s  (got=0x%0h expected=0x%0h) at t=%0t ns", name, got, expected, $time);
            fail_count++;
        end
    endtask

    // Wait n milliseconds of simulated time
    task automatic wait_ms(input integer n);
        #(n * NS_PER_MS);
    endtask

    // Wait a few clock edges (for synchroniser + output register to settle)
    // 6 clocks covers 2-FF sync + tick + output register
    task automatic settle();
        repeat(6) @(posedge sim_clk);
    endtask

    // Wait long enough to guarantee at least one full tick_1ms fires regardless of
    // current phase, then settle. Using 2 ms ensures the interval [t, t+2ms]
    // always contains at least one full 1ms tick period.
    task automatic wait_tick();
        wait_ms(2);
        settle();
    endtask

    // Drive SW_REMOTE_ARM for 5 ms (well past 2ms filter), then drop.
    // After dropping, wait 2 extra ticks so remote_arm_latch is registered
    // and visible to arm_ok on the following tick.
    task automatic pulse_remote_arm();
        SW_REMOTE_ARM = 1;
        wait_ms(5);
        SW_REMOTE_ARM = 0;
        wait_ms(3);  // rarm_re fires, latch set, visible next tick
    endtask

    // Full de-assert of all arm inputs and wait for clean state
    task automatic full_dearm();
        SW_ARM           = 0;
        MCU_CMD_ARM      = 0;
        SW_REMOTE_ARM    = 0;
        SW_REMOTE_DISARM = 1;
        wait_ms(5);
        SW_REMOTE_DISARM = 0;
        wait_ms(2);
    endtask

    // Inter-test drain: wait long enough for all blanking timers to expire,
    // ensure FAULT_LATCH=0, all outputs idle.
    task automatic inter_test_drain();
        wait_ms(520);  // > T_BLANK_OPEN_MS=500 and T_BLANK_CLOSE_MS=500
    endtask

    // Reset fault: ensure all inputs clear, then pulse RST
    task automatic do_fault_reset();
        full_dearm();
        wait_ms(2);
        SW_RST_FAULT = 1;
        wait_ms(8);   // T_DEBOUNCE_RST_MS=5 needs 7 ticks to fire; 8ms gives margin
        SW_RST_FAULT = 0;
        wait_tick();
    endtask

    // -------------------------------------------------------------------------
    // Main test sequence
    // -------------------------------------------------------------------------
    initial begin
        $display("=== tb_pdu_glue starting ===");

        // ----------------------------------------------------------------
        // T01: Power-on defaults (ESTOP_OK=1, everything else 0)
        // ----------------------------------------------------------------
        $display("\n--- T01: Power-on defaults ---");
        wait_ms(5);   // let synchronisers and output register run
        check("T01 FAULT_LATCH=0",    FAULT_LATCH,    1'b0);
        check("T01 MOTOR_EN=0",       MOTOR_EN,       1'b0);
        check("T01 K_EN=0",           K_EN,           1'b0);
        check("T01 COMPUTE_EN=0",     COMPUTE_EN,     1'b0);
        check("T01 DUMP_EN=0",        DUMP_EN,        1'b0);
        check("T01 PRECHARGE_REQ=0",  PRECHARGE_REQ,  1'b0);
        check("T01 OVUV_OK=1",        OVUV_OK,        1'b1);

        // ----------------------------------------------------------------
        // T02: COMPUTE_EN follows SW_COMPUTE
        // ----------------------------------------------------------------
        $display("\n--- T02: COMPUTE_EN tracks SW_COMPUTE ---");
        SW_COMPUTE = 1;
        wait_tick();
        check("T02 COMPUTE_EN=1 when SW_COMPUTE=1", COMPUTE_EN, 1'b1);
        SW_COMPUTE = 0;
        wait_tick();
        check("T02 COMPUTE_EN=0 when SW_COMPUTE=0", COMPUTE_EN, 1'b0);

        // ----------------------------------------------------------------
        // T03: arm_ok requires all conditions; missing one inhibits precharge
        //      We try to arm without MCU_CMD_ARM → should stay in COMPUTE
        // ----------------------------------------------------------------
        $display("\n--- T03: arm_ok inhibit (missing MCU_CMD_ARM) ---");
        SW_COMPUTE  = 1;
        SW_ARM      = 1;
        pulse_remote_arm();   // latch remote_arm_latch
        // MCU_CMD_ARM still 0 → arm_ok = 0
        wait_tick();
        check("T03 PRECHARGE_REQ=0 (no MCU_CMD_ARM)", PRECHARGE_REQ, 1'b0);
        check("T03 MOTOR_EN=0",                        MOTOR_EN,      1'b0);
        // Now also test missing ESTOP
        MCU_CMD_ARM = 1;
        ESTOP_OK    = 0;
        wait_tick();
        check("T03 PRECHARGE_REQ=0 (no ESTOP)",  PRECHARGE_REQ, 1'b0);
        ESTOP_OK    = 1;
        // Clean up — ESTOP=0 fired fault_set, so FSM may be in FAULT. Reset it.
        SW_ARM      = 0;
        MCU_CMD_ARM = 0;
        full_dearm();
        SW_COMPUTE  = 0;
        do_fault_reset();   // clears fault_latch_r if fault was latched
        wait_tick();

        // ----------------------------------------------------------------
        // T04: Normal arm path: COMPUTE → PRECHARGE → ARMED → MOTOR_EN
        // ----------------------------------------------------------------
        $display("\n--- T04: Normal precharge -> armed path ---");
        SW_COMPUTE  = 1;
        wait_tick();
        SW_ARM      = 1;
        MCU_CMD_ARM = 1;
        pulse_remote_arm();  // waits for latch to register
        // arm_ok now true; state machine needs one tick to act
        wait_ms(2);
        check("T04 PRECHARGE_REQ=1",  PRECHARGE_REQ, 1'b1);
        check("T04 K_SEL=0 (NC=pchg)", K_SEL,        1'b0);
        check("T04 MOTOR_EN=0",        MOTOR_EN,      1'b0);
        // After ksel blank clears (30ms), K_EN should assert
        wait_ms(35);
        check("T04 K_EN=1 after blank", K_EN, 1'b1);
        // Assert PRECHARGE_OK → should move to ARMED
        PRECHARGE_OK = 1;
        wait_ms(6);   // T_COMP_FILT_MS=3 filter takes ~5ms to assert pchgok_filt
        check("T04 MOTOR_EN=1 (ARMED)", MOTOR_EN,      1'b1);
        check("T04 PRECHARGE_REQ=0",    PRECHARGE_REQ, 1'b0);
        check("T04 K_EN=0 (contactor drives bus)", K_EN, 1'b0);
        // Assert FB_CLOSED within close-verify window
        FB_CLOSED = 1;
        wait_ms(10);
        check("T04 FAULT_LATCH=0 (close verify passed)", FAULT_LATCH, 1'b0);
        // Clean up
        SW_ARM      = 0;
        MCU_CMD_ARM = 0;
        full_dearm();
        FB_CLOSED    = 0;
        PRECHARGE_OK = 0;
        SW_COMPUTE   = 0;
        inter_test_drain();  // let blank timers expire

        // ----------------------------------------------------------------
        // T05: Precharge timeout → ST_PRECHARGE_ABORT → FAULT
        //      During abort-hold: FAULT_LATCH=1 (latched on entry),
        //      PRECHARGE_REQ=1, K_EN=1 (relay held to avoid dry-break arc).
        //      Asserting PRECHARGE_OK triggers early exit once min-hold expires.
        // ----------------------------------------------------------------
        $display("\n--- T05: Precharge timeout -> PRECHARGE_ABORT -> FAULT ---");
        SW_COMPUTE  = 1;
        wait_tick();
        SW_ARM      = 1;
        MCU_CMD_ARM = 1;
        pulse_remote_arm();
        wait_ms(2);
        check("T05 in PRECHARGE",      PRECHARGE_REQ, 1'b1);
        // Wait for precharge timeout (T_PRECHG_MAX_MS=8000ms fires at tick 8001)
        wait_ms(8002);
        // Now in ST_PRECHARGE_ABORT: fault latched, relay still on NC path
        check("T05 FAULT_LATCH=1 (latched on abort entry)", FAULT_LATCH,   1'b1);
        check_eq4("T05 FAULT_CODE=5",                       FAULT_CODE,    4'd5);
        check("T05 PRECHARGE_REQ=1 (relay still active)",   PRECHARGE_REQ, 1'b1);
        check("T05 K_EN=1 (relay held during hold)",        K_EN,          1'b1);
        // Assert PRECHARGE_OK to trigger early exit once min-hold expires
        PRECHARGE_OK = 1;
        wait_ms(110);  // T_PRECHG_ABORT_MIN_MS=100ms + filter margin
        check("T05 PRECHARGE_REQ=0 (exited to FAULT)",     PRECHARGE_REQ, 1'b0);
        check("T05 K_EN=0 (relay off in FAULT)",           K_EN,          1'b0);
        check("T05 FAULT_LATCH=1 (still faulted)",        FAULT_LATCH,   1'b1);
        PRECHARGE_OK = 0;
        do_fault_reset();
        SW_ARM      = 0;
        MCU_CMD_ARM = 0;
        full_dearm();
        SW_COMPUTE = 0;
        inter_test_drain();

        // ----------------------------------------------------------------
        // T06: ENABLE_FB_CLOSED=0 — close-verify disabled
        //      Holding ARMED for >T_BLANK_CLOSE_MS=500ms without asserting
        //      FB_CLOSED must NOT latch a fault.  MOTOR_EN must stay asserted.
        // ----------------------------------------------------------------
        $display("\n--- T06: ENABLE_FB_CLOSED=0: no close-verify fault ---");
        SW_COMPUTE   = 1;
        wait_tick();
        SW_ARM       = 1;
        MCU_CMD_ARM  = 1;
        PRECHARGE_OK = 1;   // bus already charged → skip precharge
        pulse_remote_arm();
        wait_ms(2);
        check("T06 MOTOR_EN=1 (ARMED)", MOTOR_EN, 1'b1);
        // Hold ARMED >500ms without FB_CLOSED; with ENABLE_FB_CLOSED=0 this is safe
        wait_ms(510);
        check("T06 FAULT_LATCH=0 (no close-verify fault)", FAULT_LATCH, 1'b0);
        check("T06 MOTOR_EN=1 (still ARMED)",              MOTOR_EN,    1'b1);
        SW_ARM       = 0;
        MCU_CMD_ARM  = 0;
        PRECHARGE_OK = 0;
        full_dearm();
        SW_COMPUTE   = 0;
        inter_test_drain();

        // ----------------------------------------------------------------
        // T07: ENABLE_FB_CLOSED=0 — weld detection disabled
        //      Asserting FB_CLOSED while MOTOR_EN=0 (weld condition) and
        //      open-blank already expired must NOT latch a fault.
        // ----------------------------------------------------------------
        $display("\n--- T07: ENABLE_FB_CLOSED=0: no weld-detection fault ---");
        // inter_test_drain above ensures t_blank_open=0 before we start
        SW_COMPUTE = 1;
        wait_tick();
        // Assert FB_CLOSED — weld condition would fire if ENABLE_FB_CLOSED=1
        FB_CLOSED = 1;
        wait_ms(510);  // hold well past any weld-detection window
        check("T07 FAULT_LATCH=0 (weld detection disabled)", FAULT_LATCH, 1'b0);
        FB_CLOSED  = 0;
        SW_COMPUTE = 0;
        wait_tick();

        // ----------------------------------------------------------------
        // T08: ESTOP fault from ARMED → FAULT code 1, K_EN drops
        // ----------------------------------------------------------------
        $display("\n--- T08: ESTOP fault from ARMED -> FAULT(1) ---");
        SW_COMPUTE   = 1;
        wait_tick();
        SW_ARM       = 1;
        MCU_CMD_ARM  = 1;
        PRECHARGE_OK = 1;
        pulse_remote_arm();
        wait_ms(2);
        FB_CLOSED = 1;
        wait_ms(5);
        check("T08 MOTOR_EN=1 pre-fault", MOTOR_EN, 1'b1);
        ESTOP_OK = 0;
        wait_ms(4);   // sync (2 cyc) + tick + output register — needs >2ms to be safe
        settle();
        check("T08 FAULT_LATCH=1",    FAULT_LATCH, 1'b1);
        check_eq4("T08 FAULT_CODE=1", FAULT_CODE,  4'd1);
        check("T08 MOTOR_EN=0",       MOTOR_EN,    1'b0);
        check("T08 K_EN=0",           K_EN,        1'b0);
        ESTOP_OK  = 1;
        FB_CLOSED = 0;
        do_fault_reset();
        PRECHARGE_OK = 0;
        SW_COMPUTE   = 0;
        inter_test_drain();

        // ----------------------------------------------------------------
        // T09: MCU_CMD_FAULT rising edge → FAULT code 6
        // ----------------------------------------------------------------
        $display("\n--- T09: MCU_CMD_FAULT injection -> FAULT(6) ---");
        SW_COMPUTE = 1;
        wait_tick();
        MCU_CMD_FAULT = 1;
        wait_tick();
        MCU_CMD_FAULT = 0;
        wait_tick();
        check("T09 FAULT_LATCH=1",    FAULT_LATCH, 1'b1);
        check_eq4("T09 FAULT_CODE=6", FAULT_CODE,  4'd6);
        // COMPUTE_EN still follows SW_COMPUTE even in FAULT
        check("T09 COMPUTE_EN=1",     COMPUTE_EN,  1'b1);
        do_fault_reset();
        SW_COMPUTE = 0;
        wait_tick();

        // ----------------------------------------------------------------
        // T10: VBUS_OV orthogonal dump — K_SEL goes high, K_EN asserts after blank
        // ----------------------------------------------------------------
        $display("\n--- T10: VBUS_OV dump (orthogonal) ---");
        SW_COMPUTE = 1;
        wait_tick();
        check("T10 K_SEL=0 pre-OV",  K_SEL, 1'b0);
        check("T10 K_EN=0 pre-OV",   K_EN,  1'b0);
        VBUS_OV = 1;
        wait_ms(6);   // T_COMP_FILT_MS=3 filter takes ~5ms; 6ms gives margin
        check("T10 K_SEL=1 on dump", K_SEL, 1'b1);
        check("T10 K_EN=0 (blanking)", K_EN, 1'b0);   // still blanking
        wait_ms(35);   // past T_KSEL_BLANK_MS=30
        check("T10 K_EN=1 after blank", K_EN,   1'b1);
        check("T10 DUMP_EN=1",          DUMP_EN, 1'b1);
        check("T10 FAULT_LATCH=0 (no fault)", FAULT_LATCH, 1'b0);
        VBUS_OV = 0;
        wait_ms(35);
        check("T10 K_SEL=0 after OV clears", K_SEL, 1'b0);
        check("T10 K_EN=0 after blank",      K_EN,  1'b0);
        SW_COMPUTE = 0;
        wait_tick();

        // ----------------------------------------------------------------
        // T11: ESTOP fault from COMPUTE → FAULT, code=1
        //      (T11 was previously VBUS_OV blocks arm entry; VBUS_OV is no
        //      longer in arm_ok — dump is orthogonal.  Slot repurposed to
        //      verify ESTOP generates fault code 1 from ST_COMPUTE.)
        // ----------------------------------------------------------------
        $display("\n--- T11: ESTOP fault from COMPUTE -> FAULT(1) ---");
        SW_COMPUTE = 1;
        wait_tick();
        ESTOP_OK   = 0;
        wait_ms(4);   // sync (2 cycles) + tick + output register
        check("T11 FAULT_LATCH=1",    FAULT_LATCH, 1'b1);
        check_eq4("T11 FAULT_CODE=1", FAULT_CODE,  4'd1);
        ESTOP_OK = 1;
        do_fault_reset();
        SW_COMPUTE = 0;
        wait_tick();

        // ----------------------------------------------------------------
        // T12: Soft de-arm — MCU_CMD_ARM drops while ARMED → back to COMPUTE
        // ----------------------------------------------------------------
        $display("\n--- T12: Soft de-arm (MCU_CMD_ARM drops) ---");
        SW_COMPUTE   = 1;
        wait_tick();
        SW_ARM       = 1;
        MCU_CMD_ARM  = 1;
        PRECHARGE_OK = 1;
        pulse_remote_arm();
        wait_ms(2);
        FB_CLOSED = 1;
        wait_ms(5);
        check("T12 MOTOR_EN=1 (ARMED)", MOTOR_EN, 1'b1);
        MCU_CMD_ARM = 0;
        wait_ms(2);
        check("T12 MOTOR_EN=0 (de-armed)", MOTOR_EN,    1'b0);
        check("T12 FAULT_LATCH=0",         FAULT_LATCH, 1'b0);
        check("T12 COMPUTE_EN=1",          COMPUTE_EN,  1'b1);
        FB_CLOSED    = 0;
        PRECHARGE_OK = 0;
        SW_ARM       = 0;
        full_dearm();
        SW_COMPUTE   = 0;
        inter_test_drain();

        // ----------------------------------------------------------------
        // T13: Fault reset succeeds while SW_ARM is held.
        //      The ~SW_ARM_S guard has been removed from the DUT; the
        //      operator may hold ARM ready for immediate re-arm after reset.
        // ----------------------------------------------------------------
        $display("\n--- T13: Fault reset succeeds with SW_ARM held ---");
        SW_COMPUTE    = 1;
        SW_ARM        = 1;      // hold throughout
        MCU_CMD_FAULT = 1;
        wait_tick();
        MCU_CMD_FAULT = 0;
        wait_tick();
        check("T13 FAULT_LATCH=1", FAULT_LATCH, 1'b1);
        // RST while SW_ARM=1 and ESTOP OK — must succeed now
        SW_RST_FAULT = 1;
        wait_ms(8);   // T_DEBOUNCE_RST_MS needs 7 ticks
        SW_RST_FAULT = 0;
        wait_tick();
        check("T13 FAULT_LATCH=0 (reset with SW_ARM held)", FAULT_LATCH, 1'b0);
        SW_ARM      = 0;
        MCU_CMD_ARM = 0;
        SW_COMPUTE  = 0;
        wait_tick();

        // ----------------------------------------------------------------
        // T14: Remote arm latch — set by pulse, cleared by disarm pulse
        //      PRECHARGE_OK=1 so arm_ok+pchgok_filt → ARMED directly
        //      (avoids ST_PRECHARGE_ABORT abort-hold contaminating T15+).
        // ----------------------------------------------------------------
        $display("\n--- T14: Remote arm/disarm latch ---");
        SW_COMPUTE   = 1;
        PRECHARGE_OK = 1;   // pre-charged: pchgok_filt asserts before arm fires
        wait_ms(6);         // pchgok_filt needs 4 ticks (~4ms+margin) to assert
        check("T14 ARM_PERMIT=0 before remote arm", ARM_PERMIT, 1'b0);
        // SW_ARM must be asserted BEFORE (or during) pulse so remote_arm_latch
        // is not immediately cleared by the ~SW_ARM_S condition on the same tick.
        SW_ARM      = 1;
        MCU_CMD_ARM = 1;
        wait_tick();            // let SW_ARM_S propagate
        pulse_remote_arm();     // rarm_re fires while SW_ARM_S=1 → latch sticks
        wait_ms(2);
        check("T14 ARM_PERMIT=1 after remote arm + sw_arm + mcu_arm", ARM_PERMIT, 1'b1);
        // Disarm via remote disarm: remote_arm_latch=0 → arm_hold=0 → ST_COMPUTE
        SW_REMOTE_DISARM = 1;
        wait_ms(3);
        SW_REMOTE_DISARM = 0;
        wait_tick();
        check("T14 ARM_PERMIT=0 after remote disarm", ARM_PERMIT, 1'b0);
        SW_ARM       = 0;
        MCU_CMD_ARM  = 0;
        PRECHARGE_OK = 0;
        full_dearm();
        SW_COMPUTE   = 0;
        inter_test_drain();

        // ----------------------------------------------------------------
        // T15: Dump timing with vbusov_filt
        //      VBUS_OV=1 → vbusov_filt asserts after T_COMP_FILT_MS=3ms
        //              → K_SEL=1; K_EN suppressed for T_KSEL_BLANK_MS=30ms.
        //      VBUS_OV=0 → vbusov_filt drops after 3ms → K_SEL=0;
        //              → K_EN drops (no precharge active, no dump).
        // ----------------------------------------------------------------
        $display("\n--- T15: Dump timing: vbusov_filt + ksel_blank ---");
        SW_COMPUTE = 1;
        wait_tick();
        VBUS_OV = 1;
        wait_ms(8);    // vbusov_filt asserts tick 4, K_SEL registers tick 5; 8ms gives margin
        check("T15 K_SEL=1 (dump path)",       K_SEL,   1'b1);
        check("T15 K_EN=0 (ksel_blank active)", K_EN,    1'b0);
        wait_ms(35);   // ksel_blank=30ms elapses → K_EN asserts
        check("T15 K_EN=1 after blank",         K_EN,    1'b1);
        check("T15 DUMP_EN=1",                  DUMP_EN, 1'b1);
        check("T15 FAULT_LATCH=0 (no fault)",   FAULT_LATCH, 1'b0);
        VBUS_OV = 0;
        wait_ms(8);    // vbusov_filt drops tick 4, K_SEL registers tick 5; 8ms gives margin
        check("T15 K_SEL=0 (dump cleared)",    K_SEL,   1'b0);
        wait_ms(35);   // K_EN drops on same tick as K_SEL (dump_req=0→ken_next=0)
        check("T15 K_EN=0 after blank",         K_EN,    1'b0);
        check("T15 DUMP_EN=0",                  DUMP_EN, 1'b0);
        SW_COMPUTE = 0;
        wait_tick();

        // ----------------------------------------------------------------
        // T16: OV fault (code 2) and UV fault (code 3)
        // ----------------------------------------------------------------
        $display("\n--- T16: OV fault -> FAULT(2), UV fault -> FAULT(3) ---");
        // --- OV ---
        SW_COMPUTE = 1;
        wait_tick();
        OV = 1;
        wait_ms(6);   // T_COMP_FILT_MS=3: ov_filt asserts ~5ms after OV rises
        check("T16 FAULT_LATCH=1 on OV",    FAULT_LATCH, 1'b1);
        check_eq4("T16 FAULT_CODE=2 on OV", FAULT_CODE,  4'd2);
        check("T16 OVUV_OK=0 on OV",        OVUV_OK,     1'b0);
        OV = 0;
        do_fault_reset();
        // --- UV ---
        SW_COMPUTE = 1;
        wait_tick();
        UV = 1;
        wait_ms(6);   // same filter latency
        check("T16 FAULT_LATCH=1 on UV",    FAULT_LATCH, 1'b1);
        check_eq4("T16 FAULT_CODE=3 on UV", FAULT_CODE,  4'd3);
        check("T16 OVUV_OK=0 on UV",        OVUV_OK,     1'b0);
        UV = 0;
        do_fault_reset();
        SW_COMPUTE = 0;
        inter_test_drain();

        // ----------------------------------------------------------------
        // T17: VBUS_OV coexists with ARMED — dump active, contactor stays closed
        // ----------------------------------------------------------------
        $display("\n--- T17: VBUS_OV coexists with ARMED (no de-arm) ---");
        SW_COMPUTE   = 1;
        wait_tick();
        SW_ARM       = 1;
        MCU_CMD_ARM  = 1;
        PRECHARGE_OK = 1;
        pulse_remote_arm();
        wait_ms(2);
        FB_CLOSED = 1;
        wait_ms(5);
        check("T17 MOTOR_EN=1 (ARMED before OV)",  MOTOR_EN,    1'b1);
        check("T17 FAULT_LATCH=0 (no fault yet)",  FAULT_LATCH, 1'b0);
        // Assert VBUS_OV — dump should activate, but arm_hold excludes VBUS_OV
        // so the contactor must NOT open and FAULT must NOT latch.
        VBUS_OV = 1;
        wait_ms(40);    // filter+ksel_blank: vbusov_filt at ~4ms, K_SEL at ~5ms,
                        // ksel_blank(30ms) from ~4ms = K_EN at ~35ms; 40ms gives margin
        check("T17 MOTOR_EN=1 (still ARMED)",      MOTOR_EN,    1'b1);
        check("T17 FAULT_LATCH=0 (no fault)",      FAULT_LATCH, 1'b0);
        check("T17 K_SEL=1 (dump path)",           K_SEL,       1'b1);
        check("T17 DUMP_EN=1",                     DUMP_EN,     1'b1);
        // Clear VBUS_OV — dump must stop, MOTOR_EN stays up
        VBUS_OV = 0;
        wait_ms(35);    // blank after K_SEL returns to 0
        check("T17 MOTOR_EN=1 (ARMED after OV)",   MOTOR_EN,    1'b1);
        check("T17 DUMP_EN=0 after OV clears",     DUMP_EN,     1'b0);
        check("T17 FAULT_LATCH=0 throughout",      FAULT_LATCH, 1'b0);
        // Clean up
        SW_ARM       = 0;
        MCU_CMD_ARM  = 0;
        FB_CLOSED    = 0;
        PRECHARGE_OK = 0;
        full_dearm();
        SW_COMPUTE   = 0;
        inter_test_drain();

        // ----------------------------------------------------------------
        // T18: Soft abort → ST_PRECHARGE_ABORT → clean exit → re-arm
        //      SW_ARM drop during PRECHARGE triggers abort-hold:
        //        relay stays closed (K_EN=1), PRECHARGE_REQ=1, FAULT_LATCH=0.
        //      Asserting PRECHARGE_OK triggers early exit once min-hold expires.
        //      After exit (to ST_COMPUTE): re-arm succeeds via remote arm pulse.
        // ----------------------------------------------------------------
        $display("\n--- T18: Soft abort-hold then clean re-arm ---");
        SW_COMPUTE  = 1;
        wait_tick();
        SW_ARM      = 1;
        MCU_CMD_ARM = 1;
        pulse_remote_arm();
        wait_ms(2);
        check("T18 in PRECHARGE",    PRECHARGE_REQ, 1'b1);
        check("T18 K_EN=1 (relay)",  K_EN,          1'b1);
        // Drop SW_ARM → soft abort → enters ST_PRECHARGE_ABORT
        SW_ARM = 0;
        wait_ms(2);
        // During abort-hold: relay still closed, PRECHARGE_REQ high, no fault
        check("T18 PRECHARGE_REQ=1 (abort-hold)",  PRECHARGE_REQ, 1'b1);
        check("T18 K_EN=1 (relay held)",            K_EN,          1'b1);
        check("T18 FAULT_LATCH=0 (soft abort)",     FAULT_LATCH,   1'b0);
        // Assert PRECHARGE_OK to trigger early exit once min-hold expires
        PRECHARGE_OK = 1;
        wait_ms(110);  // T_PRECHG_ABORT_MIN_MS=100ms + filter margin
        check("T18 PRECHARGE_REQ=0 (exited hold)",  PRECHARGE_REQ, 1'b0);
        check("T18 FAULT_LATCH=0 (no fault)",       FAULT_LATCH,   1'b0);
        check("T18 K_EN=0 (relay off in COMPUTE)",  K_EN,          1'b0);
        // Re-arm: SW_ARM=1 + remote arm in ST_COMPUTE → latch → ARMED
        SW_ARM = 1;
        pulse_remote_arm();  // state=ST_COMPUTE → guard passes → latch sets
        wait_ms(2);
        check("T18 MOTOR_EN=1 (re-armed after abort-hold)", MOTOR_EN, 1'b1);
        SW_ARM       = 0;
        MCU_CMD_ARM  = 0;
        PRECHARGE_OK = 0;
        full_dearm();
        SW_COMPUTE   = 0;
        inter_test_drain();

        // ----------------------------------------------------------------
        // T19: Fault reset guard — ESTOP must be released before RST accepted;
        //      RST pressed while fault still active must be ignored, then
        //      re-press after clearing must succeed
        // ----------------------------------------------------------------
        $display("\n--- T19: Fault reset blocked while ESTOP active; works after release ---");
        SW_COMPUTE = 1;
        wait_tick();
        ESTOP_OK = 0;
        wait_ms(4);   // let sync + tick propagate
        check("T19 FAULT_LATCH=1 (ESTOP fault)", FAULT_LATCH, 1'b1);
        // Press RST while ESTOP still held — must be rejected (fault_set still 1)
        SW_RST_FAULT = 1;
        wait_tick();
        SW_RST_FAULT = 0;
        wait_tick();
        check("T19 FAULT_LATCH still 1 (ESTOP still held)", FAULT_LATCH, 1'b1);
        // Release ESTOP, then press RST — must succeed
        ESTOP_OK = 1;
        wait_ms(4);   // sync + tick
        SW_RST_FAULT = 1;
        wait_ms(8);   // T_DEBOUNCE_RST_MS=5 needs 7 ticks; 8ms gives margin
        SW_RST_FAULT = 0;
        wait_tick();
        check("T19 FAULT_LATCH=0 after ESTOP cleared + RST", FAULT_LATCH, 1'b0);
        SW_COMPUTE = 0;
        wait_tick();

        // ----------------------------------------------------------------
        // T20: SW_ARM toggling in FAULT state never clears fault
        //      (tests that only RST can clear, not any other input)
        // ----------------------------------------------------------------
        $display("\n--- T20: SW_ARM toggling in FAULT does not clear fault ---");
        SW_COMPUTE    = 1;
        MCU_CMD_FAULT = 1;
        wait_tick();
        MCU_CMD_FAULT = 0;
        wait_tick();
        check("T20 FAULT_LATCH=1 (in fault)", FAULT_LATCH, 1'b1);
        // Toggle SW_ARM 6 times — FAULT_LATCH must stay 1 throughout
        repeat (6) begin
            SW_ARM = 1;
            wait_ms(50);   // hold for 50 ms (simulate real button press duration)
            check("T20 FAULT_LATCH=1 (SW_ARM held)", FAULT_LATCH, 1'b1);
            SW_ARM = 0;
            wait_ms(20);
            check("T20 FAULT_LATCH=1 (SW_ARM released)", FAULT_LATCH, 1'b1);
        end
        // Confirm only RST clears it
        do_fault_reset();
        check("T20 FAULT_LATCH=0 only after RST", FAULT_LATCH, 1'b0);
        SW_COMPUTE = 0;
        wait_tick();

        // ----------------------------------------------------------------
        // T21: Fault-abort → pchgok triggers exit → ST_FAULT
        //      Fault during PRECHARGE → PRECHARGE_ABORT (pchg_abort_was_fault=1)
        //      → asserting PRECHARGE_OK triggers early exit → destination=ST_FAULT.
        // ----------------------------------------------------------------
        $display("\n--- T21: Fault-abort + pchgok → ST_FAULT ---");
        SW_COMPUTE  = 1;
        wait_tick();
        SW_ARM      = 1;
        MCU_CMD_ARM = 1;
        pulse_remote_arm();
        wait_ms(2);
        check("T21 in PRECHARGE", PRECHARGE_REQ, 1'b1);
        // Trigger fault during PRECHARGE → PRECHARGE_ABORT (pchg_abort_was_fault=1)
        ESTOP_OK = 0;
        wait_ms(4);
        check("T21 FAULT_LATCH=1 (fault on abort entry)", FAULT_LATCH,   1'b1);
        check("T21 PRECHARGE_REQ=1 (abort-hold active)",  PRECHARGE_REQ, 1'b1);
        ESTOP_OK     = 1;  // clear fault source so exit condition is met
        PRECHARGE_OK = 1;  // trigger pchgok_filt
        wait_ms(110);      // min hold + filter margin
        check("T21 PRECHARGE_REQ=0 (exited to FAULT)",   PRECHARGE_REQ, 1'b0);
        check("T21 FAULT_LATCH=1 (in ST_FAULT)",         FAULT_LATCH,   1'b1);
        check("T21 MOTOR_EN=0",                          MOTOR_EN,      1'b0);
        PRECHARGE_OK = 0;
        do_fault_reset();
        SW_ARM       = 0;
        MCU_CMD_ARM  = 0;
        full_dearm();
        SW_COMPUTE   = 0;
        inter_test_drain();

        // ----------------------------------------------------------------
        // T22: Soft-abort → pchgok triggers exit → ST_COMPUTE, no fault
        //      pchg_abort_was_fault=0 → exit to ST_COMPUTE/IDLE (no fault).
        // ----------------------------------------------------------------
        $display("\n--- T22: Soft-abort + pchgok → ST_COMPUTE, no fault ---");
        SW_COMPUTE  = 1;
        wait_tick();
        SW_ARM      = 1;
        MCU_CMD_ARM = 1;
        pulse_remote_arm();
        wait_ms(2);
        check("T22 in PRECHARGE", PRECHARGE_REQ, 1'b1);
        // Soft abort: drop SW_ARM → PRECHARGE_ABORT, pchg_abort_was_fault=0
        SW_ARM       = 0;
        PRECHARGE_OK = 1;  // assert simultaneously
        wait_ms(110);
        check("T22 FAULT_LATCH=0 (no fault)",   FAULT_LATCH,   1'b0);
        check("T22 PRECHARGE_REQ=0 (in COMPUTE)", PRECHARGE_REQ, 1'b0);
        check("T22 COMPUTE_EN=1 (in ST_COMPUTE)", COMPUTE_EN,    1'b1);
        MCU_CMD_ARM  = 0;
        PRECHARGE_OK = 0;
        full_dearm();
        SW_COMPUTE   = 0;
        wait_tick();

        // ----------------------------------------------------------------
        // T23: Soft-abort hard timeout (no pchgok) → ST_COMPUTE, no fault
        //      t_prechg_abort=300ms expires → exit even without bus equalization.
        // ----------------------------------------------------------------
        $display("\n--- T23: Soft-abort hard timeout → ST_COMPUTE, no fault ---");
        SW_COMPUTE  = 1;
        wait_tick();
        SW_ARM      = 1;
        MCU_CMD_ARM = 1;
        pulse_remote_arm();
        wait_ms(2);
        check("T23 in PRECHARGE", PRECHARGE_REQ, 1'b1);
        // Drop SW_ARM → PRECHARGE_ABORT; PRECHARGE_OK stays 0
        SW_ARM = 0;
        wait_ms(310);  // T_PRECHG_ABORT_HOLD_MS=300ms + margin
        check("T23 FAULT_LATCH=0 (no fault)",     FAULT_LATCH,   1'b0);
        check("T23 PRECHARGE_REQ=0 (exited hold)", PRECHARGE_REQ, 1'b0);
        MCU_CMD_ARM = 0;
        full_dearm();
        SW_COMPUTE  = 0;
        inter_test_drain();

        // ----------------------------------------------------------------
        // T24: VBUS_OV during abort-hold — precharge_active gates dump
        //      dump_req = vbusov_filt & ~precharge_active = 0 while in hold.
        //      K_SEL must stay 0 (NC path); K_EN=1 (precharge_active relay hold).
        // ----------------------------------------------------------------
        $display("\n--- T24: VBUS_OV during abort-hold: dump gated off ---");
        SW_COMPUTE  = 1;
        wait_tick();
        SW_ARM      = 1;
        MCU_CMD_ARM = 1;
        pulse_remote_arm();
        wait_ms(2);
        check("T24 in PRECHARGE", PRECHARGE_REQ, 1'b1);
        // Soft abort → PRECHARGE_ABORT
        SW_ARM = 0;
        wait_ms(2);
        check("T24 PRECHARGE_REQ=1 (abort-hold)", PRECHARGE_REQ, 1'b1);
        // Assert VBUS_OV during abort-hold
        VBUS_OV = 1;
        wait_ms(5);   // vbusov_filt asserts after ~3ms
        // dump_req must be 0 because precharge_active=1
        check("T24 K_SEL=0 (dump gated by precharge_active)", K_SEL,   1'b0);
        check("T24 K_EN=1 (relay held by precharge_active)",  K_EN,    1'b1);
        check("T24 DUMP_EN=0 (no dump while precharge_active)", DUMP_EN, 1'b0);
        // Clear VBUS_OV; wait for abort-hold to exit
        VBUS_OV      = 0;
        PRECHARGE_OK = 1;
        wait_ms(110);
        check("T24 PRECHARGE_REQ=0 (exited hold)", PRECHARGE_REQ, 1'b0);
        check("T24 FAULT_LATCH=0",                 FAULT_LATCH,   1'b0);
        MCU_CMD_ARM  = 0;
        PRECHARGE_OK = 0;
        full_dearm();
        SW_COMPUTE   = 0;
        inter_test_drain();

        // ----------------------------------------------------------------
        // T25: Simultaneous fault + pchgok in PRECHARGE — fault wins
        //      ov_filt and pchgok_filt assert at the same tick.
        //      State machine checks fault_set before pchgok_filt → ST_FAULT.
        // ----------------------------------------------------------------
        $display("\n--- T25: Simultaneous fault+pchgok in PRECHARGE -> fault wins ---");
        SW_COMPUTE  = 1;
        wait_tick();
        SW_ARM      = 1;
        MCU_CMD_ARM = 1;
        pulse_remote_arm();
        wait_ms(2);
        check("T25 in PRECHARGE", PRECHARGE_REQ, 1'b1);
        // Assert both OV fault and PRECHARGE_OK simultaneously
        OV           = 1;
        PRECHARGE_OK = 1;
        // ov_filt asserts after tick 4 (~4ms); FAULT_LATCH visible after tick 5
        wait_ms(8);   // both filters asserted with margin
        // State should be PRECHARGE_ABORT with pchg_abort_was_fault=1
        check("T25 FAULT_LATCH=1 (fault latched)", FAULT_LATCH,   1'b1);
        check_eq4("T25 FAULT_CODE=2",              FAULT_CODE,    4'd2);
        check("T25 PRECHARGE_REQ=1 (abort-hold)",  PRECHARGE_REQ, 1'b1);
        // Exit abort-hold (pchgok_filt already high, min hold is ~100ms)
        wait_ms(110);
        check("T25 PRECHARGE_REQ=0 (exited to FAULT)", PRECHARGE_REQ, 1'b0);
        check("T25 FAULT_LATCH=1 (in ST_FAULT)",       FAULT_LATCH,   1'b1);
        OV           = 0;
        PRECHARGE_OK = 0;
        do_fault_reset();
        SW_ARM       = 0;
        MCU_CMD_ARM  = 0;
        full_dearm();
        SW_COMPUTE   = 0;
        inter_test_drain();

        // ----------------------------------------------------------------
        // T26: OV fault (code 2) during PRECHARGE (A) and during ARMED (B)
        // ----------------------------------------------------------------
        $display("\n--- T26A: OV fault during PRECHARGE -> PRECHARGE_ABORT -> FAULT ---");
        SW_COMPUTE  = 1;
        wait_tick();
        SW_ARM      = 1;
        MCU_CMD_ARM = 1;
        pulse_remote_arm();
        wait_ms(2);
        OV = 1;   // ov_filt asserts after tick 4; fault_set=1 → PRECHARGE_ABORT
        wait_ms(8);   // ov_filt tick 4 + output register tick 5; 8ms gives margin
        check("T26A FAULT_LATCH=1",    FAULT_LATCH, 1'b1);
        check_eq4("T26A FAULT_CODE=2", FAULT_CODE,  4'd2);
        check("T26A PRECHARGE_REQ=1 (abort-hold)", PRECHARGE_REQ, 1'b1);
        OV           = 0;
        PRECHARGE_OK = 1;   // trigger early exit
        wait_ms(110);
        check("T26A PRECHARGE_REQ=0 (in FAULT)", PRECHARGE_REQ, 1'b0);
        check("T26A FAULT_LATCH=1",              FAULT_LATCH,   1'b1);
        PRECHARGE_OK = 0;
        do_fault_reset();
        SW_ARM       = 0;
        MCU_CMD_ARM  = 0;
        full_dearm();
        SW_COMPUTE   = 0;
        inter_test_drain();

        $display("\n--- T26B: OV fault during ARMED -> FAULT ---");
        SW_COMPUTE   = 1;
        wait_tick();
        SW_ARM       = 1;
        MCU_CMD_ARM  = 1;
        PRECHARGE_OK = 1;
        pulse_remote_arm();
        wait_ms(2);
        check("T26B MOTOR_EN=1 (ARMED)", MOTOR_EN, 1'b1);
        OV = 1;
        wait_ms(6);
        check("T26B FAULT_LATCH=1",    FAULT_LATCH, 1'b1);
        check_eq4("T26B FAULT_CODE=2", FAULT_CODE,  4'd2);
        check("T26B MOTOR_EN=0",       MOTOR_EN,    1'b0);
        OV           = 0;
        PRECHARGE_OK = 0;
        do_fault_reset();
        SW_ARM       = 0;
        MCU_CMD_ARM  = 0;
        full_dearm();
        SW_COMPUTE   = 0;
        inter_test_drain();

        // ----------------------------------------------------------------
        // T27: PRECHARGE_LATCH set on PRECHARGE entry, cleared on exit to ARMED
        // ----------------------------------------------------------------
        $display("\n--- T27: PRECHARGE_LATCH set/clear ---");
        SW_COMPUTE  = 1;
        wait_tick();
        SW_ARM      = 1;
        MCU_CMD_ARM = 1;
        pulse_remote_arm();
        wait_ms(2);
        check("T27 PRECHARGE_LATCH=1 (in PRECHARGE)", PRECHARGE_LATCH, 1'b1);
        check("T27 PRECHARGE_REQ=1",                  PRECHARGE_REQ,  1'b1);
        // Complete precharge → ARMED
        PRECHARGE_OK = 1;
        wait_ms(6);   // pchgok_filt asserts → ARMED
        check("T27 PRECHARGE_LATCH=0 (exited to ARMED)", PRECHARGE_LATCH, 1'b0);
        check("T27 MOTOR_EN=1 (ARMED)",                  MOTOR_EN,        1'b1);
        SW_ARM       = 0;
        MCU_CMD_ARM  = 0;
        PRECHARGE_OK = 0;
        full_dearm();
        SW_COMPUTE   = 0;
        inter_test_drain();

        // ----------------------------------------------------------------
        // T28: Fault from ST_IDLE (SW_COMPUTE=0)
        // ----------------------------------------------------------------
        $display("\n--- T28: Fault from ST_IDLE ---");
        // Ensure we are in ST_IDLE
        SW_COMPUTE = 0;
        wait_tick();
        ESTOP_OK   = 0;
        wait_ms(4);
        check("T28 FAULT_LATCH=1 (faulted from IDLE)", FAULT_LATCH, 1'b1);
        check_eq4("T28 FAULT_CODE=1",                   FAULT_CODE,  4'd1);
        ESTOP_OK = 1;
        do_fault_reset();

        // ----------------------------------------------------------------
        // T29: Fault reset with SW_COMPUTE=0 → exits to ST_IDLE
        // ----------------------------------------------------------------
        $display("\n--- T29: Fault reset -> ST_IDLE when SW_COMPUTE=0 ---");
        SW_COMPUTE    = 1;
        MCU_CMD_FAULT = 1;
        wait_tick();
        MCU_CMD_FAULT = 0;
        wait_tick();
        check("T29 FAULT_LATCH=1", FAULT_LATCH, 1'b1);
        SW_COMPUTE = 0;     // drop COMPUTE before reset
        wait_tick();
        SW_RST_FAULT = 1;
        wait_ms(8);
        SW_RST_FAULT = 0;
        wait_tick();
        check("T29 FAULT_LATCH=0 (reset)",   FAULT_LATCH, 1'b0);
        check("T29 COMPUTE_EN=0 (in ST_IDLE)", COMPUTE_EN,  1'b0);

        // ----------------------------------------------------------------
        // T30: OV fires fault_set in ST_COMPUTE → FAULT, code=2
        // ----------------------------------------------------------------
        $display("\n--- T30: OV fault from ST_COMPUTE -> FAULT(2) ---");
        SW_COMPUTE = 1;
        wait_tick();
        OV = 1;
        wait_ms(6);   // ov_filt asserts after ~3ms
        check("T30 FAULT_LATCH=1",    FAULT_LATCH, 1'b1);
        check_eq4("T30 FAULT_CODE=2", FAULT_CODE,  4'd2);
        check("T30 OVUV_OK=0",        OVUV_OK,     1'b0);
        OV = 0;
        do_fault_reset();
        SW_COMPUTE = 0;
        wait_tick();

        // ----------------------------------------------------------------
        // T31: MCU_CMD_ARM drop while ARMED with SW_COMPUTE=0 → ST_IDLE
        //      arm_hold drops → ST_COMPUTE (1 tick) → ST_IDLE (next tick)
        //      since SW_COMPUTE_S=0.  FAULT_LATCH stays 0.
        // ----------------------------------------------------------------
        $display("\n--- T31: Soft de-arm with SW_COMPUTE=0 -> ST_IDLE, no fault ---");
        SW_COMPUTE   = 1;
        wait_tick();
        SW_ARM       = 1;
        MCU_CMD_ARM  = 1;
        PRECHARGE_OK = 1;
        pulse_remote_arm();
        wait_ms(2);
        check("T31 MOTOR_EN=1 (ARMED)", MOTOR_EN, 1'b1);
        SW_COMPUTE  = 0;  // drop before de-arming
        MCU_CMD_ARM = 0;  // arm_hold drops → state → ST_COMPUTE → ST_IDLE
        wait_ms(3);
        check("T31 MOTOR_EN=0 (de-armed)",       MOTOR_EN,    1'b0);
        check("T31 COMPUTE_EN=0 (in ST_IDLE)",   COMPUTE_EN,  1'b0);
        check("T31 FAULT_LATCH=0 (no fault)",    FAULT_LATCH, 1'b0);
        SW_ARM       = 0;
        PRECHARGE_OK = 0;
        full_dearm();
        wait_tick();

        // ----------------------------------------------------------------
        // T32: STATE_CODE explicit values for all states
        // ----------------------------------------------------------------
        $display("\n--- T32: STATE_CODE values ---");
        // ST_IDLE: SW_COMPUTE=0
        SW_COMPUTE = 0;
        wait_tick();
        check("T32 STATE_CODE=0 (IDLE)",    STATE_CODE == 2'd0, 1'b1);
        // ST_COMPUTE: SW_COMPUTE=1
        SW_COMPUTE = 1;
        wait_tick();
        check("T32 STATE_CODE=3 (COMPUTE)", STATE_CODE == 2'd3, 1'b1);
        // ST_PRECHARGE: arm without bus voltage
        SW_ARM      = 1;
        MCU_CMD_ARM = 1;
        pulse_remote_arm();
        wait_ms(2);
        check("T32 STATE_CODE=1 (PRECHARGE)",       STATE_CODE == 2'd1, 1'b1);
        // ST_PRECHARGE_ABORT: drop SW_ARM
        SW_ARM = 0;
        wait_ms(2);
        check("T32 STATE_CODE=1 (PRECHARGE_ABORT)", STATE_CODE == 2'd1, 1'b1);
        // Wait for abort-hold exit → ST_COMPUTE, then ARMED via PRECHARGE_OK
        PRECHARGE_OK = 1;
        wait_ms(110);
        check("T32 STATE_CODE=3 (back in COMPUTE)", STATE_CODE == 2'd3, 1'b1);
        // Re-arm to ARMED
        SW_ARM = 1;
        pulse_remote_arm();
        wait_ms(2);
        check("T32 STATE_CODE=2 (ARMED)",  STATE_CODE == 2'd2, 1'b1);
        // Fault → ST_FAULT
        MCU_CMD_FAULT = 1;
        wait_tick();
        MCU_CMD_FAULT = 0;
        wait_tick();
        check("T32 STATE_CODE=0 (FAULT)",  STATE_CODE == 2'd0, 1'b1);
        check("T32 FAULT_LATCH=1",         FAULT_LATCH,        1'b1);
        PRECHARGE_OK = 0;
        MCU_CMD_ARM  = 0;
        SW_ARM       = 0;
        full_dearm();
        do_fault_reset();
        SW_COMPUTE   = 0;
        inter_test_drain();

        // ----------------------------------------------------------------
        // T33: Soft-abort exit with SW_COMPUTE=0 → ST_IDLE, no fault
        // ----------------------------------------------------------------
        $display("\n--- T33: Soft-abort exit with SW_COMPUTE=0 -> ST_IDLE ---");
        SW_COMPUTE  = 1;
        wait_tick();
        SW_ARM      = 1;
        MCU_CMD_ARM = 1;
        pulse_remote_arm();
        wait_ms(2);
        check("T33 in PRECHARGE", PRECHARGE_REQ, 1'b1);
        // Drop both SW_ARM and SW_COMPUTE before abort-hold exits
        SW_ARM     = 0;
        SW_COMPUTE = 0;
        wait_ms(310);  // abort-hold hard timeout (T_PRECHG_ABORT_HOLD_MS=300ms)
        check("T33 FAULT_LATCH=0 (soft abort)",    FAULT_LATCH,   1'b0);
        check("T33 PRECHARGE_REQ=0 (exited hold)", PRECHARGE_REQ, 1'b0);
        check("T33 COMPUTE_EN=0 (in ST_IDLE)",     COMPUTE_EN,    1'b0);
        MCU_CMD_ARM = 0;
        full_dearm();
        wait_tick();

        // ----------------------------------------------------------------
        // T34: Remote arm pulse during abort-hold: state guard prevents latch
        //      State is ST_PRECHARGE_ABORT ≠ ST_COMPUTE → guard fails → latch=0.
        //      After abort-hold exits to ST_COMPUTE, ARM_PERMIT must be 0.
        // ----------------------------------------------------------------
        $display("\n--- T34: Remote arm pulse during abort-hold silently dropped ---");
        SW_COMPUTE  = 1;
        wait_tick();
        SW_ARM      = 1;
        MCU_CMD_ARM = 1;
        pulse_remote_arm();
        wait_ms(2);
        check("T34 in PRECHARGE", PRECHARGE_REQ, 1'b1);
        // Drop SW_ARM → PRECHARGE_ABORT; re-raise SW_ARM immediately so
        // ~SW_ARM_S does not prevent the latch set on a subsequent pulse.
        SW_ARM = 0;
        wait_ms(2);
        SW_ARM = 1;  // raise again; the remote arm guard still blocks on state
        // Pulse remote arm while in ST_PRECHARGE_ABORT — guard: state must be ST_COMPUTE
        SW_REMOTE_ARM = 1;
        wait_ms(5);  // qualify through T_REMOTE_FILT_MS=2ms filter
        SW_REMOTE_ARM = 0;
        wait_ms(3);  // rarm_re fires; state ≠ ST_COMPUTE → latch NOT set
        // Wait for abort-hold to exit (hard timeout, PRECHARGE_OK=0)
        wait_ms(310);
        check("T34 PRECHARGE_REQ=0 (exited hold)",   PRECHARGE_REQ, 1'b0);
        // ARM_PERMIT = arm_ok = SW_ARM & remote_arm_latch & ...
        // remote_arm_latch must be 0 → ARM_PERMIT=0
        check("T34 ARM_PERMIT=0 (arm pulse dropped)", ARM_PERMIT,    1'b0);
        check("T34 FAULT_LATCH=0 (soft abort)",       FAULT_LATCH,   1'b0);
        SW_ARM      = 0;
        MCU_CMD_ARM = 0;
        full_dearm();
        SW_COMPUTE  = 0;
        inter_test_drain();

        // ----------------------------------------------------------------
        // Summary
        // ----------------------------------------------------------------
        $display("\n=== SUMMARY: %0d passed, %0d failed ===", pass_count, fail_count);
        if (fail_count == 0)
            $display("ALL TESTS PASSED");
        else
            $display("*** FAILURES DETECTED ***");

        $stop;
    end

endmodule
