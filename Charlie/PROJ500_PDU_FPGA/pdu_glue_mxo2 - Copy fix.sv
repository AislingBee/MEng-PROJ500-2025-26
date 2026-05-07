`timescale 1ns/1ps
`default_nettype none
/*
PROJ500 PDU GLUE (MachXO2-256HC SG48) + Hardened Primary I2C via EFB (Wishbone)
Revision: 2026-04-21  — full implementation per spec rev 3

--------------------------------------------------------------------------------
STATE MACHINE
  IDLE      — quiescent, SW_COMPUTE=0
  COMPUTE   — SW_COMPUTE=1, no arm active   (STATE_CODE=3)
  PRECHARGE — charging bus via resistor NC path
  ARMED     — contactor closed, motor load connected
  FAULT     — latched fault; COMPUTE_EN still follows SW_COMPUTE

DUMP is an ORTHOGONAL output, not a state.
  dump_req = VBUS_OV_S & (state != PRECHARGE)
  Dump coexists with any state including ARMED and FAULT.
  VBUS_OV blocks arm entry (arm_ok) but does not drop arm_hold once ARMED.

K_SEL POLARITY:
  K_SEL = 0  → relay de-energized → NC path = PRECHARGE resistor path
  K_SEL = 1  → relay energized    → NO path = DUMP / brake resistor path
  *** Verify this polarity on bench before first K_EN assertion ***

BUILD REQUIREMENTS:
  Add BOTH files to Diamond project:
    - this pdu_glue_mxo2.sv
    - PDU_EFB_I2C.v  (IPexpress-generated EFB wrapper)
  Constrain i2c1_scl/i2c1_sda to pins 43/42 (already in LPF).

I2C REGISTER MAP (read-only slave, addr set in IPexpress):
  0x00  STATUS0        [7]=FAULT_LATCH [6]=PRECHARGE_LATCH [5]=MOTOR_EN
                       [4]=COMPUTE_EN  [3]=K_SEL           [2]=K_EN
                       [1]=OVUV_OK     [0]=ARM_PERMIT
  0x01  FAULT          [3:0]=FAULT_CODE
  0x02  STATE          [1:0]=STATE_CODE
  0x03  ACTIONS        [3]=dump_req [2]=DUMP_EN(active=dump&K_EN) [1]=PRECHARGE_REQ [0]=K_EN
  0x04  INPUTS         [7]=arm_ok  [6]=remote_arm_latch [5]=ESTOP_OK_S
                       [4]=MCU_CMD_ARM_S [3]=MCU_ALIVE_S  [2]=FB_CLOSED_S
                       [1]=PRECHARGE_OK_S [0]=VBUS_OV_S
  0x05  PCHG_TIMER_HI  high byte of t_precharge countdown (ms)
  0x06  PCHG_TIMER_LO  low byte of t_precharge countdown (ms)
  0x7F  VERSION        0xB2
--------------------------------------------------------------------------------
*/
/*
This version is built to work with the IPexpress-generated module:
    PDU_EFB_I2C.v

Key point:
- DO NOT instantiate the raw 'EFB' primitive yourself.
- Instantiate the generated wrapper module PDU_EFB_I2C, which exposes:
    wb_clk_i, wb_rst_i, wb_cyc_i, wb_stb_i, wb_we_i, wb_adr_i, wb_dat_i,
    wb_dat_o, wb_ack_o, i2c1_scl, i2c1_sda, i2c1_irqo

Your original discrete-glue intent (from pdu_glue.v) is preserved:
- OVUV_OK gating for ARM_PERMIT and COMPUTE_EN
- DUMP_EN request when VBUS_OV & contactor open & not precharging
- PRECHARGE_LATCH and FAULT_LATCH are internal state bits
- EDM weld blanking and close blanking as ms timers
- Precharge timeout window as ms timer
- Fault codes and state codes per your current mapping doc

I2C monitor protocol (simple):
- MCU writes 1 byte = register pointer
- MCU repeated-start reads N bytes = sequential register reads (auto-increment)
No writable registers implemented (monitor-only).

NOTE ON I2C ADDRESS:
- The I2C slave address is set inside IPexpress (PDU_EFB_I2C.v comment shows i2c1_sa=1010101).
- This HDL does not need to know the address.

BUILD REQUIREMENTS:
- Add BOTH files to Diamond project:
    - this pdu_glue.sv
    - PDU_EFB_I2C.v (your generated wrapper)
- Constrain i2c1_scl/i2c1_sda to pins 43/42 respectively (as you already did).
*/

module pdu_glue (
    // ---------- Inputs ----------
    input  wire SW_COMPUTE,
    input  wire SW_ARM,
    input  wire SW_RST_FAULT,
    input  wire SW_REMOTE_ARM,
    input  wire SW_REMOTE_DISARM /* synthesis syn_force_pads=1 */,
    input  wire ESTOP_OK,
    input  wire OV,
    input  wire UV,
    input  wire VBUS_OV,
    input  wire MCU_ALIVE,       // observability only — not in arm_ok or fault path
    input  wire MCU_CMD_ARM,     // soft arm permit from MCU (maintained)
    input  wire MCU_CMD_FAULT,   // hard fault injection from MCU (rising edge latches)
    input  wire PRECHARGE_OK,
    input  wire FB_CLOSED,

    // Hardened Primary I2C pins (must be top-level INOUT)
    inout  wire i2c1_scl,
    inout  wire i2c1_sda,

    // ---------- Outputs ----------
    output reg  FAULT_LATCH,
    output reg  PRECHARGE_LATCH,
    output reg  MOTOR_EN,
    output reg  COMPUTE_EN,
    output reg  K_SEL,
    output reg  K_EN,

    output reg  [3:0] FAULT_CODE,
    output reg  [1:0] STATE_CODE,
    output reg        OVUV_OK,
    output reg        ARM_PERMIT,
    output reg        PRECHARGE_REQ,
    output reg        DUMP_EN,
    output reg        SPARE_OUT_0,
    output reg        SPARE_OUT_1
);

    // =========================================================================
    // SECTION 1: CLOCK + 1ms TICK
    // =========================================================================

    // Utility: clog2 (integer log2 ceiling)
    function automatic integer clog2(input integer value);
        integer i;
        begin
            value = value - 1;
            for (i = 0; value > 0; i = i + 1) value = value >> 1;
            clog2 = i;
        end
    endfunction

    // Parameters — edit freely
    localparam string  OSCH_NOM_FREQ_MHZ = "2.08";
    localparam integer OSC_HZ            = 2080000;
    localparam integer OSC_DIV_1MS       = OSC_HZ / 1000;  // 2080 → 1 kHz tick

    // Timing constants (milliseconds)
    localparam integer T_KSEL_BLANK_MS   = 30;    // K_SEL relay changeover blank
    localparam integer T_BLANK_OPEN_MS   = 500;   // weld-detect inhibit after MOTOR_EN drops
    localparam integer T_BLANK_CLOSE_MS  = 500;   // close-verify window after MOTOR_EN asserts
    localparam integer T_PRECHG_MAX_MS   = 8000;  // precharge timeout
    localparam integer T_REMOTE_FILT_MS  = 2;     // remote receiver pulse qualify (≥2 ms)

    // =========================================================================
    // SECTION 1 (cont.): OSCH + 1ms tick
    // =========================================================================

    wire clk_osc;
    OSCH #(.NOM_FREQ(OSCH_NOM_FREQ_MHZ)) u_osch (
        .STDBY   (1'b0),
        .OSC     (clk_osc),
        .SEDSTDBY()
    );

    // 1 ms tick — fires for exactly one clk_osc cycle each millisecond
    logic [clog2(OSC_DIV_1MS)-1:0] div_cnt  = '0;
    logic                           tick_1ms = 1'b0;

    always_ff @(posedge clk_osc) begin
        tick_1ms <= 1'b0;
        if (div_cnt == OSC_DIV_1MS - 1) begin
            div_cnt  <= '0;
            tick_1ms <= 1'b1;
        end else begin
            div_cnt <= div_cnt + 1'b1;
        end
    end

    // Heartbeat toggle for SPARE_OUT_0: flips every 1 ms → 500 Hz square wave
    // Driven outside the tick-gated block so it is never stuck.
    logic spare_heartbeat = 1'b0;
    always_ff @(posedge clk_osc) begin
        if (tick_1ms) spare_heartbeat <= ~spare_heartbeat;
    end

    // =========================================================================
    // SECTION 2: INPUT SYNCHRONIZERS (2-FF metastability chains)
    // =========================================================================

    logic [1:0] sw_compute_ff  = 2'b00;
    logic [1:0] sw_arm_ff      = 2'b00;
    logic [1:0] sw_rst_ff      = 2'b00;
    logic [1:0] sw_rarm_ff     = 2'b00;
    logic [1:0] sw_rdisarm_ff  = 2'b00;
    logic [1:0] estop_ff       = 2'b00;
    logic [1:0] ov_ff          = 2'b00;
    logic [1:0] uv_ff          = 2'b00;
    logic [1:0] vbusov_ff      = 2'b00;
    logic [1:0] mcu_alive_ff   = 2'b00;
    logic [1:0] mcu_arm_ff     = 2'b00;
    logic [1:0] mcu_fault_ff   = 2'b00;
    logic [1:0] pchgok_ff      = 2'b00;
    logic [1:0] fb_closed_ff   = 2'b00;

    always_ff @(posedge clk_osc) begin
        sw_compute_ff  <= {sw_compute_ff[0],  SW_COMPUTE};
        sw_arm_ff      <= {sw_arm_ff[0],      SW_ARM};
        sw_rst_ff      <= {sw_rst_ff[0],      SW_RST_FAULT};
        sw_rarm_ff     <= {sw_rarm_ff[0],     SW_REMOTE_ARM};
        sw_rdisarm_ff  <= {sw_rdisarm_ff[0],  SW_REMOTE_DISARM};
        estop_ff       <= {estop_ff[0],       ESTOP_OK};
        ov_ff          <= {ov_ff[0],          OV};
        uv_ff          <= {uv_ff[0],          UV};
        vbusov_ff      <= {vbusov_ff[0],      VBUS_OV};
        mcu_alive_ff   <= {mcu_alive_ff[0],   MCU_ALIVE};
        mcu_arm_ff     <= {mcu_arm_ff[0],     MCU_CMD_ARM};
        mcu_fault_ff   <= {mcu_fault_ff[0],   MCU_CMD_FAULT};
        pchgok_ff      <= {pchgok_ff[0],      PRECHARGE_OK};
        fb_closed_ff   <= {fb_closed_ff[0],   FB_CLOSED};
    end

    wire SW_COMPUTE_S    = sw_compute_ff[1];
    wire SW_ARM_S        = sw_arm_ff[1];
    wire SW_RST_FAULT_S  = sw_rst_ff[1];
    wire SW_RARM_S       = sw_rarm_ff[1];
    wire SW_RDISARM_S    = sw_rdisarm_ff[1];
    wire ESTOP_OK_S      = estop_ff[1];
    wire OV_S            = ov_ff[1];
    wire UV_S            = uv_ff[1];
    wire VBUS_OV_S       = vbusov_ff[1];
    wire MCU_ALIVE_S     = mcu_alive_ff[1];
    wire MCU_CMD_ARM_S   = mcu_arm_ff[1];
    wire MCU_CMD_FAULT_S = mcu_fault_ff[1];
    wire PRECHARGE_OK_S  = pchgok_ff[1];
    wire FB_CLOSED_S     = fb_closed_ff[1];

    // =========================================================================
    // SECTION 3: EVENT GENERATION
    // =========================================================================

    // --- Remote arm/disarm: 2 ms pulse-qualify then rising-edge detect ---
    // The RF receiver gives a brief pulse on press; it has no meaningful
    // maintained state between events. We qualify for T_REMOTE_FILT_MS ticks
    // then detect the rising edge. A 2 ms filter easily passes any real button
    // press (receiver pulse is typically 50-200 ms).

    logic [clog2(T_REMOTE_FILT_MS+1)-1:0] rarm_filt    = '0;
    logic [clog2(T_REMOTE_FILT_MS+1)-1:0] rdisarm_filt = '0;
    logic rarm_qual    = 1'b0;  // qualified (filtered) remote arm signal
    logic rdisarm_qual = 1'b0;  // qualified remote disarm signal
    logic rarm_prev    = 1'b0;
    logic rdisarm_prev = 1'b0;

    always_ff @(posedge clk_osc) begin
        if (tick_1ms) begin
            // Increment filter counters while signal is high, reset when low
            if (SW_RARM_S) begin
                if (rarm_filt != T_REMOTE_FILT_MS)
                    rarm_filt <= rarm_filt + 1'b1;
            end else begin
                rarm_filt <= '0;
            end

            if (SW_RDISARM_S) begin
                if (rdisarm_filt != T_REMOTE_FILT_MS)
                    rdisarm_filt <= rdisarm_filt + 1'b1;
            end else begin
                rdisarm_filt <= '0;
            end

            rarm_qual    <= (rarm_filt    == T_REMOTE_FILT_MS);
            rdisarm_qual <= (rdisarm_filt == T_REMOTE_FILT_MS);
            rarm_prev    <= rarm_qual;
            rdisarm_prev <= rdisarm_qual;
        end
    end

    wire rarm_re    = rarm_qual    & ~rarm_prev;    // remote arm rising edge
    wire rdisarm_re = rdisarm_qual & ~rdisarm_prev; // remote disarm rising edge

    // --- SW_RST_FAULT rising edge ---
    logic sw_rst_prev = 1'b0;
    always_ff @(posedge clk_osc) begin
        if (tick_1ms) sw_rst_prev <= SW_RST_FAULT_S;
    end
    wire rst_fault_re = SW_RST_FAULT_S & ~sw_rst_prev;

    // --- MCU_CMD_FAULT rising edge ---
    logic mcu_fault_prev = 1'b0;
    always_ff @(posedge clk_osc) begin
        if (tick_1ms) mcu_fault_prev <= MCU_CMD_FAULT_S;
    end
    wire mcu_fault_re = MCU_CMD_FAULT_S & ~mcu_fault_prev;

    // --- remote_arm_latch ---
    // SET   by remote arm event
    // CLEAR by remote disarm event
    // CLEAR when SW_ARM_S drops (local disarm also clears remote)
    // CLEAR on fault entry (belt-and-suspenders, also cleared by state machine)
    logic remote_arm_latch = 1'b0;

    // Updated in Section 5 (state machine) to allow atomic clear on fault/reset.
    // Declared here so arm_ok can reference it combinationally.

    // =========================================================================
    // SECTION 4: ARM CONDITIONS (combinational)
    // =========================================================================

    // arm_ok: gates entry into PRECHARGE / ARMED from COMPUTE
    //   ~VBUS_OV_S: cannot start arming while dump would immediately activate.
    //   fault_latch_r used (not FAULT_LATCH) so arm drops the same tick a fault latches.
    wire arm_ok =   SW_ARM_S
                  & remote_arm_latch
                  & ESTOP_OK_S
                  & ~(OV_S | UV_S)
                  & ~VBUS_OV_S        // VBUS OV blocks entry
                  & MCU_CMD_ARM_S
                  & ~fault_latch_r;

    // arm_hold: gates staying in ARMED
    //   VBUS_OV_S deliberately excluded — dump may coexist with closed contactor
    //   fault_latch_r used for same-tick response
    wire arm_hold =  SW_ARM_S
                   & remote_arm_latch
                   & ESTOP_OK_S
                   & ~(OV_S | UV_S)
                   & MCU_CMD_ARM_S
                   & ~fault_latch_r;

    // Supply OV/UV combined flag
    wire ovuv_ok_w = ~(OV_S | UV_S);

    // =========================================================================
    // SECTION 5: MAIN STATE MACHINE + TIMERS
    // =========================================================================

    // State encoding
    typedef enum logic [2:0] {
        ST_IDLE      = 3'd0,
        ST_COMPUTE   = 3'd3,  // 2'd3 → STATE_CODE=3 on bench
        ST_PRECHARGE = 3'd1,
        ST_ARMED     = 3'd2,
        ST_FAULT     = 3'd4
    } state_t;

    state_t state = ST_IDLE;

    // Internal latch registers (driven by state machine)
    logic fault_latch_r     = 1'b0;
    logic precharge_latch_r = 1'b0;
    logic [3:0] fault_code_r = 4'h0;

    // Timers (ms downcounters, decremented every tick_1ms)
    logic [clog2(T_KSEL_BLANK_MS+1)-1:0]  t_ksel_blank  = '0;
    logic [clog2(T_BLANK_OPEN_MS+1)-1:0]  t_blank_open  = '0;
    logic [clog2(T_BLANK_CLOSE_MS+1)-1:0] t_blank_close = '0;
    logic [clog2(T_PRECHG_MAX_MS+1)-1:0]  t_precharge   = '0;

    wire ksel_blank_active  = (t_ksel_blank  != '0);
    wire blank_open_active  = (t_blank_open  != '0);
    wire blank_close_active = (t_blank_close != '0);

    // ---- FAULT_SET: any latching fault condition ----
    // weld: contactor reads closed when MOTOR_EN is off and open-blank has expired
    wire weld_detected   = FB_CLOSED_S & ~MOTOR_EN & ~blank_open_active;
    // close failure: close-verify window expired without FB_CLOSED
    wire no_close_fault  = (t_blank_close == '0) & ~FB_CLOSED_S & (state == ST_ARMED);
    // precharge timeout (only fires in PRECHARGE by construction)
    wire prechg_timeout  = (t_precharge == '0) & ~PRECHARGE_OK_S & (state == ST_PRECHARGE);

    wire fault_set =   ~ESTOP_OK_S      // code 1
                     | OV_S             // code 2
                     | UV_S             // code 3
                     | weld_detected    // code 4
                     | prechg_timeout   // code 5
                     | mcu_fault_re     // code 6
                     | no_close_fault;  // code 7

    // Priority-encoded fault code for the moment of fault entry
    function automatic logic [3:0] fault_code_enc;
        input logic estop, ov, uv, weld, pchg_to, mcu_f, no_close;
        if      (~estop)   fault_code_enc = 4'd1;
        else if (ov)       fault_code_enc = 4'd2;
        else if (uv)       fault_code_enc = 4'd3;
        else if (weld)     fault_code_enc = 4'd4;
        else if (pchg_to)  fault_code_enc = 4'd5;
        else if (mcu_f)    fault_code_enc = 4'd6;
        else if (no_close) fault_code_enc = 4'd7;
        else               fault_code_enc = 4'd15;
    endfunction

    // ---- Main state machine (all transitions on tick_1ms) ----
    always_ff @(posedge clk_osc) begin
        if (tick_1ms) begin

            // ---- Decrement all running timers ----
            if (t_ksel_blank  != '0) t_ksel_blank  <= t_ksel_blank  - 1'b1;
            if (t_blank_open  != '0) t_blank_open  <= t_blank_open  - 1'b1;
            if (t_blank_close != '0) t_blank_close <= t_blank_close - 1'b1;
            if (t_precharge   != '0) t_precharge   <= t_precharge   - 1'b1;

            // ---- K_SEL blank: load when ksel_next will change K_SEL ----
            // ksel_next is combinational (dump_req); K_SEL is the registered output.
            // Detect upcoming transition here so only one block drives t_ksel_blank.
            if (ksel_next != K_SEL) begin
                t_ksel_blank <= T_KSEL_BLANK_MS[clog2(T_KSEL_BLANK_MS+1)-1:0];
            end

            // ---- remote_arm_latch management ----
            if (rarm_re)
                remote_arm_latch <= 1'b1;
            if (rdisarm_re | ~SW_ARM_S)
                remote_arm_latch <= 1'b0;

            // ---- State transitions ----
            case (state)

                // ----------------------------------------------------------
                ST_IDLE: begin
                    if (fault_set) begin
                        state          <= ST_FAULT;
                        fault_latch_r  <= 1'b1;
                        fault_code_r   <= fault_code_enc(~ESTOP_OK_S, OV_S, UV_S,
                                              weld_detected, prechg_timeout,
                                              mcu_fault_re, no_close_fault);
                        remote_arm_latch <= 1'b0;
                    end else if (SW_COMPUTE_S) begin
                        state <= ST_COMPUTE;
                    end
                end

                // ----------------------------------------------------------
                ST_COMPUTE: begin
                    if (fault_set) begin
                        state          <= ST_FAULT;
                        fault_latch_r  <= 1'b1;
                        fault_code_r   <= fault_code_enc(~ESTOP_OK_S, OV_S, UV_S,
                                              weld_detected, prechg_timeout,
                                              mcu_fault_re, no_close_fault);
                        remote_arm_latch <= 1'b0;
                    end else if (~SW_COMPUTE_S) begin
                        state <= ST_IDLE;
                    end else if (arm_ok & PRECHARGE_OK_S) begin
                        // Bus already at voltage: skip precharge, close contactor
                        state             <= ST_ARMED;
                        precharge_latch_r <= 1'b0;
                        // start close-verify timer (MOTOR_EN set in output decode)
                        t_blank_close     <= T_BLANK_CLOSE_MS[clog2(T_BLANK_CLOSE_MS+1)-1:0];
                    end else if (arm_ok & ~PRECHARGE_OK_S) begin
                        state             <= ST_PRECHARGE;
                        precharge_latch_r <= 1'b1;
                        t_precharge       <= T_PRECHG_MAX_MS[clog2(T_PRECHG_MAX_MS+1)-1:0];
                        // K_SEL will go to 0 (precharge=NC); load ksel blank if it changes
                        // (handled in output decode section)
                    end
                end

                // ----------------------------------------------------------
                ST_PRECHARGE: begin
                    if (fault_set | prechg_timeout) begin
                        // prechg_timeout is already in fault_set but listed explicitly for clarity
                        state             <= ST_FAULT;
                        precharge_latch_r <= 1'b0;
                        fault_latch_r     <= 1'b1;
                        fault_code_r      <= fault_code_enc(~ESTOP_OK_S, OV_S, UV_S,
                                                weld_detected, prechg_timeout,
                                                mcu_fault_re, no_close_fault);
                        remote_arm_latch  <= 1'b0;
                        // MOTOR_EN was 0 throughout precharge; still start open-blank
                        // so weld detect is suppressed during any relay bounce
                        t_blank_open      <= T_BLANK_OPEN_MS[clog2(T_BLANK_OPEN_MS+1)-1:0];
                    end else if (~arm_ok) begin
                        // arm_ok includes ~VBUS_OV_S, so VBUS_OV during precharge
                        // aborts here; dump then activates orthogonally once back in COMPUTE
                        state             <= ST_COMPUTE;
                        precharge_latch_r <= 1'b0;
                        t_precharge       <= '0;
                    end else if (PRECHARGE_OK_S) begin
                        // Bus charged: close contactor
                        state             <= ST_ARMED;
                        precharge_latch_r <= 1'b0;
                        t_precharge       <= '0;
                        t_blank_close     <= T_BLANK_CLOSE_MS[clog2(T_BLANK_CLOSE_MS+1)-1:0];
                        // MOTOR_EN asserted in output decode; K_EN drops (handled there)
                    end
                end

                // ----------------------------------------------------------
                ST_ARMED: begin
                    if (fault_set) begin
                        state            <= ST_FAULT;
                        fault_latch_r    <= 1'b1;
                        fault_code_r     <= fault_code_enc(~ESTOP_OK_S, OV_S, UV_S,
                                               weld_detected, prechg_timeout,
                                               mcu_fault_re, no_close_fault);
                        remote_arm_latch <= 1'b0;
                        // MOTOR_EN drops in output decode; start open-blank now
                        t_blank_open     <= T_BLANK_OPEN_MS[clog2(T_BLANK_OPEN_MS+1)-1:0];
                    end else if (~arm_hold) begin
                        // Soft de-arm: MCU_CMD_ARM dropped, remote disarm, local disarm, OV/UV
                        // VBUS_OV does NOT drop arm_hold — dump can coexist with ARMED
                        state        <= ST_COMPUTE;
                        t_blank_open <= T_BLANK_OPEN_MS[clog2(T_BLANK_OPEN_MS+1)-1:0];
                    end
                    // VBUS_OV: orthogonal dump, state stays ST_ARMED
                end

                // ----------------------------------------------------------
                ST_FAULT: begin
                    // Dump remains orthogonally active in fault if VBUS_OV_S.
                    // Fault reset: all guards must be clear.
                    if (rst_fault_re
                            & ESTOP_OK_S
                            & ~SW_ARM_S
                            & ~remote_arm_latch
                            & ~MCU_CMD_ARM_S
                            & ~fault_set) begin
                        state            <= SW_COMPUTE_S ? ST_COMPUTE : ST_IDLE;
                        fault_latch_r    <= 1'b0;
                        fault_code_r     <= 4'h0;
                        remote_arm_latch <= 1'b0;
                    end
                end

                default: state <= ST_IDLE;

            endcase
        end
    end

    // =========================================================================
    // SECTION 6: DUMP LOGIC (orthogonal, combinational)
    // =========================================================================
    //
    // K_SEL polarity (MUST verify on bench before enabling K_EN):
    //   K_SEL = 0  →  relay de-energized  →  NC path  =  PRECHARGE resistor path
    //   K_SEL = 1  →  relay energized     →  NO path  =  DUMP / brake path
    //
    // Dump is active whenever VBUS_OV and we are NOT in PRECHARGE.
    // PRECHARGE state has exclusive use of the resistor via K_SEL=0 / NC path.
    // Abort of precharge on VBUS_OV (arm_ok goes false) ensures the state machine
    // leaves PRECHARGE before dump can assert.

    wire dump_req = VBUS_OV_S & (state != ST_PRECHARGE);

    // K_SEL combinational intent (before blanking)
    //   PRECHARGE: K_SEL=0 (NC = precharge path)
    //   dump_req:  K_SEL=1 (NO = dump path)
    //   otherwise: K_SEL=0 (de-energized / safe default)
    wire ksel_next = dump_req;  // 1 = dump, 0 = precharge/idle

    // ksel_changing: true on the tick K_SEL is about to change.
    // Used to guarantee K_EN stays low on the transition tick itself,
    // even before t_ksel_blank has been loaded and ksel_blank_active goes high.
    wire ksel_changing = (ksel_next != K_SEL);

    // K_EN active when resistor should be connected, subject to blank window.
    // Blocked by ksel_changing so K_EN cannot assert on the same tick K_SEL moves.
    wire precharge_active = (state == ST_PRECHARGE);
    wire ken_next = (precharge_active | dump_req) & ~ksel_blank_active & ~ksel_changing;

    // =========================================================================
    // SECTION 7: OUTPUT DECODE (all registered, updated every tick_1ms)
    // =========================================================================

    always_ff @(posedge clk_osc) begin
        if (tick_1ms) begin

            // ---- Register all outputs ----

            // COMPUTE_EN: always follows SW_COMPUTE — all states, including FAULT
            COMPUTE_EN    <= SW_COMPUTE_S;

            // MOTOR_EN: asserted only in ARMED state
            MOTOR_EN      <= (state == ST_ARMED);

            // Precharge / dump relay outputs (per Section 6)
            K_SEL         <= ksel_next;
            K_EN          <= ken_next;

            // Status latches
            FAULT_LATCH     <= fault_latch_r;
            PRECHARGE_LATCH <= precharge_latch_r;

            // Fault and state codes
            FAULT_CODE <= fault_code_r;

            case (state)
                ST_IDLE:      STATE_CODE <= 2'd0;
                ST_COMPUTE:   STATE_CODE <= 2'd3;
                ST_PRECHARGE: STATE_CODE <= 2'd1;
                ST_ARMED:     STATE_CODE <= 2'd2;
                ST_FAULT:     STATE_CODE <= 2'd0; // distinguished from IDLE by FAULT_LATCH pin
                default:      STATE_CODE <= 2'd0;
            endcase

            // Derived flag outputs
            OVUV_OK       <= ovuv_ok_w;
            ARM_PERMIT    <= arm_ok;
            PRECHARGE_REQ <= (state == ST_PRECHARGE);
            // DUMP_EN = dump physically active: VBUS_OV demanding dump AND relay
            // will be/is energised on dump path (not blanked, not switching).
            DUMP_EN       <= dump_req & ken_next;

            // Spare outputs: useful for bench scope probing
            SPARE_OUT_0 <= spare_heartbeat; // 500 Hz toggle — proves FPGA is alive
            SPARE_OUT_1 <= arm_ok;          // live arm permission wire
        end
    end

    // =========================================================================
    // SECTION 8: EFB / I2C WISHBONE ENGINE
    // (engine logic unchanged from verified implementation; reg_read expanded)
    // =========================================================================

    logic       wb_rst_i;
    logic       wb_cyc_i, wb_stb_i, wb_we_i;
    logic [7:0] wb_adr_i, wb_dat_i;
    wire  [7:0] wb_dat_o;
    wire        wb_ack_o;
    wire        i2c1_irqo;

    // Hold wb_rst_i asserted for a few ms after configuration
    logic [2:0] wb_rst_ms = 3'd5;
    always_ff @(posedge clk_osc) begin
        if (tick_1ms) begin
            if (wb_rst_ms != 0) wb_rst_ms <= wb_rst_ms - 1'b1;
        end
        wb_rst_i <= (wb_rst_ms != 0);
    end

    PDU_EFB_I2C u_efb_i2c (
        .wb_clk_i (clk_osc),
        .wb_rst_i (wb_rst_i),
        .wb_cyc_i (wb_cyc_i),
        .wb_stb_i (wb_stb_i),
        .wb_we_i  (wb_we_i),
        .wb_adr_i (wb_adr_i),
        .wb_dat_i (wb_dat_i),
        .wb_dat_o (wb_dat_o),
        .wb_ack_o (wb_ack_o),
        .i2c1_scl (i2c1_scl),
        .i2c1_sda (i2c1_sda),
        .i2c1_irqo(i2c1_irqo)
    );

    // ---- Wishbone transaction engine (single outstanding transaction) ----
    typedef enum logic [1:0] {WB_IDLE, WB_WAIT} wb_state_t;
    wb_state_t wb_state = WB_IDLE;

    logic       wb_req;
    logic       wb_req_we;
    logic [7:0] wb_req_adr;
    logic [7:0] wb_req_dat;
    logic       wb_resp_valid;
    logic [7:0] wb_resp_dat;

    always_ff @(posedge clk_osc) begin
        wb_resp_valid <= 1'b0;

        case (wb_state)
            WB_IDLE: begin
                wb_cyc_i <= 1'b0;
                wb_stb_i <= 1'b0;
                if (wb_req && !wb_rst_i) begin
                    wb_cyc_i <= 1'b1;
                    wb_stb_i <= 1'b1;
                    wb_we_i  <= wb_req_we;
                    wb_adr_i <= wb_req_adr;
                    wb_dat_i <= wb_req_dat;
                    wb_state <= WB_WAIT;
                end
            end
            WB_WAIT: begin
                if (wb_ack_o) begin
                    wb_cyc_i      <= 1'b0;
                    wb_stb_i      <= 1'b0;
                    wb_resp_dat   <= wb_dat_o;
                    wb_resp_valid <= 1'b1;
                    wb_state      <= WB_IDLE;
                end
            end
            default: wb_state <= WB_IDLE;
        endcase
    end

    // ---- I2C register map ----
    localparam logic [7:0] I2C1_TXDR = 8'h44;
    localparam logic [7:0] I2C1_SR   = 8'h45;
    localparam logic [7:0] I2C1_RXDR = 8'h47;

    localparam integer SR_BUSY  = 6;
    localparam integer SR_SRW   = 4;
    localparam integer SR_TRRDY = 2;

    function automatic logic [7:0] reg_read(input logic [7:0] a);
        case (a)
            // 0x00 STATUS0
            8'h00: reg_read = {FAULT_LATCH, PRECHARGE_LATCH, MOTOR_EN, COMPUTE_EN,
                               K_SEL, K_EN, OVUV_OK, ARM_PERMIT};
            // 0x01 FAULT
            8'h01: reg_read = {4'h0, FAULT_CODE};
            // 0x02 STATE
            8'h02: reg_read = {6'h00, STATE_CODE};
            // 0x03 ACTIONS
            // [3]=dump_req  VBUS_OV demanding dump (may still be blanked)
            // [2]=DUMP_EN   dump physically active (relay on dump path, not blanked)
            // [1]=PRECHARGE_REQ  precharge state active
            // [0]=K_EN      relay enable output (cross-check)
            8'h03: reg_read = {4'h0, dump_req, DUMP_EN, PRECHARGE_REQ, K_EN};
            // 0x04 INPUTS
            8'h04: reg_read = {arm_ok, remote_arm_latch, ESTOP_OK_S, MCU_CMD_ARM_S,
                               MCU_ALIVE_S, FB_CLOSED_S, PRECHARGE_OK_S, VBUS_OV_S};
            // 0x05/0x06 Precharge countdown timer (ms), high/low bytes
            8'h05: reg_read = t_precharge[clog2(T_PRECHG_MAX_MS+1)-1:8];
            8'h06: reg_read = t_precharge[7:0];
            // 0x7F Version
            8'h7F: reg_read = 8'hB2;
            default: reg_read = 8'h00;
        endcase
    endfunction

    // ---- I2C mailbox handler ----
    typedef enum logic [3:0] {
        I2C_IDLE   = 4'd0,
        I2C_RD_SR0,
        I2C_WAIT_SR0,
        I2C_RD_SR1,
        I2C_WAIT_SR1,
        I2C_RD_RX,
        I2C_WAIT_RX,
        I2C_WR_TX,
        I2C_WAIT_TX
    } i2c_state_t;

    i2c_state_t i2c_state = I2C_IDLE;

    logic [7:0] sr0 = 8'h00, sr1 = 8'h00;
    logic       prev_srw   = 1'b0;
    logic [7:0] reg_ptr    = 8'h00;
    logic [7:0] tx_ptr     = 8'h00;
    logic       expect_ptr = 1'b1;

    // Combinational WB request mux
    always_comb begin
        wb_req     = 1'b0;
        wb_req_we  = 1'b0;
        wb_req_adr = 8'h00;
        wb_req_dat = 8'h00;

        unique case (i2c_state)
            I2C_RD_SR0: begin wb_req = 1'b1; wb_req_adr = I2C1_SR;   end
            I2C_RD_SR1: begin wb_req = 1'b1; wb_req_adr = I2C1_SR;   end
            I2C_RD_RX:  begin wb_req = 1'b1; wb_req_adr = I2C1_RXDR; end
            I2C_WR_TX:  begin wb_req = 1'b1; wb_req_we  = 1'b1;
                              wb_req_adr = I2C1_TXDR;
                              wb_req_dat = reg_read(tx_ptr); end
            default: ;
        endcase
    end

    always_ff @(posedge clk_osc) begin
        if (wb_rst_i) begin
            i2c_state  <= I2C_IDLE;
            prev_srw   <= 1'b0;
            expect_ptr <= 1'b1;
            reg_ptr    <= 8'h00;
            tx_ptr     <= 8'h00;
            sr0        <= 8'h00;
            sr1        <= 8'h00;
        end else begin
            case (i2c_state)

                I2C_IDLE: begin
                    if (i2c1_irqo || tick_1ms) i2c_state <= I2C_RD_SR0;
                end

                I2C_RD_SR0: begin
                    if (wb_state == WB_WAIT) i2c_state <= I2C_WAIT_SR0;
                end

                I2C_WAIT_SR0: begin
                    if (wb_resp_valid) begin
                        sr0 <= wb_resp_dat;
                        if (!wb_resp_dat[SR_BUSY]) begin
                            expect_ptr <= 1'b1;
                            prev_srw   <= 1'b0;
                        end
                        if (wb_resp_dat[SR_TRRDY])
                            i2c_state <= I2C_RD_SR1;
                        else
                            i2c_state <= I2C_IDLE;
                    end
                end

                I2C_RD_SR1: begin
                    if (wb_state == WB_WAIT) i2c_state <= I2C_WAIT_SR1;
                end

                I2C_WAIT_SR1: begin
                    if (wb_resp_valid) begin
                        sr1 <= wb_resp_dat;
                        if (wb_resp_dat[SR_BUSY] && !prev_srw && wb_resp_dat[SR_SRW])
                            tx_ptr <= reg_ptr;
                        prev_srw <= wb_resp_dat[SR_SRW];
                        if (wb_resp_dat[SR_SRW] == 1'b0)
                            i2c_state <= I2C_RD_RX;
                        else
                            i2c_state <= I2C_WR_TX;
                    end
                end

                I2C_RD_RX: begin
                    if (wb_state == WB_WAIT) i2c_state <= I2C_WAIT_RX;
                end

                I2C_WAIT_RX: begin
                    if (wb_resp_valid) begin
                        if (expect_ptr) begin
                            reg_ptr    <= wb_resp_dat;
                            expect_ptr <= 1'b0;
                        end
                        i2c_state <= I2C_IDLE;
                    end
                end

                I2C_WR_TX: begin
                    if (wb_state == WB_WAIT) i2c_state <= I2C_WAIT_TX;
                end

                I2C_WAIT_TX: begin
                    if (wb_resp_valid) begin
                        tx_ptr    <= tx_ptr + 1'b1;
                        i2c_state <= I2C_IDLE;
                    end
                end

                default: i2c_state <= I2C_IDLE;
            endcase
        end
    end

endmodule

default_nettype wire
