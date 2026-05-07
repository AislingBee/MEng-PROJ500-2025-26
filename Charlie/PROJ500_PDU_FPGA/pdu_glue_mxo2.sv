`timescale 1ns/1ps
`default_nettype none
// =============================================================================
// Project  : PROJ500 â€” Humanoid Robot Power Distribution Unit
// File     : pdu_glue_mxo2.sv
// Device   : Lattice MachXO2-256HC-5SG48C  (SG48C QFN48 package)
// Clock    : Internal OSCH @ 2.08 MHz (period 480.77 ns).
//            OSC_DIV_1MS = 2080 â†’ one 1 ms tick every 2080 clock cycles.
// Purpose  : PDU sequencing state machine + hardened I2C monitoring interface.
//
// This module implements the full relay sequencing logic for the PDU main
// contactor and bus dump circuit, and exposes the internal state to the
// supervisory MCU via a read-only I2C slave (implemented via the MachXO2
// embedded function block â€” EFB).
//
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
// RELAY HARDWARE INTERFACE
//
//   The single relay is driven by two output signals:
//     K_SEL  â€” selects which resistor path the relay connects
//     K_EN   â€” energises the relay coil
//
//   K_SEL = 0  â†’  relay de-energised  â†’  NC contact active  â†’  PRECHARGE path
//                 (series precharge resistor limits inrush current)
//   K_SEL = 1  â†’  relay energised     â†’  NO contact active  â†’  DUMP path
//                 (dump / regenerative braking resistor)
//
//   NOTE: Verify K_SEL polarity on the bench before enabling K_EN for the
//   first time.  A polarity error will connect the dump resistor during
//   precharge (inadequate current limiting) or the precharge resistor during
//   dump (insufficient energy dissipation).
//
//   K_EN is suppressed for T_KSEL_BLANK_MS (30 ms) after any K_SEL transition.
//   This dead-time protects the relay coil from inrush current spikes and
//   back-EMF transients during changeover.
//
//   The main motor contactor is driven directly by MOTOR_EN.  K_SEL / K_EN
//   are separate and operate orthogonally to the contactor.
//
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
// STATE MACHINE OVERVIEW
//
//   Six states.  Transitions only on the 1 ms tick (all registered outputs
//   therefore update at 1 ms granularity; worst-case 2 ms after an input edge).
//
//   STATE_CODE is the value driven on the 2-bit STATE_CODE output (and I2C
//   register 0x02).  ST_IDLE and ST_FAULT share STATE_CODE=0; they are
//   distinguished by the FAULT_LATCH output.
//
//   â”Œâ”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”¬â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”¬â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”
//   â”‚ State               â”‚ STATE_CODE â”‚ Description                          â”‚
//   â”œâ”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”¼â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”¼â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”¤
//   â”‚ ST_IDLE             â”‚     0      â”‚ SW_COMPUTE=0; all outputs de-asserted â”‚
//   â”‚ ST_COMPUTE          â”‚     3      â”‚ SW_COMPUTE=1; awaiting arm permission â”‚
//   â”‚ ST_PRECHARGE        â”‚     1      â”‚ Bus charging via NC precharge path    â”‚
//   â”‚ ST_PRECHARGE_ABORT  â”‚     1      â”‚ Relay hold after precharge abort      â”‚
//   â”‚ ST_ARMED            â”‚     2      â”‚ Main contactor closed; motor active   â”‚
//   â”‚ ST_FAULT            â”‚     0      â”‚ Latched fault; requires manual reset  â”‚
//   â””â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”´â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”´â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”˜
//
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
// PRECHARGE ABORT HOLD (ST_PRECHARGE_ABORT)
//
//   When precharge is interrupted â€” either by a fault or by the operator
//   dropping SW_ARM â€” the relay coil must not be de-energised immediately.
//   Opening the relay under load current (bus not yet discharged) causes a
//   dry-break arc across the NC contacts, which degrades or destroys them.
//
//   The PRECHARGE_ABORT state holds the relay energised until one of:
//     (a) pchgok_filt asserts (bus voltage has equalised â€” safe to open), AND
//         T_PRECHG_ABORT_MIN_MS has elapsed since entering abort (prevents
//         premature exit on a stale filtered pchgok signal); or
//     (b) T_PRECHG_ABORT_HOLD_MS hard timeout expires regardless of pchgok
//         (failsafe â€” relay will eventually open even if comparator fails).
//
//   During abort-hold, K_EN remains asserted and PRECHARGE_REQ remains
//   asserted (both relay and MCU indication show precharge still in progress).
//   The dump path is suppressed during abort-hold (precharge_active gates
//   dump_req) because the relay is already closed on the precharge path.
//
//   If a fault fires during the abort-hold, FAULT_LATCH is latched
//   immediately, but the relay is NOT opened until the exit condition above
//   is met.  The destination on exit becomes ST_FAULT in this case.
//
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
// DUMP LOGIC
//
//   dump_req = vbusov_filt  &  ~precharge_active
//
//   VBUS_OV is the bus overvoltage comparator input (separate from the supply
//   OV comparator OV).  When the bus voltage exceeds the VBUS_OV threshold,
//   the dump resistor is connected to absorb the excess energy.
//
//   Dump is orthogonal to the arm state machine: it can coexist with
//   ST_ARMED (contactor closed; dump resistor connected in parallel with
//   motor load) or ST_FAULT (dump may remain active until bus discharges).
//
//   Dump is inhibited during precharge and abort-hold (precharge_active=1)
//   because the relay is already on the precharge/NC path.  Dump re-activates
//   automatically once the state machine exits precharge_active.
//
//   VBUS_OV does NOT block arm entry (arm_ok).  The threshold should be
//   trimmed well above the source OV threshold so that normal regenerative
//   energy return does not prevent arming.
//
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
// FAULT CODES
//
//   â”Œâ”€â”€â”€â”€â”€â”€â”¬â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”
//   â”‚ Code â”‚ Cause                                                           â”‚
//   â”œâ”€â”€â”€â”€â”€â”€â”¼â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”¤
//   â”‚  0   â”‚ No fault (reset value)                                          â”‚
//   â”‚  1   â”‚ ESTOP_OK de-asserted (emergency stop)                           â”‚
//   â”‚  2   â”‚ OV â€” supply overvoltage (comparator filtered)                   â”‚
//   â”‚  3   â”‚ UV â€” supply undervoltage (comparator filtered)                  â”‚
//   â”‚  4   â”‚ Weld detected â€” FB_CLOSED high while MOTOR_EN low (if enabled)  â”‚
//   â”‚  5   â”‚ Precharge timeout â€” bus did not reach threshold in T_PRECHG_MAX â”‚
//   â”‚  6   â”‚ MCU_CMD_FAULT rising edge â€” software-injected fault             â”‚
//   â”‚  7   â”‚ Close verify failure â€” FB_CLOSED never asserted (if enabled)    â”‚
//   â”‚ 15   â”‚ Unknown / multiple simultaneous (fault_code_enc default branch)  â”‚
//   â””â”€â”€â”€â”€â”€â”€â”´â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”˜
//
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
// I2C INTERFACE (read-only slave; address set in IPexpress â€” see PDU_EFB_I2C.v)
//
//   Protocol: MCU writes 1-byte register pointer; MCU repeated-start reads
//   N bytes.  The pointer auto-increments on each read byte, enabling burst
//   reads of the full register file.  No write registers are implemented.
//
//   Register map:
//   â”Œâ”€â”€â”€â”€â”€â”€â”¬â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”¬â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”
//   â”‚ Addr â”‚ Name             â”‚ Bit layout                                   â”‚
//   â”œâ”€â”€â”€â”€â”€â”€â”¼â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”¼â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”¤
//   â”‚ 0x00 â”‚ STATUS0          â”‚ [7]=FAULT_LATCH [6]=PRECHARGE_LATCH          â”‚
//   â”‚      â”‚                  â”‚ [5]=MOTOR_EN    [4]=COMPUTE_EN               â”‚
//   â”‚      â”‚                  â”‚ [3]=K_SEL       [2]=K_EN                     â”‚
//   â”‚      â”‚                  â”‚ [1]=OVUV_OK     [0]=ARM_PERMIT               â”‚
//   â”‚ 0x01 â”‚ FAULT            â”‚ [3:0]=FAULT_CODE                             â”‚
//   â”‚ 0x02 â”‚ STATE            â”‚ [1:0]=STATE_CODE                             â”‚
//   â”‚ 0x03 â”‚ ACTIONS          â”‚ [3]=dump_req  [2]=DUMP_EN                    â”‚
//   â”‚      â”‚                  â”‚ [1]=PRECHARGE_REQ  [0]=K_EN                  â”‚
//   â”‚ 0x04 â”‚ INPUTS           â”‚ [7]=arm_ok  [6]=remote_arm_latch             â”‚
//   â”‚      â”‚                  â”‚ [5]=ESTOP_OK_S  [4]=MCU_CMD_ARM_S            â”‚
//   â”‚      â”‚                  â”‚ [3]=MCU_ALIVE_S  [2]=FB_CLOSED_S             â”‚
//   â”‚      â”‚                  â”‚ [1]=PRECHARGE_OK_S  [0]=VBUS_OV_S            â”‚
//   â”‚ 0x05 â”‚ PCHG_TIMER_HI   â”‚ t_precharge[msb:8] countdown (ms)            â”‚
//   â”‚ 0x06 â”‚ PCHG_TIMER_LO   â”‚ t_precharge[7:0]                             â”‚
//   â”‚ 0x7F â”‚ VERSION          â”‚ 0xB2                                         â”‚
//   â””â”€â”€â”€â”€â”€â”€â”´â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”´â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”˜
//
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
// CONTACTOR AUXILIARY CONTACTS (ENABLE_FB_CLOSED)
//
//   Set ENABLE_FB_CLOSED = 1 when the main contactor has auxiliary contacts
//   wired to FB_CLOSED.  This enables:
//     - Close-verify: FAULT if FB_CLOSED does not assert within T_BLANK_CLOSE_MS
//       after MOTOR_EN rises (code 7).
//     - Weld detection: FAULT if FB_CLOSED asserts while MOTOR_EN is low and
//       the weld-detect blank has expired (code 4).
//
//   Set ENABLE_FB_CLOSED = 0 (current hardware revision) when the contactor
//   has no auxiliary contacts.  FB_CLOSED input is ignored entirely.
//
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
// BUILD INSTRUCTIONS
//
//   1. Add to Lattice Diamond project (Implementation > Source Files):
//        - pdu_glue_mxo2.sv     (this file â€” set as top-level module)
//        - PDU_EFB_I2C.v        (IPexpress-generated EFB wrapper)
//   2. Ensure the LPF contains the I2C pin constraints:
//        LOCATE COMP "i2c1_scl" SITE "43";
//        LOCATE COMP "i2c1_sda" SITE "42";
//   3. Do NOT instantiate the raw EFB primitive directly.  Use the generated
//      wrapper PDU_EFB_I2C which exposes the Wishbone bus interface.
//   4. Simulation: compile with +define+SIM to substitute sim_clk for OSCH.
//        vlog -sv -mfcu +define+SIM -work work pdu_glue_mxo2.sv tb_pdu_glue.sv
//
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
// REVISION HISTORY
//
//   2026-04-21  Initial implementation â€” state machine, dump, I2C, EFB.
//   2026-05-06  Added ST_PRECHARGE_ABORT state (abort-hold).
//               Corrected dump_req to use vbusov_filt (filtered comparator).
//               Removed VBUS_OV from arm_ok â€” dump is fully orthogonal.
//               Tightened remote_arm_latch SET guard to ST_COMPUTE only.
//               Full comment overhaul for project handover.
// =============================================================================

module pdu_glue (

    // -------------------------------------------------------------------------
    // Clock input (simulation only)
    // In hardware the OSCH primitive is used (see Section 1).
    // -------------------------------------------------------------------------
`ifdef SIM
    input  wire sim_clk,
`endif

    // -------------------------------------------------------------------------
    // Operator interface inputs
    // -------------------------------------------------------------------------
    input  wire SW_COMPUTE,          // Compute enable switch (maintained)
    input  wire SW_ARM,              // Arm switch (maintained)
    input  wire SW_RST_FAULT,        // Fault reset pushbutton (momentary, active-high)
    input  wire SW_REMOTE_ARM,       // Remote arm receiver output (pulse, active-high)
    input  wire SW_REMOTE_DISARM /* synthesis syn_force_pads=1 */,
                                     // Remote disarm receiver output (pulse, active-high)
                                     // syn_force_pads prevents Diamond merging this with
                                     // SW_REMOTE_ARM if both are tied to the same net

    // -------------------------------------------------------------------------
    // Safety and status inputs
    // -------------------------------------------------------------------------
    input  wire ESTOP_OK,            // Emergency stop chain healthy (active-high)
    input  wire OV,                  // Supply overvoltage comparator (active-high)
    input  wire UV,                  // Supply undervoltage comparator (active-high)
    input  wire VBUS_OV,             // Bus overvoltage comparator â€” triggers dump (active-high)
    input  wire MCU_ALIVE,           // MCU heartbeat â€” observability only;
                                     // not used in any fault or arm logic
    input  wire MCU_CMD_ARM,         // MCU arm permission (maintained; AND'd into arm_ok)
    input  wire MCU_CMD_FAULT,       // MCU fault injection (rising edge latches fault code 6)
    input  wire PRECHARGE_OK,        // Bus voltage at threshold â€” precharge complete
    input  wire FB_CLOSED,           // Main contactor auxiliary contact (active-high when closed)
                                     // Only used when ENABLE_FB_CLOSED = 1

    // -------------------------------------------------------------------------
    // Hardened I2C bus pins
    // Must remain at the top level of the design hierarchy; Diamond maps these
    // directly to the EFB I2C hard IP inside the MachXO2.
    // -------------------------------------------------------------------------
    inout  wire i2c1_scl,
    inout  wire i2c1_sda,

    // -------------------------------------------------------------------------
    // Status outputs (registered, updated every 1 ms tick)
    // -------------------------------------------------------------------------
    output reg  FAULT_LATCH,         // Latched fault active (cleared by SW_RST_FAULT)
    output reg  PRECHARGE_LATCH,     // Precharge in progress (cleared on exit)
    output reg  MOTOR_EN,            // Main contactor drive (high = contactor closed)
    output reg  COMPUTE_EN,          // Follows SW_COMPUTE; high in all states incl. FAULT

    // -------------------------------------------------------------------------
    // Relay drive outputs
    // -------------------------------------------------------------------------
    output reg  K_SEL,               // Relay path select: 0=NC/precharge, 1=NO/dump
    output reg  K_EN,                // Relay coil enable (blanked 30 ms on K_SEL change)

    // -------------------------------------------------------------------------
    // Telemetry and diagnostic outputs
    // -------------------------------------------------------------------------
    output reg  [3:0] FAULT_CODE,    // Encoded fault source (see fault code table above)
    output reg  [1:0] STATE_CODE,    // Encoded state (see state machine table above)
    output reg        OVUV_OK,       // High when neither OV nor UV is filtered-active
    output reg        ARM_PERMIT,    // High when all arm_ok conditions are satisfied
    output reg        PRECHARGE_REQ, // High during ST_PRECHARGE and ST_PRECHARGE_ABORT
    output reg        DUMP_EN,       // High when dump is physically active (K_SEL=1, K_EN=1)
    output reg        SPARE_OUT_0,   // 500 Hz heartbeat toggle â€” proves FPGA is running
    output reg        SPARE_OUT_1 /* synthesis syn_preserve=1 */
                                     // Live arm_ok signal â€” useful for scope probing;
                                     // syn_preserve prevents optimisation away
);

    // =========================================================================
    // SECTION 1: CLOCK AND 1 ms TICK GENERATION
    // =========================================================================

    // Integer ceiling log2 â€” used to size timer registers automatically.
    // Example: clog2(2080) = 11, giving an 11-bit counter for OSC_DIV_1MS.
    function automatic integer clog2(input integer value);
        integer i;
        begin
            value = value - 1;
            for (i = 0; value > 0; i = i + 1) value = value >> 1;
            clog2 = i;
        end
    endfunction

    // â”€â”€ Oscillator parameters â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    localparam string  OSCH_NOM_FREQ_MHZ = "2.08";
    localparam integer OSC_HZ            = 2_080_000;
    localparam integer OSC_DIV_1MS       = OSC_HZ / 1000;  // 2080 counts per ms

    // â”€â”€ Timing constants (all in milliseconds) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    //
    //   T_KSEL_BLANK_MS      Relay dead-time after K_SEL changes.  The relay
    //                        coil takes ~15 ms to change state; 30 ms gives
    //                        margin for contact bounce and back-EMF decay.
    //
    //   T_BLANK_OPEN_MS      Weld-detect inhibit window after MOTOR_EN falls.
    //                        Prevents auxiliary contact bounce from triggering
    //                        a false weld detection.
    //
    //   T_BLANK_CLOSE_MS     Close-verify window after MOTOR_EN rises.  If
    //                        FB_CLOSED does not assert within this window, a
    //                        close-verify fault is declared (code 7).
    //
    //   T_PRECHG_MAX_MS      Maximum precharge duration.  If PRECHARGE_OK has
    //                        not asserted by this deadline, a precharge timeout
    //                        fault is declared (code 5) and abort-hold begins.
    //
    //   T_PRECHG_ABORT_MIN_MS  Minimum time the relay is held closed after
    //                        precharge is aborted.  Prevents premature opening
    //                        on a stale (pre-abort) pchgok_filt assertion.
    //
    //   T_PRECHG_ABORT_HOLD_MS  Hard failsafe timeout for abort-hold.  If the
    //                        bus has not equalised by this deadline, the relay
    //                        opens regardless.  The operator must investigate.
    //
    //   T_REMOTE_FILT_MS     Minimum pulse width to qualify a remote arm or
    //                        disarm event.  Rejects receiver glitches.
    //
    //   T_DEBOUNCE_RST_MS    Mechanical debounce period for SW_RST_FAULT.
    //                        The button must be held stable for this many ms
    //                        before a reset is accepted.
    //
    //   T_COMP_FILT_MS       Up/down counter depth for comparator inputs.
    //                        An input must be stable for this many consecutive
    //                        ticks to change the filtered output state.
    //
    localparam integer T_KSEL_BLANK_MS        = 30;
    localparam integer T_BLANK_OPEN_MS        = 500;
    localparam integer T_BLANK_CLOSE_MS       = 500;
    localparam integer T_PRECHG_MAX_MS        = 8000;
    localparam integer T_PRECHG_ABORT_MIN_MS  = 100;
    localparam integer T_PRECHG_ABORT_HOLD_MS = 300;
    localparam integer T_REMOTE_FILT_MS       = 2;
    localparam integer T_DEBOUNCE_RST_MS      = 5;
    localparam integer T_COMP_FILT_MS         = 3;

    // â”€â”€ Contactor auxiliary contact feature flag â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    // Set to 1 when FB_CLOSED is wired to auxiliary contacts on the main
    // contactor to enable weld detection and close-verify.
    // Set to 0 (current hardware revision) â€” FB_CLOSED input is ignored.
    localparam bit ENABLE_FB_CLOSED = 0;

    // â”€â”€ Clock source â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    // In simulation the OSCH primitive is not available; sim_clk is used
    // directly.  Compile with +define+SIM to activate this path.
    wire clk_osc;
`ifdef SIM
    assign clk_osc = sim_clk;
`else
    OSCH #(.NOM_FREQ(OSCH_NOM_FREQ_MHZ)) u_osch (
        .STDBY   (1'b0),
        .OSC     (clk_osc),
        .SEDSTDBY()
    );
`endif

    // â”€â”€ 1 ms tick generator â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    // tick_1ms is high for exactly one clk_osc cycle every OSC_DIV_1MS cycles.
    // All timers, filters, and the state machine are gated on this signal so
    // that all time constants are expressed in round millisecond values.
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

    // â”€â”€ Heartbeat for SPARE_OUT_0 â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    // Toggles at 500 Hz (one flip per ms tick) as a live indicator that the
    // FPGA is operational.  Runs independently of the state machine.
    logic spare_heartbeat = 1'b0;
    always_ff @(posedge clk_osc) begin
        if (tick_1ms) spare_heartbeat <= ~spare_heartbeat;
    end

    // =========================================================================
    // SECTION 2: INPUT SYNCHRONISERS
    // =========================================================================
    //
    // All asynchronous inputs pass through a 2-stage flip-flop synchroniser
    // before entering any combinational or sequential logic.  This eliminates
    // metastability at the cost of up to 2 clock cycles (~1 Âµs) of latency,
    // which is negligible relative to the 1 ms tick period.
    //
    // Signal pipeline latency summary (worst case, input edge to registered output):
    //   Async input â†’ 2-FF sync (~1 Âµs) â†’ up/down filter (T_COMP_FILT_MS ticks
    //   to assert) â†’ state machine tick (1 ms) â†’ output register (1 ms)
    //   = T_COMP_FILT_MS + 2 ms minimum before a filtered fault reaches output.
    //   For T_COMP_FILT_MS=3: minimum 5 ms; practical test margin 6â€“8 ms.
    //
    //   ESTOP_OK has no comparator filter (only the 2-FF sync), so ESTOP faults
    //   appear at the output within ~2 ms of the input edge.

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

    // Synchronised signal aliases (suffix _S = synchronised)
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
    // SECTION 3: COMPARATOR GLITCH FILTERS
    // =========================================================================
    //
    // OV, UV, PRECHARGE_OK, and VBUS_OV are driven by analogue comparators
    // with no hardware hysteresis.  A T_COMP_FILT_MS-deep up/down counter
    // on each signal implements digital hysteresis:
    //
    //   - Counter increments each tick while the synchronised input is high.
    //   - Counter decrements each tick while the synchronised input is low.
    //   - Filtered output asserts when counter reaches T_COMP_FILT_MS (3).
    //   - Filtered output de-asserts when counter returns to 0.
    //
    // T_COMP_FILT_MS=3 consecutive agreeing ticks are required to change
    // state, preventing comparator chatter from causing spurious faults or
    // relay events.

    logic [clog2(T_COMP_FILT_MS+1)-1:0] ov_cnt     = '0;
    logic [clog2(T_COMP_FILT_MS+1)-1:0] uv_cnt     = '0;
    logic [clog2(T_COMP_FILT_MS+1)-1:0] pchgok_cnt = '0;
    logic [clog2(T_COMP_FILT_MS+1)-1:0] vbusov_cnt = '0;
    logic ov_filt     = 1'b0;
    logic uv_filt     = 1'b0;
    logic pchgok_filt = 1'b0;
    logic vbusov_filt = 1'b0;

    always_ff @(posedge clk_osc) begin
        if (tick_1ms) begin
            // OV filter
            if (OV_S && ov_cnt != T_COMP_FILT_MS)        ov_cnt <= ov_cnt + 1'b1;
            else if (!OV_S && ov_cnt != '0)               ov_cnt <= ov_cnt - 1'b1;
            if (ov_cnt == T_COMP_FILT_MS) ov_filt     <= 1'b1;
            if (ov_cnt == '0)             ov_filt     <= 1'b0;

            // UV filter
            if (UV_S && uv_cnt != T_COMP_FILT_MS)        uv_cnt <= uv_cnt + 1'b1;
            else if (!UV_S && uv_cnt != '0)               uv_cnt <= uv_cnt - 1'b1;
            if (uv_cnt == T_COMP_FILT_MS) uv_filt     <= 1'b1;
            if (uv_cnt == '0)             uv_filt     <= 1'b0;

            // PRECHARGE_OK filter
            if (PRECHARGE_OK_S && pchgok_cnt != T_COMP_FILT_MS) pchgok_cnt <= pchgok_cnt + 1'b1;
            else if (!PRECHARGE_OK_S && pchgok_cnt != '0)        pchgok_cnt <= pchgok_cnt - 1'b1;
            if (pchgok_cnt == T_COMP_FILT_MS) pchgok_filt <= 1'b1;
            if (pchgok_cnt == '0)             pchgok_filt <= 1'b0;

            // VBUS_OV filter
            if (VBUS_OV_S && vbusov_cnt != T_COMP_FILT_MS) vbusov_cnt <= vbusov_cnt + 1'b1;
            else if (!VBUS_OV_S && vbusov_cnt != '0)        vbusov_cnt <= vbusov_cnt - 1'b1;
            if (vbusov_cnt == T_COMP_FILT_MS) vbusov_filt <= 1'b1;
            if (vbusov_cnt == '0)             vbusov_filt <= 1'b0;
        end
    end

    // =========================================================================
    // SECTION 4: EVENT GENERATION
    // =========================================================================

    // â”€â”€ Remote arm / disarm pulse qualification â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    //
    // The RF remote receiver asserts a pulse for each button press.  Pulses
    // are typically 50â€“200 ms long; the T_REMOTE_FILT_MS=2 ms filter easily
    // captures any genuine press while rejecting sub-2 ms glitches.
    //
    // A rising edge is detected on the qualified signal (rarm_qual, rdisarm_qual)
    // rather than on the raw synchronised input, so the event fires once at the
    // start of a qualified pulse regardless of how long the button is held.

    logic [clog2(T_REMOTE_FILT_MS+1)-1:0] rarm_filt    = '0;
    logic [clog2(T_REMOTE_FILT_MS+1)-1:0] rdisarm_filt = '0;
    logic rarm_qual    = 1'b0;
    logic rdisarm_qual = 1'b0;
    logic rarm_prev    = 1'b0;
    logic rdisarm_prev = 1'b0;

    always_ff @(posedge clk_osc) begin
        if (tick_1ms) begin
            if (SW_RARM_S)    rarm_filt    <= (rarm_filt    < T_REMOTE_FILT_MS) ? rarm_filt    + 1'b1 : rarm_filt;
            else               rarm_filt    <= '0;
            if (SW_RDISARM_S) rdisarm_filt <= (rdisarm_filt < T_REMOTE_FILT_MS) ? rdisarm_filt + 1'b1 : rdisarm_filt;
            else               rdisarm_filt <= '0;

            rarm_qual    <= (rarm_filt    == T_REMOTE_FILT_MS);
            rdisarm_qual <= (rdisarm_filt == T_REMOTE_FILT_MS);
            rarm_prev    <= rarm_qual;
            rdisarm_prev <= rdisarm_qual;
        end
    end

    wire rarm_re    = rarm_qual    & ~rarm_prev;    // remote arm rising edge
    wire rdisarm_re = rdisarm_qual & ~rdisarm_prev; // remote disarm rising edge

    // â”€â”€ SW_RST_FAULT debounce â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    //
    // Mechanical pushbutton.  The button must remain stable for T_DEBOUNCE_RST_MS
    // ticks before a rising edge event is generated.  This prevents bounce
    // glitches from triggering multiple reset attempts.

    logic [clog2(T_DEBOUNCE_RST_MS+1)-1:0] rst_filt = '0;
    logic rst_qual = 1'b0;
    logic rst_prev = 1'b0;

    always_ff @(posedge clk_osc) begin
        if (tick_1ms) begin
            if (SW_RST_FAULT_S) rst_filt <= (rst_filt < T_DEBOUNCE_RST_MS) ? rst_filt + 1'b1 : rst_filt;
            else                 rst_filt <= '0;
            rst_qual <= (rst_filt == T_DEBOUNCE_RST_MS);
            rst_prev <= rst_qual;
        end
    end

    wire rst_fault_re = rst_qual & ~rst_prev; // debounced rising edge

    // â”€â”€ MCU_CMD_FAULT rising-edge detector â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    // Sampled on the 1 ms tick so that a pulse shorter than 1 ms but longer
    // than the synchroniser latency (~1 Âµs) will still be captured reliably.

    logic mcu_fault_prev = 1'b0;
    always_ff @(posedge clk_osc) begin
        if (tick_1ms) mcu_fault_prev <= MCU_CMD_FAULT_S;
    end
    wire mcu_fault_re = MCU_CMD_FAULT_S & ~mcu_fault_prev;

    // â”€â”€ Remote arm latch â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    //
    // The remote arm receiver sends a momentary pulse on arm and a separate
    // pulse on disarm.  This latch captures the arm state from those events.
    //
    // SET conditions  : rarm_re fires AND state is ST_COMPUTE AND SW_COMPUTE_S=1.
    //                   The guard prevents phantom set events in IDLE, PRECHARGE,
    //                   PRECHARGE_ABORT, ARMED, or FAULT.
    //
    // CLEAR conditions: rdisarm_re  (remote disarm command received)
    //                   ~SW_ARM_S   (local disarm clears remote latch)
    //                   Fault entry (cleared atomically by the state machine)
    //                   Soft abort entry (cleared when arm_ok drops in PRECHARGE)
    //
    // Declared here so it can be referenced combinationally by arm_ok (Section 5).
    logic remote_arm_latch = 1'b0;

    // =========================================================================
    // SECTION 5: ARM CONDITIONS (combinational)
    // =========================================================================

    // fault_latch_r is declared here rather than alongside the other state
    // machine registers in Section 6 because it is referenced by arm_ok.
    // Using the internal register (not the FAULT_LATCH output) gives same-tick
    // response: arm drops on the same tick that a fault is latched, before the
    // output register propagates it.
    logic fault_latch_r = 1'b0;

    // arm_ok â€” all conditions required to enter or skip precharge.
    //
    //   SW_ARM_S           Physical arm switch held by operator.
    //   remote_arm_latch   Remote arm event received and not yet cleared.
    //   ESTOP_OK_S         Emergency stop chain healthy.
    //   ~(ov_filt|uv_filt) Supply voltage within operating range.
    //   MCU_CMD_ARM_S      MCU has granted arm permission.
    //   ~fault_latch_r     No active latched fault.
    //
    //   Note: VBUS_OV is intentionally absent.  Dump is orthogonal to arming;
    //   the VBUS_OV threshold should be trimmed above the source OV threshold
    //   so that normal bus transients do not block arm entry.
    wire arm_ok =   SW_ARM_S
                  & remote_arm_latch
                  & ESTOP_OK_S
                  & ~(ov_filt | uv_filt)
                  & MCU_CMD_ARM_S
                  & ~fault_latch_r;

    // arm_hold â€” all conditions required to remain in ST_ARMED.
    // Identical to arm_ok: any condition that would have blocked arming also
    // triggers a soft de-arm.  VBUS_OV is absent here too; dump coexists
    // safely with the contactor closed.
    wire arm_hold =  SW_ARM_S
                   & remote_arm_latch
                   & ESTOP_OK_S
                   & ~(ov_filt | uv_filt)
                   & MCU_CMD_ARM_S
                   & ~fault_latch_r;

    // Convenience: supply OV/UV combined health flag
    wire ovuv_ok_w = ~(ov_filt | uv_filt);

    // =========================================================================
    // SECTION 6: MAIN STATE MACHINE AND TIMERS
    // =========================================================================

    // â”€â”€ State encoding â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    typedef enum logic [2:0] {
        ST_IDLE             = 3'd0,  // STATE_CODE = 0
        ST_COMPUTE          = 3'd3,  // STATE_CODE = 3
        ST_PRECHARGE        = 3'd1,  // STATE_CODE = 1
        ST_PRECHARGE_ABORT  = 3'd5,  // STATE_CODE = 1 (same as PRECHARGE â€” relay still active)
        ST_ARMED            = 3'd2,  // STATE_CODE = 2
        ST_FAULT            = 3'd4   // STATE_CODE = 0 (FAULT_LATCH distinguishes from IDLE)
    } state_t;

    state_t state = ST_IDLE;

    // â”€â”€ Internal state registers â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    logic precharge_latch_r    = 1'b0;
    logic [3:0] fault_code_r   = 4'h0;
    logic pchg_abort_was_fault = 1'b0;

    // â”€â”€ Timer downcounters â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    logic [clog2(T_KSEL_BLANK_MS+1)-1:0]        t_ksel_blank       = '0;
    logic [clog2(T_BLANK_OPEN_MS+1)-1:0]         t_blank_open       = '0;
    logic [clog2(T_BLANK_CLOSE_MS+1)-1:0]        t_blank_close      = '0;
    logic [clog2(T_PRECHG_MAX_MS+1)-1:0]         t_precharge        = '0;
    logic [clog2(T_PRECHG_ABORT_HOLD_MS+1)-1:0]  t_prechg_abort     = '0;
    logic [clog2(T_PRECHG_ABORT_MIN_MS+1)-1:0]   t_prechg_abort_min = '0;

    wire ksel_blank_active  = (t_ksel_blank  != '0);
    wire blank_open_active  = (t_blank_open  != '0);
    wire blank_close_active = (t_blank_close != '0);

    // â”€â”€ Dump / relay combinational wires â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    // precharge_active covers both PRECHARGE and PRECHARGE_ABORT.
    wire precharge_active = (state == ST_PRECHARGE) | (state == ST_PRECHARGE_ABORT);
    wire dump_req         = vbusov_filt & ~precharge_active;
    wire ksel_next        = dump_req;
    wire ksel_changing    = (ksel_next != K_SEL);
    wire ken_next         = (precharge_active | dump_req) & ~ksel_blank_active & ~ksel_changing;

    // â”€â”€ Fault condition wires â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    wire weld_detected  = ENABLE_FB_CLOSED & FB_CLOSED_S & ~MOTOR_EN & ~blank_open_active;
    wire no_close_fault = ENABLE_FB_CLOSED & (t_blank_close == '0) & ~FB_CLOSED_S & (state == ST_ARMED);
    wire prechg_timeout = (t_precharge == '0) & ~pchgok_filt & (state == ST_PRECHARGE);

    wire fault_set =   ~ESTOP_OK_S
                     | ov_filt
                     | uv_filt
                     | weld_detected
                     | prechg_timeout
                     | mcu_fault_re
                     | no_close_fault;

    function automatic logic [3:0] fault_code_enc(
        input logic estop, ov, uv, weld, pchg_to, mcu_f, no_close
    );
        if      (estop)    fault_code_enc = 4'd1;
        else if (ov)       fault_code_enc = 4'd2;
        else if (uv)       fault_code_enc = 4'd3;
        else if (weld)     fault_code_enc = 4'd4;
        else if (pchg_to)  fault_code_enc = 4'd5;
        else if (mcu_f)    fault_code_enc = 4'd6;
        else if (no_close) fault_code_enc = 4'd7;
        else               fault_code_enc = 4'd15;
    endfunction

    always_ff @(posedge clk_osc) begin
        if (tick_1ms) begin

            // â”€â”€ Decrement all running timers â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
            if (t_ksel_blank       != '0) t_ksel_blank       <= t_ksel_blank       - 1'b1;
            if (t_blank_open       != '0) t_blank_open       <= t_blank_open       - 1'b1;
            if (t_blank_close      != '0) t_blank_close      <= t_blank_close      - 1'b1;
            if (t_precharge        != '0) t_precharge        <= t_precharge        - 1'b1;
            if (t_prechg_abort     != '0) t_prechg_abort     <= t_prechg_abort     - 1'b1;
            if (t_prechg_abort_min != '0) t_prechg_abort_min <= t_prechg_abort_min - 1'b1;

            // â”€â”€ K_SEL blank timer reload â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
            if (ksel_next != K_SEL)
                t_ksel_blank <= T_KSEL_BLANK_MS[clog2(T_KSEL_BLANK_MS+1)-1:0];

            // â”€â”€ Remote arm latch â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
            // SET only in ST_COMPUTE with SW_COMPUTE_S.
            if (rarm_re && (state == ST_COMPUTE) && SW_COMPUTE_S)
                remote_arm_latch <= 1'b1;
            if (rdisarm_re | ~SW_ARM_S)
                remote_arm_latch <= 1'b0;

            // â”€â”€ State transitions â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
            case (state)

                // â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
                // ST_IDLE: SW_COMPUTE=0; system quiescent.
                // â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
                ST_IDLE: begin
                    if (fault_set) begin
                        state            <= ST_FAULT;
                        fault_latch_r    <= 1'b1;
                        fault_code_r     <= fault_code_enc(~ESTOP_OK_S, ov_filt, uv_filt,
                                               weld_detected, prechg_timeout,
                                               mcu_fault_re, no_close_fault);
                        remote_arm_latch <= 1'b0;
                    end else if (SW_COMPUTE_S) begin
                        state <= ST_COMPUTE;
                    end
                end

                // â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
                // ST_COMPUTE: system ready; waiting for full arm permission.
                // â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
                ST_COMPUTE: begin
                    if (fault_set) begin
                        state            <= ST_FAULT;
                        fault_latch_r    <= 1'b1;
                        fault_code_r     <= fault_code_enc(~ESTOP_OK_S, ov_filt, uv_filt,
                                               weld_detected, prechg_timeout,
                                               mcu_fault_re, no_close_fault);
                        remote_arm_latch <= 1'b0;
                    end else if (~SW_COMPUTE_S) begin
                        state <= ST_IDLE;
                    end else if (arm_ok & pchgok_filt) begin
                        state             <= ST_ARMED;
                        precharge_latch_r <= 1'b0;
                        if (ENABLE_FB_CLOSED)
                            t_blank_close <= T_BLANK_CLOSE_MS[clog2(T_BLANK_CLOSE_MS+1)-1:0];
                    end else if (arm_ok & ~pchgok_filt) begin
                        state             <= ST_PRECHARGE;
                        precharge_latch_r <= 1'b1;
                        t_precharge       <= T_PRECHG_MAX_MS[clog2(T_PRECHG_MAX_MS+1)-1:0];
                    end
                end

                // â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
                // ST_PRECHARGE: bus charging through NC precharge resistor.
                // â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
                ST_PRECHARGE: begin
                    if (fault_set) begin
                        state                <= ST_PRECHARGE_ABORT;
                        precharge_latch_r    <= 1'b0;
                        fault_latch_r        <= 1'b1;
                        fault_code_r         <= fault_code_enc(~ESTOP_OK_S, ov_filt, uv_filt,
                                                   weld_detected, prechg_timeout,
                                                   mcu_fault_re, no_close_fault);
                        remote_arm_latch     <= 1'b0;
                        pchg_abort_was_fault <= 1'b1;
                        t_prechg_abort       <= T_PRECHG_ABORT_HOLD_MS[clog2(T_PRECHG_ABORT_HOLD_MS+1)-1:0];
                        t_prechg_abort_min   <= T_PRECHG_ABORT_MIN_MS[clog2(T_PRECHG_ABORT_MIN_MS+1)-1:0];
                    end else if (~arm_ok) begin
                        state                <= ST_PRECHARGE_ABORT;
                        precharge_latch_r    <= 1'b0;
                        remote_arm_latch     <= 1'b0;
                        pchg_abort_was_fault <= 1'b0;
                        t_prechg_abort       <= T_PRECHG_ABORT_HOLD_MS[clog2(T_PRECHG_ABORT_HOLD_MS+1)-1:0];
                        t_prechg_abort_min   <= T_PRECHG_ABORT_MIN_MS[clog2(T_PRECHG_ABORT_MIN_MS+1)-1:0];
                    end else if (pchgok_filt) begin
                        state             <= ST_ARMED;
                        precharge_latch_r <= 1'b0;
                        t_precharge       <= '0;
                        if (ENABLE_FB_CLOSED)
                            t_blank_close <= T_BLANK_CLOSE_MS[clog2(T_BLANK_CLOSE_MS+1)-1:0];
                    end
                end

                // â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
                // ST_PRECHARGE_ABORT: relay held closed after precharge abort.
                // â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
                ST_PRECHARGE_ABORT: begin
                    if (fault_set && !pchg_abort_was_fault) begin
                        fault_latch_r        <= 1'b1;
                        fault_code_r         <= fault_code_enc(~ESTOP_OK_S, ov_filt, uv_filt,
                                                   weld_detected, prechg_timeout,
                                                   mcu_fault_re, no_close_fault);
                        pchg_abort_was_fault <= 1'b1;
                        remote_arm_latch     <= 1'b0;
                    end
                    if ((t_prechg_abort_min == '0) &&
                        (pchgok_filt || (t_prechg_abort == '0))) begin
                        t_blank_open         <= T_BLANK_OPEN_MS[clog2(T_BLANK_OPEN_MS+1)-1:0];
                        pchg_abort_was_fault <= 1'b0;
                        if (pchg_abort_was_fault || fault_set)
                            state <= ST_FAULT;
                        else
                            state <= SW_COMPUTE_S ? ST_COMPUTE : ST_IDLE;
                    end
                end

                // â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
                // ST_ARMED: main contactor closed; motor load connected.
                // â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
                ST_ARMED: begin
                    if (fault_set) begin
                        state            <= ST_FAULT;
                        fault_latch_r    <= 1'b1;
                        fault_code_r     <= fault_code_enc(~ESTOP_OK_S, ov_filt, uv_filt,
                                               weld_detected, prechg_timeout,
                                               mcu_fault_re, no_close_fault);
                        remote_arm_latch <= 1'b0;
                        t_blank_open     <= T_BLANK_OPEN_MS[clog2(T_BLANK_OPEN_MS+1)-1:0];
                    end else if (~arm_hold) begin
                        state        <= ST_COMPUTE;
                        t_blank_open <= T_BLANK_OPEN_MS[clog2(T_BLANK_OPEN_MS+1)-1:0];
                    end
                end

                // â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
                // ST_FAULT: latched fault; all drive outputs de-asserted.
                // â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
                ST_FAULT: begin
                    if (rst_fault_re
                            & ESTOP_OK_S
                            & ~remote_arm_latch
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
    // SECTION 7: DUMP LOGIC
    // =========================================================================
    //
    // dump_req = vbusov_filt & ~precharge_active.
    // All combinational wires (precharge_active, dump_req, ksel_next,
    // ksel_changing, ken_next) declared in Section 6 to avoid forward-reference
    // issues.  See file header for full description and K_SEL polarity notes.

    // =========================================================================
    // SECTION 8: OUTPUT DECODE (all registered, updated every 1 ms tick)
    // =========================================================================

    always_ff @(posedge clk_osc) begin
        if (tick_1ms) begin

            COMPUTE_EN      <= SW_COMPUTE_S;
            MOTOR_EN        <= (state == ST_ARMED);
            K_SEL           <= ksel_next;
            K_EN            <= ken_next;
            FAULT_LATCH     <= fault_latch_r;
            PRECHARGE_LATCH <= precharge_latch_r;
            FAULT_CODE      <= fault_code_r;

            // ST_PRECHARGE_ABORT outputs STATE_CODE=1 (same as ST_PRECHARGE â€”
            // relay still active on precharge path).
            case (state)
                ST_IDLE:             STATE_CODE <= 2'd0;
                ST_COMPUTE:          STATE_CODE <= 2'd3;
                ST_PRECHARGE:        STATE_CODE <= 2'd1;
                ST_PRECHARGE_ABORT:  STATE_CODE <= 2'd1;
                ST_ARMED:            STATE_CODE <= 2'd2;
                ST_FAULT:            STATE_CODE <= 2'd0;
                default:             STATE_CODE <= 2'd0;
            endcase

            OVUV_OK       <= ovuv_ok_w;
            ARM_PERMIT    <= arm_ok;
            PRECHARGE_REQ <= precharge_active;
            DUMP_EN       <= dump_req & ken_next;
            SPARE_OUT_0   <= spare_heartbeat;
            SPARE_OUT_1   <= arm_ok;
        end
    end

    // =========================================================================
    // SECTION 9: EFB I2C WISHBONE ENGINE
    // =========================================================================
    //
    // The MachXO2 EFB contains a hardened I2C slave controller accessible via
    // an internal Wishbone bus.  This engine manages the Wishbone transactions
    // needed to service the I2C interface.
    //
    // The I2C slave is configured as read-only monitor: MCU writes a 1-byte
    // register pointer, then reads back N bytes (auto-incrementing pointer).
    //
    // Key design constraints:
    //   - The handler must respond to TRRDY within a few I2C bit-periods.
    //     At 100 kHz I2C, one bit is 10 Âµs; the Wishbone round-trip at
    //     2.08 MHz is ~4 cycles (~2 Âµs), so timing is comfortably met.
    //   - The handler is woken by i2c1_irqo (IRQ from EFB) rather than
    //     polling on tick_1ms.  At 2.08 MHz / 100 kHz I2C, a full byte
    //     takes 80 cycles (~40 Âµs); polling every 1 ms would miss the byte.
    //   - After each byte service (RX or TX) the handler returns directly to
    //     I2C_RD_SR rather than back to I2C_IDLE, enabling back-to-back burst
    //     reads without a round-trip through IDLE.
    //   - A tick_1ms fallback wakeup is included to recover from any missed IRQ.

    logic       wb_rst_i;
    logic       wb_cyc_i, wb_stb_i, wb_we_i;
    logic [7:0] wb_adr_i, wb_dat_i;
    wire  [7:0] wb_dat_o;
    wire        wb_ack_o;
    wire        i2c1_irqo;

    // Hold Wishbone reset asserted for several ms after FPGA configuration.
    logic [2:0] wb_rst_ms = 3'd5;
    always_ff @(posedge clk_osc) begin
        if (tick_1ms)
            if (wb_rst_ms != 0) wb_rst_ms <= wb_rst_ms - 1'b1;
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

    // â”€â”€ Wishbone transaction engine â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
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

    localparam integer SR_SRW   = 1;  // I2C1SR[1]: 0=slave receiving, 1=slave transmitting
    localparam integer SR_TRRDY = 2;  // I2C1SR[2]: TX/RX data register ready

    function automatic logic [7:0] reg_read(input logic [7:0] a);
        case (a)
            8'h00: reg_read = {FAULT_LATCH, PRECHARGE_LATCH, MOTOR_EN, COMPUTE_EN,
                               K_SEL, K_EN, OVUV_OK, ARM_PERMIT};
            8'h01: reg_read = {4'h0, FAULT_CODE};
            8'h02: reg_read = {6'h00, STATE_CODE};
            8'h03: reg_read = {4'h0, dump_req, DUMP_EN, PRECHARGE_REQ, K_EN};
            8'h04: reg_read = {arm_ok, remote_arm_latch, ESTOP_OK_S, MCU_CMD_ARM_S,
                               MCU_ALIVE_S, FB_CLOSED_S, PRECHARGE_OK_S, VBUS_OV_S};
            8'h05: reg_read = t_precharge[clog2(T_PRECHG_MAX_MS+1)-1:8];
            8'h06: reg_read = t_precharge[7:0];
            8'h7F: reg_read = 8'hB2;
            default: reg_read = 8'h00;
        endcase
    endfunction

    // â”€â”€ I2C handler state machine â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    typedef enum logic [2:0] {
        I2C_IDLE    = 3'd0,
        I2C_RD_SR   = 3'd1,
        I2C_WAIT_SR = 3'd2,
        I2C_RD_RX   = 3'd3,
        I2C_WAIT_RX = 3'd4,
        I2C_WR_TX   = 3'd5,
        I2C_WAIT_TX = 3'd6
    } i2c_state_t;

    i2c_state_t i2c_state = I2C_IDLE;
    logic [7:0]  tx_ptr   = 8'h00;

    always_comb begin
        wb_req     = 1'b0;
        wb_req_we  = 1'b0;
        wb_req_adr = 8'h00;
        wb_req_dat = 8'h00;
        unique case (i2c_state)
            I2C_RD_SR: begin wb_req = 1'b1; wb_req_adr = 8'h45; end
            I2C_RD_RX: begin wb_req = 1'b1; wb_req_adr = 8'h47; end
            I2C_WR_TX: begin wb_req = 1'b1; wb_req_we  = 1'b1;
                             wb_req_adr = 8'h44;
                             wb_req_dat = reg_read(tx_ptr); end
            default: ;
        endcase
    end

    always_ff @(posedge clk_osc) begin
        if (wb_rst_i) begin
            i2c_state <= I2C_IDLE;
            tx_ptr    <= 8'h00;
        end else begin
            case (i2c_state)
                I2C_IDLE:    if (i2c1_irqo || tick_1ms) i2c_state <= I2C_RD_SR;
                I2C_RD_SR:   if (wb_state == WB_WAIT) i2c_state <= I2C_WAIT_SR;
                I2C_WAIT_SR: if (wb_resp_valid) begin
                    if (wb_resp_dat[SR_TRRDY])
                        i2c_state <= wb_resp_dat[SR_SRW] ? I2C_WR_TX : I2C_RD_RX;
                    else
                        i2c_state <= I2C_IDLE;
                end
                I2C_RD_RX:   if (wb_state == WB_WAIT) i2c_state <= I2C_WAIT_RX;
                I2C_WAIT_RX: if (wb_resp_valid) begin tx_ptr <= wb_resp_dat;        i2c_state <= I2C_RD_SR; end
                I2C_WR_TX:   if (wb_state == WB_WAIT) i2c_state <= I2C_WAIT_TX;
                I2C_WAIT_TX: if (wb_resp_valid) begin tx_ptr <= tx_ptr + 1'b1;      i2c_state <= I2C_RD_SR; end
                default:     i2c_state <= I2C_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
