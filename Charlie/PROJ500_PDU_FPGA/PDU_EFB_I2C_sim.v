// PDU_EFB_I2C_sim.v — Simulation stub for the EFB I2C wrapper.
// Replaces PDU_EFB_I2C.v during QuestaSim runs.
// The EFB hard macro has no Verilog simulation model; this stub provides:
//   - 1-cycle Wishbone ACK (data=0x00) so the WB engine never stalls
//   - I2C pins left high-Z
//   - No IRQ
// All functional tests target the PDU state machine, not the I2C peripheral.

`timescale 1ns/1ps

module PDU_EFB_I2C (
    input  wire       wb_clk_i,
    input  wire       wb_rst_i,
    input  wire       wb_cyc_i,
    input  wire       wb_stb_i,
    input  wire       wb_we_i,
    input  wire [7:0] wb_adr_i,
    input  wire [7:0] wb_dat_i,
    output reg  [7:0] wb_dat_o,
    output reg        wb_ack_o,
    inout  wire       i2c1_scl,
    inout  wire       i2c1_sda,
    output wire       i2c1_irqo
);
    assign i2c1_scl  = 1'bz;
    assign i2c1_sda  = 1'bz;
    assign i2c1_irqo = 1'b0;

    always @(posedge wb_clk_i) begin
        if (wb_rst_i) begin
            wb_ack_o <= 1'b0;
            wb_dat_o <= 8'h00;
        end else begin
            // Single-cycle ACK on any active transaction
            wb_ack_o <= wb_cyc_i & wb_stb_i & ~wb_ack_o;
            wb_dat_o <= 8'h00;
        end
    end
endmodule
