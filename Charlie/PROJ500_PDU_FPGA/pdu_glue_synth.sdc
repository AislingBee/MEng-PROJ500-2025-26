# Synthesis timing constraint for OSCH 2.08 MHz internal oscillator
# Period = 1/2.08MHz = 480.77 ns
create_clock -name {clk_osc} -period 480.77 [get_nets {clk_osc}]
