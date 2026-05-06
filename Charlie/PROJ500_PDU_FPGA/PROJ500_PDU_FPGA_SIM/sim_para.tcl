lappend auto_path "C:/Programs/Lattice Diamond/data/script"
package require simulation_generation
set ::bali::simulation::Para(DEVICEFAMILYNAME) {MachXO2}
set ::bali::simulation::Para(PROJECT) {PROJ500_PDU_FPGA_SIM}
set ::bali::simulation::Para(PROJECTPATH) {C:/Users/charl/Documents/TEMP BACKUPS/17.03.26/PROJ500 (Humanoid)/FPGA/PROJ500_PDU_FPGA}
set ::bali::simulation::Para(FILELIST) {"C:/Users/charl/Documents/TEMP BACKUPS/17.03.26/PROJ500 (Humanoid)/FPGA/PROJ500_PDU_FPGA/pdu_selftest_mxo2.sv" "C:/Users/charl/Documents/TEMP BACKUPS/17.03.26/PROJ500 (Humanoid)/FPGA/PROJ500_PDU_FPGA/tb_pdu_selftest_mxo2.sv" }
set ::bali::simulation::Para(GLBINCLIST) {}
set ::bali::simulation::Para(INCLIST) {"none" "none"}
set ::bali::simulation::Para(WORKLIBLIST) {"work" "work" }
set ::bali::simulation::Para(COMPLIST) {"VERILOG" "VERILOG" }
set ::bali::simulation::Para(LANGSTDLIST) {"System Verilog" "System Verilog" }
set ::bali::simulation::Para(SIMLIBLIST) {pmi_work ovi_machxo2}
set ::bali::simulation::Para(MACROLIST) {}
set ::bali::simulation::Para(SIMULATIONTOPMODULE) {tb_pdu_selftest_mxo2}
set ::bali::simulation::Para(SIMULATIONINSTANCE) {}
set ::bali::simulation::Para(LANGUAGE) {}
set ::bali::simulation::Para(SDFPATH)  {}
set ::bali::simulation::Para(INSTALLATIONPATH) {C:/Programs/Lattice Diamond}
set ::bali::simulation::Para(ADDTOPLEVELSIGNALSTOWAVEFORM)  {1}
set ::bali::simulation::Para(RUNSIMULATION)  {1}
set ::bali::simulation::Para(SIMULATION_RESOLUTION)  {default}
set ::bali::simulation::Para(HDLPARAMETERS) {}
set ::bali::simulation::Para(POJO2LIBREFRESH)    {}
set ::bali::simulation::Para(POJO2MODELSIMLIB)   {}
set ::bali::simulation::Para(OPTIMIZEARGS)  {+acc}
set ::bali::simulation::Para(OPTIMIZATION_DEBUG)  {1}
::bali::simulation::QuestaSim_Run
