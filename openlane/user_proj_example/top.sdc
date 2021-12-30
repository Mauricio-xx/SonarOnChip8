set_units -time ns
create_clock -name wb_clk_i -period 40 {wb_clk_i }
set_timing_derate -early 0.9
set_timing_derate -late  1.1
set_clock_uncertainty  0.25 [get_clocks wb_clk_i]
set_clock_transition   0.15 [get_clocks wb_clk_i]
# --------------------------------- GLOABAL -------------------------------------------------------
# --------------------------------- PIN SPECIFIC VALUES----------------------------------------------------
# set input delay
set_input_delay -clock wb_clk_i 5  [get_ports wb_rst_i]
set_input_delay -clock wb_clk_i 5  [get_ports wbs_stb_i]
set_input_delay -clock wb_clk_i 5  [get_ports wbs_cyc_i]
set_input_delay -clock wb_clk_i 5  [get_ports wbs_dat_i*]
set_input_delay -clock wb_clk_i 5  [get_ports wbs_adr_i*]
set_input_delay -clock wb_clk_i 5  [get_ports wbs_sel_i*]
set_input_delay -clock wb_clk_i 5  [get_ports wbs_we_i]
set_input_delay -clock wb_clk_i 5  [get_ports la_data_i*]
set_input_delay -clock wb_clk_i 5  [get_ports la_oenb*]
# set input driving cell
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_8  [get_ports wb_rst_i]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_8  [get_ports wbs_stb_i]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_8  [get_ports wbs_cyc_i]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_8  [get_ports wbs_we_i]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_8  [get_ports wbs_sel_i*]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_8  [get_ports wbs_dat_i*]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_8  [get_ports wbs_adr_i*]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_8  [get_ports la_data_in*]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_8  [get_ports la_oenb*]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_8  [get_ports io_in*]
# set input delay and driving cell for the external inputs of Mega Project Area
set_input_delay  -clock wb_clk_i                  5   [get_ports io_in*]
set_driving_cell -lib_cell sky130_fd_sc_hd__einvp_8   [get_ports io_in*]
#set outputa  delays to the Caravel (by default calculated by RCX)
set_output_delay -clock wb_clk_i 5 [get_ports wbs_ack_o*]
set_output_delay -clock wb_clk_i 5 [get_ports wbs_dat_o*]
set_output_delay -clock wb_clk_i 5 [get_ports la_data_out*]
set_output_delay -clock wb_clk_i 5 [get_ports irq*]
# # outputs loads to the Caravel SoC
set_load 0.035 [get_ports wbs_ack_o*]
set_load 0.035 [get_ports wbs_dat_o*]
set_load 0.035 [get_ports la_data_out*]
set_load 0.035 [get_ports irq*]
# set outputs delay for the external outputs of Mega Project Area
set_output_delay -clock wb_clk_i   5 [get_ports io_out*]
set_output_delay -clock wb_clk_i   5 [get_ports io_oeb*]
# set load to the capacitance of "sky130_fd_sc_hd__mux2_1"
set_load 0.035 [get_ports io_out*]
set_load 0.035 [get_ports io_oeb*]
#









