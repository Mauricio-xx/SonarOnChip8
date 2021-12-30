# SPDX-FileCopyrightText: 2020 Efabless Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# SPDX-License-Identifier: Apache-2.0

set ::env(PDK) "sky130A"
set ::env(STD_CELL_LIBRARY) "sky130_fd_sc_hd"

set script_dir [file dirname [file normalize [info script]]]

set ::env(DESIGN_NAME) user_proj_example

set ::env(VERILOG_FILES) "\
    $::env(CARAVEL_ROOT)/verilog/rtl/defines.v \
	$script_dir/../../verilog/rtl/SonarOnChip.v \
	$script_dir/../../verilog/rtl/user_proj_example.v"




set ::env(DESIGN_IS_CORE) 0

set ::env(CLOCK_PORT) "wb_clk_i"
set ::env(CLOCK_NET) "wb_clk_i"
set ::env(CLOCK_PERIOD) "40"

set ::env(FP_SIZING) absolute
#set ::env(DIE_AREA) "0 0 2700 3100"
set ::env(DIE_AREA) "0 0 2700 3100"

set ::env(FP_PIN_ORDER_CFG) $script_dir/pin_order.cfg
set ::env(PL_SKIP_INITIAL_PLACEMENT) 0
#set ::env(PL_BASIC_PLACEMENT) 1
set ::env(PL_TARGET_DENSITY) 0.25
set ::env(FP_CORE_UTIL) 60
set ::env(GLB_RT_ALLOW_CONGESTION) 1
set ::env(ROUTING_CORES) 8

#set ::env(PL_RESIZER_MAX_WIRE_LENGTH) 100
#set ::env(PL_RESIZER_HOLD_SLACK_MARGIN) 0.2  4 chann
#set ::env(PL_RESIZER_HOLD_SLACK_MARGIN) 0.25  
#8 chann placement 0.35
set ::env(PL_RESIZER_MAX_SLEW_MARGIN) 30
set ::env(PL_RESIZER_MAX_CAP_MARGIN) 30
#set ::env(WIRE_RC_LAYER) "met2"

set ::env(PL_RESIZER_HOLD_SLACK_MARGIN) 0.5
#set ::env(PL_RESIZER_HOLD_BUFFER_PERCENT) 90
#set ::env(SPEF_WIRE_MODEL) "Pi"

#set ::env(DECAP_CELL) "sky130_ef_sc_hd__decap_12 sky130_fd_sc_hd__decap_3 sky130_fd_sc_hd__decap_4 sky130_fd_sc_hd__decap_6 sky130_fd_sc_hd__decap_8"

set ::env(BASE_SDC_FILE) "$script_dir/top.sdc"

# Maximum layer used for routing is metal 4.
# This is because this macro will be inserted in a top level (user_project_wrapper) 
# where the PDN is planned on metal 5. So, to avoid having shorts between routes
# in this macro and the top level metal 5 stripes, we have to restrict routes to metal4.  
set ::env(GLB_RT_MAXLAYER) 5

# You can draw more power domains if you need to 
set ::env(VDD_NETS) [list {vccd1}]
set ::env(GND_NETS) [list {vssd1}]


set ::env(DIODE_INSERTION_STRATEGY) 4
# If you're going to use multiple power domains, then disable cvc run.

set ::env(RUN_CVC) 1

#............. only for testing .....
#set ::env(MAGIC_DRC_USE_GDS) 0
#set ::env(KLAYOUT_XOR_GDS) 0
#set ::env(KLAYOUT_XOR_XML) 0
#set ::env(RUN_KLAYOUT_XOR) 0
#set ::env(USE_ARC_ANTENNA_CHECK) 1

